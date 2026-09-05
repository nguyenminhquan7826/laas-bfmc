#include "Executive.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "../laas_core/Time.hpp"

namespace laas {

namespace {

const char* operatingModeToString(OperatingMode mode)
{
    switch (mode) {
    case OperatingMode::LANE_DRIVING: return "LANE";
    case OperatingMode::PARKING: return "PARKING";
    default: return "UNKNOWN";
    }
}

const char* behaviorToString(BehaviorMode mode)
{
    switch (mode) {
    case BehaviorMode::STOP: return "STOP";
    case BehaviorMode::KEEP_LANE: return "KEEP";
    case BehaviorMode::FOLLOW_LANE: return "FOLLOW";
    case BehaviorMode::AVOID_OBSTACLE: return "AVOID";
    case BehaviorMode::EMERGENCY_STOP: return "ESTOP";
    default: return "UNKNOWN";
    }
}

const char* plannerStateToString(PlannerState state)
{
    switch (state) {
    case PlannerState::KEEP_LANE: return "KEEP";
    case PlannerState::CHANGE_USING_DASHED: return "CHANGE";
    case PlannerState::FOLLOW_LANE: return "FOLLOW";
    default: return "UNKNOWN";
    }
}

const char* directionToString(ChangeDirection direction)
{
    switch (direction) {
    case ChangeDirection::LEFT: return "LEFT";
    case ChangeDirection::RIGHT: return "RIGHT";
    case ChangeDirection::NONE:
    default: return "NONE";
    }
}

int getchNonBlocking()
{
    termios oldt{};
    termios newt{};

    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {
        return -1;
    }

    newt = oldt;
    newt.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

    const int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, old_flags);

    return ch;
}

}  // namespace

Executive::Executive(const Config& config)
    : config_(config),
      camera_(config_),
      yolo_(config_),
      vehicle_(config_),
#ifdef LAAS_ENABLE_PARKING_CLIENT
      parking_server_(config_),
#endif
      lane_perception_(config_),
      parking_status_bench_source_(config_),
      vehicle_pose_estimator_(config_),
      mission_(config_),
      planner_(config_),
      pure_pursuit_(config_),
      parking_trajectory_tracker_(config_),
#ifdef LAAS_ENABLE_MPC
      mpc_(config_),
#endif
      safety_(config_),
      parking_trajectory_validator_(config_),
      parking_safety_filter_(config_)
{
    configureScheduler();
}

Executive::~Executive()
{
    stop();
}

void Executive::configureScheduler()
{
    scheduler_.configure(config_.runtime.camera_period_ms,
                         config_.runtime.yolo_period_ms,
                         config_.runtime.perception_period_ms,
                         config_.runtime.decision_period_ms,
                         config_.runtime.planning_period_ms,
                         config_.runtime.control_period_ms,
                         config_.runtime.logging_period_ms);
}

bool Executive::init()
{
    state_.store(RuntimeState::INIT);

    if (!camera_.init()) {
        std::cerr << "[EXEC] Camera init failed.\n";
        state_.store(RuntimeState::ERROR);
        return false;
    }

    if (!yolo_.init()) {
        std::cerr << "[EXEC] UDP YOLO init failed.\n";
        state_.store(RuntimeState::ERROR);
        return false;
    }

    if (!vehicle_.init()) {
        std::cerr << "[EXEC] Vehicle UART init failed.\n";
        state_.store(RuntimeState::ERROR);
        return false;
    }

#ifdef LAAS_ENABLE_PARKING_CLIENT
    if (config_.parking.enable) {
        parking_server_.init();
    }
#else
    if (config_.parking.enable) {
        std::cerr << "[PARKING] Parking client unavailable in this build. "
                  << "Install libjson-c-dev and rebuild.\n";
    }
#endif

    const uint64_t now = nowMs();
    scheduler_.reset(now);
    state_.store(RuntimeState::READY);

    std::cout << "[EXEC] Ready. Keyboard: R=run, S=stop, Q=quit.\n";
    return true;
}

void Executive::run()
{
    if (state_.load() == RuntimeState::INIT) {
        if (!init()) {
            return;
        }
    }

    running_.store(true);

    while (running_.load()) {
        const uint64_t now = nowMs();

        if (config_.runtime.enable_keyboard) {
            handleKeyboardTick();
        }

        // Telemetry reception is independent of keyboard input. Headless
        // operation must continue draining STM32 telemetry every loop.
        telemetryTick();
        parkingNetworkTick();

        if (scheduler_.camera.ready(now)) {
            scheduler_.camera.mark(now);
            cameraTick();
        }

        if (scheduler_.yolo.ready(now)) {
            scheduler_.yolo.mark(now);
            yoloTick();
        }

        if (scheduler_.perception.ready(now)) {
            scheduler_.perception.mark(now);
            perceptionTick();
        }

        if (scheduler_.decision.ready(now)) {
            scheduler_.decision.mark(now);
            decisionTick();
        }

        if (scheduler_.planning.ready(now)) {
            scheduler_.planning.mark(now);
            planningTick();
        }

        if (scheduler_.control.ready(now)) {
            scheduler_.control.mark(now);
            controlTick();
            parkingBenchControlTick();
        }

        if (scheduler_.logging.ready(now)) {
            scheduler_.logging.mark(now);
            loggingTick();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ControlCmdMsg stop_cmd;
    stop_cmd.header.valid = true;
    stop_cmd.header.timestamp_ms = nowMs();
    stop_cmd.speed_mps = 0.0f;
    stop_cmd.steering_deg = 0.0f;
    stop_cmd.servo_cmd = static_cast<int>(std::lround(config_.vehicle.servo_center));
    vehicle_.send(stop_cmd);

    state_.store(RuntimeState::STOPPED);
    std::cout << "[EXEC] Stopped cleanly.\n";
}

void Executive::stop()
{
    running_.store(false);
    yolo_.close();
#ifdef LAAS_ENABLE_PARKING_CLIENT
    parking_server_.close();
#endif
    vehicle_.close();
    camera_.close();
}

void Executive::setUserRunRequest(bool enabled)
{
    user_run_request_.store(enabled);
}

void Executive::handleKeyboardTick()
{
    const int key = getchNonBlocking();
    if (key == 'r' || key == 'R') {
        setUserRunRequest(true);
        std::cout << "[EXEC] User request: RUN\n";
    } else if (key == 's' || key == 'S') {
        setUserRunRequest(false);
        std::cout << "[EXEC] User request: STOP\n";
    } else if (key == 'q' || key == 'Q' || key == 27) {
        setUserRunRequest(false);
        running_.store(false);
        std::cout << "[EXEC] User request: QUIT\n";
    }
}

void Executive::cameraTick()
{
    FrameMsg frame;
    if (camera_.grab(frame)) {
        blackboard_.setFrame(frame);
    }
}

void Executive::yoloTick()
{
    if (!config_.runtime.enable_yolo_udp) {
        return;
    }

    const FrameMsg frame = blackboard_.frame();
    if (frame.header.valid &&
        frame.header.timestamp_ms != last_yolo_frame_timestamp_ms_ &&
        isFresh(nowMs(), frame.header.timestamp_ms,
                config_.runtime.frame_timeout_ms)) {
        if (yolo_.sendFrame(frame, 85)) {
            last_yolo_frame_timestamp_ms_ = frame.header.timestamp_ms;
        }
    }

    ObstacleMsg obstacle;
    if (yolo_.receiveObstacle(obstacle)) {
        blackboard_.setObstacle(obstacle);
    }
}

void Executive::perceptionTick()
{
    const FrameMsg frame = blackboard_.frame();
    if (!frame.header.valid ||
        frame.header.timestamp_ms == last_perception_frame_timestamp_ms_ ||
        !isFresh(nowMs(), frame.header.timestamp_ms,
                 config_.runtime.frame_timeout_ms)) {
        return;
    }

    last_perception_frame_timestamp_ms_ = frame.header.timestamp_ms;

    LanePerceptionMsg lane;
    if (lane_perception_.process(frame, lane)) {
        blackboard_.setLane(lane);
        if (config_.runtime.enable_yolo_udp && !lane.bird_eye_view.empty()) {
            yolo_.sendDebugFrame(lane.bird_eye_view, 80);
        }
    }
}

void Executive::decisionTick()
{
    const LanePerceptionMsg lane = blackboard_.lane();
    const ObstacleMsg obstacle = blackboard_.obstacle();

    BehaviorRequest behavior = mission_.update(lane, obstacle, user_run_request_.load());
    blackboard_.setBehavior(behavior);

    if (behavior.mode == BehaviorMode::EMERGENCY_STOP) {
        state_.store(RuntimeState::EMERGENCY_STOP);
    } else if (behavior.mode == BehaviorMode::STOP) {
        state_.store(RuntimeState::READY);
    } else {
        state_.store(RuntimeState::RUNNING);
    }
}

void Executive::planningTick()
{
    const LanePerceptionMsg lane = blackboard_.lane();
    const ObstacleMsg obstacle = blackboard_.obstacle();
    const BehaviorRequest behavior = blackboard_.behavior();

    TrajectoryMsg trajectory;
    planner_.process(lane, obstacle, behavior, trajectory);

    // Always publish the latest result, including a planning failure. Keeping
    // the previous valid trajectory here could make the vehicle continue on a
    // stale path after the environment becomes blocked.
    blackboard_.setTrajectory(trajectory);
}

ControlCmdMsg Executive::computeRawCommand(const TrajectoryMsg& trajectory,
                                           const LanePerceptionMsg& lane,
                                           const BehaviorRequest& behavior)
{
    ControlCmdMsg raw;
    raw.header.timestamp_ms = nowMs();
    raw.header.valid = true;
    raw.speed_mps = 0.0f;
    raw.steering_deg = 0.0f;

    if (behavior.mode == BehaviorMode::STOP || behavior.mode == BehaviorMode::EMERGENCY_STOP) {
        return raw;
    }

    if (!trajectory.header.valid || !trajectory.collision_free) {
        return raw;
    }

#ifdef LAAS_ENABLE_MPC
    if (behavior.control_mode == ControlMode::MPC) {
        if (mpc_.process(trajectory, lane, behavior, raw)) {
            return raw;
        }
        std::cerr << "[EXEC] MPC failed, falling back to Pure Pursuit for this tick.\n";
    }
#endif

    BehaviorRequest pp_behavior = behavior;
    pp_behavior.control_mode = ControlMode::PURE_PURSUIT;
    if (pure_pursuit_.process(trajectory, lane, pp_behavior, raw)) {
        return raw;
    }

    raw.header.valid = true;
    raw.speed_mps = 0.0f;
    raw.steering_deg = 0.0f;
    return raw;
}

void Executive::telemetryTick()
{
    VehicleTelemetryMsg telemetry;

    if (!vehicle_.receiveLatest(telemetry)) {
        return;
    }

    const std::uint64_t now = nowMs();

    ++telemetry_received_frames_;
    ++telemetry_window_frames_;

    if (telemetry_window_start_ms_ == 0U) {
        telemetry_window_start_ms_ = now;
    }

    const std::uint64_t window_elapsed_ms =
        now - telemetry_window_start_ms_;

    if (window_elapsed_ms >= 1000U) {
        telemetry_rx_hz_ =
            static_cast<double>(telemetry_window_frames_) *
            1000.0 /
            static_cast<double>(window_elapsed_ms);

        telemetry_window_start_ms_ = now;
        telemetry_window_frames_ = 0U;
    }

    if (have_telemetry_sequence_) {
        const std::uint32_t difference =
            telemetry.packet_sequence -
            last_telemetry_sequence_;

        if (difference == 0U) {
            ++telemetry_duplicate_frames_;
        } else if (difference < 0x80000000U) {
            telemetry_sequence_gaps_ +=
                static_cast<std::uint64_t>(difference - 1U);
        } else {
            // STM32 reset hoặc sequence quay về giá trị nhỏ.
            ++telemetry_sequence_resets_;
        }
    } else {
        have_telemetry_sequence_ = true;
    }

    last_telemetry_sequence_ =
        telemetry.packet_sequence;

    latest_telemetry_ = telemetry;

    VehiclePoseMsg pose;
    if (vehicle_pose_estimator_.process(telemetry, pose)) {
        blackboard_.setVehiclePose(pose);
    }
}

void Executive::parkingNetworkTick()
{
#ifdef LAAS_ENABLE_PARKING_CLIENT
    if (!config_.parking.enable) {
        return;
    }

    ParkingStatusMsg bench_status;
    if (parking_status_bench_source_.process(nowMs(), bench_status)) {
        blackboard_.setParkingStatus(bench_status);
    }

    parking_server_.service();
    parking_server_connected_ = parking_server_.connected();

    ParkingServerMessage server_message;
    while (parking_server_.popMessage(server_message)) {
        if (server_message.type == ParkingServerMessageType::TRAJECTORY) {
            const VehiclePoseMsg current_pose = blackboard_.vehiclePose();
            const ParkingTrajectoryValidationResult validation =
                parking_trajectory_validator_.validate(
                    server_message.trajectory, current_pose);

            if (validation.accepted) {
                blackboard_.setParkingTrajectory(server_message.trajectory);
                parking_server_.sendTrajectoryStatus(
                    parking_status_tx_sequence_++,
                    server_message.trajectory.trajectory_id,
                    "RECEIVED", validation.reason);
                std::cout << "[PARKING] trajectory tid="
                          << server_message.trajectory.trajectory_id
                          << " slot=" << server_message.trajectory.target_slot
                          << " points=" << server_message.trajectory.points.size()
                          << " PiCheck=PASS (bench only)\n";
            } else {
                parking_server_.sendTrajectoryStatus(
                    parking_status_tx_sequence_++,
                    server_message.trajectory.trajectory_id,
                    "REJECTED", validation.reason);
                std::cerr << "[PARKING] trajectory tid="
                          << server_message.trajectory.trajectory_id
                          << " PiCheck=REJECT reason=" << validation.reason
                          << "\n";
            }
        } else if (server_message.type == ParkingServerMessageType::PLANNING_RESULT) {
            std::cout << "[PARKING] planning_result=" << server_message.status
                      << " reason=" << server_message.reason << "\n";
        } else if (server_message.type == ParkingServerMessageType::ERROR) {
            std::cerr << "[PARKING] protocol/network error: "
                      << server_message.reason << "\n";
        }
    }

    if (!parking_server_connected_) {
        return;
    }

    const VehiclePoseMsg pose = blackboard_.vehiclePose();
    if (pose.header.valid &&
        (!have_sent_pose_sequence_ || pose.sequence != last_sent_pose_sequence_)) {
        if (parking_server_.sendVehiclePose(pose)) {
            last_sent_pose_sequence_ = pose.sequence;
            have_sent_pose_sequence_ = true;
        }
    }

    const ParkingStatusMsg parking_status = blackboard_.parkingStatus();
    if (parking_status.header.valid &&
        (!have_sent_parking_status_sequence_ ||
         parking_status.sequence != last_sent_parking_status_sequence_)) {
        if (parking_server_.sendParkingStatus(parking_status)) {
            last_sent_parking_status_sequence_ = parking_status.sequence;
            have_sent_parking_status_sequence_ = true;
        }
    }
#endif
}

void Executive::parkingBenchControlTick()
{
    parking_bench_raw_command_ = ControlCmdMsg{};
    parking_bench_safe_command_ = ControlCmdMsg{};
    parking_tracker_debug_ = ParkingTrackerDebug{};
    parking_safety_result_ = ParkingSafetyResult{};

    if (!config_.parking.enable || !config_.parking.bench_mode ||
        !config_.parking.enable_bench_tracker) {
        return;
    }

    const VehiclePoseMsg pose = blackboard_.vehiclePose();
    const ParkingTrajectoryMsg trajectory = blackboard_.parkingTrajectory();
    const ObstacleMsg obstacle = blackboard_.obstacle();

    // Tracker output is a raw parking command only. It is never sent to UART.
    parking_trajectory_tracker_.process(
        pose, trajectory, parking_bench_raw_command_, &parking_tracker_debug_);

    bool server_connected = false;
#ifdef LAAS_ENABLE_PARKING_CLIENT
    server_connected = parking_server_connected_;
#endif

    parking_bench_safe_command_ = parking_safety_filter_.filter(
        parking_bench_raw_command_, parking_tracker_debug_, pose, trajectory,
        latest_telemetry_, obstacle, server_connected, &parking_safety_result_);

    // Deliberately no vehicle_.send(...) here. Step-10 is observation/bench only.
}

void Executive::controlTick()
{
    const LanePerceptionMsg lane = blackboard_.lane();
    const BehaviorRequest behavior = blackboard_.behavior();
    const TrajectoryMsg trajectory = blackboard_.trajectory();

    ControlCmdMsg raw = computeRawCommand(trajectory, lane, behavior);
    ControlCmdMsg safe = safety_.filter(raw, behavior, lane, trajectory);

    blackboard_.setRawCommand(raw);
    blackboard_.setSafeCommand(safe);

    vehicle_.send(safe);
}

void Executive::loggingTick() const
{
    const VehicleTelemetryMsg telemetry = latest_telemetry_;

    const std::uint64_t now = nowMs();

    std::uint64_t telemetry_age_ms = 0U;

    if (telemetry.header.valid &&
        now >= telemetry.header.timestamp_ms) {
        telemetry_age_ms =
            now - telemetry.header.timestamp_ms;
    }

    const char* telemetry_state = "NONE";

    if (telemetry.header.valid) {
        telemetry_state =
            telemetry_age_ms <= 200U ? "OK" : "STALE";
    }


    const LanePerceptionMsg lane = blackboard_.lane();
    const ObstacleMsg obstacle = blackboard_.obstacle();
    const BehaviorRequest behavior = blackboard_.behavior();
    const TrajectoryMsg trajectory = blackboard_.trajectory();
    const ControlCmdMsg safe = blackboard_.safeCommand();

    if (config_.parking.enable && config_.parking.bench_mode &&
        config_.parking.enable_bench_tracker && parking_safety_result_.evaluated) {
        std::cout << "[PARKING_BENCH] tid=" << parking_tracker_debug_.trajectory_id
                  << " idx=" << parking_tracker_debug_.nearest_index
                  << "->" << parking_tracker_debug_.target_index
                  << " dir="
                  << (parking_tracker_debug_.direction == MotionDirection::REVERSE
                          ? "REV" : "FWD")
                  << " raw_v=" << parking_bench_raw_command_.speed_mps
                  << " safe_v=" << parking_bench_safe_command_.speed_mps
                  << " steer_deg=" << parking_bench_safe_command_.steering_deg
                  << " cte=" << parking_tracker_debug_.nearest_distance_m
                  << " safety=" << parking_safety_result_.reason
                  << " [NO_UART]\n";
    }

    std::cout   << "[EXEC] "
                << "op=" << operatingModeToString(operating_mode_.load())
                << " mode=" << behaviorToString(behavior.mode)
                << " planner=" << plannerStateToString(trajectory.planner_state)
                << " plan=" << ((trajectory.header.valid && trajectory.collision_free) ? "SAFE" : "BLOCKED")
                << " dir=" << directionToString(trajectory.direction)
                << " lane=" << (lane.header.valid ? "OK" : "BAD")
                << " obs=" << obstacle.distance_m
                << " minD=" << trajectory.min_distance_m
                << " ttc=" << trajectory.min_ttc_s
                << " steer=" << safe.steering_deg
                << " servo=" << safe.servo_cmd
                << " speed=" << safe.speed_mps
                << " tel=" << telemetry_state
                << " ageMs=" << telemetry_age_ms
                << " rxHz=" << telemetry_rx_hz_
                << " telSeq=" << telemetry.packet_sequence
                << " seqGap=" << telemetry_sequence_gaps_
                << " dup=" << telemetry_duplicate_frames_
                << " seqReset=" << telemetry_sequence_resets_

                << " encV=" << (telemetry.encoder.valid ? 1 : 0)
                << " encSeq=" << telemetry.encoder.sequence
                << " ticks=" << telemetry.encoder.total_ticks
                << " dTicks=" << telemetry.encoder.delta_ticks
                << " encSpeed=" << telemetry.encoder.speed_mps

                << " imuV=" << (telemetry.imu.valid ? 1 : 0)
                << " imuSeq=" << telemetry.imu.sequence
                << " ax=" << telemetry.imu.linear_accel_x_mps2
                << " ay=" << telemetry.imu.linear_accel_y_mps2
                << " gz=" << telemetry.imu.gyro_z_dps
                << " yaw=" << telemetry.imu.yaw_deg
                << " calib=" << static_cast<unsigned>(
                    telemetry.imu.calibration_raw)
                << "\n";
}

}  // namespace laas
