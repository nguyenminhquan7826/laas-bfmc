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

// Step-11 mapping from the dedicated Pi parking safety gate
// to Parking Server V1.
//
// Only transitions are transmitted. A persistent fault is not
// retransmitted at every 20 ms control cycle.
bool mapParkingSafetyReasonToServerEvent(
    const std::string& reason,
    std::string& event,
    bool& requires_clear)
{
    requires_clear = false;

    if (reason == "SERVER_DISCONNECTED") {
        event = "SERVER_TIMEOUT";
        requires_clear = true;
        return true;
    }

    if (reason == "LOCAL_OBSTACLE_BLOCKING") {
        event = "CRITICAL_OBSTACLE";
        requires_clear = true;
        return true;
    }

    if (reason == "RAW_COMMAND_INVALID" ||
        reason == "RAW_COMMAND_STALE" ||
        reason == "TRACKER_STATE_INVALID" ||
        reason == "POSE_INVALID" ||
        reason == "POSE_STALE" ||
        reason == "TRAJECTORY_INVALID" ||
        reason == "SPEED_LIMIT_EXCEEDED" ||
        reason == "STEERING_LIMIT_EXCEEDED" ||
        reason == "DIRECTION_SPEED_MISMATCH") {

        event = "TRAJECTORY_INVALID";
        return true;
    }

    return false;
}

bool parkingSafetyReasonIsClear(
    const std::string& reason)
{
    return reason == "PASS_BENCH_ONLY" ||
           reason == "GOAL_HOLD";
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

// [SCHEDULER_AUDIT_V1]
// [SCHEDULER_JITTER_MONITOR_V1]
// Cooperative scheduler helper. Each task checks its deadline using a fresh
// monotonic timestamp. Normal ticks keep start-to-start cadence. If a task
// itself overruns its period, reset the phase at completion so the next loop
// cannot issue a catch-up burst immediately after the overrun.
//
// Diagnostics are observational only: they never change safety state or motion
// authorization. A missed period is counted only when lateness reaches at least
// one complete task period; small scheduler jitter remains visible as lateMs.
template <typename Function>
void runPeriodicTask(PeriodicTimer& timer,
                     PeriodicTaskDiagnostics& diagnostics,
                     Function&& task)
{
    const std::uint64_t start_ms = nowMs();
    if (!timer.ready(start_ms)) {
        return;
    }

    const std::uint64_t period_ms =
        static_cast<std::uint64_t>(timer.periodMs());
    const std::uint64_t expected_start_ms =
        timer.lastRunMs() + period_ms;

    diagnostics.onStart(start_ms, expected_start_ms, timer.periodMs());
    timer.mark(start_ms);

    const auto exec_start = std::chrono::steady_clock::now();
    task();
    const auto exec_end = std::chrono::steady_clock::now();

    const auto exec_us_signed =
        std::chrono::duration_cast<std::chrono::microseconds>(
            exec_end - exec_start).count();
    diagnostics.onFinish(
        exec_us_signed > 0
            ? static_cast<std::uint64_t>(exec_us_signed)
            : 0U);

    const std::uint64_t end_ms = nowMs();
    if (end_ms >= start_ms &&
        end_ms - start_ms >= period_ms) {
        timer.mark(end_ms);
    }
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
        parking_trajectory_status_policy_.reset();
        parking_session_query_pending_ = false;
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "NOT_CONNECTED";
    }
#else
    if (config_.parking.enable) {
        std::cerr << "[PARKING] Parking client unavailable in this build. "
                  << "Install libjson-c-dev and rebuild.\n";
    }
#endif

    const uint64_t now = nowMs();
    scheduler_.reset(now);
    scheduler_diagnostics_.reset();
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

    // Keyboard is operator I/O, not a 1 kHz control task. Polling at 20 ms
    // keeps it responsive while avoiding repeated tcgetattr/tcsetattr/fcntl
    // calls on every 1 ms scheduler spin.
    constexpr int kKeyboardPollPeriodMs = 20;
    PeriodicTimer keyboard_timer(kKeyboardPollPeriodMs);
    keyboard_timer.reset(nowMs());

    while (running_.load()) {
        if (config_.runtime.enable_keyboard) {
            runPeriodicTask(keyboard_timer, scheduler_diagnostics_.keyboard, [this]() {
                handleKeyboardTick();
            });

            // Q/ESC must not allow another camera/planning/control tick in the
            // same scheduler iteration. The normal shutdown STOP is sent below.
            if (!running_.load()) {
                break;
            }
        }

        // Telemetry consumption must never depend on keyboard availability.
        // receiveLatest() is non-blocking and drains the latest UART RX sample.
        telemetryTick();

        runPeriodicTask(scheduler_.camera, scheduler_diagnostics_.camera, [this]() {
            cameraTick();
        });

        runPeriodicTask(scheduler_.yolo, scheduler_diagnostics_.yolo, [this]() {
            yoloTick();
        });

        runPeriodicTask(scheduler_.perception, scheduler_diagnostics_.perception, [this]() {
            perceptionTick();
        });

        runPeriodicTask(scheduler_.decision, scheduler_diagnostics_.decision, [this]() {
            decisionTick();
        });

        runPeriodicTask(scheduler_.planning, scheduler_diagnostics_.planning, [this]() {
            planningTick();
        });

        runPeriodicTask(scheduler_.control, scheduler_diagnostics_.control, [this]() {
#ifdef LAAS_ENABLE_PARKING_CLIENT
            // Step-11: service parking TCP/protocol before parking safety.
            // This remains independent of OperatingMode::PARKING.
            parkingNetworkTick();
#endif

            controlTick();
            parkingBenchControlTick();
        });

        runPeriodicTask(scheduler_.logging, scheduler_diagnostics_.logging, [this]() {
            loggingTick();
        });

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

    // ========================================================
    // Step-11 BENCH ONLY STATIC POSE
    //
    // Used only when:
    //   - parking bench mode is enabled
    //   - UART is disabled
    //   - real pose estimator is disabled
    //   - an explicit initial pose is configured
    //
    // NEVER use this path for real vehicle localization.
    // ========================================================
    if (config_.parking.bench_mode &&
        !config_.runtime.enable_uart &&
        !config_.parking.enable_pose_estimator &&
        config_.parking.initial_pose_valid) {

        static std::uint64_t bench_pose_sequence = 1U;
        static std::uint64_t last_bench_pose_ms = 0U;

        const std::uint64_t now = nowMs();
        constexpr std::uint64_t kBenchPosePeriodMs = 100U;

        if (last_bench_pose_ms == 0U ||
            now - last_bench_pose_ms >= kBenchPosePeriodMs) {

            VehiclePoseMsg pose;

            pose.header.valid = true;
            pose.header.timestamp_ms = now;

            pose.sequence = bench_pose_sequence++;

            pose.map_id = config_.parking.map_id;
            pose.source = "BENCH_STATIC_INITIAL_POSE";

            pose.x_m = config_.parking.initial_x_m;
            pose.y_m = config_.parking.initial_y_m;
            pose.yaw_rad = config_.parking.initial_yaw_rad;

            blackboard_.setVehiclePose(pose);

            last_bench_pose_ms = now;
        }
    }

    ParkingStatusMsg bench_status;
    if (parking_status_bench_source_.process(nowMs(), bench_status)) {
        blackboard_.setParkingStatus(bench_status);
    }

    const bool was_connected = parking_server_connected_;
    parking_server_.service();
    parking_server_connected_ = parking_server_.connected();

    if (was_connected && !parking_server_connected_) {
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "SERVER_DISCONNECTED";
        parking_session_query_pending_ = false;
        std::cout << "[PARKING][SYNC] disconnected -> HOLD\n";
    }

    if (!was_connected && parking_server_connected_) {
        // A TCP connection alone never authorizes parking motion.
        // Reconcile Server session state before releasing the hold.
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "AWAITING_SESSION_SYNC";
        parking_session_query_pending_ = true;

        // Force the newest bench/local inputs to be republished on a
        // fresh TCP session even if their sequence did not change.
        have_sent_pose_sequence_ = false;
        have_sent_parking_status_sequence_ = false;

        std::cout << "[PARKING][SYNC] connected -> HOLD awaiting session\n";
    }

    // A safety event may have been generated while TCP was down.
    // Flush it before session_query so SERVER_TIMEOUT is ordered
    // ahead of the snapshot request on the same TCP stream.
    flushParkingSafetyEvents();

    if (parking_server_connected_ && parking_session_query_pending_) {
        if (parking_server_.sendSessionQuery()) {
            parking_session_query_pending_ = false;
            std::cout << "[PARKING][SYNC] session_query sent\n";
        }
    }

    ParkingServerMessage server_message;
    while (parking_server_.popMessage(server_message)) {
        if (server_message.type == ParkingServerMessageType::TRAJECTORY) {
            const VehiclePoseMsg current_pose = blackboard_.vehiclePose();
            const ParkingTrajectoryValidationResult validation =
                parking_trajectory_validator_.validate(
                    server_message.trajectory, current_pose);

            if (validation.accepted) {
                blackboard_.setParkingTrajectory(server_message.trajectory);
                parking_trajectory_status_policy_.onTrajectoryReceived(
                    server_message.trajectory.trajectory_id);

                // The Server attaches its TRAJECTORY_READY snapshot to
                // trajectory messages. Reconcile only after the Pi has
                // validated/stored the same trajectory ID.
                if (server_message.has_session) {
                    applyParkingSessionSnapshot(server_message.session);
                }

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
                parking_session_sync_hold_ = true;
                parking_session_sync_reason_ =
                    "TRAJECTORY_REJECTED_" + validation.reason;
                clearLocalParkingTrajectory(parking_session_sync_reason_);

                parking_server_.sendTrajectoryStatus(
                    parking_status_tx_sequence_++,
                    server_message.trajectory.trajectory_id,
                    "REJECTED", validation.reason);
                std::cerr << "[PARKING] trajectory tid="
                          << server_message.trajectory.trajectory_id
                          << " PiCheck=REJECT reason=" << validation.reason
                          << "\n";
            }
        } else if (server_message.type == ParkingServerMessageType::ACK) {
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            }
            if (!server_message.accepted) {
                std::cerr << "[PARKING][ACK] rejected reason="
                          << server_message.reason << "\n";
            }
        } else if (server_message.type == ParkingServerMessageType::SESSION_STATUS) {
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            } else {
                parking_session_sync_hold_ = true;
                parking_session_sync_reason_ = "SESSION_STATUS_MISSING_SNAPSHOT";
                std::cerr << "[PARKING][SYNC] invalid session_status -> HOLD\n";
            }
        } else if (server_message.type == ParkingServerMessageType::PLANNING_RESULT) {
            std::cout << "[PARKING] planning_result=" << server_message.status
                      << " reason=" << server_message.reason << "\n";
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            }
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


#ifdef LAAS_ENABLE_PARKING_CLIENT
void Executive::clearLocalParkingTrajectory(
    const std::string& reason)
{
    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();
    if (local.header.valid && local.trajectory_id > 0U) {
        std::cout << "[PARKING][SYNC] clear local tid="
                  << local.trajectory_id
                  << " reason=" << reason << "\n";
    }

    blackboard_.setParkingTrajectory(ParkingTrajectoryMsg{});
    parking_trajectory_tracker_.reset();
    parking_trajectory_status_policy_.reset();
}

void Executive::applyParkingSessionSnapshot(
    const ParkingSessionSnapshot& session)
{
    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();
    const ParkingSessionSyncDecision decision =
        ParkingSessionSyncPolicy::evaluate(session, local);

    parking_session_sync_hold_ = decision.hold_motion;
    parking_session_sync_reason_ = decision.reason;

    if (decision.clear_local_trajectory) {
        clearLocalParkingTrajectory(decision.reason);
    }

    if (decision.request_replan && parking_server_connected_) {
        const bool sent = parking_server_.sendTrajectoryStatus(
            parking_status_tx_sequence_++,
            decision.replan_trajectory_id,
            "REPLAN_REQUESTED",
            "STEP12_SESSION_RESYNC_" + decision.reason);

        if (!sent) {
            // Re-query on a later tick so the fail-closed decision can be
            // retried without assuming delivery.
            parking_session_query_pending_ = true;
        }
    }

    std::cout << "[PARKING][SYNC] serverState=" << session.state
              << " serverTid="
              << (session.has_active_trajectory
                      ? std::to_string(session.active_trajectory_id)
                      : std::string("none"))
              << " localTid="
              << ((local.header.valid && local.trajectory_id > 0U)
                      ? std::to_string(local.trajectory_id)
                      : std::string("none"))
              << " hold=" << (parking_session_sync_hold_ ? 1 : 0)
              << " reason=" << parking_session_sync_reason_ << "\n";
}

void Executive::parkingTrajectoryStatusSyncTick(
    const ParkingTrajectoryMsg& trajectory)
{
    // Step-12B remains bench-only. Hard-lock physical execution authorization
    // to false: static/bench tracking cannot emit EXECUTING or COMPLETED.
    constexpr bool kActuationAuthorized = false;

    const ParkingTrajectoryStatusUpdate update =
        parking_trajectory_status_policy_.evaluate(
            trajectory,
            parking_tracker_debug_,
            parking_safety_result_,
            kActuationAuthorized);

    if (!update.emit || !parking_server_connected_) {
        return;
    }

    if (parking_server_.sendTrajectoryStatus(
            parking_status_tx_sequence_++,
            update.trajectory_id,
            update.status,
            update.reason)) {
        std::cout << "[PARKING][STATUS_TX] tid="
                  << update.trajectory_id
                  << " status=" << update.status
                  << " reason=" << update.reason << "\n";
    }
}
#endif

void Executive::queueParkingSafetyEvent(
    const std::string& event,
    std::uint64_t trajectory_id,
    bool has_trajectory_id)
{
    constexpr std::size_t kMaxPendingSafetyEvents = 8U;

    if (event.empty()) {
        return;
    }

    // Avoid duplicate adjacent events.
    if (!parking_safety_event_queue_.empty()) {
        const PendingParkingSafetyEvent& back =
            parking_safety_event_queue_.back();

        if (back.event == event &&
            back.trajectory_id == trajectory_id &&
            back.has_trajectory_id == has_trajectory_id) {
            return;
        }
    }

    if (parking_safety_event_queue_.size() >=
        kMaxPendingSafetyEvents) {

        std::cerr
            << "[PARKING][SAFETY] event queue full; "
            << "drop=" << event << "\n";

        return;
    }

    PendingParkingSafetyEvent pending;

    pending.event = event;
    pending.trajectory_id = trajectory_id;
    pending.has_trajectory_id = has_trajectory_id;

    parking_safety_event_queue_.push_back(pending);
}

void Executive::flushParkingSafetyEvents()
{
#ifdef LAAS_ENABLE_PARKING_CLIENT

    if (!config_.parking.enable ||
        !parking_server_connected_) {
        return;
    }

    while (!parking_safety_event_queue_.empty()) {

        const PendingParkingSafetyEvent& pending =
            parking_safety_event_queue_.front();

        if (!parking_server_.sendSafetyEvent(
                pending.trajectory_id,
                pending.has_trajectory_id,
                pending.event)) {
            return;
        }

        std::cout
            << "[PARKING][SAFETY_TX] event="
            << pending.event;

        if (pending.has_trajectory_id) {
            std::cout
                << " tid="
                << pending.trajectory_id;
        } else {
            std::cout << " tid=none";
        }

        std::cout << "\n";

        parking_safety_event_queue_.pop_front();
    }

#endif
}

void Executive::parkingSafetyEventSyncTick(
    const ParkingTrajectoryMsg& trajectory)
{
    if (!parking_safety_result_.evaluated) {
        return;
    }

    const std::string& reason =
        parking_safety_result_.reason;

    // Safety events in Step-11 are trajectory/session events.
    // Do not emit SERVER_TIMEOUT during startup before a valid
    // parking trajectory exists.
    const bool has_active_trajectory =
        trajectory.header.valid &&
        trajectory.trajectory_id > 0U;

    if (!has_active_trajectory) {
        last_parking_safety_reason_ = reason;
        flushParkingSafetyEvents();
        return;
    }

    // No state transition: do not spam Server.
    if (reason == last_parking_safety_reason_) {
        flushParkingSafetyEvents();
        return;
    }

    const std::uint64_t trajectory_id =
        trajectory.trajectory_id;

    std::string event;
    bool requires_clear = false;

    if (mapParkingSafetyReasonToServerEvent(
            reason,
            event,
            requires_clear)) {

        queueParkingSafetyEvent(
            event,
            trajectory_id,
            true);

        // SERVER_TIMEOUT and CRITICAL_OBSTACLE pause the
        // server session and later require SAFETY_CLEARED.
        //
        // TRAJECTORY_INVALID already requests replan by itself,
        // therefore we must NOT automatically send
        // SAFETY_CLEARED after that event.
        parking_blocking_safety_active_ =
            requires_clear;

        std::cout
            << "[PARKING][SAFETY] reason="
            << reason
            << " -> event="
            << event
            << (requires_clear
                    ? " [PAUSE]"
                    : " [REPLAN]")
            << "\n";
    }
    else if (
        parkingSafetyReasonIsClear(reason) &&
        parking_blocking_safety_active_) {

        queueParkingSafetyEvent(
            "SAFETY_CLEARED",
            trajectory_id,
            true);

        parking_blocking_safety_active_ = false;

        std::cout
            << "[PARKING][SAFETY] clear"
            << " -> SAFETY_CLEARED [REPLAN]\n";
    }

    last_parking_safety_reason_ = reason;

    flushParkingSafetyEvents();
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

#ifdef LAAS_ENABLE_PARKING_CLIENT
    // Session synchronization is an independent fail-closed gate. It never
    // sends UART; it only prevents a locally computed bench command from being
    // treated as runnable while Pi and Server disagree.
    if (parking_session_sync_hold_) {
        parking_bench_safe_command_.header.valid = true;
        parking_bench_safe_command_.header.timestamp_ms = nowMs();
        parking_bench_safe_command_.speed_mps = 0.0F;
        parking_bench_safe_command_.steering_deg = 0.0F;
    }
#endif

    // Step-11 sends safety state to the Server only.
    // Pi local stop authority remains independent of Server response.
    parkingSafetyEventSyncTick(trajectory);

#ifdef LAAS_ENABLE_PARKING_CLIENT
    parkingTrajectoryStatusSyncTick(trajectory);
#endif

    // Deliberately no vehicle_.send(...) here.
    // Parking remains bench-only.
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
    const UartRxStats uart_stats = vehicle_.rxStats();

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
#ifdef LAAS_ENABLE_PARKING_CLIENT
                  << " sync=" << (parking_session_sync_hold_ ? "HOLD" : "READY")
                  << " syncReason=" << parking_session_sync_reason_
#endif
                  << " [NO_UART]\n";
    }

    const SchedulerDiagnostics& sd = scheduler_diagnostics_;
    std::cout << "[SCHED]"
              << " controlDtMs=" << sd.control.last_interval_ms
              << " controlMaxDtMs=" << sd.control.max_interval_ms
              << " controlExecUs=" << sd.control.last_exec_us
              << " controlMaxExecUs=" << sd.control.max_exec_us
              << " controlLateMs=" << sd.control.last_lateness_ms
              << " controlMaxLateMs=" << sd.control.max_lateness_ms
              << " controlMissed=" << sd.control.missed_periods
              << " cameraExecUs=" << sd.camera.last_exec_us
              << " cameraMaxExecUs=" << sd.camera.max_exec_us
              << " cameraLateMs=" << sd.camera.last_lateness_ms
              << " cameraMissed=" << sd.camera.missed_periods
              << " yoloExecUs=" << sd.yolo.last_exec_us
              << " yoloMaxExecUs=" << sd.yolo.max_exec_us
              << " yoloLateMs=" << sd.yolo.last_lateness_ms
              << " yoloMissed=" << sd.yolo.missed_periods
              << " perceptionExecUs=" << sd.perception.last_exec_us
              << " planningExecUs=" << sd.planning.last_exec_us
              << " decisionExecUs=" << sd.decision.last_exec_us
              << "\n";

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

                // Executive-side consume statistics.
                << " consumeHz=" << telemetry_rx_hz_
                << " telSeq=" << telemetry.packet_sequence
                << " consumeSeqSkip=" << telemetry_sequence_gaps_
                << " consumeDup=" << telemetry_duplicate_frames_
                << " consumeReset=" << telemetry_sequence_resets_

                // True UART RX-thread statistics.
                << " uartHz=" << uart_stats.parsed_hz
                << " uartFrames=" << uart_stats.parsed_frames
                << " uartGap=" << uart_stats.transport_sequence_gaps
                << " uartDup=" << uart_stats.duplicate_frames
                << " uartReset=" << uart_stats.sequence_resets
                << " qDrop=" << uart_stats.queue_dropped_frames
                << " latestSkip=" << uart_stats.latest_skipped_frames

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
