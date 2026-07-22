#include "Executive.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>

#include "../laas_core/Time.hpp"

namespace laas {

namespace {

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
      lane_perception_(config_),
      mission_(config_),
      planner_(config_),
      pure_pursuit_(config_),
#ifdef LAAS_ENABLE_MPC
      mpc_(config_),
#endif
      safety_(config_)
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
            const cv::Mat& bev = lane.bird_eye_view;

            const float safe_range =
                std::max(config_.camera.bev_forward_range_m, 0.20f);

            int valid_width = static_cast<int>(std::lround(
                static_cast<float>(bev.rows) *
                std::max(config_.planner.lane_width_m, 0.05f) /
                safe_range
            ));

            valid_width = std::min(
                std::max(valid_width, 40),
                static_cast<int>(0.8f * bev.cols)
            );

            const int x0 = (bev.cols - valid_width) / 2;

            cv::Mat cropped_bev =
                bev(cv::Rect(x0, 0, valid_width, bev.rows)).clone();

            cv::Mat debug_bev;
            cv::resize(
                cropped_bev,
                debug_bev,
                bev.size(),
                0.0,
                0.0,
                cv::INTER_LINEAR
            );

            yolo_.sendDebugFrame(debug_bev, 80);
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
    const LanePerceptionMsg lane = blackboard_.lane();
    const ObstacleMsg obstacle = blackboard_.obstacle();
    const BehaviorRequest behavior = blackboard_.behavior();
    const TrajectoryMsg trajectory = blackboard_.trajectory();
    const ControlCmdMsg safe = blackboard_.safeCommand();

    std::cout << "[EXEC] "
              << "mode=" << behaviorToString(behavior.mode)
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
              << "\n";
}

}  // namespace laas
