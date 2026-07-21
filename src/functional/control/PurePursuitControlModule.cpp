#include "PurePursuitControlModule.hpp"

#include <algorithm>

#include "../../laas_core/Time.hpp"

namespace laas {

PurePursuitControlModule::PurePursuitControlModule(const Config& config)
    : config_(config)
{
    controller_.setWheelbase(config_.vehicle.wheelbase_m);
    controller_.setMaxSteeringDeg(config_.vehicle.steering_limit_deg);
}

std::vector<cv::Point> PurePursuitControlModule::selectPath(const TrajectoryMsg& trajectory,
                                                            const LanePerceptionMsg& lane) const
{
    (void)lane;

    if (trajectory.header.valid &&
        trajectory.collision_free &&
        trajectory.target_centerline.size() >= 3) {
        return trajectory.target_centerline;
    }

    return {};
}

void PurePursuitControlModule::updateScaleFromLaneWidth(const LanePerceptionMsg& lane)
{
    if (lane.lane_width_px > 50.0f && config_.planner.lane_width_m > 0.05f) {
        controller_.setPixelPerMeter(lane.lane_width_px / config_.planner.lane_width_m);
    }
}

bool PurePursuitControlModule::process(const TrajectoryMsg& trajectory,
                                       const LanePerceptionMsg& lane,
                                       const BehaviorRequest& behavior,
                                       ControlCmdMsg& command)
{
    command = ControlCmdMsg{};
    command.header.timestamp_ms = nowMs();
    command.speed_mps = 0.0f;
    command.steering_deg = 0.0f;

    if (behavior.control_mode != ControlMode::PURE_PURSUIT) {
        command.header.valid = false;
        return false;
    }

    if (behavior.mode == BehaviorMode::STOP ||
        behavior.mode == BehaviorMode::EMERGENCY_STOP) {
        command.header.valid = true;
        return true;
    }

    if (!trajectory.header.valid || !trajectory.collision_free) {
        command.header.valid = false;
        return false;
    }

    if (!lane.header.valid || lane.bird_eye_view.empty()) {
        command.header.valid = false;
        return false;
    }

    const std::vector<cv::Point> path = selectPath(trajectory, lane);
    if (path.size() < 3) {
        command.header.valid = false;
        return false;
    }

    updateScaleFromLaneWidth(lane);

    const float speed_mps = std::max(behavior.desired_speed_mps, 0.0f);
    float steering_deg = 0.0f;
    if (!controller_.computeSteeringAngle(path,
                                          lane.bird_eye_view.size(),
                                          speed_mps,
                                          steering_deg)) {
        command.header.valid = false;
        return false;
    }

    command.steering_deg = steering_deg;
    command.speed_mps = speed_mps;
    command.header.valid = true;
    return true;
}

}  // namespace laas
