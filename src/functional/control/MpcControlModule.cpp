#include "MpcControlModule.hpp"

#include <algorithm>
#include <cmath>

#include "../../laas_core/Time.hpp"

namespace laas {

MpcControlModule::MpcControlModule(const Config& config)
    : config_(config),
      last_state_(config.mpc.horizon)
{
    controller_.setPredictionHorizon(config_.mpc.horizon);
    controller_.setVehicleParams(config_.vehicle.wheelbase_m,
                                 2.3f,
                                 0.132f,
                                 0.12f,
                                 0.04f,
                                 0.02f,
                                 0.04f);
    controller_.init(config_.mpc.q_lateral,
                     config_.mpc.q_yaw,
                     config_.mpc.r_steering);
}

std::vector<cv::Point> MpcControlModule::selectPath(const TrajectoryMsg& trajectory,
                                                    const LanePerceptionMsg& lane) const
{
    if (trajectory.header.valid && trajectory.target_centerline.size() >= 3) {
        return trajectory.target_centerline;
    }

    if (trajectory.base_centerline.size() >= 3) {
        return trajectory.base_centerline;
    }

    return lane.centerline;
}

void MpcControlModule::updateScaleFromLaneWidth(const LanePerceptionMsg& lane)
{
    if (lane.lane_width_px > 50.0f && config_.planner.lane_width_m > 0.05f) {
        controller_.setMeterPerPixel(config_.planner.lane_width_m / lane.lane_width_px);
    }
}

bool MpcControlModule::process(const TrajectoryMsg& trajectory,
                               const LanePerceptionMsg& lane,
                               const BehaviorRequest& behavior,
                               ControlCmdMsg& command)
{
    command = ControlCmdMsg{};
    command.header.timestamp_ms = nowMs();
    command.speed_mps = 0.0f;
    command.steering_deg = 0.0f;

    if (behavior.control_mode != ControlMode::MPC) {
        command.header.valid = false;
        return false;
    }

    if (behavior.mode == BehaviorMode::STOP ||
        behavior.mode == BehaviorMode::EMERGENCY_STOP) {
        command.header.valid = true;
        return true;
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
    last_state_ = controller_.computeMpcParameters(path, lane.bird_eye_view);

    command.steering_deg = controller_.computeSteeringAngle(last_state_, speed_mps);
    command.speed_mps = speed_mps;
    command.header.valid = true;
    return true;
}

}  // namespace laas
