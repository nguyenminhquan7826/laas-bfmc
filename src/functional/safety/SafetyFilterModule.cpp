#include "SafetyFilterModule.hpp"
#include <algorithm>
#include <cmath>

#include "../../laas_core/Time.hpp"

namespace laas {

namespace {

template <typename T>
T clampValue(T value, T minimum, T maximum)
{
    return std::min(std::max(value, minimum), maximum);
}

}  // namespace

SafetyFilterModule::SafetyFilterModule(const Config& config)
    : config_(config)
{
}

ControlCmdMsg SafetyFilterModule::filter(const ControlCmdMsg& raw_cmd,
                                         const BehaviorRequest& behavior,
                                         const LanePerceptionMsg& lane,
                                         const TrajectoryMsg& trajectory) const
{
    const uint64_t now = nowMs();
    ControlCmdMsg out = raw_cmd;
    const bool raw_values_finite =
        std::isfinite(raw_cmd.speed_mps) &&
        std::isfinite(raw_cmd.steering_deg);

    if (!std::isfinite(out.speed_mps)) {
        out.speed_mps = 0.0f;
    }

    if (!std::isfinite(out.steering_deg)) {
        out.steering_deg = 0.0f;
    }

    const bool stop_required =
        !raw_values_finite ||
        !raw_cmd.header.valid ||
        !isFresh(now, raw_cmd.header.timestamp_ms,
                 config_.runtime.control_command_timeout_ms) ||
        !behavior.header.valid ||
        !isFresh(now, behavior.header.timestamp_ms,
                 config_.runtime.behavior_timeout_ms) ||
        behavior.mode == BehaviorMode::STOP ||
        behavior.mode == BehaviorMode::EMERGENCY_STOP ||
        !lane.header.valid ||
        !isFresh(now, lane.header.timestamp_ms, config_.runtime.lane_timeout_ms) ||
        lane.centerline.size() < 3 ||
        !trajectory.header.valid ||
        !trajectory.collision_free ||
        !isFresh(now, trajectory.header.timestamp_ms,
                 config_.runtime.trajectory_timeout_ms);

    if (stop_required) {
        out.speed_mps = 0.0f;
        out.steering_deg = 0.0f;
    }

    out.speed_mps = clampValue(out.speed_mps, 0.0f, config_.vehicle.desired_speed_mps);
    out.steering_deg = clampValue(out.steering_deg,
                                  -config_.vehicle.steering_limit_deg,
                                  config_.vehicle.steering_limit_deg);
    out.servo_cmd = steeringToServo(out.steering_deg);
    out.header.timestamp_ms = now;
    out.header.valid = true;
    return out;
}

int SafetyFilterModule::steeringToServo(float steering_deg) const
{
    const float raw_servo = config_.vehicle.servo_center + steering_deg;
    const int servo = static_cast<int>(std::lround(raw_servo));
    return clampValue(servo, config_.vehicle.servo_min, config_.vehicle.servo_max);
}

}  // namespace laas
