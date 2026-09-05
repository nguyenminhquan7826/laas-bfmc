#include "ParkingSafetyFilter.hpp"

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
}

ParkingSafetyFilter::ParkingSafetyFilter(const Config& config)
    : config_(config)
{
}

bool ParkingSafetyFilter::finiteCommand(const ControlCmdMsg& command)
{
    return std::isfinite(command.speed_mps) &&
           std::isfinite(command.steering_deg);
}

int ParkingSafetyFilter::steeringToServo(float steering_deg) const
{
    const float raw_servo = config_.vehicle.servo_center + steering_deg;
    const int servo = static_cast<int>(std::lround(raw_servo));
    return clampValue(servo, config_.vehicle.servo_min, config_.vehicle.servo_max);
}

ControlCmdMsg ParkingSafetyFilter::filter(
    const ControlCmdMsg& raw_cmd,
    const ParkingTrackerDebug& tracker,
    const VehiclePoseMsg& pose,
    const ParkingTrajectoryMsg& trajectory,
    const VehicleTelemetryMsg& telemetry,
    const ObstacleMsg& obstacle,
    bool server_connected,
    ParkingSafetyResult* result) const
{
    ParkingSafetyResult local;
    local.evaluated = true;
    local.motion_allowed = false;
    local.reason = "STOP_UNKNOWN";

    const std::uint64_t now = nowMs();
    ControlCmdMsg out = raw_cmd;
    out.header.valid = true;
    out.header.timestamp_ms = now;

    auto stop = [&](const char* reason) {
        out.speed_mps = 0.0f;
        out.steering_deg = 0.0f;
        out.servo_cmd = steeringToServo(0.0f);
        local.motion_allowed = false;
        local.reason = reason;
        if (result) *result = local;
        return out;
    };

    if (!config_.parking.enable) return stop("PARKING_DISABLED");
    if (!config_.parking.bench_mode) return stop("NON_BENCH_NOT_AUTHORIZED");

    // Step-10 integration remains bench-only. This gate prevents accidental
    // actuator use before full-body geometry and the final parking safety
    // thresholds are verified on the real vehicle.
    if (config_.runtime.enable_uart) return stop("UART_MUST_BE_DISABLED_IN_BENCH");
    if (!server_connected) return stop("SERVER_DISCONNECTED");

    if (!raw_cmd.header.valid || !finiteCommand(raw_cmd))
        return stop("RAW_COMMAND_INVALID");
    if (!isFresh(now, raw_cmd.header.timestamp_ms,
                 config_.runtime.control_command_timeout_ms))
        return stop("RAW_COMMAND_STALE");

    if (!tracker.valid || tracker.trajectory_id == 0 ||
        tracker.trajectory_id != trajectory.trajectory_id)
        return stop("TRACKER_STATE_INVALID");

    if (!pose.header.valid || pose.map_id != config_.parking.map_id ||
        !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
        !std::isfinite(pose.yaw_rad))
        return stop("POSE_INVALID");
    if (!isFresh(now, pose.header.timestamp_ms,
                 config_.parking.tracker_pose_timeout_ms))
        return stop("POSE_STALE");

    if (!trajectory.header.valid || trajectory.trajectory_id == 0 ||
        trajectory.map_id != config_.parking.map_id ||
        trajectory.reference_point != "rear_axle_center" ||
        trajectory.validation != "PASS" || trajectory.points.size() < 2)
        return stop("TRAJECTORY_INVALID");

    const float max_speed = std::max(config_.parking.max_parking_speed_mps, 0.0f);
    if (std::fabs(raw_cmd.speed_mps) > max_speed + 1e-5f)
        return stop("SPEED_LIMIT_EXCEEDED");
    if (std::fabs(raw_cmd.steering_deg) >
        config_.vehicle.steering_limit_deg + 1e-5f)
        return stop("STEERING_LIMIT_EXCEEDED");

    if ((tracker.direction == MotionDirection::FORWARD && raw_cmd.speed_mps < -1e-5f) ||
        (tracker.direction == MotionDirection::REVERSE && raw_cmd.speed_mps > 1e-5f))
        return stop("DIRECTION_SPEED_MISMATCH");

    // Existing camera/YOLO obstacle data can conservatively force a stop. Its
    // absence is NOT treated as proof that the parking area is clear.
    if (obstacle.header.valid &&
        isFresh(now, obstacle.header.timestamp_ms, config_.runtime.obstacle_timeout_ms) &&
        obstacle.has_obstacle && std::isfinite(obstacle.distance_m) &&
        obstacle.distance_m >= 0.0f &&
        obstacle.distance_m <= config_.parking.local_obstacle_stop_distance_m) {
        return stop("LOCAL_OBSTACLE_BLOCKING");
    }

    // When real UART is authorized in a later phase, telemetry must be fresh.
    // The current bench gate above makes this branch unreachable today, but the
    // contract is kept here so live enabling cannot silently omit STM32 health.
    if (config_.runtime.enable_uart &&
        (!telemetry.header.valid ||
         !isFresh(now, telemetry.header.timestamp_ms,
                  config_.parking.telemetry_timeout_ms))) {
        return stop("STM32_TELEMETRY_STALE");
    }

    out.speed_mps = clampValue(out.speed_mps, -max_speed, max_speed);
    out.steering_deg = clampValue(out.steering_deg,
                                  -config_.vehicle.steering_limit_deg,
                                  config_.vehicle.steering_limit_deg);
    out.servo_cmd = steeringToServo(out.steering_deg);
    out.header.valid = true;

    local.motion_allowed = true;
    local.reason = tracker.goal_reached ? "GOAL_HOLD" : "PASS_BENCH_ONLY";
    if (tracker.goal_reached) {
        out.speed_mps = 0.0f;
        out.steering_deg = 0.0f;
        out.servo_cmd = steeringToServo(0.0f);
        local.motion_allowed = false;
    }
    if (result) *result = local;
    return out;
}

}  // namespace laas
