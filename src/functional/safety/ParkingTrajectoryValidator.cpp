#include "ParkingTrajectoryValidator.hpp"

#include <algorithm>
#include <cmath>

namespace laas {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ParkingTrajectoryValidator::ParkingTrajectoryValidator(const Config& config)
    : config_(config)
{
}

double ParkingTrajectoryValidator::wrapAngle(double angle_rad)
{
    while (angle_rad > kPi) angle_rad -= 2.0 * kPi;
    while (angle_rad < -kPi) angle_rad += 2.0 * kPi;
    return angle_rad;
}

ParkingTrajectoryValidationResult ParkingTrajectoryValidator::validate(
    const ParkingTrajectoryMsg& trajectory,
    const VehiclePoseMsg& current_pose) const
{
    ParkingTrajectoryValidationResult result;

    auto reject = [&result](const char* reason) {
        result.accepted = false;
        result.reason = reason;
        return result;
    };

    if (!trajectory.header.valid) {
        return reject("trajectory_header_invalid");
    }
    if (trajectory.protocol_version != 1U) {
        return reject("trajectory_protocol_version_mismatch");
    }
    if (trajectory.trajectory_id == 0U) {
        return reject("trajectory_id_invalid");
    }
    if (trajectory.map_id != config_.parking.map_id) {
        return reject("trajectory_map_id_mismatch");
    }
    if (trajectory.reference_point != "rear_axle_center") {
        return reject("trajectory_reference_point_mismatch");
    }
    if (trajectory.validation != "PASS") {
        return reject("server_validation_not_pass");
    }
    if (trajectory.target_slot.empty() || trajectory.goal_mode.empty()) {
        return reject("trajectory_metadata_missing");
    }
    if (trajectory.points.size() < 2U ||
        trajectory.points.size() >
            static_cast<std::size_t>(config_.parking.trajectory_max_points)) {
        return reject("trajectory_point_count_invalid");
    }

    if (!current_pose.header.valid ||
        current_pose.map_id != config_.parking.map_id ||
        !std::isfinite(current_pose.x_m) ||
        !std::isfinite(current_pose.y_m) ||
        !std::isfinite(current_pose.yaw_rad)) {
        return reject("current_pose_invalid");
    }

    const ParkingTrajectoryPoint& first = trajectory.points.front();
    const double start_distance = std::hypot(first.x_m - current_pose.x_m,
                                             first.y_m - current_pose.y_m);
    if (!std::isfinite(start_distance) ||
        start_distance > config_.parking.trajectory_start_max_position_error_m) {
        return reject("trajectory_start_position_mismatch");
    }
    const double start_yaw_error =
        std::abs(wrapAngle(first.yaw_rad - current_pose.yaw_rad));
    if (!std::isfinite(start_yaw_error) ||
        start_yaw_error > config_.parking.trajectory_start_max_yaw_error_rad) {
        return reject("trajectory_start_yaw_mismatch");
    }

    const double max_speed =
        static_cast<double>(config_.parking.max_parking_speed_mps) + 1e-6;

    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
        const ParkingTrajectoryPoint& point = trajectory.points[i];
        if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
            !std::isfinite(point.yaw_rad) || !std::isfinite(point.v_ref_mps)) {
            return reject("trajectory_point_non_finite");
        }
        const double speed = static_cast<double>(point.v_ref_mps);
        if (std::abs(speed) > max_speed) {
            return reject("trajectory_speed_exceeds_pi_limit");
        }
        if (point.direction == MotionDirection::FORWARD && speed < -1e-6) {
            return reject("trajectory_forward_speed_negative");
        }
        if (point.direction == MotionDirection::REVERSE && speed > 1e-6) {
            return reject("trajectory_reverse_speed_positive");
        }

        if (i == 0U) {
            continue;
        }
        const ParkingTrajectoryPoint& previous = trajectory.points[i - 1U];
        const double spacing = std::hypot(point.x_m - previous.x_m,
                                          point.y_m - previous.y_m);
        if (!std::isfinite(spacing) ||
            spacing > config_.parking.trajectory_max_point_spacing_m) {
            return reject("trajectory_spacing_too_large");
        }
        const double yaw_step =
            std::abs(wrapAngle(point.yaw_rad - previous.yaw_rad));
        if (!std::isfinite(yaw_step) ||
            yaw_step > config_.parking.trajectory_max_yaw_step_rad) {
            return reject("trajectory_yaw_step_too_large");
        }
    }

    result.accepted = true;
    result.reason = "PASS_PI_CONTRACT_CHECKS_ONLY";
    return result;
}

}  // namespace laas
