#include "VehiclePoseEstimator.hpp"

#include <cmath>

namespace laas {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
}

VehiclePoseEstimator::VehiclePoseEstimator(const Config& config)
    : config_(config)
{
    reset();
}

void VehiclePoseEstimator::reset()
{
    initialized_ = false;
    last_receive_timestamp_ms_ = 0U;
    imu_reference_yaw_rad_ = 0.0;

    x_m_ = config_.parking.initial_x_m;
    y_m_ = config_.parking.initial_y_m;
    yaw_rad_ = wrapAngle(config_.parking.initial_yaw_rad);
}

double VehiclePoseEstimator::wrapAngle(double angle_rad)
{
    while (angle_rad > kPi) {
        angle_rad -= 2.0 * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0 * kPi;
    }
    return angle_rad;
}

bool VehiclePoseEstimator::process(const VehicleTelemetryMsg& telemetry,
                                   VehiclePoseMsg& out)
{
    out = VehiclePoseMsg{};

    if (!config_.parking.enable ||
        !config_.parking.enable_pose_estimator ||
        !config_.parking.initial_pose_valid) {
        return false;
    }

    if (!telemetry.header.valid ||
        !telemetry.encoder.valid ||
        !telemetry.imu.valid ||
        !std::isfinite(telemetry.encoder.speed_mps) ||
        !std::isfinite(telemetry.imu.yaw_deg)) {
        return false;
    }

    const std::uint64_t receive_ms = telemetry.header.timestamp_ms;
    const double imu_yaw_rad =
        static_cast<double>(telemetry.imu.yaw_deg) * kDegToRad;

    if (!initialized_) {
        initialized_ = true;
        last_receive_timestamp_ms_ = receive_ms;
        imu_reference_yaw_rad_ = imu_yaw_rad;

        x_m_ = config_.parking.initial_x_m;
        y_m_ = config_.parking.initial_y_m;
        yaw_rad_ = wrapAngle(config_.parking.initial_yaw_rad);
    } else {
        if (receive_ms <= last_receive_timestamp_ms_) {
            return false;
        }

        const std::uint64_t dt_ms = receive_ms - last_receive_timestamp_ms_;
        last_receive_timestamp_ms_ = receive_ms;

        const double new_yaw = wrapAngle(
            config_.parking.initial_yaw_rad +
            wrapAngle(imu_yaw_rad - imu_reference_yaw_rad_));

        // Do not integrate position over a large telemetry gap. Updating the
        // yaw reference is safe; inventing a long travelled distance is not.
        if (dt_ms <= static_cast<std::uint64_t>(
                config_.parking.pose_max_integration_dt_ms)) {
            const double dt_s = static_cast<double>(dt_ms) / 1000.0;
            const double yaw_delta = wrapAngle(new_yaw - yaw_rad_);
            const double yaw_mid = wrapAngle(yaw_rad_ + 0.5 * yaw_delta);
            const double distance_m =
                static_cast<double>(telemetry.encoder.speed_mps) * dt_s;

            x_m_ += distance_m * std::cos(yaw_mid);
            y_m_ += distance_m * std::sin(yaw_mid);
        }

        yaw_rad_ = new_yaw;
    }

    out.header.valid = true;
    out.header.timestamp_ms = receive_ms;
    out.sequence = telemetry.packet_sequence;
    out.map_id = config_.parking.map_id;
    out.source = "ENCODER_IMU_DEAD_RECKONING";
    out.x_m = x_m_;
    out.y_m = y_m_;
    out.yaw_rad = yaw_rad_;
    return true;
}

}  // namespace laas
