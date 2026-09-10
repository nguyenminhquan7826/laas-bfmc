#include <cmath>
#include <cstdlib>
#include <iostream>

#include "functional/localization/VehiclePoseEstimator.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << "\n";
        std::exit(1);
    }
}

laas::VehicleTelemetryMsg telemetry(
    std::uint32_t sequence,
    std::uint64_t receive_ms,
    float imu_yaw_deg)
{
    laas::VehicleTelemetryMsg value;
    value.header.valid = true;
    value.header.timestamp_ms = receive_ms;
    value.packet_sequence = sequence;
    value.encoder.valid = true;
    value.encoder.speed_mps = 0.0F;
    value.imu.valid = true;
    value.imu.yaw_deg = imu_yaw_deg;
    return value;
}

}  // namespace

int main()
{
    laas::Config config;
    config.parking.enable = true;
    config.parking.enable_pose_estimator = true;
    config.parking.initial_pose_valid = true;
    config.parking.initial_yaw_rad = 0.0;
    config.parking.imu_yaw_positive_clockwise = true;

    laas::VehiclePoseEstimator estimator(config);
    laas::VehiclePoseMsg pose;

    require(estimator.process(telemetry(1U, 1000U, 10.0F), pose),
            "initial telemetry must initialize pose");
    require(std::fabs(pose.yaw_rad) < 1e-12,
            "first IMU yaw must become the relative reference");

    require(estimator.process(telemetry(2U, 1020U, 40.0F), pose),
            "second telemetry must update pose");
    require(std::fabs(pose.yaw_rad - (-30.0 * kPi / 180.0)) < 1e-6,
            "clockwise-positive IMU delta must become negative map yaw");

    require(estimator.process(telemetry(3U, 1040U, -20.0F), pose),
            "counterclockwise sample must update pose");
    require(std::fabs(pose.yaw_rad - (30.0 * kPi / 180.0)) < 1e-6,
            "counterclockwise physical delta must become positive map yaw");

    std::cout
        << "[PASS] clockwise-positive IMU is converted to map yaw and can be "
        << "displayed as clockwise-positive\n";
    return 0;
}
