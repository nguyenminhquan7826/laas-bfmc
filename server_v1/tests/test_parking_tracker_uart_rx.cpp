#include <cmath>
#include <iostream>
#include <stdexcept>

#include "functional/control/ParkingTrajectoryTracker.hpp"
#include "laas_core/Time.hpp"

namespace {

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

laas::Config makePoseBenchConfig()
{
    laas::Config config;
    config.parking.enable = true;
    config.parking.bench_mode = true;
    config.parking.enable_bench_tracker = true;
    config.parking.map_id = "map_v1";
    config.parking.tracker_lookahead_m = 0.20;
    config.parking.tracker_pose_timeout_ms = 300;
    config.parking.max_parking_speed_mps = 0.10F;

    // Pose bench policy: UART is open for encoder/IMU RX, while actuator TX is
    // independently and explicitly disabled.
    config.runtime.enable_uart = true;
    config.runtime.enable_uart_tx = false;
    return config;
}

laas::VehiclePoseMsg makePose()
{
    laas::VehiclePoseMsg pose;
    pose.header.valid = true;
    pose.header.timestamp_ms = laas::nowMs();
    pose.sequence = 1;
    pose.map_id = "map_v1";
    pose.source = "ENCODER_IMU";
    pose.x_m = 0.0;
    pose.y_m = 0.0;
    pose.yaw_rad = 0.0;
    return pose;
}

laas::ParkingTrajectoryMsg makeTrajectory()
{
    laas::ParkingTrajectoryMsg trajectory;
    trajectory.header.valid = true;
    trajectory.header.timestamp_ms = laas::nowMs();
    trajectory.protocol_version = 1;
    trajectory.trajectory_id = 42;
    trajectory.map_id = "map_v1";
    trajectory.target_slot = "P_B2";
    trajectory.reference_point = "rear_axle_center";
    trajectory.validation = "PASS";

    laas::ParkingTrajectoryPoint start;
    start.x_m = 0.0;
    start.y_m = 0.0;
    start.yaw_rad = 0.0;
    start.v_ref_mps = 0.05F;
    start.direction = laas::MotionDirection::FORWARD;

    laas::ParkingTrajectoryPoint target = start;
    target.x_m = 0.30;

    trajectory.points.push_back(start);
    trajectory.points.push_back(target);
    return trajectory;
}

}  // namespace

int main()
{
    try {
        const laas::Config config = makePoseBenchConfig();
        laas::ParkingTrajectoryTracker tracker(config);
        laas::ControlCmdMsg command;
        laas::ParkingTrackerDebug debug;

        const bool accepted = tracker.process(
            makePose(), makeTrajectory(), command, &debug);

        CHECK_TRUE(accepted, "tracker_rejected_pose_bench_with_uart_rx");
        CHECK_TRUE(command.header.valid, "tracker_command_invalid");
        CHECK_TRUE(debug.valid, "tracker_debug_invalid");
        CHECK_TRUE(debug.trajectory_id == 42, "trajectory_id_mismatch");
        CHECK_TRUE(command.speed_mps > 0.0F, "tracker_did_not_generate_motion");
        CHECK_TRUE(std::isfinite(command.steering_deg), "steering_not_finite");
        CHECK_TRUE(!config.runtime.enable_uart_tx, "test_tx_gate_not_disabled");

        std::cout << "[PASS] tracker accepts Encoder-IMU pose with UART RX enabled "
                     "and UART TX disabled\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "[FAIL] " << exc.what() << "\n";
        return 1;
    }
}
