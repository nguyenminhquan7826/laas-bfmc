#pragma once

#include <string>
#include "Messages.hpp"
#include "SteeringCalibration.hpp"

namespace laas {

struct CameraConfig {
    // Raspberry Pi CSI camera. Pass another value through main() when a video
    // file or a V4L2 USB camera is required.
    std::string device = "libcamera";
    int width = 640;
    int height = 480;
    int fps = 30;
    float bev_forward_range_m = 2.0f;
};

struct UartConfig {
    std::string port = "/dev/ttyACM0";
    int baudrate = 115200;
};

struct UdpConfig {
    // Safety-critical AI traffic stays inside the Pi and does not use Wi-Fi.
    std::string local_ai_ip = "127.0.0.1";

    // Bird-eye debug streaming is observational only and can be disabled
    // independently from the local YOLO safety path.
    bool enable_debug_stream = true;

    // Change this to the current IPv4 address of the monitoring laptop.
    std::string monitor_ip = "192.168.1.253";

    // C++ camera -> local Python ONNX process.
    int yolo_send_port = 9996;

    // C++ bird-eye-view -> monitoring laptop.
    int debug_send_port = 9997;

    // Local Python ONNX process -> C++ ObstacleMsg.
    int distance_recv_port = 8888;

    // Limit the bird-eye-view stream to 10 FPS to reduce Wi-Fi load.
    int monitor_period_ms = 100;
};

struct VehicleConfig {
    // Physical measurements verified on the current shared mechanical platform.
    float wheelbase_m = 0.250f;
    float desired_speed_mps = 0.15f;

    // Bicycle-equivalent steering range derived from the thesis Ackermann table.
    // This is a software baseline pending final wheel-angle validation on the
    // current vehicle; parking actuation remains disabled.
    float steering_limit_deg = steering_calibration::kBicycleSteeringMaxDeg;

    // Absolute STM32 servo commands physically verified on the current vehicle.
    float servo_center = 75.0f;
    int servo_min = 45;
    int servo_max = 105;
};


struct PlannerConfig {
    float lane_width_m = 0.40f;
    float vehicle_width_m = 0.21f;
    float vehicle_length_m = 0.432f;
    float obstacle_width_m = 0.20f;
    float obstacle_length_m = 0.22f;
    float safe_margin_m = 0.10f;
    // Conservative first value. Recalibrate from measured end-to-end
    // perception latency and the real braking distance.
    float emergency_stop_distance_m = 0.50f;
    float trigger_distance_m = 1.10f;
    float lane_change_commit_time_s = 4.0f;
};

struct MpcConfig {
    int horizon = 10;
    float q_lateral = 1000.0f;
    float q_yaw = 50.0f;
    float r_steering = 5.0f;
};

struct ParkingConfig {
    // Parking integration is disabled until an initial map pose is measured.
    bool enable = false;
    bool bench_mode = true;

    std::string map_id = "map_v1";
    std::string server_host = "127.0.0.1";
    int server_port = 5000;
    int reconnect_period_ms = 1000;
    int max_ndjson_line_bytes = 65536;

    // Pi-side trajectory contract checks. These are integration defaults, not
    // final real-vehicle safety thresholds.
    float max_parking_speed_mps = 0.10f;
    double trajectory_start_max_position_error_m = 0.20;
    double trajectory_start_max_yaw_error_rad = 0.35;
    double trajectory_max_point_spacing_m = 0.15;
    double trajectory_max_yaw_step_rad = 0.40;
    int trajectory_max_points = 5000;

    // Bench-only parking occupancy source. Defaults are deliberately UNKNOWN;
    // no slot is considered FREE unless a test explicitly configures it.
    bool enable_bench_parking_status = false;
    int bench_parking_status_period_ms = 200;
    float bench_slot_confidence = 1.0f;
    ParkingSlotState bench_p_b1 = ParkingSlotState::UNKNOWN;
    ParkingSlotState bench_p_b2 = ParkingSlotState::UNKNOWN;
    ParkingSlotState bench_p_t1 = ParkingSlotState::UNKNOWN;
    ParkingSlotState bench_p_t2 = ParkingSlotState::UNKNOWN;

    // Bench-only map-frame parking tracker. Its output is logged only and must
    // not be routed to UART until the dedicated parking safety layer and real
    // vehicle geometry are verified.
    bool enable_bench_tracker = false;
    double tracker_lookahead_m = 0.20;
    int tracker_nearest_search_points = 30;
    double tracker_max_cross_track_error_m = 0.30;
    double tracker_cusp_position_tolerance_m = 0.06;
    double tracker_goal_position_tolerance_m = 0.06;
    double tracker_goal_yaw_tolerance_rad = 0.15;
    int tracker_pose_timeout_ms = 300;
    int telemetry_timeout_ms = 250;
    float local_obstacle_stop_distance_m = 0.50f;

    // Provisional encoder + IMU dead reckoning. It must not publish a valid
    // map pose until the vehicle's initial rear-axle-center pose is known.
    bool enable_pose_estimator = false;
    bool initial_pose_valid = false;
    double initial_x_m = 0.0;
    double initial_y_m = 0.0;
    double initial_yaw_rad = 0.0;
    int pose_max_integration_dt_ms = 200;
};

struct RuntimeConfig {
    int obstacle_timeout_ms = 500;

    int frame_timeout_ms = 150;
    int lane_timeout_ms = 300;
    int behavior_timeout_ms = 200;
    int trajectory_timeout_ms = 200;
    int control_command_timeout_ms = 100;

    ControlMode control_mode = ControlMode::MPC;

    int camera_period_ms = 33;
    int yolo_period_ms = 333;  // ~3.00 FPS sustained AI rate
    // Pi 5 benchmark: lane perception typically takes ~40 ms (P95 ~47 ms).
    // A 50 ms period gives a sustainable 20 Hz start-to-start cadence instead
    // of repeatedly overrunning the former 33 ms target. Control and camera
    // remain independently scheduled at 20 ms and 33 ms respectively.
    int perception_period_ms = 50;
    int decision_period_ms = 50;
    int planning_period_ms = 50;
    int control_period_ms = 20;
    int logging_period_ms = 100;

    bool enable_keyboard = true;
    bool enable_yolo_udp = true;

    // UART RX/connection enable.
    bool enable_uart = true;

    // Master TX gate.
    // Giữ false cho đến khi neutral-only build đã được xác nhận.
    bool enable_uart_tx = false;

    // Bench safety:
    // Khi true, mọi command thực tế gửi xuống STM32 đều bị ép:
    //   speed = 0.00 m/s
    //   servo = servo_center
    // kể cả MPC/PP hoặc người dùng yêu cầu RUN.
    bool enable_uart_tx_neutral_only = true;

    // Bench motor-test safety.
    // Chỉ có tác dụng khi neutral_only được tắt có chủ đích.
    float uart_tx_test_max_speed_mps = 0.03f;

    // Trong motor bench test đầu tiên, luôn khóa servo ở center.
    bool uart_tx_force_center = true;
};

struct Config {
    CameraConfig camera;
    UartConfig uart;
    UdpConfig udp;
    VehicleConfig vehicle;
    PlannerConfig planner;
    MpcConfig mpc;
    ParkingConfig parking;
    RuntimeConfig runtime;
};


}  // namespace laas
