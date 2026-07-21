#pragma once

#include <string>
#include "Messages.hpp"

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

    // Change this to the current IPv4 address of the monitoring laptop.
    std::string monitor_ip = "192.168.1.104";

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
    float wheelbase_m = 0.2515f;
    float desired_speed_mps = 0.15f;
    float steering_limit_deg = 25.0f;
    float servo_center = 94.5f;
    int servo_min = 60;
    int servo_max = 130;
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

struct RuntimeConfig {
    int obstacle_timeout_ms = 500;

    int frame_timeout_ms = 150;
    int lane_timeout_ms = 300;
    int behavior_timeout_ms = 200;
    int trajectory_timeout_ms = 200;
    int control_command_timeout_ms = 100;

    ControlMode control_mode = ControlMode::MPC;

    int camera_period_ms = 33;
    int yolo_period_ms = 33;
    int perception_period_ms = 33;
    int decision_period_ms = 50;
    int planning_period_ms = 50;
    int control_period_ms = 20;
    int logging_period_ms = 100;

    bool enable_keyboard = true;
    bool enable_yolo_udp = true;

    // Giữ false trong giai đoạn kiểm thử an toàn
    bool enable_uart = false;
};

struct Config {
    CameraConfig camera;
    UartConfig uart;
    UdpConfig udp;
    VehicleConfig vehicle;
    PlannerConfig planner;
    MpcConfig mpc;
    RuntimeConfig runtime;
};


}  // namespace laas
