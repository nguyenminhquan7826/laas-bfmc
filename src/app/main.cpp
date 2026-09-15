#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "../laas_core/Config.hpp"
#include "../laas_core/CpuAffinity.hpp"
#include "../execution_control/Executive.hpp"

namespace {

bool parseHomography(const std::string& text, std::array<double, 9>& output)
{
    std::stringstream stream(text);
    std::string token;
    std::size_t index = 0U;
    try {
        while (std::getline(stream, token, ',')) {
            if (index >= output.size()) {
                return false;
            }
            std::size_t consumed = 0U;
            output[index] = std::stod(token, &consumed);
            if (consumed != token.size()) {
                return false;
            }
            ++index;
        }
    } catch (const std::exception&) {
        return false;
    }
    return index == output.size();
}

}  // namespace

int main(int argc, char* argv[])
{
    // Raspberry Pi 5 measured policy after the 20 ms control-thread split:
    //   laas_pp -> affinity OFF; Linux may schedule its threads on CPU0-3.
    //   AI      -> CPU2,3 via ai/run_ai_affinity.py.
    // Pinning the whole laas_pp process to CPU1 or CPU0,1 reduced measured
    // scheduler/vision performance, so LAAS_MAIN_CPU is diagnostic only.
    if (!laas::configureMainCpuAffinity()) {
        return 2;
    }

    laas::Config config;

    if (argc > 1) {
        std::string mode = argv[1];
        if (mode == "pp" || mode == "pure_pursuit") {
            config.runtime.control_mode = laas::ControlMode::PURE_PURSUIT;
        } else if (mode == "mpc") {
            config.runtime.control_mode = laas::ControlMode::MPC;
        } else {
            std::cerr << "Usage: " << argv[0] << " [pp|mpc] [camera_or_video_path]\n";
            return 1;
        }
    }

    if (argc > 2) {
        config.camera.device = argv[2];
    }

#ifndef LAAS_ENABLE_MPC
    if (config.runtime.control_mode == laas::ControlMode::MPC) {
        std::cout << "[APP] MPC requested but LAAS_ENABLE_MPC was not defined. "
                  << "Executive will fall back to Pure Pursuit.\n";
    }
#endif

    // Step-11/12 bench runtime profile.
    // Enabled only when LAAS_PARKING_BENCH=1.
    const char* bench_env = std::getenv("LAAS_PARKING_BENCH");

    // Physical telemetry / pose integration bench.
    // This profile keeps parking actuation disabled while allowing real
    // STM32 encoder + IMU telemetry to feed VehiclePoseEstimator.
    const char* pose_bench_env =
        std::getenv("LAAS_PARKING_POSE_BENCH");

    const bool parking_bench_enabled =
        bench_env && std::string(bench_env) == "1";

    const bool parking_pose_bench_enabled =
        pose_bench_env && std::string(pose_bench_env) == "1";

    // Fail closed if two mutually exclusive parking bench profiles are
    // accidentally enabled together.
    if (parking_bench_enabled && parking_pose_bench_enabled) {
        std::cerr
            << "[APP] ERROR: LAAS_PARKING_BENCH and "
            << "LAAS_PARKING_POSE_BENCH cannot both be enabled.\n";
        return 3;
    }

    if (parking_bench_enabled) {

        // HARD safety gate: parking bench must never use UART.
        // No secondary bench option below is allowed to change this value.
        config.runtime.enable_uart = false;

        // Default parking handshake bench does not need AI. For scheduler/load
        // testing, LAAS_PARKING_BENCH_YOLO=1 enables only the local UDP YOLO
        // path while UART remains hard-disabled above.
        const char* bench_yolo_env =
            std::getenv("LAAS_PARKING_BENCH_YOLO");
        const bool bench_yolo_enabled =
            bench_yolo_env && std::string(bench_yolo_env) == "1";
        config.runtime.enable_yolo_udp = bench_yolo_enabled;

        // Bird-eye monitor traffic is not safety-critical and adds JPEG encode
        // work to perception. Keep it OFF for bench/load tests unless explicitly
        // requested. This never changes the UART safety gate above.
        const char* bench_debug_env =
            std::getenv("LAAS_PARKING_BENCH_DEBUG");
        const bool bench_debug_enabled =
            bench_yolo_enabled && bench_debug_env &&
            std::string(bench_debug_env) == "1";
        config.udp.enable_debug_stream = bench_debug_enabled;

        config.parking.enable = true;
        config.parking.bench_mode = true;

        config.parking.enable_bench_parking_status = true;
        config.parking.enable_bench_tracker = true;
        config.parking.enable_local_parking_planner = true;

        // No encoder/IMU telemetry in this bench build.
        config.parking.enable_pose_estimator = false;

        // Static initial map pose for Server V1 planning bench.
        config.parking.initial_pose_valid = true;
        config.parking.initial_x_m = 1.300;
        config.parking.initial_y_m = 0.751;
        config.parking.initial_yaw_rad = 0.0;

        // Explicit parking occupancy.
        config.parking.bench_p_b1 =
            laas::ParkingSlotState::OCCUPIED;

        config.parking.bench_p_b2 =
            laas::ParkingSlotState::FREE;

        config.parking.bench_p_t1 =
            laas::ParkingSlotState::OCCUPIED;

        config.parking.bench_p_t2 =
            laas::ParkingSlotState::OCCUPIED;

        // Server host can be changed without recompiling.
        const char* host =
            std::getenv("LAAS_PARKING_SERVER_HOST");

        if (host && *host) {
            config.parking.server_host = host;
        }

        const char* port =
            std::getenv("LAAS_PARKING_SERVER_PORT");

        if (port && *port) {
            const int value = std::atoi(port);

            if (value > 0 && value <= 65535) {
                config.parking.server_port = value;
            }
        }

        std::cout
            << "[APP][PARKING_BENCH]"
            << " UART=OFF"
            << " YOLO=" << (bench_yolo_enabled ? "ON" : "OFF")
            << " DEBUG=" << (bench_debug_enabled ? "ON" : "OFF")
            << " server="
            << config.parking.server_host
            << ":"
            << config.parking.server_port
            << " pose=("
            << config.parking.initial_x_m
            << ","
            << config.parking.initial_y_m
            << ","
            << config.parking.initial_yaw_rad
            << ")"
            << " FREE=P_B2"
            << "\n";
    }


    if (parking_pose_bench_enabled) {

        // Real STM32 telemetry is required for encoder + IMU pose estimation.
        config.runtime.enable_uart = true;

        // HARD safety gate: receive telemetry only.
        // Parking/control commands must not be transmitted to STM32.
        config.runtime.enable_uart_tx = false;

        // Secondary safety state remains neutral-only even though TX is
        // completely disabled above.
        config.runtime.enable_uart_tx_neutral_only = true;

        // Pose bench normally does not require YOLO. Explicitly allow the
        // local C++ -> AI frame path for model/homography calibration; this
        // does not change the UART TX hard gate above.
        const char* pose_bench_yolo_env =
            std::getenv("LAAS_PARKING_BENCH_YOLO");
        const bool pose_bench_yolo_enabled =
            pose_bench_yolo_env &&
            std::string(pose_bench_yolo_env) == "1";
        config.runtime.enable_yolo_udp = pose_bench_yolo_enabled;
        config.udp.enable_debug_stream = false;

        config.parking.enable = true;

        // Keep all parking execution in bench semantics.
        config.parking.bench_mode = true;
        config.parking.enable_bench_parking_status = true;
        config.parking.enable_bench_tracker = true;
        config.parking.enable_local_parking_planner = true;

        // Real encoder + IMU telemetry drives VehiclePoseEstimator.
        config.parking.enable_pose_estimator = true;

        // Known initial rear-axle-center map pose.
        // The IMU yaw at estimator startup becomes the relative yaw reference.
        config.parking.initial_pose_valid = true;
        config.parking.initial_x_m = 1.300;
        config.parking.initial_y_m = 0.751;
        config.parking.initial_yaw_rad = 0.0;

        // Explicit parking occupancy used only for the planning bench.
        config.parking.bench_p_b1 =
            laas::ParkingSlotState::OCCUPIED;

        config.parking.bench_p_b2 =
            laas::ParkingSlotState::FREE;

        config.parking.bench_p_t1 =
            laas::ParkingSlotState::OCCUPIED;

        config.parking.bench_p_t2 =
            laas::ParkingSlotState::OCCUPIED;

        const char* host =
            std::getenv("LAAS_PARKING_SERVER_HOST");

        if (host && *host) {
            config.parking.server_host = host;
        }

        const char* port =
            std::getenv("LAAS_PARKING_SERVER_PORT");

        if (port && *port) {
            const int value = std::atoi(port);

            if (value > 0 && value <= 65535) {
                config.parking.server_port = value;
            }
        }

        std::cout
            << "[APP][PARKING_POSE_BENCH]"
            << " UART_RX=ON"
            << " UART_TX=OFF"
            << " YOLO=" << (pose_bench_yolo_enabled ? "ON" : "OFF")
            << " POSE=ENCODER_IMU"
            << " TRACKER=BENCH_ONLY"
            << " server="
            << config.parking.server_host
            << ":"
            << config.parking.server_port
            << " initialPose=("
            << config.parking.initial_x_m
            << ","
            << config.parking.initial_y_m
            << ","
            << config.parking.initial_yaw_rad
            << ")"
            << " FREE=P_B2"
            << "\n";
    }

    // Real parking perception is an explicit opt-in on top of a parking
    // profile. It replaces the four synthetic slot states while preserving the
    // existing RX-only UART safety boundary.
    const char* perception_env = std::getenv("LAAS_PARKING_PERCEPTION");
    const bool parking_perception_enabled =
        perception_env && std::string(perception_env) == "1";
    if (parking_perception_enabled) {
        if (!config.parking.enable) {
            std::cerr << "[APP] LAAS_PARKING_PERCEPTION requires "
                      << "LAAS_PARKING_POSE_BENCH=1.\n";
            return 4;
        }

        const char* homography_env =
            std::getenv("LAAS_PARKING_IMAGE_TO_GROUND_H");
        if (!homography_env ||
            !parseHomography(homography_env,
                             config.parking.image_to_ground_homography)) {
            std::cerr
                << "[APP] LAAS_PARKING_IMAGE_TO_GROUND_H must contain "
                << "9 comma-separated coefficients.\n";
            return 5;
        }

        const char* map_file_env =
            std::getenv("LAAS_PARKING_MAP_FILE");
        if (map_file_env && *map_file_env) {
            config.parking.slot_map_file = map_file_env;
        }

        config.runtime.enable_yolo_udp = true;
        config.parking.enable_bench_parking_status = false;
        config.parking.enable_camera_parking_perception = true;
        config.parking.image_to_ground_homography_valid = true;

        std::cout
            << "[APP][PARKING_PERCEPTION] YOLO=ON STATUS=CAMERA_MAP "
            << "fakeSlots=OFF map="
            << config.parking.slot_map_file
            << " UART_TX="
            << (config.runtime.enable_uart_tx ? "ON" : "OFF")
            << "\n";
    }

    laas::Executive executive(config);
    if (!executive.init()) {
        return 1;
    }

    executive.run();
    return 0;
}
