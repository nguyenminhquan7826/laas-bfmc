#include <cstdlib>
#include <iostream>
#include <string>

#include "../laas_core/Config.hpp"
#include "../laas_core/CpuAffinity.hpp"
#include "../execution_control/Executive.hpp"

int main(int argc, char* argv[])
{
    // Raspberry Pi 5 measured CPU policy:
    //   laas_pp -> affinity OFF by default so Linux can schedule the current
    //              cooperative camera/perception/control workload freely.
    //   AI      -> CPU2,3 via ai/run_ai_affinity.py.
    // LAAS_MAIN_CPU=<index> remains available for explicit benchmark tests.
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

    // Step-11 bench runtime profile.
    // Enabled only when LAAS_PARKING_BENCH=1.
    const char* bench_env = std::getenv("LAAS_PARKING_BENCH");

    if (bench_env && std::string(bench_env) == "1") {

        // HARD safety gate: parking bench must never use UART.
        config.runtime.enable_uart = false;

        // We do not need YOLO for the first network/safety handshake test.
        config.runtime.enable_yolo_udp = false;

        config.parking.enable = true;
        config.parking.bench_mode = true;

        config.parking.enable_bench_parking_status = true;
        config.parking.enable_bench_tracker = true;

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

    laas::Executive executive(config);
    if (!executive.init()) {
        return 1;
    }

    executive.run();
    return 0;
}
