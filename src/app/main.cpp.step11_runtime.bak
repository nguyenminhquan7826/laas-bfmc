#include <iostream>
#include <string>

#include "../laas_core/Config.hpp"
#include "../execution_control/Executive.hpp"

int main(int argc, char* argv[])
{
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

    laas::Executive executive(config);
    if (!executive.init()) {
        return 1;
    }

    executive.run();
    return 0;
}
