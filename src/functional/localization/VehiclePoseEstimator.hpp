#pragma once

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

// Provisional rear-axle-center map pose estimator for parking integration.
// Position is dead-reckoned from signed encoder speed and yaw is aligned to the
// map frame from the first valid IMU sample. This is not a replacement for a
// final absolute localization system.
class VehiclePoseEstimator {
public:
    explicit VehiclePoseEstimator(const Config& config);

    void reset();
    bool process(const VehicleTelemetryMsg& telemetry, VehiclePoseMsg& out);

private:
    static double wrapAngle(double angle_rad);

    const Config& config_;

    bool initialized_{false};
    std::uint64_t last_receive_timestamp_ms_{0};
    double imu_reference_yaw_rad_{0.0};

    double x_m_{0.0};
    double y_m_{0.0};
    double yaw_rad_{0.0};
};

}  // namespace laas
