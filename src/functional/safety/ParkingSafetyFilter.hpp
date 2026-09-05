#pragma once

#include <string>

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"
#include "../control/ParkingTrajectoryTracker.hpp"

namespace laas {

struct ParkingSafetyResult {
    bool evaluated = false;
    bool motion_allowed = false;
    std::string reason = "NOT_EVALUATED";
};

// Dedicated safety gate for map-frame parking control.
//
// It intentionally does not reuse SafetyFilterModule because the lane filter
// requires lane freshness and clamps speed to non-negative values. Parking has
// a different contract and must support reverse motion while remaining
// fail-closed on stale pose/network/trajectory/control data.
class ParkingSafetyFilter {
public:
    explicit ParkingSafetyFilter(const Config& config);

    ControlCmdMsg filter(const ControlCmdMsg& raw_cmd,
                         const ParkingTrackerDebug& tracker,
                         const VehiclePoseMsg& pose,
                         const ParkingTrajectoryMsg& trajectory,
                         const VehicleTelemetryMsg& telemetry,
                         const ObstacleMsg& obstacle,
                         bool server_connected,
                         ParkingSafetyResult* result = nullptr) const;

private:
    int steeringToServo(float steering_deg) const;
    static bool finiteCommand(const ControlCmdMsg& command);

private:
    Config config_;
};

}  // namespace laas
