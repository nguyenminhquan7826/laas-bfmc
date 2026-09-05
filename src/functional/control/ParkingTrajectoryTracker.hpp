#pragma once

#include <cstddef>
#include <cstdint>

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

struct ParkingTrackerDebug {
    bool valid = false;
    bool goal_reached = false;
    std::uint64_t trajectory_id = 0;
    std::size_t nearest_index = 0;
    std::size_t target_index = 0;
    double nearest_distance_m = 0.0;
    double target_distance_m = 0.0;
    MotionDirection direction = MotionDirection::FORWARD;
};

// Map-frame Pure Pursuit tracker for Server V1 parking trajectories.
//
// This controller is deliberately separate from PurePursuitController, which
// tracks BEV/pixel lane centerlines. ParkingTrajectoryTracker consumes a
// rear-axle-center pose and map-frame trajectory points in metres/radians and
// supports both FORWARD and REVERSE primitives.
//
// Current integration use is bench-only: Executive logs its output but does
// not route the command to UartVehicleInterface.
class ParkingTrajectoryTracker {
public:
    explicit ParkingTrajectoryTracker(const Config& config);

    void reset();

    bool process(const VehiclePoseMsg& pose,
                 const ParkingTrajectoryMsg& trajectory,
                 ControlCmdMsg& command,
                 ParkingTrackerDebug* debug = nullptr);

private:
    std::size_t findNearestIndex(const VehiclePoseMsg& pose,
                                 const ParkingTrajectoryMsg& trajectory);

    std::size_t findLookaheadIndex(std::size_t nearest_index,
                                   const ParkingTrajectoryMsg& trajectory,
                                   MotionDirection direction) const;

    bool computeSteering(const VehiclePoseMsg& pose,
                         const ParkingTrajectoryPoint& target,
                         MotionDirection direction,
                         float& steering_deg,
                         double& target_distance_m) const;

    static double normalizeAngle(double angle_rad);
    static double distance(double x0, double y0, double x1, double y1);

private:
    Config config_;
    std::uint64_t active_trajectory_id_ = 0;
    std::size_t progress_index_ = 0;
};

}  // namespace laas
