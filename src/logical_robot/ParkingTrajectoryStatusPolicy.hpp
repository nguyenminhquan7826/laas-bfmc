#pragma once

#include <cstdint>
#include <string>

#include "../functional/control/ParkingTrajectoryTracker.hpp"
#include "../functional/safety/ParkingSafetyFilter.hpp"

namespace laas {

struct ParkingTrajectoryStatusUpdate {
    bool emit{false};
    std::uint64_t trajectory_id{0};
    std::string status;
    std::string reason;
};

// Step-12 transition policy for trajectory_status messages.
//
// This class does not send network messages and never commands UART. Executive
// may use its output to publish status only after a trajectory was accepted.
//
// Critical invariants:
// - No EXECUTING/COMPLETED status without explicit actuation authorization.
// - A blocking pause latches the current trajectory. Clearing the condition
//   does NOT resume the same trajectory; a newly received trajectory is
//   required before EXECUTING can be emitted again.
// - Validation/replan faults remain owned by the existing safety_event path,
//   avoiding duplicate REPLAN_REQUESTED + TRAJECTORY_INVALID races.
class ParkingTrajectoryStatusPolicy {
public:
    void reset()
    {
        active_trajectory_id_ = 0U;
        last_status_.clear();
        pause_latched_ = false;
    }

    void onTrajectoryReceived(std::uint64_t trajectory_id)
    {
        active_trajectory_id_ = trajectory_id;
        last_status_ = trajectory_id > 0U ? "RECEIVED" : std::string();
        pause_latched_ = false;
    }

    ParkingTrajectoryStatusUpdate evaluate(
        const ParkingTrajectoryMsg& trajectory,
        const ParkingTrackerDebug& tracker,
        const ParkingSafetyResult& safety,
        bool actuation_authorized)
    {
        ParkingTrajectoryStatusUpdate out;

        if (!trajectory.header.valid || trajectory.trajectory_id == 0U) {
            return out;
        }

        if (trajectory.trajectory_id != active_trajectory_id_) {
            // Executive sends RECEIVED after Pi validation. Treat a new ID as
            // a new status epoch, but do not fabricate RECEIVED here.
            onTrajectoryReceived(trajectory.trajectory_id);
            return out;
        }

        out.trajectory_id = trajectory.trajectory_id;

        // Bench/static tracking must never make Server believe physical motion
        // is executing or completed.
        if (!actuation_authorized) {
            return out;
        }

        if (!safety.evaluated) {
            return out;
        }

        if (isBlockingPauseReason(safety.reason)) {
            pause_latched_ = true;
            return transition("PAUSED", safety.reason);
        }

        // No blind resume: once this trajectory was paused, only reception of
        // a different/new trajectory resets the latch.
        if (pause_latched_) {
            return out;
        }

        if (safety.reason == "GOAL_HOLD" &&
            tracker.valid && tracker.goal_reached &&
            tracker.trajectory_id == trajectory.trajectory_id) {
            return transition("COMPLETED", "TRACKER_GOAL_REACHED");
        }

        // Faults such as TRAJECTORY_INVALID, POSE_STALE, etc. are already
        // translated to TRAJECTORY_INVALID safety events by Executive. Do not
        // race that path with a second trajectory_status replan trigger.
        if (!safety.motion_allowed) {
            return out;
        }

        if (tracker.valid && !tracker.goal_reached &&
            tracker.trajectory_id == trajectory.trajectory_id) {
            return transition("EXECUTING", "ACTUATION_AUTHORIZED_TRACKING");
        }

        return out;
    }

    bool pauseLatched() const { return pause_latched_; }
    std::uint64_t activeTrajectoryId() const { return active_trajectory_id_; }
    const std::string& lastStatus() const { return last_status_; }

private:
    static bool isBlockingPauseReason(const std::string& reason)
    {
        return reason == "LOCAL_OBSTACLE_BLOCKING" ||
               reason == "SERVER_DISCONNECTED";
    }

    ParkingTrajectoryStatusUpdate transition(
        const std::string& status,
        const std::string& reason)
    {
        ParkingTrajectoryStatusUpdate out;
        out.trajectory_id = active_trajectory_id_;

        if (status == last_status_) {
            return out;
        }

        last_status_ = status;
        out.emit = true;
        out.status = status;
        out.reason = reason;
        return out;
    }

private:
    std::uint64_t active_trajectory_id_{0U};
    std::string last_status_;
    bool pause_latched_{false};
};

}  // namespace laas
