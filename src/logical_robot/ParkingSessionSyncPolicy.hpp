#pragma once

#include <cstdint>
#include <string>

#include "ParkingProtocol.hpp"

namespace laas {

struct ParkingSessionSyncDecision {
    bool session_valid{false};
    bool trajectory_consistent{false};

    // True means the local parking controller must remain stopped even if TCP
    // is connected. This is a synchronization hold, not an actuator command.
    bool hold_motion{true};

    // Clear an old/mismatched trajectory from the Pi blackboard.
    bool clear_local_trajectory{false};

    // Ask Server V1 to replace its active trajectory because Pi does not have
    // the same trajectory. Never request this while the server is PAUSED;
    // safety clear/replan owns that transition.
    bool request_replan{false};
    std::uint64_t replan_trajectory_id{0};

    std::string reason;
};

// Step-12 policy for reconciling the Pi's local trajectory with Server V1's
// session snapshot after connect/reconnect.
//
// Safety invariants:
// - Mismatch never authorizes motion.
// - PAUSED/REPLAN/PLANNING/WAITING_INPUT/COMPLETED always hold motion.
// - A PAUSED server is never forced out of pause by this policy.
// - This class has no UART or socket access; Executive remains the orchestrator.
class ParkingSessionSyncPolicy {
public:
    static ParkingSessionSyncDecision evaluate(
        const ParkingSessionSnapshot& server,
        const ParkingTrajectoryMsg& local)
    {
        ParkingSessionSyncDecision out;

        if (!validState(server.state)) {
            out.clear_local_trajectory = local.header.valid &&
                                         local.trajectory_id > 0U;
            out.reason = "INVALID_SERVER_SESSION_STATE";
            return out;
        }

        out.session_valid = true;

        const bool local_active =
            local.header.valid && local.trajectory_id > 0U;

        if (server.has_active_trajectory) {
            if (server.active_trajectory_id == 0U) {
                out.clear_local_trajectory = local_active;
                out.reason = "INVALID_SERVER_ACTIVE_TRAJECTORY_ID";
                return out;
            }

            if (local_active &&
                local.trajectory_id == server.active_trajectory_id) {
                out.trajectory_consistent = true;

                // Matching IDs are necessary but not sufficient to move.
                // Server pause/replan states remain an explicit hold.
                if (server.state == "TRAJECTORY_READY" ||
                    server.state == "EXECUTING") {
                    out.hold_motion = false;
                    out.reason = "SESSION_TRAJECTORY_MATCH";
                } else {
                    out.reason = "SERVER_STATE_HOLD_" + server.state;
                }
                return out;
            }

            out.clear_local_trajectory = local_active;
            out.reason = local_active
                ? "TRAJECTORY_ID_MISMATCH"
                : "PI_MISSING_SERVER_TRAJECTORY";

            // Only request replacement when the server considers the current
            // trajectory runnable. A PAUSED trajectory must wait for the
            // safety-clear path; forcing REPLAN here would bypass pause safety.
            if (server.state == "TRAJECTORY_READY" ||
                server.state == "EXECUTING") {
                out.request_replan = true;
                out.replan_trajectory_id = server.active_trajectory_id;
            }
            return out;
        }

        // Server has no active trajectory. Any local trajectory is stale and
        // must be discarded. Waiting/planning/replan states will produce a new
        // trajectory through the normal Server V1 planning flow.
        if (local_active) {
            out.clear_local_trajectory = true;
            out.reason = "SERVER_HAS_NO_ACTIVE_TRAJECTORY";
            return out;
        }

        out.trajectory_consistent = true;
        out.reason = "NO_ACTIVE_TRAJECTORY";
        return out;
    }

private:
    static bool validState(const std::string& state)
    {
        return state == "IDLE" ||
               state == "WAITING_INPUT" ||
               state == "PLANNING" ||
               state == "TRAJECTORY_READY" ||
               state == "EXECUTING" ||
               state == "PAUSED" ||
               state == "REPLAN" ||
               state == "COMPLETED";
    }
};

}  // namespace laas
