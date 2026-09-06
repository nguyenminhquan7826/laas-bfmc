#pragma once

#include <cstdint>

#include "ParkingProtocol.hpp"

namespace laas {

// Small pure policy used by Executive when replaying safety events after a
// reconnect. It has no socket/UART access and deliberately treats a Server
// restart with no active trajectory as a new session scope.
class ParkingSafetyEventSyncPolicy {
public:
    static bool sessionOwnsTrajectory(
        const ParkingSessionSnapshot& session,
        std::uint64_t trajectory_id)
    {
        return trajectory_id > 0U &&
               session.has_active_trajectory &&
               session.active_trajectory_id == trajectory_id;
    }

    static bool canEmitSafetyCleared(
        bool session_snapshot_received,
        bool blocking_safety_active,
        bool safety_reason_is_clear)
    {
        return session_snapshot_received &&
               blocking_safety_active &&
               safety_reason_is_clear;
    }
};

}  // namespace laas
