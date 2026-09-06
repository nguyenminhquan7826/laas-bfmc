#include <cstdint>
#include <iostream>
#include <string>

#include "logical_robot/ParkingSafetyEventSyncPolicy.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& label)
{
    if (!condition) {
        std::cerr << "[FAIL] " << label << "\n";
        ++failures;
    } else {
        std::cout << "[PASS] " << label << "\n";
    }
}

}  // namespace

int main()
{
    using laas::ParkingSafetyEventSyncPolicy;
    using laas::ParkingSessionSnapshot;

    ParkingSessionSnapshot restarted;
    restarted.state = "IDLE";
    restarted.has_active_trajectory = false;
    restarted.active_trajectory_id = 0U;

    expect(
        !ParkingSafetyEventSyncPolicy::sessionOwnsTrajectory(restarted, 1U),
        "server restart with no active trajectory invalidates queued tid=1 events");

    ParkingSessionSnapshot same_session;
    same_session.state = "TRAJECTORY_READY";
    same_session.has_active_trajectory = true;
    same_session.active_trajectory_id = 7U;

    expect(
        ParkingSafetyEventSyncPolicy::sessionOwnsTrajectory(same_session, 7U),
        "same active trajectory may retain its queued safety event");
    expect(
        !ParkingSafetyEventSyncPolicy::sessionOwnsTrajectory(same_session, 8U),
        "mismatched trajectory event is stale after reconnect");

    expect(
        !ParkingSafetyEventSyncPolicy::canEmitSafetyCleared(false, true, true),
        "SAFETY_CLEARED is blocked before session snapshot");
    expect(
        !ParkingSafetyEventSyncPolicy::canEmitSafetyCleared(true, false, true),
        "SAFETY_CLEARED requires an active blocking safety transition");
    expect(
        !ParkingSafetyEventSyncPolicy::canEmitSafetyCleared(true, true, false),
        "SAFETY_CLEARED requires a locally clear safety reason");
    expect(
        ParkingSafetyEventSyncPolicy::canEmitSafetyCleared(true, true, true),
        "SAFETY_CLEARED is allowed only after session reconciliation");

    if (failures != 0) {
        std::cerr << failures << " reconnect safety policy checks failed\n";
        return 1;
    }

    std::cout << "[PASS] reconnect safety event sync policy\n";
    return 0;
}
