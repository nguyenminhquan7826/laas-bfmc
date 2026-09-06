#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected 1 match, found {count}")
    return text.replace(old, new, 1)


root = Path(__file__).resolve().parents[1]
cpp_path = root / "src/execution_control/Executive.cpp"
hpp_path = root / "src/execution_control/Executive.hpp"
policy_path = root / "src/logical_robot/ParkingSafetyEventSyncPolicy.hpp"
test_path = root / "server_v1/tests/test_parking_safety_event_sync_policy.cpp"

cpp = cpp_path.read_text()
hpp = hpp_path.read_text()

cpp = replace_once(
    cpp,
    '#include <chrono>\n',
    '#include <algorithm>\n#include <chrono>\n',
    'add algorithm include')

hpp = replace_once(
    hpp,
    '#include "../logical_robot/ParkingSessionSyncPolicy.hpp"\n#include "../logical_robot/ParkingTrajectoryStatusPolicy.hpp"',
    '#include "../logical_robot/ParkingSessionSyncPolicy.hpp"\n#include "../logical_robot/ParkingSafetyEventSyncPolicy.hpp"\n#include "../logical_robot/ParkingTrajectoryStatusPolicy.hpp"',
    'include safety event sync policy')

hpp = replace_once(
    hpp,
    '    void flushParkingSafetyEvents();\n\n#ifdef LAAS_ENABLE_PARKING_CLIENT\n    void applyParkingSessionSnapshot(',
    '    void flushParkingSafetyEvents();\n\n#ifdef LAAS_ENABLE_PARKING_CLIENT\n    void discardParkingSafetyEventsForTrajectory(\n        std::uint64_t trajectory_id,\n        const std::string& reason);\n\n    void reconcileParkingSafetyEventsWithSession(\n        const ParkingSessionSnapshot& session);\n\n    void applyParkingSessionSnapshot(',
    'declare safety queue reconciliation helpers')

hpp = replace_once(
    hpp,
    '    // Step-11: transition-based parking safety synchronization.\n    // Events generated during a TCP outage remain queued and are\n    // transmitted after the connection is restored.\n',
    '    // Step-11/12: transition-based parking safety synchronization.\n    // Events generated during a TCP outage remain queued, but reconnect\n    // never replays them until the new Server session snapshot is known.\n    // Trajectory-scoped events from a stale/restarted session are discarded.\n',
    'update safety queue comment')

hpp = replace_once(
    hpp,
    '    bool parking_session_query_pending_{false};\n    bool parking_session_sync_hold_{true};\n',
    '    bool parking_session_query_pending_{false};\n    bool parking_session_snapshot_received_{false};\n    bool parking_session_sync_hold_{true};\n',
    'add snapshot received gate')

cpp = replace_once(
    cpp,
    '        parking_trajectory_status_policy_.reset();\n        parking_session_query_pending_ = false;\n        parking_session_sync_hold_ = true;\n',
    '        parking_trajectory_status_policy_.reset();\n        parking_session_query_pending_ = false;\n        parking_session_snapshot_received_ = false;\n        parking_session_sync_hold_ = true;\n',
    'initialize snapshot gate')

cpp = replace_once(
    cpp,
    '        parking_session_sync_hold_ = true;\n        parking_session_sync_reason_ = "SERVER_DISCONNECTED";\n        parking_session_query_pending_ = false;\n        std::cout << "[PARKING][SYNC] disconnected -> HOLD\\n";\n',
    '        parking_session_sync_hold_ = true;\n        parking_session_sync_reason_ = "SERVER_DISCONNECTED";\n        parking_session_query_pending_ = false;\n        parking_session_snapshot_received_ = false;\n        std::cout << "[PARKING][SYNC] disconnected -> HOLD\\n";\n',
    'clear snapshot gate on disconnect')

cpp = replace_once(
    cpp,
    '        parking_session_sync_hold_ = true;\n        parking_session_sync_reason_ = "AWAITING_SESSION_SYNC";\n        parking_session_query_pending_ = true;\n\n        // Force the newest bench/local inputs to be republished on a\n',
    '        parking_session_sync_hold_ = true;\n        parking_session_sync_reason_ = "AWAITING_SESSION_SYNC";\n        parking_session_query_pending_ = true;\n        parking_session_snapshot_received_ = false;\n\n        // Force the newest bench/local inputs to be republished on a\n',
    'clear snapshot gate on reconnect')

cpp = replace_once(
    cpp,
    '    // A safety event may have been generated while TCP was down.\n    // Flush it before session_query so SERVER_TIMEOUT is ordered\n    // ahead of the snapshot request on the same TCP stream.\n    flushParkingSafetyEvents();\n\n    if (parking_server_connected_ && parking_session_query_pending_) {\n',
    '    // Do not replay trajectory-scoped safety events yet. A reconnect may\n    // be a fresh Server process with no active trajectory, so the session\n    // snapshot must be reconciled before any queued event can be trusted.\n    if (parking_server_connected_ && parking_session_query_pending_) {\n',
    'remove pre-session safety replay')

cpp = replace_once(
    cpp,
    '    if (!parking_server_connected_) {\n        return;\n    }\n\n    const VehiclePoseMsg pose = blackboard_.vehiclePose();\n',
    '    // applyParkingSessionSnapshot() reconciles queued safety events with\n    // the Server-owned trajectory. flushParkingSafetyEvents() itself remains\n    // gated until at least one snapshot has been received on this connection.\n    flushParkingSafetyEvents();\n\n    if (!parking_server_connected_) {\n        return;\n    }\n\n    const VehiclePoseMsg pose = blackboard_.vehiclePose();\n',
    'flush only after session message processing')

old_clear = '''void Executive::clearLocalParkingTrajectory(\n    const std::string& reason)\n{\n    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();\n    if (local.header.valid && local.trajectory_id > 0U) {\n        std::cout << "[PARKING][SYNC] clear local tid="\n                  << local.trajectory_id\n                  << " reason=" << reason << "\\n";\n    }\n\n    blackboard_.setParkingTrajectory(ParkingTrajectoryMsg{});\n    parking_trajectory_tracker_.reset();\n    parking_trajectory_status_policy_.reset();\n}\n\nvoid Executive::applyParkingSessionSnapshot(\n'''
new_clear = '''void Executive::discardParkingSafetyEventsForTrajectory(\n    std::uint64_t trajectory_id,\n    const std::string& reason)\n{\n    if (trajectory_id == 0U) {\n        return;\n    }\n\n    const std::size_t before = parking_safety_event_queue_.size();\n    parking_safety_event_queue_.erase(\n        std::remove_if(\n            parking_safety_event_queue_.begin(),\n            parking_safety_event_queue_.end(),\n            [trajectory_id](const PendingParkingSafetyEvent& pending) {\n                return pending.has_trajectory_id &&\n                       pending.trajectory_id == trajectory_id;\n            }),\n        parking_safety_event_queue_.end());\n\n    const std::size_t dropped =\n        before - parking_safety_event_queue_.size();\n    if (dropped > 0U) {\n        std::cout << "[PARKING][SAFETY] drop stale queued events count="\n                  << dropped\n                  << " tid=" << trajectory_id\n                  << " reason=" << reason << "\\n";\n    }\n}\n\nvoid Executive::reconcileParkingSafetyEventsWithSession(\n    const ParkingSessionSnapshot& session)\n{\n    const std::size_t before = parking_safety_event_queue_.size();\n\n    parking_safety_event_queue_.erase(\n        std::remove_if(\n            parking_safety_event_queue_.begin(),\n            parking_safety_event_queue_.end(),\n            [&session](const PendingParkingSafetyEvent& pending) {\n                if (!pending.has_trajectory_id) {\n                    return false;\n                }\n                return !ParkingSafetyEventSyncPolicy::sessionOwnsTrajectory(\n                    session, pending.trajectory_id);\n            }),\n        parking_safety_event_queue_.end());\n\n    const std::size_t dropped =\n        before - parking_safety_event_queue_.size();\n    if (dropped > 0U) {\n        parking_blocking_safety_active_ = false;\n        std::cout << "[PARKING][SAFETY] reconcile dropped="\n                  << dropped\n                  << " serverTid="\n                  << (session.has_active_trajectory\n                          ? std::to_string(session.active_trajectory_id)\n                          : std::string("none"))\n                  << "\\n";\n    }\n}\n\nvoid Executive::clearLocalParkingTrajectory(\n    const std::string& reason)\n{\n    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();\n    if (local.header.valid && local.trajectory_id > 0U) {\n        std::cout << "[PARKING][SYNC] clear local tid="\n                  << local.trajectory_id\n                  << " reason=" << reason << "\\n";\n        discardParkingSafetyEventsForTrajectory(\n            local.trajectory_id, reason);\n    }\n\n    // Blocking safety state belongs to the trajectory/session being cleared.\n    // Never let it generate SAFETY_CLEARED for a later replacement trajectory.\n    parking_blocking_safety_active_ = false;\n    last_parking_safety_reason_ = "NOT_EVALUATED";\n\n    blackboard_.setParkingTrajectory(ParkingTrajectoryMsg{});\n    parking_trajectory_tracker_.reset();\n    parking_trajectory_status_policy_.reset();\n}\n\nvoid Executive::applyParkingSessionSnapshot(\n'''
cpp = replace_once(cpp, old_clear, new_clear, 'add safety queue reconciliation helpers')

cpp = replace_once(
    cpp,
    'void Executive::applyParkingSessionSnapshot(\n    const ParkingSessionSnapshot& session)\n{\n    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();\n',
    'void Executive::applyParkingSessionSnapshot(\n    const ParkingSessionSnapshot& session)\n{\n    reconcileParkingSafetyEventsWithSession(session);\n    parking_session_snapshot_received_ = true;\n\n    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();\n',
    'mark reconciled session snapshot')

cpp = replace_once(
    cpp,
    '    if (!config_.parking.enable ||\n        !parking_server_connected_) {\n        return;\n    }\n',
    '    if (!config_.parking.enable ||\n        !parking_server_connected_ ||\n        !parking_session_snapshot_received_) {\n        return;\n    }\n',
    'gate safety flush on session snapshot')

old_clear_branch = '''    else if (\n        parkingSafetyReasonIsClear(reason) &&\n        parking_blocking_safety_active_) {\n\n        queueParkingSafetyEvent(\n            "SAFETY_CLEARED",\n            trajectory_id,\n            true);\n'''
new_clear_branch = '''    else if (\n        ParkingSafetyEventSyncPolicy::canEmitSafetyCleared(\n            parking_session_snapshot_received_,\n            parking_blocking_safety_active_,\n            parkingSafetyReasonIsClear(reason))) {\n\n        queueParkingSafetyEvent(\n            "SAFETY_CLEARED",\n            trajectory_id,\n            true);\n'''
cpp = replace_once(cpp, old_clear_branch, new_clear_branch, 'gate SAFETY_CLEARED on session reconciliation')

policy_path.write_text(r'''#pragma once

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
''')

test_path.write_text(r'''#include <cstdint>
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
''')

# Structural regression checks for the exact field failure observed on Pi:
# queued outage events must not be sent before session_query/snapshot, and a
# clear event must be impossible while the connection is awaiting sync.
network_start = cpp.index('void Executive::parkingNetworkTick()')
network_end = cpp.index('#ifdef LAAS_ENABLE_PARKING_CLIENT\nvoid Executive::discardParkingSafetyEventsForTrajectory', network_start)
network = cpp[network_start:network_end]
assert network.index('sendSessionQuery()') < network.index('flushParkingSafetyEvents();')
assert 'reconcileParkingSafetyEventsWithSession(session);' in cpp
assert 'parking_session_snapshot_received_ = true;' in cpp
assert 'ParkingSafetyEventSyncPolicy::canEmitSafetyCleared' in cpp
assert '!parking_session_snapshot_received_' in cpp
assert 'drop stale queued events' in cpp

cpp_path.write_text(cpp)
hpp_path.write_text(hpp)

print('[PASS] applied reconnect safety-event reconciliation patch')
