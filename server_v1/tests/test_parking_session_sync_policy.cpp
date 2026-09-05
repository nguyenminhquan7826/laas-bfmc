#include <iostream>
#include <stdexcept>
#include <string>

#include "logical_robot/ParkingSessionSyncPolicy.hpp"

namespace {

using laas::ParkingSessionSnapshot;
using laas::ParkingSessionSyncDecision;
using laas::ParkingSessionSyncPolicy;
using laas::ParkingTrajectoryMsg;

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

ParkingTrajectoryMsg localTrajectory(std::uint64_t id)
{
    ParkingTrajectoryMsg trajectory;
    trajectory.header.valid = id > 0U;
    trajectory.trajectory_id = id;
    trajectory.map_id = "map_v1";
    return trajectory;
}

ParkingSessionSnapshot serverSession(
    const std::string& state,
    bool has_active,
    std::uint64_t trajectory_id = 0U)
{
    ParkingSessionSnapshot session;
    session.session_id = 1U;
    session.state = state;
    session.has_active_trajectory = has_active;
    session.active_trajectory_id = trajectory_id;
    session.target_slot = has_active ? "P_B2" : "";
    return session;
}

void testMatchingReadyTrajectoryCanLeaveSyncHold()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("TRAJECTORY_READY", true, 11U),
        localTrajectory(11U));

    CHECK_TRUE(decision.session_valid, "ready_session_invalid");
    CHECK_TRUE(decision.trajectory_consistent, "ready_not_consistent");
    CHECK_TRUE(!decision.hold_motion, "ready_unexpected_hold");
    CHECK_TRUE(!decision.clear_local_trajectory, "ready_clear_local");
    CHECK_TRUE(!decision.request_replan, "ready_replan_requested");
}

void testPausedMatchingTrajectoryRemainsHeld()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("PAUSED", true, 12U),
        localTrajectory(12U));

    CHECK_TRUE(decision.trajectory_consistent, "paused_match_not_consistent");
    CHECK_TRUE(decision.hold_motion, "paused_match_motion_allowed");
    CHECK_TRUE(!decision.request_replan, "paused_match_forced_replan");
}

void testMismatchedReadyTrajectoryClearsAndRequestsReplan()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("TRAJECTORY_READY", true, 20U),
        localTrajectory(19U));

    CHECK_TRUE(decision.hold_motion, "mismatch_motion_allowed");
    CHECK_TRUE(decision.clear_local_trajectory, "mismatch_local_not_cleared");
    CHECK_TRUE(decision.request_replan, "mismatch_replan_missing");
    CHECK_TRUE(decision.replan_trajectory_id == 20U,
               "mismatch_wrong_replan_tid");
}

void testMissingLocalReadyTrajectoryRequestsReplacement()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("EXECUTING", true, 21U),
        localTrajectory(0U));

    CHECK_TRUE(decision.hold_motion, "missing_local_motion_allowed");
    CHECK_TRUE(!decision.clear_local_trajectory, "missing_local_clear_flag");
    CHECK_TRUE(decision.request_replan, "missing_local_replan_missing");
    CHECK_TRUE(decision.replan_trajectory_id == 21U,
               "missing_local_wrong_tid");
}

void testPausedMismatchDoesNotBypassSafetyPause()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("PAUSED", true, 30U),
        localTrajectory(29U));

    CHECK_TRUE(decision.hold_motion, "paused_mismatch_motion_allowed");
    CHECK_TRUE(decision.clear_local_trajectory,
               "paused_mismatch_local_not_cleared");
    CHECK_TRUE(!decision.request_replan,
               "paused_mismatch_bypassed_pause_with_replan");
}

void testServerWithoutActiveTrajectoryClearsLocal()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("REPLAN", false),
        localTrajectory(40U));

    CHECK_TRUE(decision.hold_motion, "no_server_tid_motion_allowed");
    CHECK_TRUE(decision.clear_local_trajectory,
               "no_server_tid_local_not_cleared");
    CHECK_TRUE(!decision.request_replan,
               "no_server_tid_duplicate_replan");
}

void testNoActiveTrajectoryBothSidesIsConsistentButHeld()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("WAITING_INPUT", false),
        localTrajectory(0U));

    CHECK_TRUE(decision.session_valid, "waiting_session_invalid");
    CHECK_TRUE(decision.trajectory_consistent,
               "waiting_no_trajectory_not_consistent");
    CHECK_TRUE(decision.hold_motion, "waiting_motion_allowed");
}

void testInvalidServerStateFailsClosed()
{
    const auto decision = ParkingSessionSyncPolicy::evaluate(
        serverSession("UNKNOWN_STATE", true, 50U),
        localTrajectory(50U));

    CHECK_TRUE(!decision.session_valid, "invalid_state_accepted");
    CHECK_TRUE(decision.hold_motion, "invalid_state_motion_allowed");
    CHECK_TRUE(decision.clear_local_trajectory,
               "invalid_state_local_not_cleared");
}

struct TestCase {
    const char* name;
    void (*fn)();
};

}  // namespace

int main()
{
    const TestCase tests[] = {
        {"matching_ready", testMatchingReadyTrajectoryCanLeaveSyncHold},
        {"paused_matching_hold", testPausedMatchingTrajectoryRemainsHeld},
        {"mismatched_ready", testMismatchedReadyTrajectoryClearsAndRequestsReplan},
        {"missing_local_ready", testMissingLocalReadyTrajectoryRequestsReplacement},
        {"paused_mismatch_no_bypass", testPausedMismatchDoesNotBypassSafetyPause},
        {"server_no_active_clears_local", testServerWithoutActiveTrajectoryClearsLocal},
        {"both_no_active_held", testNoActiveTrajectoryBothSidesIsConsistentButHeld},
        {"invalid_server_state", testInvalidServerStateFailsClosed},
    };

    int failures = 0;
    for (const auto& test : tests) {
        try {
            test.fn();
            std::cout << "[PASS] " << test.name << "\n";
        } catch (const std::exception& exc) {
            ++failures;
            std::cerr << "[FAIL] " << test.name << ": " << exc.what() << "\n";
        }
    }

    if (failures != 0) {
        std::cerr << "[SUMMARY] failures=" << failures << "\n";
        return 1;
    }

    std::cout << "[SUMMARY] all Step-12 session sync policy tests passed\n";
    return 0;
}
