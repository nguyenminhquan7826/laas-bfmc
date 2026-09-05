#include <iostream>
#include <stdexcept>
#include <string>

#include "logical_robot/ParkingTrajectoryStatusPolicy.hpp"

namespace {

using laas::ParkingSafetyResult;
using laas::ParkingTrackerDebug;
using laas::ParkingTrajectoryMsg;
using laas::ParkingTrajectoryStatusPolicy;

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

ParkingTrajectoryMsg trajectory(std::uint64_t id)
{
    ParkingTrajectoryMsg msg;
    msg.header.valid = true;
    msg.trajectory_id = id;
    return msg;
}

ParkingTrackerDebug tracker(std::uint64_t id, bool goal = false)
{
    ParkingTrackerDebug debug;
    debug.valid = true;
    debug.goal_reached = goal;
    debug.trajectory_id = id;
    return debug;
}

ParkingSafetyResult safety(
    bool motion_allowed,
    const std::string& reason)
{
    ParkingSafetyResult result;
    result.evaluated = true;
    result.motion_allowed = motion_allowed;
    result.reason = reason;
    return result;
}

void testBenchNeverEmitsExecuting()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(10U);

    const auto update = policy.evaluate(
        trajectory(10U), tracker(10U), safety(true, "PASS_BENCH_ONLY"), false);

    CHECK_TRUE(!update.emit, "bench_emitted_status");
    CHECK_TRUE(policy.lastStatus() == "RECEIVED", "bench_changed_status");
}

void testAuthorizedTrackingEmitsExecutingOnce()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(11U);

    auto update = policy.evaluate(
        trajectory(11U), tracker(11U), safety(true, "PASS_BENCH_ONLY"), true);
    CHECK_TRUE(update.emit, "executing_not_emitted");
    CHECK_TRUE(update.status == "EXECUTING", "executing_wrong_status");

    update = policy.evaluate(
        trajectory(11U), tracker(11U), safety(true, "PASS_BENCH_ONLY"), true);
    CHECK_TRUE(!update.emit, "executing_spammed");
}

void testObstaclePauseLatchesAndPreventsBlindResume()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(12U);

    auto update = policy.evaluate(
        trajectory(12U), tracker(12U), safety(true, "PASS_BENCH_ONLY"), true);
    CHECK_TRUE(update.emit && update.status == "EXECUTING",
               "pre_pause_not_executing");

    update = policy.evaluate(
        trajectory(12U), tracker(12U),
        safety(false, "LOCAL_OBSTACLE_BLOCKING"), true);
    CHECK_TRUE(update.emit && update.status == "PAUSED", "pause_not_emitted");
    CHECK_TRUE(policy.pauseLatched(), "pause_not_latched");

    // Safety clears, but the old trajectory must never automatically resume.
    update = policy.evaluate(
        trajectory(12U), tracker(12U), safety(true, "PASS_BENCH_ONLY"), true);
    CHECK_TRUE(!update.emit, "old_trajectory_blind_resumed");
    CHECK_TRUE(policy.lastStatus() == "PAUSED", "pause_status_lost");
}

void testNewTrajectoryClearsPauseLatch()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(13U);
    policy.evaluate(
        trajectory(13U), tracker(13U),
        safety(false, "LOCAL_OBSTACLE_BLOCKING"), true);
    CHECK_TRUE(policy.pauseLatched(), "precondition_pause_not_latched");

    policy.onTrajectoryReceived(14U);
    CHECK_TRUE(!policy.pauseLatched(), "new_trajectory_did_not_clear_latch");

    const auto update = policy.evaluate(
        trajectory(14U), tracker(14U), safety(true, "PASS_BENCH_ONLY"), true);
    CHECK_TRUE(update.emit && update.status == "EXECUTING",
               "new_trajectory_cannot_execute");
}

void testGoalCompletedOnlyWithAuthorization()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(15U);

    auto update = policy.evaluate(
        trajectory(15U), tracker(15U, true), safety(false, "GOAL_HOLD"), false);
    CHECK_TRUE(!update.emit, "bench_emitted_completed");

    update = policy.evaluate(
        trajectory(15U), tracker(15U, true), safety(false, "GOAL_HOLD"), true);
    CHECK_TRUE(update.emit && update.status == "COMPLETED",
               "authorized_goal_not_completed");
}

void testValidationFaultDoesNotRaceSafetyReplan()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(16U);

    const auto update = policy.evaluate(
        trajectory(16U), tracker(16U),
        safety(false, "TRAJECTORY_INVALID"), true);

    CHECK_TRUE(!update.emit, "validation_fault_emitted_duplicate_status");
    CHECK_TRUE(policy.lastStatus() == "RECEIVED",
               "validation_fault_changed_status_epoch");
}

void testServerDisconnectPausesWhenAuthorized()
{
    ParkingTrajectoryStatusPolicy policy;
    policy.onTrajectoryReceived(17U);
    policy.evaluate(
        trajectory(17U), tracker(17U), safety(true, "PASS_BENCH_ONLY"), true);

    const auto update = policy.evaluate(
        trajectory(17U), tracker(17U),
        safety(false, "SERVER_DISCONNECTED"), true);

    CHECK_TRUE(update.emit && update.status == "PAUSED",
               "disconnect_pause_not_emitted");
    CHECK_TRUE(policy.pauseLatched(), "disconnect_pause_not_latched");
}

struct TestCase {
    const char* name;
    void (*fn)();
};

}  // namespace

int main()
{
    const TestCase tests[] = {
        {"bench_suppresses_executing", testBenchNeverEmitsExecuting},
        {"authorized_executing_transition", testAuthorizedTrackingEmitsExecutingOnce},
        {"pause_latch_no_blind_resume", testObstaclePauseLatchesAndPreventsBlindResume},
        {"new_trajectory_clears_pause", testNewTrajectoryClearsPauseLatch},
        {"completed_requires_authorization", testGoalCompletedOnlyWithAuthorization},
        {"validation_fault_no_status_race", testValidationFaultDoesNotRaceSafetyReplan},
        {"disconnect_pause_latched", testServerDisconnectPausesWhenAuthorized},
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

    std::cout << "[SUMMARY] all Step-12 trajectory status policy tests passed\n";
    return 0;
}
