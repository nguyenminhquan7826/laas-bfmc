#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "functional/safety/ParkingTrajectoryValidator.hpp"
#include "laas_core/Time.hpp"
#include "logical_robot/ParkingServerClient.hpp"

namespace {

using laas::Config;
using laas::ParkingServerClient;
using laas::ParkingServerMessage;
using laas::ParkingServerMessageType;
using laas::ParkingSlotObservation;
using laas::ParkingSlotState;
using laas::ParkingStatusMsg;
using laas::ParkingTrajectoryMsg;
using laas::ParkingTrajectoryValidator;
using laas::VehiclePoseMsg;

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

Config makeConfig(int port)
{
    Config config;
    config.parking.enable = true;
    config.parking.bench_mode = true;
    config.parking.server_host = "127.0.0.1";
    config.parking.server_port = port;
    config.parking.reconnect_period_ms = 20;
    config.parking.max_ndjson_line_bytes = 65536;
    config.parking.map_id = "map_v1";
    return config;
}

ParkingSlotObservation makeSlot(
    const std::string& id,
    ParkingSlotState state)
{
    ParkingSlotObservation slot;
    slot.id = id;
    slot.state = state;
    slot.confidence = 1.0F;
    return slot;
}

VehiclePoseMsg makePose(std::uint64_t sequence)
{
    VehiclePoseMsg pose;
    pose.header.valid = true;
    pose.header.timestamp_ms = laas::nowMs();
    pose.sequence = sequence;
    pose.map_id = "map_v1";
    pose.source = "CI_STATIC_BENCH_POSE";
    pose.x_m = 1.300;
    pose.y_m = 0.751;
    pose.yaw_rad = 0.0;
    return pose;
}

ParkingStatusMsg makeParkingStatus(std::uint64_t sequence)
{
    ParkingStatusMsg status;
    status.header.valid = true;
    status.header.timestamp_ms = laas::nowMs();
    status.sequence = sequence;
    status.map_id = "map_v1";
    status.slots.push_back(makeSlot("P_B1", ParkingSlotState::OCCUPIED));
    status.slots.push_back(makeSlot("P_B2", ParkingSlotState::FREE));
    status.slots.push_back(makeSlot("P_T1", ParkingSlotState::OCCUPIED));
    status.slots.push_back(makeSlot("P_T2", ParkingSlotState::OCCUPIED));
    return status;
}

bool waitConnected(ParkingServerClient& client, int timeout_ms = 3000)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        client.service();
        if (client.connected()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
}

ParkingServerMessage waitForType(
    ParkingServerClient& client,
    ParkingServerMessageType expected,
    int timeout_ms,
    std::uint64_t ignore_trajectory_id = 0)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        client.service();

        ParkingServerMessage message;
        while (client.popMessage(message)) {
            if (message.type == ParkingServerMessageType::ERROR) {
                throw std::runtime_error(
                    std::string("server_protocol_error:") + message.reason);
            }

            if (message.type == ParkingServerMessageType::PLANNING_RESULT) {
                std::cout << "[E2E] planning_result status=" << message.status
                          << " reason=" << message.reason << "\n";
            }

            if (message.type != expected) {
                continue;
            }

            if (expected == ParkingServerMessageType::TRAJECTORY &&
                ignore_trajectory_id != 0 &&
                message.trajectory.trajectory_id == ignore_trajectory_id) {
                continue;
            }

            return message;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    throw std::runtime_error("timeout_waiting_for_server_message");
}

void refreshInputs(
    ParkingServerClient& client,
    std::uint64_t sequence)
{
    const VehiclePoseMsg pose = makePose(sequence);
    const ParkingStatusMsg status = makeParkingStatus(sequence);
    CHECK_TRUE(client.sendVehiclePose(pose), "refresh_pose_send_failed");
    CHECK_TRUE(client.sendParkingStatus(status), "refresh_parking_send_failed");
}

void assertSessionState(
    ParkingServerClient& client,
    const std::string& expected_state,
    std::uint64_t expected_trajectory_id = 0)
{
    CHECK_TRUE(client.sendSessionQuery(), "session_query_send_failed");
    const ParkingServerMessage status = waitForType(
        client, ParkingServerMessageType::SESSION_STATUS, 3000);

    CHECK_TRUE(status.has_session, "session_status_missing_snapshot");
    CHECK_TRUE(status.session.state == expected_state,
               std::string("unexpected_session_state:") + status.session.state);

    if (expected_trajectory_id != 0) {
        CHECK_TRUE(status.session.has_active_trajectory,
                   "session_missing_active_trajectory");
        CHECK_TRUE(status.session.active_trajectory_id == expected_trajectory_id,
                   "session_active_trajectory_mismatch");
    }
}

ParkingTrajectoryMsg waitForValidatedTrajectory(
    ParkingServerClient& client,
    const Config& config,
    const VehiclePoseMsg& pose,
    std::uint64_t previous_trajectory_id = 0,
    int timeout_ms = 30000)
{
    const ParkingServerMessage message = waitForType(
        client,
        ParkingServerMessageType::TRAJECTORY,
        timeout_ms,
        previous_trajectory_id);

    ParkingTrajectoryValidator validator(config);
    const auto validation = validator.validate(message.trajectory, pose);
    CHECK_TRUE(validation.accepted,
               std::string("pi_trajectory_validation_failed:") +
                   validation.reason);

    std::cout << "[E2E] trajectory tid="
              << message.trajectory.trajectory_id
              << " slot=" << message.trajectory.target_slot
              << " points=" << message.trajectory.points.size()
              << " PiCheck=PASS\n";

    return message.trajectory;
}

void runScenario(int port)
{
    Config config = makeConfig(port);
    ParkingServerClient client(config);
    CHECK_TRUE(client.init(), "client_init_failed");
    CHECK_TRUE(waitConnected(client), "client_connect_timeout");
    std::cout << "[E2E] connected\n";

    std::uint64_t input_sequence = 1;
    VehiclePoseMsg pose = makePose(input_sequence);
    ParkingStatusMsg parking = makeParkingStatus(input_sequence);

    CHECK_TRUE(client.sendVehiclePose(pose), "initial_pose_send_failed");
    CHECK_TRUE(client.sendParkingStatus(parking), "initial_parking_send_failed");

    ParkingTrajectoryMsg trajectory = waitForValidatedTrajectory(
        client, config, pose);
    CHECK_TRUE(client.sendTrajectoryStatus(
                   1, trajectory.trajectory_id, "RECEIVED", "CI_E2E_ACCEPT"),
               "received_status_send_failed");
    assertSessionState(client, "TRAJECTORY_READY", trajectory.trajectory_id);
    std::cout << "[PASS] initial_handshake_and_pi_validation\n";

    CHECK_TRUE(client.sendSafetyEvent(
                   trajectory.trajectory_id, true, "CRITICAL_OBSTACLE"),
               "critical_obstacle_send_failed");
    assertSessionState(client, "PAUSED", trajectory.trajectory_id);
    std::cout << "[PASS] critical_obstacle_pauses_session\n";

    ++input_sequence;
    refreshInputs(client, input_sequence);
    pose = makePose(input_sequence);
    CHECK_TRUE(client.sendSafetyEvent(
                   trajectory.trajectory_id, true, "SAFETY_CLEARED"),
               "critical_clear_send_failed");

    ParkingTrajectoryMsg after_obstacle = waitForValidatedTrajectory(
        client, config, pose, trajectory.trajectory_id);
    CHECK_TRUE(after_obstacle.trajectory_id != trajectory.trajectory_id,
               "critical_clear_did_not_replan");
    CHECK_TRUE(client.sendTrajectoryStatus(
                   2, after_obstacle.trajectory_id, "RECEIVED", "CI_E2E_ACCEPT"),
               "second_received_status_send_failed");
    std::cout << "[PASS] safety_clear_replans_after_obstacle\n";

    CHECK_TRUE(client.sendSafetyEvent(
                   after_obstacle.trajectory_id, true, "SERVER_TIMEOUT"),
               "server_timeout_event_send_failed");
    assertSessionState(client, "PAUSED", after_obstacle.trajectory_id);
    std::cout << "[PASS] server_timeout_pauses_session\n";

    ++input_sequence;
    refreshInputs(client, input_sequence);
    pose = makePose(input_sequence);
    CHECK_TRUE(client.sendSafetyEvent(
                   after_obstacle.trajectory_id, true, "SAFETY_CLEARED"),
               "timeout_clear_send_failed");

    ParkingTrajectoryMsg after_timeout = waitForValidatedTrajectory(
        client, config, pose, after_obstacle.trajectory_id);
    CHECK_TRUE(after_timeout.trajectory_id != after_obstacle.trajectory_id,
               "timeout_clear_did_not_replan");
    CHECK_TRUE(client.sendTrajectoryStatus(
                   3, after_timeout.trajectory_id, "RECEIVED", "CI_E2E_ACCEPT"),
               "third_received_status_send_failed");
    std::cout << "[PASS] safety_clear_replans_after_timeout\n";

    ++input_sequence;
    refreshInputs(client, input_sequence);
    pose = makePose(input_sequence);
    CHECK_TRUE(client.sendSafetyEvent(
                   after_timeout.trajectory_id, true, "TRAJECTORY_INVALID"),
               "trajectory_invalid_send_failed");

    ParkingTrajectoryMsg after_invalid = waitForValidatedTrajectory(
        client, config, pose, after_timeout.trajectory_id);
    CHECK_TRUE(after_invalid.trajectory_id != after_timeout.trajectory_id,
               "trajectory_invalid_did_not_replan");
    std::cout << "[PASS] trajectory_invalid_replans_without_clear\n";

    client.close();
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: test_parking_end_to_end <port>\n";
        return 2;
    }

    try {
        const int port = std::stoi(argv[1]);
        runScenario(port);
        std::cout << "[SUMMARY] C++ client <-> Python Server E2E passed\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "[FAIL] E2E: " << exc.what() << "\n";
        return 1;
    }
}
