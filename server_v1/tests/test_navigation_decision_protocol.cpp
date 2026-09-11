#include <cstdlib>
#include <iostream>
#include <string>

#include "logical_robot/ParkingProtocol.hpp"

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "[FAIL] " << message << "\n";
        std::exit(1);
    }
}

}  // namespace

int main()
{
    const std::string hash =
        "185f84b2e16ff12177a5658480f263285a93123cec7a01a487483d39869c70a7";
    const std::string line =
        "{\"type\":\"navigation_decision\",\"version\":1,"
        "\"decision_id\":4,\"source_seq\":12,\"timestamp_ms\":1000,"
        "\"map_id\":\"map_v1\",\"map_package_sha256\":\"" + hash +
        "\",\"planning_owner\":\"client\","
        "\"maneuver\":\"PARK_AT_SLOT\",\"target_slot\":\"P_B2\","
        "\"local_planner\":\"HYBRID_A_STAR\","
        "\"trigger\":\"parking_status_initial\"}";

    laas::ParkingServerMessage decoded;
    std::string reason;
    require(
        laas::ParkingProtocol::decodeServerLine(
            line, "map_v1", decoded, reason),
        "navigation_decision must decode");
    require(
        decoded.type == laas::ParkingServerMessageType::NAVIGATION_DECISION,
        "decoded message type must be navigation decision");
    require(decoded.navigation_decision.decision_id == 4U,
            "decision id must be preserved");
    require(decoded.navigation_decision.target_slot == "P_B2",
            "target slot must be preserved");
    require(decoded.navigation_decision.map_package_sha256 == hash,
            "map hash must be preserved");

    std::string status_line;
    require(
        laas::ParkingProtocol::encodeNavigationDecisionStatus(
            9U, 1100U, "map_v1", 4U, "ACCEPTED",
            "MAP_PACKAGE_ID_MATCH_LOCAL_VERIFY_PENDING",
            status_line, reason),
        "navigation decision status must encode");
    require(status_line.find("navigation_decision_status") != std::string::npos,
            "encoded status must carry its type");
    require(status_line.find("\"decision_id\":4") != std::string::npos,
            "encoded status must carry decision id");

    laas::VehiclePoseMsg pose;
    pose.header.valid = true;
    pose.map_id = "map_v1";
    pose.x_m = 1.3;
    pose.y_m = 0.751;
    pose.yaw_rad = 0.0;
    laas::ParkingStatusMsg parking;
    parking.header.valid = true;
    parking.map_id = "map_v1";
    parking.slots = {
        {"P_B1", laas::ParkingSlotState::OCCUPIED, 1.0F},
        {"P_B2", laas::ParkingSlotState::FREE, 1.0F},
        {"P_T1", laas::ParkingSlotState::OCCUPIED, 1.0F},
        {"P_T2", laas::ParkingSlotState::OCCUPIED, 1.0F},
    };
    std::string request_line;
    require(
        laas::ParkingProtocol::encodeLocalPlanningRequest(
            decoded.navigation_decision, pose, parking, request_line, reason),
        "local planning request must encode");
    require(request_line.find("\"target_slot\":\"P_B2\"") != std::string::npos,
            "local request must carry target slot");
    require(request_line.find("\"pose\"") != std::string::npos,
            "local request must carry pose");

    laas::ParkingTrajectoryMsg trajectory;
    trajectory.header.valid = true;
    trajectory.protocol_version = laas::ParkingProtocol::kVersion;
    trajectory.trajectory_id = 4U;
    trajectory.source_seq = 12U;
    trajectory.map_id = "map_v1";
    trajectory.target_slot = "P_B2";
    trajectory.reference_point = "rear_axle_center";
    trajectory.goal_mode = "forward";
    trajectory.validation = "PASS";
    trajectory.map_package_sha256 = hash;
    trajectory.planning_owner = "client";
    trajectory.points = {
        {1.3, 0.751, 0.0, 0.1F, laas::MotionDirection::FORWARD},
        {1.4, 0.751, 0.0, 0.1F, laas::MotionDirection::FORWARD},
    };
    std::string telemetry_line;
    require(
        laas::ParkingProtocol::encodeLocalTrajectoryTelemetry(
            10U, 1200U, decoded.navigation_decision, trajectory,
            telemetry_line, reason),
        "local trajectory telemetry must encode");
    require(telemetry_line.find("\"type\":\"local_trajectory\"") !=
                std::string::npos,
            "local trajectory telemetry must carry its type");
    require(telemetry_line.find("\"trajectory_id\":4") !=
                std::string::npos,
            "local trajectory telemetry must carry decision trajectory id");
    require(telemetry_line.find("\"points\":[") != std::string::npos,
            "local trajectory telemetry must carry all path points");

    std::cout << "[PASS] navigation decision protocol\n";
    return 0;
}
