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
        "6a61bff5fcd280bf3f630d6644ee65b0d07842e75bc6a03f6fe8f2db519cb9ed";
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

    std::cout << "[PASS] navigation decision protocol\n";
    return 0;
}
