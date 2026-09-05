#pragma once

#include <cstdint>
#include <string>

#include "../laas_core/Messages.hpp"

namespace laas {

enum class ParkingServerMessageType {
    NONE = 0,
    ACK,
    TRAJECTORY,
    PLANNING_RESULT,
    SESSION_STATUS,
    ERROR
};

struct ParkingSessionSnapshot {
    std::uint64_t session_id = 0;
    std::string state;
    std::uint64_t active_trajectory_id = 0;
    bool has_active_trajectory = false;
    std::string target_slot;
    std::uint64_t replan_count = 0;
};

struct ParkingServerMessage {
    ParkingServerMessageType type = ParkingServerMessageType::NONE;
    Header header;

    std::uint32_t protocol_version = 0;
    std::uint64_t sequence = 0;
    bool has_sequence = false;
    bool accepted = false;

    std::uint64_t source_seq = 0;
    bool has_source_seq = false;

    std::string map_id;
    std::string status;
    std::string reason;

    ParkingSessionSnapshot session;
    bool has_session = false;

    ParkingTrajectoryMsg trajectory;
};

class ParkingProtocol {
public:
    static constexpr std::uint32_t kVersion = 1;

    static bool encodeVehiclePose(const VehiclePoseMsg& msg,
                                  std::string& line,
                                  std::string& reason);
    static bool encodeParkingStatus(const ParkingStatusMsg& msg,
                                    std::string& line,
                                    std::string& reason);
    static bool encodePlanRequest(std::uint64_t sequence,
                                  std::uint64_t timestamp_ms,
                                  const std::string& map_id,
                                  bool new_session,
                                  std::string& line,
                                  std::string& reason);
    static bool encodeSafetyEvent(std::uint64_t timestamp_ms,
                                  std::uint64_t trajectory_id,
                                  bool has_trajectory_id,
                                  const std::string& event,
                                  std::string& line,
                                  std::string& reason);
    static bool encodeTrajectoryStatus(std::uint64_t sequence,
                                       std::uint64_t timestamp_ms,
                                       std::uint64_t trajectory_id,
                                       const std::string& status,
                                       const std::string& reason_text,
                                       std::string& line,
                                       std::string& reason);
    static bool encodeSessionQuery(const std::string& map_id,
                                   std::string& line,
                                   std::string& reason);

    static bool decodeServerLine(const std::string& line,
                                 const std::string& expected_map_id,
                                 ParkingServerMessage& out,
                                 std::string& reason);
};

}  // namespace laas
