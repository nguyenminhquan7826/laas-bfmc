#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include "../laas_core/Config.hpp"
#include "ParkingProtocol.hpp"

namespace laas {

class ParkingServerClient {
public:
    explicit ParkingServerClient(const Config& config);
    ~ParkingServerClient();

    bool init();
    void close();
    void service();

    bool connected() const { return state_ == ConnectionState::CONNECTED; }
    std::uint64_t lastRxTimestampMs() const { return last_rx_timestamp_ms_; }

    bool sendVehiclePose(const VehiclePoseMsg& msg);
    bool sendParkingStatus(const ParkingStatusMsg& msg);
    bool sendPlanRequest(std::uint64_t sequence, bool new_session);
    bool sendSafetyEvent(std::uint64_t trajectory_id,
                         bool has_trajectory_id,
                         const std::string& event);
    bool sendTrajectoryStatus(std::uint64_t sequence,
                              std::uint64_t trajectory_id,
                              const std::string& status,
                              const std::string& reason_text = std::string());
    bool sendSessionQuery();

    bool popMessage(ParkingServerMessage& out);

private:
    enum class ConnectionState { DISCONNECTED = 0, CONNECTING, CONNECTED };

    void beginConnect(std::uint64_t now_ms);
    void checkConnecting();
    void markDisconnected();
    void receiveAvailable();
    void flushTx();
    bool queueLine(const std::string& line);
    void handleLine(const std::string& line);

    const Config& config_;
    int socket_fd_{-1};
    ConnectionState state_{ConnectionState::DISCONNECTED};
    std::uint64_t last_connect_attempt_ms_{0};
    std::uint64_t last_rx_timestamp_ms_{0};

    std::string rx_buffer_;
    std::deque<std::string> tx_queue_;
    std::size_t tx_offset_{0};
    std::deque<ParkingServerMessage> rx_messages_;
};

}  // namespace laas
