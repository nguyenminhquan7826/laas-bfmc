#include "ParkingServerClient.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../laas_core/Time.hpp"

namespace laas {

namespace {
constexpr std::size_t kMaxQueuedMessages = 32U;
constexpr std::size_t kMaxTxMessages = 16U;
}

ParkingServerClient::ParkingServerClient(const Config& config)
    : config_(config)
{
}

ParkingServerClient::~ParkingServerClient()
{
    close();
}

bool ParkingServerClient::init()
{
    close();
    last_connect_attempt_ms_ = 0U;
    last_rx_timestamp_ms_ = 0U;
    return true;
}

void ParkingServerClient::close()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    state_ = ConnectionState::DISCONNECTED;
    rx_buffer_.clear();
    rx_messages_.clear();
    tx_queue_.clear();
    tx_offset_ = 0U;
}

void ParkingServerClient::markDisconnected()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    state_ = ConnectionState::DISCONNECTED;
    rx_buffer_.clear();
    // Decoded messages are scoped to the TCP session that produced them.
    // Never let an ACK/session snapshot from a dead connection survive into
    // the next connection and overwrite Executive's fail-closed HOLD state.
    rx_messages_.clear();
    tx_queue_.clear();
    tx_offset_ = 0U;
}

void ParkingServerClient::beginConnect(std::uint64_t now_ms)
{
    last_connect_attempt_ms_ = now_ms;

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* result = nullptr;
    const std::string port = std::to_string(config_.parking.server_port);
    const int gai = getaddrinfo(config_.parking.server_host.c_str(), port.c_str(),
                                &hints, &result);
    if (gai != 0 || result == nullptr) {
        return;
    }

    for (addrinfo* ai = result; ai != nullptr; ai = ai->ai_next) {
        const int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0) {
            continue;
        }

        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            ::close(fd);
            continue;
        }

        const int rc = ::connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc == 0) {
            socket_fd_ = fd;
            state_ = ConnectionState::CONNECTED;
            break;
        }
        if (errno == EINPROGRESS) {
            socket_fd_ = fd;
            state_ = ConnectionState::CONNECTING;
            break;
        }
        ::close(fd);
    }

    freeaddrinfo(result);
}

void ParkingServerClient::checkConnecting()
{
    if (state_ != ConnectionState::CONNECTING || socket_fd_ < 0) {
        return;
    }

    pollfd pfd{};
    pfd.fd = socket_fd_;
    pfd.events = POLLOUT;
    const int rc = poll(&pfd, 1, 0);
    if (rc <= 0) {
        return;
    }

    int error = 0;
    socklen_t len = sizeof(error);
    if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len) != 0 ||
        error != 0) {
        markDisconnected();
        return;
    }
    state_ = ConnectionState::CONNECTED;
}

void ParkingServerClient::service()
{
    if (!config_.parking.enable) {
        return;
    }

    const std::uint64_t now = nowMs();
    if (state_ == ConnectionState::DISCONNECTED) {
        if (last_connect_attempt_ms_ == 0U ||
            now - last_connect_attempt_ms_ >=
                static_cast<std::uint64_t>(config_.parking.reconnect_period_ms)) {
            beginConnect(now);
        }
    }

    checkConnecting();
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    flushTx();
    receiveAvailable();
}

bool ParkingServerClient::queueLine(const std::string& line)
{
    if (state_ != ConnectionState::CONNECTED || line.empty() ||
        line.size() > static_cast<std::size_t>(config_.parking.max_ndjson_line_bytes) ||
        tx_queue_.size() >= kMaxTxMessages) {
        return false;
    }
    tx_queue_.push_back(line + "\n");
    flushTx();
    return true;
}

void ParkingServerClient::flushTx()
{
    while (state_ == ConnectionState::CONNECTED && !tx_queue_.empty()) {
        const std::string& current = tx_queue_.front();
        const char* data = current.data() + tx_offset_;
        const std::size_t remaining = current.size() - tx_offset_;
        const ssize_t sent = ::send(socket_fd_, data, remaining, MSG_NOSIGNAL);
        if (sent > 0) {
            tx_offset_ += static_cast<std::size_t>(sent);
            if (tx_offset_ == current.size()) {
                tx_queue_.pop_front();
                tx_offset_ = 0U;
            }
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return;
        }
        markDisconnected();
        return;
    }
}

void ParkingServerClient::receiveAvailable()
{
    char buffer[4096];
    while (state_ == ConnectionState::CONNECTED) {
        const ssize_t count = ::recv(socket_fd_, buffer, sizeof(buffer), 0);
        if (count > 0) {
            last_rx_timestamp_ms_ = nowMs();
            rx_buffer_.append(buffer, static_cast<std::size_t>(count));

            const std::size_t hard_limit =
                static_cast<std::size_t>(config_.parking.max_ndjson_line_bytes) * 2U;
            if (rx_buffer_.size() > hard_limit) {
                ParkingServerMessage error;
                error.type = ParkingServerMessageType::ERROR;
                error.header.valid = true;
                error.header.timestamp_ms = nowMs();
                error.reason = "rx_buffer_overflow";
                if (rx_messages_.size() >= kMaxQueuedMessages) {
                    rx_messages_.pop_front();
                }
                rx_messages_.push_back(error);
                markDisconnected();
                return;
            }

            std::size_t newline = std::string::npos;
            while ((newline = rx_buffer_.find('\n')) != std::string::npos) {
                std::string line = rx_buffer_.substr(0, newline);
                rx_buffer_.erase(0, newline + 1U);
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                if (line.size() > static_cast<std::size_t>(
                        config_.parking.max_ndjson_line_bytes)) {
                    ParkingServerMessage error;
                    error.type = ParkingServerMessageType::ERROR;
                    error.header.valid = true;
                    error.header.timestamp_ms = nowMs();
                    error.reason = "ndjson_line_too_large";
                    if (rx_messages_.size() >= kMaxQueuedMessages) {
                        rx_messages_.pop_front();
                    }
                    rx_messages_.push_back(error);
                    continue;
                }
                if (!line.empty()) {
                    handleLine(line);
                }
            }
            continue;
        }
        if (count == 0) {
            markDisconnected();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }
        if (errno == EINTR) {
            continue;
        }
        markDisconnected();
        return;
    }
}

void ParkingServerClient::handleLine(const std::string& line)
{
    ParkingServerMessage message;
    std::string reason;
    if (!ParkingProtocol::decodeServerLine(line, config_.parking.map_id,
                                           message, reason)) {
        message = ParkingServerMessage{};
        message.type = ParkingServerMessageType::ERROR;
        message.header.valid = true;
        message.header.timestamp_ms = nowMs();
        message.reason = "protocol_decode:" + reason;
    }
    if (rx_messages_.size() >= kMaxQueuedMessages) {
        rx_messages_.pop_front();
    }
    rx_messages_.push_back(std::move(message));
}

bool ParkingServerClient::sendVehiclePose(const VehiclePoseMsg& msg)
{
    std::string line, reason;
    return ParkingProtocol::encodeVehiclePose(msg, line, reason) && queueLine(line);
}

bool ParkingServerClient::sendParkingStatus(const ParkingStatusMsg& msg)
{
    std::string line, reason;
    return ParkingProtocol::encodeParkingStatus(msg, line, reason) && queueLine(line);
}

bool ParkingServerClient::sendPlanRequest(std::uint64_t sequence, bool new_session)
{
    std::string line, reason;
    return ParkingProtocol::encodePlanRequest(sequence, nowMs(),
                                              config_.parking.map_id, new_session,
                                              line, reason) && queueLine(line);
}

bool ParkingServerClient::sendSafetyEvent(std::uint64_t trajectory_id,
                                          bool has_trajectory_id,
                                          const std::string& event)
{
    std::string line, reason;
    return ParkingProtocol::encodeSafetyEvent(nowMs(), trajectory_id,
                                              has_trajectory_id, event,
                                              line, reason) && queueLine(line);
}

bool ParkingServerClient::sendTrajectoryStatus(std::uint64_t sequence,
                                               std::uint64_t trajectory_id,
                                               const std::string& status,
                                               const std::string& reason_text)
{
    std::string line, reason;
    return ParkingProtocol::encodeTrajectoryStatus(sequence, nowMs(), trajectory_id,
                                                   status, reason_text,
                                                   line, reason) && queueLine(line);
}

bool ParkingServerClient::sendSessionQuery()
{
    std::string line, reason;
    return ParkingProtocol::encodeSessionQuery(config_.parking.map_id,
                                               line, reason) && queueLine(line);
}

bool ParkingServerClient::popMessage(ParkingServerMessage& out)
{
    if (rx_messages_.empty()) {
        return false;
    }
    out = std::move(rx_messages_.front());
    rx_messages_.pop_front();
    return true;
}

}  // namespace laas
