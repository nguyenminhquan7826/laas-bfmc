#include <arpa/inet.h>
#include <json-c/json.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "logical_robot/ParkingServerClient.hpp"

namespace {

using laas::Config;
using laas::ParkingServerClient;
using laas::ParkingServerMessage;
using laas::ParkingServerMessageType;
using laas::VehiclePoseMsg;

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

void sendAll(int fd, const std::string& data)
{
    std::size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t sent = ::send(fd, data.data() + offset,
                                    data.size() - offset, MSG_NOSIGNAL);
        if (sent <= 0) {
            throw std::runtime_error("mock_server_send_failed");
        }
        offset += static_cast<std::size_t>(sent);
    }
}

std::string readLine(int fd)
{
    std::string line;
    char byte = 0;
    while (true) {
        const ssize_t count = ::recv(fd, &byte, 1, 0);
        if (count == 0) {
            break;
        }
        if (count < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error("mock_server_recv_failed");
        }
        if (byte == '\n') {
            break;
        }
        line.push_back(byte);
    }
    return line;
}

int acceptClient(int listen_fd)
{
    const int fd = ::accept(listen_fd, nullptr, nullptr);
    if (fd < 0) {
        throw std::runtime_error("mock_server_accept_failed");
    }

    timeval timeout{};
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    return fd;
}

class ScriptedServer {
public:
    explicit ScriptedServer(std::function<void(int)> script)
        : script_(std::move(script))
    {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_fd_ < 0) {
            throw std::runtime_error("mock_server_socket_failed");
        }

        int reuse = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR,
                     &reuse, sizeof(reuse));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(0);

        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address),
                   sizeof(address)) != 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            throw std::runtime_error("mock_server_bind_failed");
        }

        if (::listen(listen_fd_, 4) != 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            throw std::runtime_error("mock_server_listen_failed");
        }

        socklen_t length = sizeof(address);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address),
                          &length) != 0) {
            ::close(listen_fd_);
            listen_fd_ = -1;
            throw std::runtime_error("mock_server_getsockname_failed");
        }
        port_ = ntohs(address.sin_port);

        thread_ = std::thread([this]() {
            try {
                script_(listen_fd_);
            } catch (...) {
                error_ = std::current_exception();
            }
        });
    }

    ~ScriptedServer()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
        if (listen_fd_ >= 0) {
            ::close(listen_fd_);
        }
    }

    std::uint16_t port() const { return port_; }

    void joinAndRethrow()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
        if (error_) {
            std::rethrow_exception(error_);
        }
    }

private:
    int listen_fd_{-1};
    std::uint16_t port_{0};
    std::function<void(int)> script_;
    std::thread thread_;
    std::exception_ptr error_;
};

Config makeConfig(std::uint16_t port, int max_line_bytes = 4096)
{
    Config config;
    config.parking.enable = true;
    config.parking.server_host = "127.0.0.1";
    config.parking.server_port = static_cast<int>(port);
    config.parking.reconnect_period_ms = 10;
    config.parking.max_ndjson_line_bytes = max_line_bytes;
    return config;
}

bool waitConnected(ParkingServerClient& client, int timeout_ms = 1000)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        client.service();
        if (client.connected()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

std::vector<ParkingServerMessage> collectMessages(
    ParkingServerClient& client,
    std::size_t count,
    int timeout_ms = 1500)
{
    std::vector<ParkingServerMessage> messages;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline &&
           messages.size() < count) {
        client.service();

        ParkingServerMessage message;
        while (client.popMessage(message)) {
            messages.push_back(message);
            if (messages.size() >= count) {
                break;
            }
        }

        if (messages.size() < count) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return messages;
}

void testConnectAndVehiclePoseTx()
{
    std::string received_line;
    ScriptedServer server([&received_line](int listen_fd) {
        const int fd = acceptClient(listen_fd);
        received_line = readLine(fd);
        ::close(fd);
    });

    Config config = makeConfig(server.port());
    ParkingServerClient client(config);
    CHECK_TRUE(client.init(), "client_init_failed");
    CHECK_TRUE(waitConnected(client), "client_did_not_connect");

    VehiclePoseMsg pose;
    pose.header.valid = true;
    pose.header.timestamp_ms = 1234;
    pose.sequence = 42;
    pose.map_id = "map_v1";
    pose.source = "CPP_TEST";
    pose.x_m = 1.3;
    pose.y_m = 0.751;
    pose.yaw_rad = 0.0;

    CHECK_TRUE(client.sendVehiclePose(pose), "vehicle_pose_send_failed");
    server.joinAndRethrow();

    json_object* root = json_tokener_parse(received_line.c_str());
    CHECK_TRUE(root != nullptr, "vehicle_pose_tx_not_json");

    json_object* type = nullptr;
    json_object* seq = nullptr;
    json_object* map_id = nullptr;
    json_object* pose_obj = nullptr;
    json_object* x_m = nullptr;

    CHECK_TRUE(json_object_object_get_ex(root, "type", &type) != 0,
               "vehicle_pose_tx_missing_type");
    CHECK_TRUE(std::string(json_object_get_string(type)) == "vehicle_pose",
               "vehicle_pose_tx_wrong_type");
    CHECK_TRUE(json_object_object_get_ex(root, "seq", &seq) != 0 &&
                   json_object_get_int64(seq) == 42,
               "vehicle_pose_tx_wrong_seq");
    CHECK_TRUE(json_object_object_get_ex(root, "map_id", &map_id) != 0 &&
                   std::string(json_object_get_string(map_id)) == "map_v1",
               "vehicle_pose_tx_wrong_map");
    CHECK_TRUE(json_object_object_get_ex(root, "pose", &pose_obj) != 0 &&
                   json_object_object_get_ex(pose_obj, "x_m", &x_m) != 0,
               "vehicle_pose_tx_missing_pose");
    CHECK_TRUE(std::abs(json_object_get_double(x_m) - 1.3) < 1e-9,
               "vehicle_pose_tx_wrong_x");

    json_object_put(root);
    client.close();
}

void testFragmentedAndMultipleNdjson()
{
    ScriptedServer server([](int listen_fd) {
        const int fd = acceptClient(listen_fd);
        sendAll(fd,
                "{\"type\":\"ack\",\"version\":1,\"accepted\":true,\"seq\":11");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        sendAll(fd,
                "}\n{\"type\":\"ack\",\"version\":1,\"accepted\":false,\"seq\":12}\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ::close(fd);
    });

    Config config = makeConfig(server.port());
    ParkingServerClient client(config);
    client.init();
    CHECK_TRUE(waitConnected(client), "fragment_test_connect_failed");

    const auto messages = collectMessages(client, 2);
    server.joinAndRethrow();

    CHECK_TRUE(messages.size() == 2, "fragment_test_message_count");
    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ACK,
               "fragment_test_first_not_ack");
    CHECK_TRUE(messages[0].has_sequence && messages[0].sequence == 11,
               "fragment_test_first_seq");
    CHECK_TRUE(messages[0].accepted, "fragment_test_first_accept");
    CHECK_TRUE(messages[1].type == ParkingServerMessageType::ACK,
               "fragment_test_second_not_ack");
    CHECK_TRUE(messages[1].has_sequence && messages[1].sequence == 12,
               "fragment_test_second_seq");
    CHECK_TRUE(!messages[1].accepted, "fragment_test_second_accept");
    client.close();
}

void testInvalidJsonDoesNotLoseFollowingMessage()
{
    ScriptedServer server([](int listen_fd) {
        const int fd = acceptClient(listen_fd);
        sendAll(fd,
                "{bad-json}\n"
                "{\"type\":\"ack\",\"version\":1,\"accepted\":true,\"seq\":21}\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ::close(fd);
    });

    Config config = makeConfig(server.port());
    ParkingServerClient client(config);
    client.init();
    CHECK_TRUE(waitConnected(client), "invalid_json_connect_failed");

    const auto messages = collectMessages(client, 2);
    server.joinAndRethrow();

    CHECK_TRUE(messages.size() == 2, "invalid_json_message_count");
    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ERROR,
               "invalid_json_first_not_error");
    CHECK_TRUE(messages[0].reason.find("protocol_decode:invalid_json") == 0,
               "invalid_json_reason");
    CHECK_TRUE(messages[1].type == ParkingServerMessageType::ACK,
               "invalid_json_following_ack_missing");
    CHECK_TRUE(messages[1].sequence == 21,
               "invalid_json_following_ack_seq");
    client.close();
}

void testOversizedNdjsonLineProducesError()
{
    ScriptedServer server([](int listen_fd) {
        const int fd = acceptClient(listen_fd);
        sendAll(fd, std::string(65, 'x') + "\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ::close(fd);
    });

    Config config = makeConfig(server.port(), 64);
    ParkingServerClient client(config);
    client.init();
    CHECK_TRUE(waitConnected(client), "oversize_connect_failed");

    const auto messages = collectMessages(client, 1);
    server.joinAndRethrow();

    CHECK_TRUE(messages.size() == 1, "oversize_message_count");
    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ERROR,
               "oversize_not_error");
    CHECK_TRUE(messages[0].reason == "ndjson_line_too_large",
               "oversize_wrong_reason");
    client.close();
}

void testReconnectAfterPeerClose()
{
    ScriptedServer server([](int listen_fd) {
        const int first = acceptClient(listen_fd);
        ::close(first);

        const int second = acceptClient(listen_fd);
        sendAll(second,
                "{\"type\":\"ack\",\"version\":1,\"accepted\":true,\"seq\":77}\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ::close(second);
    });

    Config config = makeConfig(server.port());
    ParkingServerClient client(config);
    client.init();

    const auto messages = collectMessages(client, 1, 2000);
    server.joinAndRethrow();

    CHECK_TRUE(messages.size() == 1, "reconnect_no_message");
    CHECK_TRUE(messages[0].type == ParkingServerMessageType::ACK,
               "reconnect_message_not_ack");
    CHECK_TRUE(messages[0].has_sequence && messages[0].sequence == 77,
               "reconnect_wrong_seq");
    client.close();
}

struct TestCase {
    const char* name;
    void (*fn)();
};

}  // namespace

int main()
{
    const TestCase tests[] = {
        {"connect_and_vehicle_pose_tx", testConnectAndVehiclePoseTx},
        {"fragmented_and_multiple_ndjson", testFragmentedAndMultipleNdjson},
        {"invalid_json_followed_by_valid_message", testInvalidJsonDoesNotLoseFollowingMessage},
        {"oversized_ndjson_line", testOversizedNdjsonLineProducesError},
        {"reconnect_after_peer_close", testReconnectAfterPeerClose},
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

    std::cout << "[SUMMARY] all ParkingServerClient tests passed\n";
    return 0;
}
