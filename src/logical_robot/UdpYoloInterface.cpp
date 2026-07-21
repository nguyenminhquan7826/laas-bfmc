#include "UdpYoloInterface.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../laas_core/Time.hpp"

namespace laas {

namespace {

constexpr std::array<std::uint8_t, 4> kJpegMagic{{'L', 'J', 'P', 'G'}};
constexpr std::size_t kJpegHeaderSize = 16;
constexpr std::size_t kMaxDatagramSize = 1400;
constexpr std::size_t kMaxChunkPayload = kMaxDatagramSize - kJpegHeaderSize;
constexpr std::size_t kMaxJpegSize = 4U * 1024U * 1024U;

bool makeAddress(const std::string& ip, int port, sockaddr_in& out)
{
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port = htons(static_cast<std::uint16_t>(port));
    return port > 0 && port <= 65535 &&
           inet_pton(AF_INET, ip.c_str(), &out.sin_addr) == 1;
}

int makeSendSocket()
{
    const int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return -1;
    }

    const int buffer_size = 4 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
    return sock;
}

int makeReceiveSocket(const std::string& bind_ip, int port, int timeout_us)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return -1;
    }

    sockaddr_in address{};
    if (!makeAddress(bind_ip, port, address)) {
        ::close(sock);
        errno = EINVAL;
        return -1;
    }

    const int buffer_size = 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));

    if (bind(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(sock);
        return -1;
    }

    timeval timeout{};
    timeout.tv_sec = timeout_us / 1000000;
    timeout.tv_usec = timeout_us % 1000000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return sock;
}

void writeU16(std::uint8_t* destination, std::uint16_t value)
{
    const std::uint16_t network_value = htons(value);
    std::memcpy(destination, &network_value, sizeof(network_value));
}

void writeU32(std::uint8_t* destination, std::uint32_t value)
{
    const std::uint32_t network_value = htonl(value);
    std::memcpy(destination, &network_value, sizeof(network_value));
}

bool sendJpegToSocket(int sock,
                      const sockaddr_in& address,
                      const cv::Mat& frame,
                      int quality,
                      std::uint32_t& frame_id)
{
    if (sock < 0 || frame.empty()) {
        return false;
    }

    quality = std::min(std::max(quality, 1), 100);
    std::vector<uchar> jpeg;
    const std::vector<int> parameters = {cv::IMWRITE_JPEG_QUALITY, quality};
    if (!cv::imencode(".jpg", frame, jpeg, parameters) || jpeg.empty()) {
        return false;
    }
    if (jpeg.size() > kMaxJpegSize) {
        std::cerr << "[UDP-YOLO] JPEG is too large: " << jpeg.size() << " bytes\n";
        return false;
    }

    const std::size_t chunk_count_size =
        (jpeg.size() + kMaxChunkPayload - 1U) / kMaxChunkPayload;
    if (chunk_count_size == 0U ||
        chunk_count_size > std::numeric_limits<std::uint16_t>::max()) {
        return false;
    }

    ++frame_id;
    const auto chunk_count = static_cast<std::uint16_t>(chunk_count_size);
    const auto total_size = static_cast<std::uint32_t>(jpeg.size());

    for (std::uint16_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        const std::size_t offset =
            static_cast<std::size_t>(chunk_index) * kMaxChunkPayload;
        const std::size_t payload_size =
            std::min(kMaxChunkPayload, jpeg.size() - offset);

        std::vector<std::uint8_t> packet(kJpegHeaderSize + payload_size);
        std::copy(kJpegMagic.begin(), kJpegMagic.end(), packet.begin());
        writeU32(packet.data() + 4, frame_id);
        writeU16(packet.data() + 8, chunk_index);
        writeU16(packet.data() + 10, chunk_count);
        writeU32(packet.data() + 12, total_size);
        std::memcpy(packet.data() + kJpegHeaderSize,
                    jpeg.data() + offset,
                    payload_size);

        const ssize_t sent = sendto(
            sock,
            packet.data(),
            packet.size(),
            0,
            reinterpret_cast<const sockaddr*>(&address),
            sizeof(address));
        if (sent != static_cast<ssize_t>(packet.size())) {
            return false;
        }
    }
    return true;
}

bool hasOnlyTrailingWhitespace(const std::string& text, std::size_t position)
{
    return text.find_first_not_of(" \t\r\n", position) == std::string::npos;
}

}  // namespace

struct UdpYoloInterface::Impl {
    explicit Impl(const Config& cfg)
        : config(cfg)
    {
    }

    Config config;
    int yolo_send_sock = -1;
    int debug_send_sock = -1;
    int distance_recv_sock = -1;
    sockaddr_in yolo_address{};
    sockaddr_in debug_address{};
    std::uint32_t yolo_frame_id = 0;
    std::uint32_t debug_frame_id = 0;
    std::uint64_t last_debug_send_ms = 0;
    bool initialized = false;
};

UdpYoloInterface::UdpYoloInterface(const Config& config)
    : impl_(std::make_unique<Impl>(config))
{
}

UdpYoloInterface::~UdpYoloInterface()
{
    close();
}

bool UdpYoloInterface::init()
{
    if (impl_->initialized) {
        return true;
    }

    if (!impl_->config.runtime.enable_yolo_udp) {
        std::cout << "[UDP-YOLO] Disabled by config.\n";
        return true;
    }

    const UdpConfig& udp = impl_->config.udp;
    if (!makeAddress(udp.local_ai_ip, udp.yolo_send_port, impl_->yolo_address)) {
        std::cerr << "[UDP-YOLO] Invalid local AI address: "
                  << udp.local_ai_ip << ":" << udp.yolo_send_port << "\n";
        return false;
    }
    if (!makeAddress(udp.monitor_ip, udp.debug_send_port, impl_->debug_address)) {
        std::cerr << "[UDP-YOLO] Invalid monitor address: "
                  << udp.monitor_ip << ":" << udp.debug_send_port << "\n";
        return false;
    }

    impl_->yolo_send_sock = makeSendSocket();
    impl_->debug_send_sock = makeSendSocket();
    impl_->distance_recv_sock = makeReceiveSocket(
        udp.local_ai_ip, udp.distance_recv_port, 1000);

    if (impl_->yolo_send_sock < 0 ||
        impl_->debug_send_sock < 0 ||
        impl_->distance_recv_sock < 0) {
        std::cerr << "[UDP-YOLO] Socket init failed: " << std::strerror(errno) << "\n";
        close();
        return false;
    }

    impl_->initialized = true;
    std::cout << "[UDP-YOLO] Raw frame -> "
              << udp.local_ai_ip << ":" << udp.yolo_send_port
              << ", distance <- " << udp.local_ai_ip << ":"
              << udp.distance_recv_port
              << ", bird-eye -> " << udp.monitor_ip << ":"
              << udp.debug_send_port << "\n";
    return true;
}

bool UdpYoloInterface::sendFrame(const FrameMsg& frame, int quality)
{
    if (!impl_->config.runtime.enable_yolo_udp ||
        !impl_->initialized ||
        !frame.header.valid) {
        return false;
    }
    return sendJpegToSocket(
        impl_->yolo_send_sock,
        impl_->yolo_address,
        frame.frame_bgr,
        quality,
        impl_->yolo_frame_id);
}

bool UdpYoloInterface::sendDebugFrame(const cv::Mat& frame, int quality)
{
    if (!impl_->config.runtime.enable_yolo_udp || !impl_->initialized) {
        return false;
    }

    const std::uint64_t now = nowMs();
    const int period_ms = std::max(1, impl_->config.udp.monitor_period_ms);
    if (impl_->last_debug_send_ms != 0 &&
        now >= impl_->last_debug_send_ms &&
        now - impl_->last_debug_send_ms < static_cast<std::uint64_t>(period_ms)) {
        return true;
    }

    const bool sent = sendJpegToSocket(
        impl_->debug_send_sock,
        impl_->debug_address,
        frame,
        quality,
        impl_->debug_frame_id);
    if (sent) {
        impl_->last_debug_send_ms = now;
    }
    return sent;
}

bool UdpYoloInterface::receiveObstacle(ObstacleMsg& obstacle)
{
    obstacle = ObstacleMsg{};

    if (!impl_->config.runtime.enable_yolo_udp ||
        !impl_->initialized ||
        impl_->distance_recv_sock < 0) {
        return false;
    }

    char buffer[128] = {0};
    sockaddr_in sender_address{};
    socklen_t sender_length = sizeof(sender_address);
    const int received = recvfrom(
        impl_->distance_recv_sock,
        buffer,
        sizeof(buffer) - 1,
        0,
        reinterpret_cast<sockaddr*>(&sender_address),
        &sender_length);
    if (received <= 0) {
        return false;
    }
    buffer[received] = '\0';

    try {
        const std::string payload(buffer);
        std::size_t parsed = 0;
        const float distance = std::stof(payload, &parsed);
        if (!hasOnlyTrailingWhitespace(payload, parsed) || !std::isfinite(distance)) {
            throw std::invalid_argument("invalid distance payload");
        }

        obstacle.header.timestamp_ms = nowMs();
        obstacle.header.valid = true;
        obstacle.distance_m = distance;
        obstacle.has_obstacle = distance > 0.05F;
        obstacle.confidence = obstacle.has_obstacle ? 1.0F : 0.0F;
        return true;
    } catch (const std::exception&) {
        std::cerr << "[UDP-YOLO] Failed to parse distance: '" << buffer << "'\n";
        return false;
    }
}

void UdpYoloInterface::close()
{
    if (!impl_) {
        return;
    }

    if (impl_->yolo_send_sock >= 0) {
        ::close(impl_->yolo_send_sock);
    }
    if (impl_->debug_send_sock >= 0) {
        ::close(impl_->debug_send_sock);
    }
    if (impl_->distance_recv_sock >= 0) {
        ::close(impl_->distance_recv_sock);
    }

    impl_->yolo_send_sock = -1;
    impl_->debug_send_sock = -1;
    impl_->distance_recv_sock = -1;
    impl_->initialized = false;
}

bool UdpYoloInterface::isInitialized() const
{
    return impl_ && impl_->initialized;
}

}  // namespace laas
