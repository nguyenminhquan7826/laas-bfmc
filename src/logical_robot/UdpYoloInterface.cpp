#include "UdpYoloInterface.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
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

std::vector<std::string> splitCsv(const std::string& payload)
{
    std::vector<std::string> fields;
    std::size_t start = 0U;
    while (start <= payload.size()) {
        const std::size_t comma = payload.find(',', start);
        fields.push_back(payload.substr(
            start,
            comma == std::string::npos ? std::string::npos : comma - start));
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1U;
    }
    return fields;
}

template <typename Parse>
bool parseWholeField(const std::string& text, Parse parse)
{
    try {
        std::size_t consumed = 0U;
        parse(consumed);
        return consumed == text.size();
    } catch (const std::exception&) {
        return false;
    }
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
    std::deque<std::pair<std::uint32_t, std::uint64_t>> sent_frames;
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
    if (udp.enable_debug_stream &&
        !makeAddress(udp.monitor_ip, udp.debug_send_port, impl_->debug_address)) {
        std::cerr << "[UDP-YOLO] Invalid monitor address: "
                  << udp.monitor_ip << ":" << udp.debug_send_port << "\n";
        return false;
    }

    impl_->yolo_send_sock = makeSendSocket();
    if (udp.enable_debug_stream) {
        impl_->debug_send_sock = makeSendSocket();
    }
    impl_->distance_recv_sock = makeReceiveSocket(
        udp.local_ai_ip, udp.distance_recv_port, 1000);

    if (impl_->yolo_send_sock < 0 ||
        (udp.enable_debug_stream && impl_->debug_send_sock < 0) ||
        impl_->distance_recv_sock < 0) {
        std::cerr << "[UDP-YOLO] Socket init failed: " << std::strerror(errno) << "\n";
        close();
        return false;
    }

    impl_->initialized = true;
    std::cout << "[UDP-YOLO] Raw frame -> "
              << udp.local_ai_ip << ":" << udp.yolo_send_port
              << ", distance <- " << udp.local_ai_ip << ":"
              << udp.distance_recv_port;
    if (udp.enable_debug_stream) {
        std::cout << ", bird-eye -> " << udp.monitor_ip << ":"
                  << udp.debug_send_port;
    } else {
        std::cout << ", bird-eye=OFF";
    }
    std::cout << "\n";
    return true;
}

bool UdpYoloInterface::sendFrame(const FrameMsg& frame, int quality)
{
    if (!impl_->config.runtime.enable_yolo_udp ||
        !impl_->initialized ||
        !frame.header.valid) {
        return false;
    }
    const bool sent = sendJpegToSocket(
        impl_->yolo_send_sock,
        impl_->yolo_address,
        frame.frame_bgr,
        quality,
        impl_->yolo_frame_id);
    if (sent) {
        impl_->sent_frames.emplace_back(
            impl_->yolo_frame_id, frame.header.timestamp_ms);
        while (impl_->sent_frames.size() > 64U) {
            impl_->sent_frames.pop_front();
        }
    }
    return sent;
}

bool UdpYoloInterface::sendDebugFrame(const cv::Mat& frame, int quality)
{
    if (!impl_->config.runtime.enable_yolo_udp || !impl_->initialized) {
        return false;
    }
    if (!impl_->config.udp.enable_debug_stream) {
        return true;
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

bool UdpYoloInterface::receivePerception(YoloPerceptionMsg& perception)
{
    perception = YoloPerceptionMsg{};

    if (!impl_->config.runtime.enable_yolo_udp ||
        !impl_->initialized ||
        impl_->distance_recv_sock < 0) {
        return false;
    }

    char buffer[8192] = {0};
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
        if (payload.compare(0U, 5U, "PDET,") == 0) {
            const std::vector<std::string> fields = splitCsv(payload);
            constexpr std::size_t kHeaderFields = 7U;
            constexpr std::size_t kDetectionFields = 6U;
            constexpr std::size_t kMaxDetections = 32U;
            if (fields.size() < kHeaderFields || fields[1] != "1") {
                throw std::invalid_argument("invalid PDET header");
            }

            unsigned long parsed_frame_id = 0UL;
            int width = 0;
            int height = 0;
            float distance = -1.0F;
            unsigned long count = 0UL;
            if (!parseWholeField(fields[2], [&](std::size_t& n) {
                    parsed_frame_id = std::stoul(fields[2], &n);
                }) ||
                !parseWholeField(fields[3], [&](std::size_t& n) {
                    width = std::stoi(fields[3], &n);
                }) ||
                !parseWholeField(fields[4], [&](std::size_t& n) {
                    height = std::stoi(fields[4], &n);
                }) ||
                !parseWholeField(fields[5], [&](std::size_t& n) {
                    distance = std::stof(fields[5], &n);
                }) ||
                !parseWholeField(fields[6], [&](std::size_t& n) {
                    count = std::stoul(fields[6], &n);
                }) ||
                parsed_frame_id > std::numeric_limits<std::uint32_t>::max() ||
                width <= 0 || height <= 0 ||
                !std::isfinite(distance) || count > kMaxDetections ||
                fields.size() != kHeaderFields + count * kDetectionFields) {
                throw std::invalid_argument("invalid PDET values");
            }

            perception.frame_id = static_cast<std::uint32_t>(parsed_frame_id);
            perception.image_width = width;
            perception.image_height = height;
            perception.has_detection_payload = true;
            perception.detections.reserve(static_cast<std::size_t>(count));

            for (std::size_t i = 0U; i < count; ++i) {
                const std::size_t base = kHeaderFields + i * kDetectionFields;
                YoloDetection detection;
                if (!parseWholeField(fields[base], [&](std::size_t& n) {
                        detection.class_id = std::stoi(fields[base], &n);
                    }) ||
                    !parseWholeField(fields[base + 1U], [&](std::size_t& n) {
                        detection.confidence = std::stof(fields[base + 1U], &n);
                    }) ||
                    !parseWholeField(fields[base + 2U], [&](std::size_t& n) {
                        detection.x1_px = std::stof(fields[base + 2U], &n);
                    }) ||
                    !parseWholeField(fields[base + 3U], [&](std::size_t& n) {
                        detection.y1_px = std::stof(fields[base + 3U], &n);
                    }) ||
                    !parseWholeField(fields[base + 4U], [&](std::size_t& n) {
                        detection.x2_px = std::stof(fields[base + 4U], &n);
                    }) ||
                    !parseWholeField(fields[base + 5U], [&](std::size_t& n) {
                        detection.y2_px = std::stof(fields[base + 5U], &n);
                    }) ||
                    !std::isfinite(detection.confidence) ||
                    !std::isfinite(detection.x1_px) ||
                    !std::isfinite(detection.y1_px) ||
                    !std::isfinite(detection.x2_px) ||
                    !std::isfinite(detection.y2_px) ||
                    detection.confidence < 0.0F ||
                    detection.confidence > 1.0F ||
                    detection.x1_px < 0.0F || detection.y1_px < 0.0F ||
                    detection.x2_px <= detection.x1_px ||
                    detection.y2_px <= detection.y1_px ||
                    detection.x2_px > static_cast<float>(width) ||
                    detection.y2_px > static_cast<float>(height)) {
                    throw std::invalid_argument("invalid PDET detection");
                }
                detection.class_name = detection.class_id == 0
                    ? "parking_sign"
                    : detection.class_id == 1 ? "vehicle" : "unknown";
                perception.detections.push_back(detection);
            }

            const std::uint64_t received_ms = nowMs();
            std::uint64_t capture_ms = received_ms;
            for (const auto& sent : impl_->sent_frames) {
                if (sent.first == perception.frame_id) {
                    capture_ms = sent.second;
                    break;
                }
            }
            while (!impl_->sent_frames.empty() &&
                   impl_->sent_frames.front().first != perception.frame_id) {
                impl_->sent_frames.pop_front();
            }
            if (!impl_->sent_frames.empty()) {
                impl_->sent_frames.pop_front();
            }

            perception.header.timestamp_ms = capture_ms;
            perception.header.valid = true;
            perception.obstacle.header.timestamp_ms = received_ms;
            perception.obstacle.header.valid = true;
            perception.obstacle.distance_m = distance;
            perception.obstacle.has_obstacle = distance > 0.05F;
            perception.obstacle.confidence =
                perception.obstacle.has_obstacle ? 1.0F : 0.0F;
            return true;
        }

        std::size_t parsed = 0;
        const float distance = std::stof(payload, &parsed);
        if (!hasOnlyTrailingWhitespace(payload, parsed) || !std::isfinite(distance)) {
            throw std::invalid_argument("invalid distance payload");
        }

        perception.header.timestamp_ms = nowMs();
        perception.header.valid = true;
        perception.frame_id = 0U;
        perception.image_width = impl_->config.camera.width;
        perception.image_height = impl_->config.camera.height;
        perception.obstacle.header = perception.header;
        perception.obstacle.distance_m = distance;
        perception.obstacle.has_obstacle = distance > 0.05F;
        perception.obstacle.confidence =
            perception.obstacle.has_obstacle ? 1.0F : 0.0F;
        return true;
    } catch (const std::exception&) {
        std::cerr << "[UDP-YOLO] Failed to parse result packet\n";
        return false;
    }
}

bool UdpYoloInterface::receiveObstacle(ObstacleMsg& obstacle)
{
    YoloPerceptionMsg perception;
    if (!receivePerception(perception)) {
        obstacle = ObstacleMsg{};
        return false;
    }
    obstacle = perception.obstacle;
    return true;
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
    impl_->sent_frames.clear();
}

bool UdpYoloInterface::isInitialized() const
{
    return impl_ && impl_->initialized;
}

}  // namespace laas
