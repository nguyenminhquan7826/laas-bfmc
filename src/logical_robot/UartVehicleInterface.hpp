#pragma once

#include <cstdint>
#include <memory>

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"

namespace laas {

struct UartRxStats {
    std::uint64_t parsed_frames{0U};
    double parsed_hz{0.0};

    // Chỉ được tính trên TOÀN BỘ frame TEL hợp lệ ngay tại RX thread.
    std::uint64_t transport_sequence_gaps{0U};
    std::uint64_t duplicate_frames{0U};
    std::uint64_t sequence_resets{0U};

    // Queue đầy 32 frame và phải pop_front().
    std::uint64_t queue_dropped_frames{0U};

    // Frame hợp lệ bị bỏ có chủ đích khi receiveLatest()
    // lấy frame cuối rồi clear backlog.
    std::uint64_t latest_skipped_frames{0U};
};

class UartVehicleInterface {
public:
    explicit UartVehicleInterface(const Config& config);
    ~UartVehicleInterface();

    bool init();
    bool send(const ControlCmdMsg& command);

    // Lấy frame telemetry mới nhất.
    // Các frame cũ hơn trong queue sẽ bị loại bỏ.
    bool receiveLatest(VehicleTelemetryMsg& telemetry);

    // Thống kê ở tầng RX thật, độc lập với tốc độ Executive consume.
    UartRxStats rxStats() const;

    void close();
    bool isOpened() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace laas