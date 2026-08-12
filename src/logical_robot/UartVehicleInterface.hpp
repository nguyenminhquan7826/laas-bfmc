#pragma once

#include <memory>

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"

namespace laas {

class UartVehicleInterface {
public:
    explicit UartVehicleInterface(const Config& config);
    ~UartVehicleInterface();

    bool init();
    bool send(const ControlCmdMsg& command);

    // Lấy frame telemetry mới nhất.
    // Các frame cũ hơn trong queue sẽ bị loại bỏ.
    bool receiveLatest(VehicleTelemetryMsg& telemetry);

    void close();
    bool isOpened() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace laas