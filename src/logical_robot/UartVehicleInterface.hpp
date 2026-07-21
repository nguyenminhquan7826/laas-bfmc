#pragma once

#include <memory>
#include <string>

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"

namespace laas {

class UartVehicleInterface {
public:
    explicit UartVehicleInterface(const Config& config);
    ~UartVehicleInterface();

    bool init();
    bool send(const ControlCmdMsg& command);
    void close();
    bool isOpened() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace laas
