#include "UartVehicleInterface.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

#ifdef LAAS_USE_LIBSERIAL
#include <libserial/SerialPort.h>
#endif

namespace laas {

struct UartVehicleInterface::Impl {
    explicit Impl(const Config& cfg)
        : config(cfg)
    {
    }

    Config config;
    bool opened = false;

#ifdef LAAS_USE_LIBSERIAL
    LibSerial::SerialPort serial;
#endif
};

UartVehicleInterface::UartVehicleInterface(const Config& config)
    : impl_(std::make_unique<Impl>(config))
{
}

UartVehicleInterface::~UartVehicleInterface()
{
    close();
}

bool UartVehicleInterface::init()
{
    if (!impl_->config.runtime.enable_uart) {
        std::cout << "[UART] Disabled by config. Commands will be printed only.\n";
        impl_->opened = false;
        return true;
    }

#ifdef LAAS_USE_LIBSERIAL
    try {
        impl_->serial.Open(impl_->config.uart.port);

        switch (impl_->config.uart.baudrate) {
        case 9600:
            impl_->serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
            break;
        case 115200:
        default:
            impl_->serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            break;
        }

        impl_->serial.SetSerialPortBlockingStatus(false);
        impl_->opened = true;
        std::cout << "[UART] Connected on " << impl_->config.uart.port
                  << " at " << impl_->config.uart.baudrate << " baud.\n";
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[UART] Open failed: " << e.what() << "\n";
        impl_->opened = false;
        return false;
    }
#else
    impl_->opened = true;
    std::cout << "[UART] Simulation mode. Define LAAS_USE_LIBSERIAL to enable real UART.\n";
    return true;
#endif
}

bool UartVehicleInterface::send(const ControlCmdMsg& command)
{
    if (!command.header.valid) {
        return false;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "CMD," << command.speed_mps << "," << command.servo_cmd << "\r\n";

#ifdef LAAS_USE_LIBSERIAL
    if (!impl_->config.runtime.enable_uart || !impl_->opened || !impl_->serial.IsOpen()) {
        return false;
    }

    try {
        impl_->serial.Write(oss.str());
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[UART] Write error: " << e.what() << "\n";
        return false;
    }
#else
    if (!impl_->config.runtime.enable_uart) {
        return true;
    }

    std::cout << "[UART SIM] " << oss.str();
    return true;
#endif
}

void UartVehicleInterface::close()
{
#ifdef LAAS_USE_LIBSERIAL
    if (impl_ && impl_->serial.IsOpen()) {
        impl_->serial.Close();
    }
#endif
    if (impl_) {
        impl_->opened = false;
    }
}

bool UartVehicleInterface::isOpened() const
{
    return impl_ && impl_->opened;
}

}  // namespace laas
