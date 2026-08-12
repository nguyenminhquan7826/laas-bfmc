#include "UartVehicleInterface.hpp"

#include "TelemetryProtocol.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#ifdef LAAS_USE_LIBSERIAL
#include <libserial/SerialPort.h>
#endif

namespace laas {

namespace {

constexpr std::size_t kMaxRxLineLength = 256U;
constexpr std::size_t kMaxTelemetryQueueSize = 32U;

std::uint64_t monotonicTimeMs()
{
    const auto now =
        std::chrono::steady_clock::now().time_since_epoch();

    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<
            std::chrono::milliseconds>(now).count());
}

}  // namespace

struct UartVehicleInterface::Impl {
    explicit Impl(const Config& cfg)
        : config(cfg)
    {
    }

    void pushTelemetry(VehicleTelemetryMsg telemetry)
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex);

        if (telemetry_queue.size() >=
            kMaxTelemetryQueueSize) {
            telemetry_queue.pop_front();
            ++dropped_frames;
        }

        telemetry_queue.emplace_back(
            std::move(telemetry));
    }

#ifdef LAAS_USE_LIBSERIAL
    void reportParseError(
        TelemetryParseError error)
    {
        ++parse_error_count;

        // Tránh in lỗi liên tục làm chậm chương trình.
        if (parse_error_count <= 5U ||
            (parse_error_count % 100U) == 0U) {
            std::cerr
                << "[UART RX] Invalid telemetry frame: "
                << telemetryParseErrorName(error)
                << " (count=" << parse_error_count
                << ")\n";
        }
    }

    void processRxByte(char byte)
    {
        // Nếu dòng trước vượt chiều dài cho phép,
        // bỏ toàn bộ dữ liệu cho đến ký tự xuống dòng.
        if (discard_until_newline) {
            if (byte == '\n') {
                discard_until_newline = false;
                rx_line.clear();
            }
            return;
        }

        if (byte == '\n') {
            // Bỏ qua dòng trống.
            if (rx_line.empty() ||
                rx_line == "\r") {
                rx_line.clear();
                return;
            }

            // Khôi phục ký tự kết thúc dòng cho parser.
            rx_line.push_back('\n');

            VehicleTelemetryMsg telemetry{};
            TelemetryParseError error =
                TelemetryParseError::NONE;

            const bool valid =
                parseTelemetryLine(
                    rx_line,
                    monotonicTimeMs(),
                    telemetry,
                    &error);

            rx_line.clear();

            if (!valid) {
                reportParseError(error);
                return;
            }

            pushTelemetry(std::move(telemetry));
            ++valid_frame_count;

            return;
        }

        // Chừa một byte để thêm '\n' khi kết thúc dòng.
        if (rx_line.size() >=
            (kMaxRxLineLength - 1U)) {
            rx_line.clear();
            discard_until_newline = true;
            ++overflow_count;

            if (overflow_count <= 5U ||
                (overflow_count % 100U) == 0U) {
                std::cerr
                    << "[UART RX] Line too long; "
                    << "discarding until newline"
                    << " (count=" << overflow_count
                    << ")\n";
            }

            return;
        }

        rx_line.push_back(byte);
    }

    void rxLoop()
    {
        while (running.load()) {
            char byte = '\0';
            bool received_byte = false;

            try {
                {
                    std::lock_guard<std::mutex>
                        serial_lock(serial_mutex);

                    if (!running.load() ||
                        !opened.load() ||
                        !serial.IsOpen()) {
                        break;
                    }

                    if (serial.IsDataAvailable()) {
                        // Timeout ngắn để close() không bị treo.
                        serial.ReadByte(byte, 2U);
                        received_byte = true;
                    }
                }
            } catch (const LibSerial::ReadTimeout&) {
                // Timeout là trạng thái bình thường,
                // không phải lỗi kết nối.
            } catch (const std::exception& e) {
                if (running.load()) {
                    std::cerr
                        << "[UART RX] Read error: "
                        << e.what() << "\n";
                }

                opened.store(false);
                running.store(false);
                break;
            }

            if (received_byte) {
                processRxByte(byte);
            } else {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1));
            }
        }
    }
#endif

    Config config;

    std::atomic<bool> opened{false};
    std::atomic<bool> running{false};

    std::mutex telemetry_mutex;
    std::deque<VehicleTelemetryMsg>
        telemetry_queue;

    std::uint64_t dropped_frames = 0U;

#ifdef LAAS_USE_LIBSERIAL
    LibSerial::SerialPort serial;
    std::mutex serial_mutex;
    std::thread rx_thread;

    std::string rx_line;
    bool discard_until_newline = false;

    std::uint64_t valid_frame_count = 0U;
    std::uint64_t parse_error_count = 0U;
    std::uint64_t overflow_count = 0U;
#endif
};

UartVehicleInterface::UartVehicleInterface(
    const Config& config)
    : impl_(std::make_unique<Impl>(config))
{
}

UartVehicleInterface::~UartVehicleInterface()
{
    close();
}

bool UartVehicleInterface::init()
{
    if (!impl_) {
        return false;
    }

    // Cho phép init() lại an toàn.
    close();

    if (!impl_->config.runtime.enable_uart) {
        std::cout
            << "[UART] Disabled by config. "
            << "Commands will be printed only.\n";

        impl_->opened.store(false);
        return true;
    }

#ifdef LAAS_USE_LIBSERIAL
    try {
        {
            std::lock_guard<std::mutex>
                serial_lock(impl_->serial_mutex);

            impl_->serial.Open(
                impl_->config.uart.port);

            if (impl_->config.uart.baudrate ==
                9600) {
                impl_->serial.SetBaudRate(
                    LibSerial::BaudRate::BAUD_9600);
            } else if (
                impl_->config.uart.baudrate ==
                115200) {
                impl_->serial.SetBaudRate(
                    LibSerial::BaudRate::BAUD_115200);
            } else {
                std::cerr
                    << "[UART] Unsupported baudrate: "
                    << impl_->config.uart.baudrate
                    << "\n";

                impl_->serial.Close();
                return false;
            }

            // Cấu hình UART 8N1, không flow control.
            impl_->serial.SetCharacterSize(
                LibSerial::CharacterSize::CHAR_SIZE_8);

            impl_->serial.SetFlowControl(
                LibSerial::FlowControl::
                    FLOW_CONTROL_NONE);

            impl_->serial.SetParity(
                LibSerial::Parity::PARITY_NONE);

            impl_->serial.SetStopBits(
                LibSerial::StopBits::STOP_BITS_1);

            /*
             * ReadByte() sử dụng timeout 2 ms,
             * vì vậy có thể để SerialPort ở blocking mode.
             */
            impl_->serial.SetSerialPortBlockingStatus(
                true);

            // Xóa dữ liệu cũ còn trong driver UART.
            impl_->serial.FlushInputBuffer();
        }

        impl_->opened.store(true);
        impl_->running.store(true);

        impl_->rx_thread =
            std::thread(
                &UartVehicleInterface::Impl::rxLoop,
                impl_.get());

        std::cout
            << "[UART] Connected on "
            << impl_->config.uart.port
            << " at "
            << impl_->config.uart.baudrate
            << " baud; RX thread started.\n";

        return true;
    } catch (const std::exception& e) {
        std::cerr
            << "[UART] Open/init failed: "
            << e.what() << "\n";

        impl_->running.store(false);
        impl_->opened.store(false);

        try {
            std::lock_guard<std::mutex>
                serial_lock(impl_->serial_mutex);

            if (impl_->serial.IsOpen()) {
                impl_->serial.Close();
            }
        } catch (...) {
            // Không để cleanup che mất lỗi ban đầu.
        }

        return false;
    }
#else
    impl_->opened.store(true);

    std::cout
        << "[UART] Simulation mode. "
        << "Define LAAS_USE_LIBSERIAL "
        << "to enable real UART.\n";

    return true;
#endif
}

bool UartVehicleInterface::send(
    const ControlCmdMsg& command)
{
    if (!command.header.valid) {
        return false;
    }

    std::ostringstream oss;

    oss << std::fixed
        << std::setprecision(2)
        << "CMD,"
        << command.speed_mps
        << ","
        << command.servo_cmd
        << "\r\n";

#ifdef LAAS_USE_LIBSERIAL
    if (!impl_ ||
        !impl_->config.runtime.enable_uart ||
        !impl_->opened.load()) {
        return false;
    }

    try {
        std::lock_guard<std::mutex>
            serial_lock(impl_->serial_mutex);

        // Kiểm tra lại sau khi lấy mutex.
        if (!impl_->opened.load() ||
            !impl_->serial.IsOpen()) {
            return false;
        }

        impl_->serial.Write(oss.str());
        return true;
    } catch (const std::exception& e) {
        std::cerr
            << "[UART] Write error: "
            << e.what() << "\n";

        impl_->opened.store(false);
        impl_->running.store(false);

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

bool UartVehicleInterface::receiveLatest(
    VehicleTelemetryMsg& telemetry)
{
    if (!impl_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(
        impl_->telemetry_mutex);

    if (impl_->telemetry_queue.empty()) {
        return false;
    }

    // Lấy frame mới nhất và bỏ backlog.
    telemetry =
        impl_->telemetry_queue.back();

    impl_->telemetry_queue.clear();

    return true;
}

void UartVehicleInterface::close()
{
    if (!impl_) {
        return;
    }

#ifdef LAAS_USE_LIBSERIAL
    // Yêu cầu RX thread dừng trước khi đóng cổng.
    impl_->running.store(false);

    if (impl_->rx_thread.joinable()) {
        impl_->rx_thread.join();
    }

    try {
        std::lock_guard<std::mutex>
            serial_lock(impl_->serial_mutex);

        if (impl_->serial.IsOpen()) {
            impl_->serial.Close();
        }
    } catch (const std::exception& e) {
        std::cerr
            << "[UART] Close error: "
            << e.what() << "\n";
    }
#endif

    impl_->opened.store(false);

    {
        std::lock_guard<std::mutex> lock(
            impl_->telemetry_mutex);

        impl_->telemetry_queue.clear();
    }
}

bool UartVehicleInterface::isOpened() const
{
    return impl_ && impl_->opened.load();
}

}  // namespace laas