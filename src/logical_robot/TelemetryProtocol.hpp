#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "../laas_core/Messages.hpp"

namespace laas {

constexpr std::uint8_t kTelemetryProtocolVersion = 1U;
constexpr std::uint8_t kTelemetryFlagEncoderValid = 0x01U;
constexpr std::uint8_t kTelemetryFlagImuValid = 0x02U;
constexpr std::size_t kTelemetryFieldCount = 20U;
constexpr std::size_t kTelemetryMaxLineLength = 256U;

enum class TelemetryParseError : std::uint8_t {
    NONE = 0,
    EMPTY_LINE,
    LINE_TOO_LONG,
    WRONG_FIELD_COUNT,
    WRONG_PREFIX,
    UNSUPPORTED_VERSION,
    INVALID_NUMBER,
    INVALID_CRC_TEXT,
    CRC_MISMATCH
};

std::uint16_t crc16CcittFalse(
    const std::uint8_t* data,
    std::size_t length);

bool parseTelemetryLine(
    const std::string& line,
    std::uint64_t receive_timestamp_ms,
    VehicleTelemetryMsg& output,
    TelemetryParseError* error = nullptr);

const char* telemetryParseErrorName(
    TelemetryParseError error);

}  // namespace laas