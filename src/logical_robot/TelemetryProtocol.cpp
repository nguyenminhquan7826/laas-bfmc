#include "TelemetryProtocol.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace laas {
namespace {

bool fail(
    TelemetryParseError value,
    TelemetryParseError* error)
{
    if (error != nullptr) {
        *error = value;
    }
    return false;
}

bool isDecimalDigits(
    const std::string& text,
    bool allow_sign)
{
    if (text.empty()) {
        return false;
    }

    std::size_t index = 0U;

    if (allow_sign &&
        (text[0] == '-' || text[0] == '+')) {
        index = 1U;
    }

    if (index == text.size()) {
        return false;
    }

    for (; index < text.size(); ++index) {
        if (text[index] < '0' ||
            text[index] > '9') {
            return false;
        }
    }

    return true;
}

bool isHexDigits(const std::string& text)
{
    if (text.empty()) {
        return false;
    }

    for (char value : text) {
        const bool digit =
            value >= '0' && value <= '9';

        const bool lower =
            value >= 'a' && value <= 'f';

        const bool upper =
            value >= 'A' && value <= 'F';

        if (!digit && !lower && !upper) {
            return false;
        }
    }

    return true;
}

bool parseUnsigned(
    const std::string& text,
    int base,
    std::uint64_t maximum,
    std::uint64_t& output)
{
    if ((base == 10 &&
         !isDecimalDigits(text, false)) ||
        (base == 16 &&
         !isHexDigits(text))) {
        return false;
    }

    errno = 0;
    char* end = nullptr;

    const unsigned long long value =
        std::strtoull(text.c_str(), &end, base);

    if (errno == ERANGE ||
        end == text.c_str() ||
        *end != '\0' ||
        value > maximum) {
        return false;
    }

    output = static_cast<std::uint64_t>(value);
    return true;
}

bool parseSigned(
    const std::string& text,
    std::int64_t minimum,
    std::int64_t maximum,
    std::int64_t& output)
{
    if (!isDecimalDigits(text, true)) {
        return false;
    }

    errno = 0;
    char* end = nullptr;

    const long long value =
        std::strtoll(text.c_str(), &end, 10);

    if (errno == ERANGE ||
        end == text.c_str() ||
        *end != '\0' ||
        value < minimum ||
        value > maximum) {
        return false;
    }

    output = static_cast<std::int64_t>(value);
    return true;
}

bool parseFloat(
    const std::string& text,
    float& output)
{
    if (text.empty()) {
        return false;
    }

    errno = 0;
    char* end = nullptr;

    const double value =
        std::strtod(text.c_str(), &end);

    const double max_float =
        static_cast<double>(
            std::numeric_limits<float>::max());

    if (errno == ERANGE ||
        end == text.c_str() ||
        *end != '\0' ||
        !std::isfinite(value) ||
        value < -max_float ||
        value > max_float) {
        return false;
    }

    output = static_cast<float>(value);
    return true;
}

std::vector<std::string> splitFields(
    const std::string& line)
{
    std::vector<std::string> fields;
    std::size_t start = 0U;

    for (;;) {
        const std::size_t delimiter =
            line.find(',', start);

        if (delimiter == std::string::npos) {
            fields.push_back(line.substr(start));
            break;
        }

        fields.push_back(
            line.substr(start, delimiter - start));

        start = delimiter + 1U;
    }

    return fields;
}

}  // namespace

std::uint16_t crc16CcittFalse(
    const std::uint8_t* data,
    std::size_t length)
{
    std::uint16_t crc = 0xFFFFU;

    if (data == nullptr) {
        return crc;
    }

    for (std::size_t index = 0U;
         index < length;
         ++index) {
        crc ^= static_cast<std::uint16_t>(
                   data[index])
               << 8U;

        for (std::uint8_t bit = 0U;
             bit < 8U;
             ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>(
                    (crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(
                    crc << 1U);
            }
        }
    }

    return crc;
}

bool parseTelemetryLine(
    const std::string& line,
    std::uint64_t receive_timestamp_ms,
    VehicleTelemetryMsg& output,
    TelemetryParseError* error)
{
    if (error != nullptr) {
        *error = TelemetryParseError::NONE;
    }

    std::string normalized = line;

    while (!normalized.empty() &&
           (normalized.back() == '\r' ||
            normalized.back() == '\n')) {
        normalized.pop_back();
    }

    if (normalized.empty()) {
        return fail(
            TelemetryParseError::EMPTY_LINE,
            error);
    }

    if (normalized.size() >
        kTelemetryMaxLineLength) {
        return fail(
            TelemetryParseError::LINE_TOO_LONG,
            error);
    }

    const std::vector<std::string> fields =
        splitFields(normalized);

    if (fields.size() != kTelemetryFieldCount) {
        return fail(
            TelemetryParseError::WRONG_FIELD_COUNT,
            error);
    }

    if (fields[0] != "TEL") {
        return fail(
            TelemetryParseError::WRONG_PREFIX,
            error);
    }

    const std::size_t crc_delimiter =
        normalized.rfind(',');

    if (crc_delimiter == std::string::npos ||
        fields[19].size() != 4U) {
        return fail(
            TelemetryParseError::INVALID_CRC_TEXT,
            error);
    }

    std::uint64_t unsigned_value = 0U;

    if (!parseUnsigned(
            fields[19],
            16,
            0xFFFFU,
            unsigned_value)) {
        return fail(
            TelemetryParseError::INVALID_CRC_TEXT,
            error);
    }

    const std::uint16_t received_crc =
        static_cast<std::uint16_t>(
            unsigned_value);

    const std::string payload =
        normalized.substr(0U, crc_delimiter);

    const std::uint16_t calculated_crc =
        crc16CcittFalse(
            reinterpret_cast<const std::uint8_t*>(
                payload.data()),
            payload.size());

    if (received_crc != calculated_crc) {
        return fail(
            TelemetryParseError::CRC_MISMATCH,
            error);
    }

    VehicleTelemetryMsg parsed;
    parsed.received_crc16 = received_crc;

    if (!parseUnsigned(
            fields[1], 10, 0xFFU,
            unsigned_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.protocol_version =
        static_cast<std::uint8_t>(
            unsigned_value);

    if (parsed.protocol_version !=
        kTelemetryProtocolVersion) {
        return fail(
            TelemetryParseError::
                UNSUPPORTED_VERSION,
            error);
    }

    auto parseU32 =
        [&unsigned_value](
            const std::string& field,
            std::uint32_t& destination) {
            if (!parseUnsigned(
                    field,
                    10,
                    0xFFFFFFFFULL,
                    unsigned_value)) {
                return false;
            }

            destination =
                static_cast<std::uint32_t>(
                    unsigned_value);

            return true;
        };

    if (!parseU32(
            fields[2],
            parsed.packet_sequence) ||
        !parseU32(
            fields[3],
            parsed.stm32_tx_timestamp_ms) ||
        !parseU32(
            fields[4],
            parsed.encoder.sequence) ||
        !parseU32(
            fields[5],
            parsed.encoder.timestamp_ms)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    std::int64_t signed_value = 0;

    if (!parseSigned(
            fields[6],
            INT32_MIN,
            INT32_MAX,
            signed_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.encoder.total_ticks =
        static_cast<std::int32_t>(signed_value);

    if (!parseSigned(
            fields[7],
            INT16_MIN,
            INT16_MAX,
            signed_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.encoder.delta_ticks =
        static_cast<std::int16_t>(signed_value);

    if (!parseFloat(
            fields[8],
            parsed.encoder.speed_mps)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    if (!parseSigned(
            fields[9],
            INT32_MIN,
            INT32_MAX,
            signed_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.steering_command_deg =
        static_cast<std::int32_t>(signed_value);

    if (!parseSigned(
            fields[10],
            INT32_MIN,
            INT32_MAX,
            signed_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.pwm_command =
        static_cast<std::int32_t>(signed_value);

    if (!parseU32(
            fields[11],
            parsed.imu.sequence) ||
        !parseU32(
            fields[12],
            parsed.imu.timestamp_ms)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    if (!parseFloat(
            fields[13],
            parsed.imu.linear_accel_x_mps2) ||
        !parseFloat(
            fields[14],
            parsed.imu.linear_accel_y_mps2) ||
        !parseFloat(
            fields[15],
            parsed.imu.gyro_z_dps) ||
        !parseFloat(
            fields[16],
            parsed.imu.yaw_deg)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    if (fields[17].size() != 2U ||
        !parseUnsigned(
            fields[17],
            16,
            0xFFU,
            unsigned_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.imu.calibration_raw =
        static_cast<std::uint8_t>(
            unsigned_value);

    if (!parseUnsigned(
            fields[18],
            10,
            0xFFU,
            unsigned_value)) {
        return fail(
            TelemetryParseError::INVALID_NUMBER,
            error);
    }

    parsed.flags =
        static_cast<std::uint8_t>(
            unsigned_value);

    parsed.encoder.valid =
        (parsed.flags &
         kTelemetryFlagEncoderValid) != 0U;

    parsed.imu.valid =
        (parsed.flags &
         kTelemetryFlagImuValid) != 0U;

    parsed.header.timestamp_ms =
        receive_timestamp_ms;

    parsed.header.valid = true;

    output = parsed;
    return true;
}

const char* telemetryParseErrorName(
    TelemetryParseError error)
{
    switch (error) {
    case TelemetryParseError::NONE:
        return "none";
    case TelemetryParseError::EMPTY_LINE:
        return "empty_line";
    case TelemetryParseError::LINE_TOO_LONG:
        return "line_too_long";
    case TelemetryParseError::WRONG_FIELD_COUNT:
        return "wrong_field_count";
    case TelemetryParseError::WRONG_PREFIX:
        return "wrong_prefix";
    case TelemetryParseError::UNSUPPORTED_VERSION:
        return "unsupported_version";
    case TelemetryParseError::INVALID_NUMBER:
        return "invalid_number";
    case TelemetryParseError::INVALID_CRC_TEXT:
        return "invalid_crc_text";
    case TelemetryParseError::CRC_MISMATCH:
        return "crc_mismatch";
    }

    return "unknown";
}

}  // namespace laas