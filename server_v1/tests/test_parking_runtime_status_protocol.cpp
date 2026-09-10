#include <iostream>
#include <stdexcept>
#include <string>

#include <json-c/json.h>

#include "logical_robot/ParkingProtocol.hpp"

namespace {

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                     \
        if (!(condition)) {                                                  \
            throw std::runtime_error(message);                               \
        }                                                                    \
    } while (false)

laas::ParkingRuntimeStatusMsg makeStatus()
{
    laas::ParkingRuntimeStatusMsg status;
    status.header.valid = true;
    status.header.timestamp_ms = 1000;
    status.sequence = 7;
    status.vehicle_id = "car_01";
    status.map_id = "map_v1";
    status.operating_mode = laas::OperatingMode::PARKING;
    status.tracker_valid = true;
    status.trajectory_id = 42;
    status.nearest_index = 8;
    status.target_index = 11;
    status.cross_track_error_m = 0.025;
    status.safety_evaluated = true;
    status.safety_motion_allowed = false;
    status.safety_reason = "PASS_BENCH_ONLY";
    status.uart_rx_enabled = true;
    status.uart_tx_enabled = false;
    status.telemetry_valid = true;
    status.telemetry_age_ms = 16;
    status.session_sync_hold = false;
    status.session_sync_reason = "SYNC_READY";
    return status;
}

}  // namespace

int main()
{
    try {
        laas::ParkingRuntimeStatusMsg status = makeStatus();
        std::string line;
        std::string reason;
        CHECK_TRUE(
            laas::ParkingProtocol::encodeRuntimeStatus(status, line, reason),
            "runtime_status_encode_failed");

        json_object* root = json_tokener_parse(line.c_str());
        CHECK_TRUE(root != nullptr, "encoded_json_invalid");

        json_object* type = nullptr;
        json_object* uart = nullptr;
        json_object* tracker = nullptr;
        CHECK_TRUE(json_object_object_get_ex(root, "type", &type), "type_missing");
        CHECK_TRUE(std::string(json_object_get_string(type)) == "runtime_status",
                   "type_mismatch");
        CHECK_TRUE(json_object_object_get_ex(root, "uart", &uart), "uart_missing");
        CHECK_TRUE(json_object_object_get_ex(root, "tracker", &tracker),
                   "tracker_missing");

        json_object* rx = nullptr;
        json_object* tx = nullptr;
        json_object* cte = nullptr;
        CHECK_TRUE(json_object_object_get_ex(uart, "rx_enabled", &rx),
                   "rx_enabled_missing");
        CHECK_TRUE(json_object_object_get_ex(uart, "tx_enabled", &tx),
                   "tx_enabled_missing");
        CHECK_TRUE(json_object_get_boolean(rx) != 0, "uart_rx_not_enabled");
        CHECK_TRUE(json_object_get_boolean(tx) == 0, "uart_tx_not_disabled");
        CHECK_TRUE(json_object_object_get_ex(tracker, "cross_track_error_m", &cte),
                   "cte_missing");
        CHECK_TRUE(json_object_get_double(cte) == 0.025, "cte_mismatch");
        json_object_put(root);

        status.trajectory_id = 0;
        CHECK_TRUE(
            !laas::ParkingProtocol::encodeRuntimeStatus(status, line, reason),
            "invalid_tracker_without_trajectory_accepted");

        std::cout << "[PASS] runtime monitoring protocol is typed and read-only\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "[FAIL] " << exc.what() << "\n";
        return 1;
    }
}
