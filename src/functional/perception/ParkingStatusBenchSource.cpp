#include "ParkingStatusBenchSource.hpp"

#include <cmath>

namespace laas {

ParkingStatusBenchSource::ParkingStatusBenchSource(const Config& config)
    : config_(config)
{
}

bool ParkingStatusBenchSource::process(std::uint64_t now_ms,
                                       ParkingStatusMsg& out)
{
    out = ParkingStatusMsg{};

    if (!config_.parking.enable ||
        !config_.parking.bench_mode ||
        !config_.parking.enable_bench_parking_status) {
        return false;
    }

    if (config_.parking.bench_parking_status_period_ms <= 0 ||
        !std::isfinite(config_.parking.bench_slot_confidence) ||
        config_.parking.bench_slot_confidence < 0.0F ||
        config_.parking.bench_slot_confidence > 1.0F) {
        return false;
    }

    if (last_publish_ms_ != 0U &&
        now_ms - last_publish_ms_ < static_cast<std::uint64_t>(
            config_.parking.bench_parking_status_period_ms)) {
        return false;
    }

    last_publish_ms_ = now_ms;
    out.header.valid = true;
    out.header.timestamp_ms = now_ms;
    out.sequence = sequence_++;
    out.map_id = config_.parking.map_id;
    out.slots = {
        {"P_B1", config_.parking.bench_p_b1, config_.parking.bench_slot_confidence},
        {"P_B2", config_.parking.bench_p_b2, config_.parking.bench_slot_confidence},
        {"P_T1", config_.parking.bench_p_t1, config_.parking.bench_slot_confidence},
        {"P_T2", config_.parking.bench_p_t2, config_.parking.bench_slot_confidence},
    };
    return true;
}

}  // namespace laas
