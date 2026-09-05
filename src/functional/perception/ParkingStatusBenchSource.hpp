#pragma once

#include <cstdint>

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

// Explicit bench/integration source. It never infers occupancy from camera data.
class ParkingStatusBenchSource {
public:
    explicit ParkingStatusBenchSource(const Config& config);

    bool process(std::uint64_t now_ms, ParkingStatusMsg& out);

private:
    const Config& config_;
    std::uint64_t sequence_{1};
    std::uint64_t last_publish_ms_{0};
};

}  // namespace laas
