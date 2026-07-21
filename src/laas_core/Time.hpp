#pragma once

#include <chrono>
#include <cstdint>

namespace laas {

inline std::uint64_t nowMs()
{
    using namespace std::chrono;
    return static_cast<std::uint64_t>(
        duration_cast<milliseconds>(
            steady_clock::now().time_since_epoch()).count());
}

inline bool isFresh(std::uint64_t now_ms,
                    std::uint64_t timestamp_ms,
                    int timeout_ms)
{
    if (timestamp_ms == 0 || timeout_ms < 0) {
        return false;
    }
    return now_ms >= timestamp_ms &&
           now_ms - timestamp_ms <= static_cast<std::uint64_t>(timeout_ms);
}

}  // namespace laas
