#pragma once

#include <algorithm>
#include <cstdint>

namespace laas {

struct PeriodicTaskDiagnostics {
    std::uint64_t runs{0};
    std::uint64_t missed_periods{0};

    std::uint64_t last_start_ms{0};
    std::uint64_t last_interval_ms{0};
    std::uint64_t max_interval_ms{0};

    std::uint64_t last_exec_us{0};
    std::uint64_t max_exec_us{0};

    std::uint64_t last_lateness_ms{0};
    std::uint64_t max_lateness_ms{0};

    void reset()
    {
        *this = PeriodicTaskDiagnostics{};
    }

    void onStart(std::uint64_t start_ms,
                 std::uint64_t expected_start_ms,
                 int period_ms)
    {
        if (last_start_ms != 0U && start_ms >= last_start_ms) {
            last_interval_ms = start_ms - last_start_ms;
            max_interval_ms = std::max(max_interval_ms, last_interval_ms);
        }

        last_lateness_ms =
            start_ms > expected_start_ms
                ? start_ms - expected_start_ms
                : 0U;

        max_lateness_ms =
            std::max(max_lateness_ms, last_lateness_ms);

        const std::uint64_t period =
            static_cast<std::uint64_t>(std::max(1, period_ms));

        // Count only completely skipped periods. A small 1-2 ms scheduling
        // jitter is late, but it is not a missed control cycle.
        missed_periods += last_lateness_ms / period;

        last_start_ms = start_ms;
        ++runs;
    }

    void onFinish(std::uint64_t exec_us)
    {
        last_exec_us = exec_us;
        max_exec_us = std::max(max_exec_us, exec_us);
    }
};

struct SchedulerDiagnostics {
    PeriodicTaskDiagnostics keyboard;
    PeriodicTaskDiagnostics camera;
    PeriodicTaskDiagnostics yolo;
    PeriodicTaskDiagnostics perception;
    PeriodicTaskDiagnostics decision;
    PeriodicTaskDiagnostics planning;
    PeriodicTaskDiagnostics control;
    PeriodicTaskDiagnostics logging;

    void reset()
    {
        keyboard.reset();
        camera.reset();
        yolo.reset();
        perception.reset();
        decision.reset();
        planning.reset();
        control.reset();
        logging.reset();
    }
};

}  // namespace laas
