#include <cassert>
#include <cstdint>
#include <iostream>

#include "execution_control/SchedulerDiagnostics.hpp"

int main()
{
    using laas::PeriodicTaskDiagnostics;
    using laas::SchedulerDiagnostics;

    PeriodicTaskDiagnostics d;

    d.onStart(100U, 100U, 20);
    d.onFinish(1500U);
    assert(d.runs == 1U);
    assert(d.last_lateness_ms == 0U);
    assert(d.missed_periods == 0U);
    assert(d.last_exec_us == 1500U);
    assert(d.max_exec_us == 1500U);

    // 2 ms late is jitter, not a missed 20 ms cycle.
    d.onStart(122U, 120U, 20);
    d.onFinish(800U);
    assert(d.runs == 2U);
    assert(d.last_interval_ms == 22U);
    assert(d.max_interval_ms == 22U);
    assert(d.last_lateness_ms == 2U);
    assert(d.max_lateness_ms == 2U);
    assert(d.missed_periods == 0U);
    assert(d.max_exec_us == 1500U);

    // 45 ms late on a 20 ms task means two complete periods were skipped.
    d.onStart(185U, 140U, 20);
    d.onFinish(3000U);
    assert(d.runs == 3U);
    assert(d.last_interval_ms == 63U);
    assert(d.max_interval_ms == 63U);
    assert(d.last_lateness_ms == 45U);
    assert(d.max_lateness_ms == 45U);
    assert(d.missed_periods == 2U);
    assert(d.max_exec_us == 3000U);

    d.reset();
    assert(d.runs == 0U);
    assert(d.missed_periods == 0U);
    assert(d.max_interval_ms == 0U);
    assert(d.max_exec_us == 0U);
    assert(d.max_lateness_ms == 0U);

    SchedulerDiagnostics all;
    all.control.onStart(1000U, 1000U, 20);
    all.control.onFinish(200U);
    all.camera.onStart(1033U, 1033U, 33);
    all.camera.onFinish(10000U);
    assert(all.control.runs == 1U);
    assert(all.camera.max_exec_us == 10000U);

    all.reset();
    assert(all.control.runs == 0U);
    assert(all.camera.runs == 0U);

    std::cout << "[PASS] Scheduler jitter diagnostics accounting\n";
    return 0;
}
