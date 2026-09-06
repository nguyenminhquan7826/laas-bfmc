#!/usr/bin/env python3
from pathlib import Path

p = Path("src/execution_control/Executive.cpp")
s = p.read_text()

if "[SCHEDULER_JITTER_MONITOR_V1]" in s:
    print("[SKIP] scheduler jitter monitor already integrated")
    raise SystemExit(0)

old_helper = '''// [SCHEDULER_AUDIT_V1]
// Cooperative scheduler helper. Each task checks its deadline using a fresh
// monotonic timestamp. Normal ticks keep start-to-start cadence. If a task
// itself overruns its period, reset the phase at completion so the next loop
// cannot issue a catch-up burst immediately after the overrun.
template <typename Function>
void runPeriodicTask(PeriodicTimer& timer, Function&& task)
{
    const std::uint64_t start_ms = nowMs();
    if (!timer.ready(start_ms)) {
        return;
    }

    timer.mark(start_ms);
    task();

    const std::uint64_t end_ms = nowMs();
    if (end_ms >= start_ms &&
        end_ms - start_ms >= static_cast<std::uint64_t>(timer.periodMs())) {
        timer.mark(end_ms);
    }
}
'''

new_helper = '''// [SCHEDULER_AUDIT_V1]
// [SCHEDULER_JITTER_MONITOR_V1]
// Cooperative scheduler helper. Each task checks its deadline using a fresh
// monotonic timestamp. Normal ticks keep start-to-start cadence. If a task
// itself overruns its period, reset the phase at completion so the next loop
// cannot issue a catch-up burst immediately after the overrun.
//
// Diagnostics are observational only: they never change safety state or motion
// authorization. A missed period is counted only when lateness reaches at least
// one complete task period; small scheduler jitter remains visible as lateMs.
template <typename Function>
void runPeriodicTask(PeriodicTimer& timer,
                     PeriodicTaskDiagnostics& diagnostics,
                     Function&& task)
{
    const std::uint64_t start_ms = nowMs();
    if (!timer.ready(start_ms)) {
        return;
    }

    const std::uint64_t period_ms =
        static_cast<std::uint64_t>(timer.periodMs());
    const std::uint64_t expected_start_ms =
        timer.lastRunMs() + period_ms;

    diagnostics.onStart(start_ms, expected_start_ms, timer.periodMs());
    timer.mark(start_ms);

    const auto exec_start = std::chrono::steady_clock::now();
    task();
    const auto exec_end = std::chrono::steady_clock::now();

    const auto exec_us_signed =
        std::chrono::duration_cast<std::chrono::microseconds>(
            exec_end - exec_start).count();
    diagnostics.onFinish(
        exec_us_signed > 0
            ? static_cast<std::uint64_t>(exec_us_signed)
            : 0U);

    const std::uint64_t end_ms = nowMs();
    if (end_ms >= start_ms &&
        end_ms - start_ms >= period_ms) {
        timer.mark(end_ms);
    }
}
'''

if s.count(old_helper) != 1:
    raise SystemExit(f"[FAIL] helper marker count={s.count(old_helper)}")
s = s.replace(old_helper, new_helper, 1)

old_reset = '''    const uint64_t now = nowMs();
    scheduler_.reset(now);
    state_.store(RuntimeState::READY);
'''
new_reset = '''    const uint64_t now = nowMs();
    scheduler_.reset(now);
    scheduler_diagnostics_.reset();
    state_.store(RuntimeState::READY);
'''
if s.count(old_reset) != 1:
    raise SystemExit(f"[FAIL] scheduler reset marker count={s.count(old_reset)}")
s = s.replace(old_reset, new_reset, 1)

replacements = [
    ('''            runPeriodicTask(keyboard_timer, [this]() {\n''',
     '''            runPeriodicTask(keyboard_timer, scheduler_diagnostics_.keyboard, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.camera, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.camera, scheduler_diagnostics_.camera, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.yolo, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.yolo, scheduler_diagnostics_.yolo, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.perception, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.perception, scheduler_diagnostics_.perception, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.decision, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.decision, scheduler_diagnostics_.decision, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.planning, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.planning, scheduler_diagnostics_.planning, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.control, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.control, scheduler_diagnostics_.control, [this]() {\n'''),
    ('''        runPeriodicTask(scheduler_.logging, [this]() {\n''',
     '''        runPeriodicTask(scheduler_.logging, scheduler_diagnostics_.logging, [this]() {\n'''),
]

for old, new in replacements:
    if s.count(old) != 1:
        raise SystemExit(f"[FAIL] periodic call marker count={s.count(old)} for {old.strip()}")
    s = s.replace(old, new, 1)

exec_marker = '''    std::cout   << "[EXEC] "
'''

sched_log = '''    const SchedulerDiagnostics& sd = scheduler_diagnostics_;
    std::cout << "[SCHED]"
              << " controlDtMs=" << sd.control.last_interval_ms
              << " controlMaxDtMs=" << sd.control.max_interval_ms
              << " controlExecUs=" << sd.control.last_exec_us
              << " controlMaxExecUs=" << sd.control.max_exec_us
              << " controlLateMs=" << sd.control.last_lateness_ms
              << " controlMaxLateMs=" << sd.control.max_lateness_ms
              << " controlMissed=" << sd.control.missed_periods
              << " cameraExecUs=" << sd.camera.last_exec_us
              << " cameraMaxExecUs=" << sd.camera.max_exec_us
              << " cameraLateMs=" << sd.camera.last_lateness_ms
              << " cameraMissed=" << sd.camera.missed_periods
              << " yoloExecUs=" << sd.yolo.last_exec_us
              << " yoloMaxExecUs=" << sd.yolo.max_exec_us
              << " yoloLateMs=" << sd.yolo.last_lateness_ms
              << " yoloMissed=" << sd.yolo.missed_periods
              << " perceptionExecUs=" << sd.perception.last_exec_us
              << " planningExecUs=" << sd.planning.last_exec_us
              << " decisionExecUs=" << sd.decision.last_exec_us
              << "\n";

    std::cout   << "[EXEC] "
'''

if s.count(exec_marker) != 1:
    raise SystemExit(f"[FAIL] logging marker count={s.count(exec_marker)}")
s = s.replace(exec_marker, sched_log, 1)

p.write_text(s)
print("[PASS] scheduler jitter monitor integrated")
