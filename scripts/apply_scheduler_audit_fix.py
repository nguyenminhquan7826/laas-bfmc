#!/usr/bin/env python3
from pathlib import Path

exec_path = Path("src/execution_control/Executive.cpp")
text = exec_path.read_text()

MARKER = "[SCHEDULER_AUDIT_V1]"
if MARKER in text:
    print("[SKIP] scheduler audit fix already applied")
else:
    old = '''int getchNonBlocking()\n{\n    termios oldt{};\n    termios newt{};\n\n    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {\n        return -1;\n    }\n\n    newt = oldt;\n    newt.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));\n    tcsetattr(STDIN_FILENO, TCSANOW, &newt);\n\n    const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);\n    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);\n\n    const int ch = getchar();\n\n    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);\n    fcntl(STDIN_FILENO, F_SETFL, old_flags);\n\n    return ch;\n}\n\n}  // namespace\n'''

    new = '''int getchNonBlocking()\n{\n    termios oldt{};\n    termios newt{};\n\n    if (tcgetattr(STDIN_FILENO, &oldt) != 0) {\n        return -1;\n    }\n\n    newt = oldt;\n    newt.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));\n    tcsetattr(STDIN_FILENO, TCSANOW, &newt);\n\n    const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);\n    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);\n\n    const int ch = getchar();\n\n    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);\n    fcntl(STDIN_FILENO, F_SETFL, old_flags);\n\n    return ch;\n}\n\n// [SCHEDULER_AUDIT_V1]\n// Cooperative scheduler helper. Each task checks its deadline using a fresh\n// monotonic timestamp. Normal ticks keep start-to-start cadence. If a task\n// itself overruns its period, reset the phase at completion so the next loop\n// cannot issue a catch-up burst immediately after the overrun.\ntemplate <typename Function>\nvoid runPeriodicTask(PeriodicTimer& timer, Function&& task)\n{\n    const std::uint64_t start_ms = nowMs();\n    if (!timer.ready(start_ms)) {\n        return;\n    }\n\n    timer.mark(start_ms);\n    task();\n\n    const std::uint64_t end_ms = nowMs();\n    if (end_ms >= start_ms &&\n        end_ms - start_ms >= static_cast<std::uint64_t>(timer.periodMs())) {\n        timer.mark(end_ms);\n    }\n}\n\n}  // namespace\n'''

    if text.count(old) != 1:
        raise SystemExit(f"[FAIL] helper insertion marker count={text.count(old)}")
    text = text.replace(old, new, 1)

    old_loop = '''    running_.store(true);\n\n    while (running_.load()) {\n        const uint64_t now = nowMs();\n\n        if (config_.runtime.enable_keyboard) {\n            handleKeyboardTick();\n        }\n\n        if (config_.runtime.enable_keyboard) {\n            handleKeyboardTick();\n            telemetryTick();\n        }\n\n        if (scheduler_.camera.ready(now)) {\n            scheduler_.camera.mark(now);\n            cameraTick();\n        }\n\n        if (scheduler_.yolo.ready(now)) {\n            scheduler_.yolo.mark(now);\n            yoloTick();\n        }\n\n        if (scheduler_.perception.ready(now)) {\n            scheduler_.perception.mark(now);\n            perceptionTick();\n        }\n\n        if (scheduler_.decision.ready(now)) {\n            scheduler_.decision.mark(now);\n            decisionTick();\n        }\n\n        if (scheduler_.planning.ready(now)) {\n            scheduler_.planning.mark(now);\n            planningTick();\n        }\n\n        if (scheduler_.control.ready(now)) {\n            scheduler_.control.mark(now);\n\n#ifdef LAAS_ENABLE_PARKING_CLIENT\n            // Step-11: service parking TCP/protocol before parking safety.\n            //\n            // This is intentionally independent of OperatingMode::PARKING.\n            // The Server connection must be maintained during bench testing\n            // even while the real vehicle operating mode remains LANE_DRIVING.\n            parkingNetworkTick();\n#endif\n\n            controlTick();\n            parkingBenchControlTick();\n        }\n\n        if (scheduler_.logging.ready(now)) {\n            scheduler_.logging.mark(now);\n            loggingTick();\n        }\n\n        std::this_thread::sleep_for(std::chrono::milliseconds(1));\n    }\n'''

    new_loop = '''    running_.store(true);\n\n    // Keyboard is operator I/O, not a 1 kHz control task. Polling at 20 ms\n    // keeps it responsive while avoiding repeated tcgetattr/tcsetattr/fcntl\n    // calls on every 1 ms scheduler spin.\n    constexpr int kKeyboardPollPeriodMs = 20;\n    PeriodicTimer keyboard_timer(kKeyboardPollPeriodMs);\n    keyboard_timer.reset(nowMs());\n\n    while (running_.load()) {\n        if (config_.runtime.enable_keyboard) {\n            runPeriodicTask(keyboard_timer, [this]() {\n                handleKeyboardTick();\n            });\n\n            // Q/ESC must not allow another camera/planning/control tick in the\n            // same scheduler iteration. The normal shutdown STOP is sent below.\n            if (!running_.load()) {\n                break;\n            }\n        }\n\n        // Telemetry consumption must never depend on keyboard availability.\n        // receiveLatest() is non-blocking and drains the latest UART RX sample.\n        telemetryTick();\n\n        runPeriodicTask(scheduler_.camera, [this]() {\n            cameraTick();\n        });\n\n        runPeriodicTask(scheduler_.yolo, [this]() {\n            yoloTick();\n        });\n\n        runPeriodicTask(scheduler_.perception, [this]() {\n            perceptionTick();\n        });\n\n        runPeriodicTask(scheduler_.decision, [this]() {\n            decisionTick();\n        });\n\n        runPeriodicTask(scheduler_.planning, [this]() {\n            planningTick();\n        });\n\n        runPeriodicTask(scheduler_.control, [this]() {\n#ifdef LAAS_ENABLE_PARKING_CLIENT\n            // Step-11: service parking TCP/protocol before parking safety.\n            // This remains independent of OperatingMode::PARKING.\n            parkingNetworkTick();\n#endif\n\n            controlTick();\n            parkingBenchControlTick();\n        });\n\n        runPeriodicTask(scheduler_.logging, [this]() {\n            loggingTick();\n        });\n\n        std::this_thread::sleep_for(std::chrono::milliseconds(1));\n    }\n'''

    if text.count(old_loop) != 1:
        raise SystemExit(f"[FAIL] Executive run-loop marker count={text.count(old_loop)}")
    text = text.replace(old_loop, new_loop, 1)
    exec_path.write_text(text)
    print("[PASS] Executive scheduler audit fix applied")

# Permanent scheduler unit test.
test_path = Path("server_v1/tests/test_scheduler.cpp")
test_path.write_text(r'''#include <cassert>
#include <cstdint>
#include <iostream>

#include "execution_control/Scheduler.hpp"

int main()
{
    using laas::PeriodicTimer;
    using laas::Scheduler;

    PeriodicTimer timer(20);
    timer.reset(100U);
    assert(!timer.ready(119U));
    assert(timer.ready(120U));

    timer.mark(127U);
    assert(!timer.ready(146U));
    assert(timer.ready(147U));

    timer.setPeriod(0);
    assert(timer.periodMs() == 1);
    timer.reset(200U);
    assert(!timer.ready(200U));
    assert(timer.ready(201U));

    Scheduler scheduler;
    scheduler.configure(33, 333, 33, 50, 50, 20, 100);
    assert(scheduler.camera.periodMs() == 33);
    assert(scheduler.yolo.periodMs() == 333);
    assert(scheduler.perception.periodMs() == 33);
    assert(scheduler.decision.periodMs() == 50);
    assert(scheduler.planning.periodMs() == 50);
    assert(scheduler.control.periodMs() == 20);
    assert(scheduler.logging.periodMs() == 100);

    scheduler.reset(1000U);
    assert(!scheduler.control.ready(1019U));
    assert(scheduler.control.ready(1020U));
    assert(!scheduler.logging.ready(1099U));
    assert(scheduler.logging.ready(1100U));

    std::cout << "[PASS] Scheduler periodic semantics\n";
    return 0;
}
''')

workflow_path = Path(".github/workflows/scheduler-audit-tests.yml")
workflow_path.write_text(r'''name: Scheduler Audit Tests

on:
  push:
    branches:
      - step12-status-sync
  pull_request:
    branches:
      - main

jobs:
  scheduler-audit:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Build scheduler unit test
        run: |
          g++ -std=c++14 -Wall -Wextra -Wpedantic \
            -Isrc \
            server_v1/tests/test_scheduler.cpp \
            src/execution_control/Scheduler.cpp \
            -o /tmp/test_scheduler

      - name: Run scheduler unit test
        run: /tmp/test_scheduler

      - name: Audit Executive scheduler structure
        run: |
          python - <<'PY'
          from pathlib import Path

          text = Path('src/execution_control/Executive.cpp').read_text()
          start = text.index('void Executive::run()')
          end = text.index('void Executive::stop()', start)
          run = text[start:end]

          assert run.count('handleKeyboardTick();') == 1, 'keyboard tick must occur exactly once'
          assert run.count('telemetryTick();') == 1, 'telemetry tick must occur exactly once'
          assert 'runPeriodicTask(scheduler_.control' in run
          assert 'runPeriodicTask(scheduler_.camera' in run
          assert '[SCHEDULER_AUDIT_V1]' in text

          control_start = run.index('runPeriodicTask(scheduler_.control')
          control_end = run.index('runPeriodicTask(scheduler_.logging', control_start)
          control = run[control_start:control_end]
          assert control.index('parkingNetworkTick();') < control.index('controlTick();') < control.index('parkingBenchControlTick();')

          bench_start = text.index('void Executive::parkingBenchControlTick()')
          bench_end = text.index('void Executive::controlTick()', bench_start)
          bench = '\n'.join(
              line for line in text[bench_start:bench_end].splitlines()
              if not line.lstrip().startswith('//')
          )
          assert 'vehicle_.send' not in bench, 'parking bench must remain NO UART'
          print('[PASS] Executive scheduler structure and NO PARKING UART')
          PY
''')

print("[SUMMARY] scheduler audit files ready")
