#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


config_path = Path("src/laas_core/Config.hpp")
config = config_path.read_text()
config = replace_once(
    config,
    "    int perception_period_ms = 33;\n",
    "    // Pi 5 benchmark: lane perception typically takes ~40 ms (P95 ~47 ms).\n"
    "    // A 50 ms period gives a sustainable 20 Hz start-to-start cadence instead\n"
    "    // of repeatedly overrunning the former 33 ms target. Control and camera\n"
    "    // remain independently scheduled at 20 ms and 33 ms respectively.\n"
    "    int perception_period_ms = 50;\n",
    "perception period",
)
config_path.write_text(config)

exec_path = Path("src/execution_control/Executive.cpp")
text = exec_path.read_text()
text = replace_once(
    text,
    "#include <iostream>\n#include <thread>\n",
    "#include <iostream>\n#include <sstream>\n#include <thread>\n",
    "sstream include",
)
old = '''    std::cout << "[SCHED]"
              << " controlThread=1"
              << " cameraThread=1"
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
              << " perceptionMaxExecUs=" << sd.perception.max_exec_us
              << " perceptionLateMs=" << sd.perception.last_lateness_ms
              << " perceptionMissed=" << sd.perception.missed_periods
              << " planningExecUs=" << sd.planning.last_exec_us
              << " decisionExecUs=" << sd.decision.last_exec_us
              << "\\n";
'''
new = '''    // Build the diagnostics line first, then emit it in one insertion. This
    // avoids partial-field interleaving with logs produced by worker threads.
    std::ostringstream sched_line;
    sched_line << "[SCHED]"
               << " controlThread=1"
               << " cameraThread=1"
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
               << " perceptionMaxExecUs=" << sd.perception.max_exec_us
               << " perceptionLateMs=" << sd.perception.last_lateness_ms
               << " perceptionMissed=" << sd.perception.missed_periods
               << " planningExecUs=" << sd.planning.last_exec_us
               << " decisionExecUs=" << sd.decision.last_exec_us;
    std::cout << sched_line.str() << "\\n";
'''
text = replace_once(text, old, new, "scheduler log block")
exec_path.write_text(text)

print("[OK] perception 20Hz finalization applied")
