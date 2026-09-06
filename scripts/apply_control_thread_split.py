#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one match, found {count}")
    return text.replace(old, new, 1)


root = Path(__file__).resolve().parents[1]
hpp_path = root / "src/execution_control/Executive.hpp"
cpp_path = root / "src/execution_control/Executive.cpp"
ci_path = root / ".github/workflows/scheduler-audit-tests.yml"

hpp = hpp_path.read_text()
hpp = replace_once(
    hpp,
    '#include <memory>\n#include <string>\n',
    '#include <memory>\n#include <mutex>\n#include <string>\n#include <thread>\n',
    'hpp includes',
)
hpp = replace_once(
    hpp,
    '    void configureScheduler();\n    void handleKeyboardTick();\n',
    '    void configureScheduler();\n    void handleKeyboardTick();\n    void controlWorkerLoop();\n    void joinControlWorker();\n',
    'hpp worker declarations',
)
hpp = replace_once(
    hpp,
    '    std::atomic<OperatingMode> operating_mode_{OperatingMode::LANE_DRIVING};\n\n    // A camera frame may remain on the blackboard after a failed grab. These\n',
    '    std::atomic<OperatingMode> operating_mode_{OperatingMode::LANE_DRIVING};\n\n'
    '    // [CONTROL_THREAD_SPLIT_V1]\n'
    '    // Vision stays on the Executive thread. The 20 ms control path runs\n'
    '    // independently so camera/perception overruns cannot block control.\n'
    '    std::thread control_thread_;\n'
    '    mutable std::mutex control_state_mutex_;\n'
    '    mutable std::mutex diagnostics_mutex_;\n\n'
    '    // A camera frame may remain on the blackboard after a failed grab. These\n',
    'hpp worker members',
)
hpp_path.write_text(hpp)

cpp = cpp_path.read_text()
cpp = replace_once(
    cpp,
    '    running_.store(true);\n\n    // Keyboard is operator I/O, not a 1 kHz control task. Polling at 20 ms\n',
    '    running_.store(true);\n\n'
    '    // [CONTROL_THREAD_SPLIT_V1]\n'
    '    // Start the independent 20 ms control path before entering the\n'
    '    // cooperative vision/planning loop. Blackboard access is already\n'
    '    // synchronized; non-blackboard control state is protected separately.\n'
    '    try {\n'
    '        control_thread_ = std::thread(&Executive::controlWorkerLoop, this);\n'
    '    } catch (const std::exception& e) {\n'
    '        std::cerr << "[CONTROL_THREAD] start failed: " << e.what() << "\\n";\n'
    '        running_.store(false);\n'
    '        state_.store(RuntimeState::ERROR);\n'
    '        return;\n'
    '    }\n\n'
    '    std::cout << "[CONTROL_THREAD] started periodMs="\n'
    '              << config_.runtime.control_period_ms << "\\n";\n\n'
    '    // Keyboard is operator I/O, not a 1 kHz control task. Polling at 20 ms\n',
    'run start worker',
)
cpp = replace_once(
    cpp,
    '        // Telemetry consumption must never depend on keyboard availability.\n'
    '        // receiveLatest() is non-blocking and drains the latest UART RX sample.\n'
    '        telemetryTick();\n\n',
    '        // Telemetry is consumed by the independent control worker so a\n'
    '        // camera/perception overrun cannot delay the latest control state.\n\n',
    'remove main telemetry',
)
old_control_block = '''        runPeriodicTask(scheduler_.control, scheduler_diagnostics_.control, [this]() {
#ifdef LAAS_ENABLE_PARKING_CLIENT
            // Step-11: service parking TCP/protocol before parking safety.
            // This remains independent of OperatingMode::PARKING.
            parkingNetworkTick();
#endif

            controlTick();
            parkingBenchControlTick();
        });

'''
cpp = replace_once(
    cpp,
    old_control_block,
    '        // Control runs on controlWorkerLoop(); do not execute it here.\n\n',
    'remove cooperative control block',
)
cpp = replace_once(
    cpp,
    '    ControlCmdMsg stop_cmd;\n',
    '    // Prevent any further worker command before issuing the final STOP.\n'
    '    running_.store(false);\n'
    '    joinControlWorker();\n\n'
    '    ControlCmdMsg stop_cmd;\n',
    'join before final stop',
)
cpp = replace_once(
    cpp,
    'void Executive::stop()\n{\n    running_.store(false);\n    yolo_.close();\n',
    'void Executive::stop()\n{\n'
    '    running_.store(false);\n'
    '    joinControlWorker();\n'
    '    yolo_.close();\n',
    'stop joins worker',
)
insert_before = 'void Executive::setUserRunRequest(bool enabled)\n'
worker_code = r'''void Executive::joinControlWorker()
{
    if (!control_thread_.joinable()) {
        return;
    }

    // stop() is not expected to execute on the control worker, but avoid a
    // self-join deadlock if that assumption changes later.
    if (control_thread_.get_id() == std::this_thread::get_id()) {
        return;
    }

    control_thread_.join();
}

void Executive::controlWorkerLoop()
{
    PeriodicTaskDiagnostics local_control_diagnostics;

    while (running_.load()) {
        const std::uint64_t runs_before = local_control_diagnostics.runs;

        runPeriodicTask(
            scheduler_.control,
            local_control_diagnostics,
            [this]() {
                // All mutable non-blackboard control/parking state is owned by
                // this worker and observed by logging under the same mutex.
                std::lock_guard<std::mutex> lock(control_state_mutex_);

                // UART RX parsing already runs in its own thread. Consume the
                // newest telemetry sample here so perception cannot delay it.
                telemetryTick();

#ifdef LAAS_ENABLE_PARKING_CLIENT
                // Preserve the Step-11/12 ordering exactly:
                // network/session -> lane control -> parking bench safety.
                parkingNetworkTick();
#endif
                controlTick();
                parkingBenchControlTick();
            });

        if (local_control_diagnostics.runs != runs_before) {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            scheduler_diagnostics_.control = local_control_diagnostics;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

'''
cpp = replace_once(cpp, insert_before, worker_code + insert_before, 'insert worker methods')
cpp = replace_once(
    cpp,
    '    const BehaviorRequest behavior = blackboard_.behavior();\n    const TrajectoryMsg trajectory = blackboard_.trajectory();\n\n    ControlCmdMsg raw = computeRawCommand(trajectory, lane, behavior);\n',
    '    BehaviorRequest behavior = blackboard_.behavior();\n'
    '    const TrajectoryMsg trajectory = blackboard_.trajectory();\n\n'
    '    // Independent hard stop gate for the threaded control path. Q/S can\n'
    '    // clear user_run_request_ while the slower decision loop still holds an\n'
    '    // older KEEP/FOLLOW behavior snapshot; never send that stale motion.\n'
    '    if (!user_run_request_.load()) {\n'
    '        behavior.mode = BehaviorMode::STOP;\n'
    '    }\n\n'
    '    ControlCmdMsg raw = computeRawCommand(trajectory, lane, behavior);\n',
    'control user stop gate',
)
old_logging_start = '''void Executive::loggingTick() const
{
    const VehicleTelemetryMsg telemetry = latest_telemetry_;
    const UartRxStats uart_stats = vehicle_.rxStats();
'''
new_logging_start = '''void Executive::loggingTick() const
{
    VehicleTelemetryMsg telemetry;
    ControlCmdMsg parking_bench_raw_command;
    ControlCmdMsg parking_bench_safe_command;
    ParkingTrackerDebug parking_tracker_debug;
    ParkingSafetyResult parking_safety_result;
    double telemetry_rx_hz = 0.0;
    std::uint64_t telemetry_sequence_gaps = 0U;
    std::uint64_t telemetry_duplicate_frames = 0U;
    std::uint64_t telemetry_sequence_resets = 0U;
#ifdef LAAS_ENABLE_PARKING_CLIENT
    bool parking_session_sync_hold = true;
    std::string parking_session_sync_reason{"NOT_CONNECTED"};
#endif

    {
        std::lock_guard<std::mutex> lock(control_state_mutex_);
        telemetry = latest_telemetry_;
        parking_bench_raw_command = parking_bench_raw_command_;
        parking_bench_safe_command = parking_bench_safe_command_;
        parking_tracker_debug = parking_tracker_debug_;
        parking_safety_result = parking_safety_result_;
        telemetry_rx_hz = telemetry_rx_hz_;
        telemetry_sequence_gaps = telemetry_sequence_gaps_;
        telemetry_duplicate_frames = telemetry_duplicate_frames_;
        telemetry_sequence_resets = telemetry_sequence_resets_;
#ifdef LAAS_ENABLE_PARKING_CLIENT
        parking_session_sync_hold = parking_session_sync_hold_;
        parking_session_sync_reason = parking_session_sync_reason_;
#endif
    }

    const UartRxStats uart_stats = vehicle_.rxStats();
'''
cpp = replace_once(cpp, old_logging_start, new_logging_start, 'logging state snapshot')
repls = {
    'parking_safety_result_.evaluated': 'parking_safety_result.evaluated',
    'parking_tracker_debug_.trajectory_id': 'parking_tracker_debug.trajectory_id',
    'parking_tracker_debug_.nearest_index': 'parking_tracker_debug.nearest_index',
    'parking_tracker_debug_.target_index': 'parking_tracker_debug.target_index',
    'parking_tracker_debug_.direction': 'parking_tracker_debug.direction',
    'parking_bench_raw_command_.speed_mps': 'parking_bench_raw_command.speed_mps',
    'parking_bench_safe_command_.speed_mps': 'parking_bench_safe_command.speed_mps',
    'parking_bench_safe_command_.steering_deg': 'parking_bench_safe_command.steering_deg',
    'parking_tracker_debug_.nearest_distance_m': 'parking_tracker_debug.nearest_distance_m',
    'parking_safety_result_.reason': 'parking_safety_result.reason',
    'parking_session_sync_hold_ ? "HOLD" : "READY"': 'parking_session_sync_hold ? "HOLD" : "READY"',
    'parking_session_sync_reason_': 'parking_session_sync_reason',
    'telemetry_rx_hz_': 'telemetry_rx_hz',
    'telemetry_sequence_gaps_': 'telemetry_sequence_gaps',
    'telemetry_duplicate_frames_': 'telemetry_duplicate_frames',
    'telemetry_sequence_resets_': 'telemetry_sequence_resets',
}
for old, new in repls.items():
    # These identifiers exist outside logging too. Replace only in logging tail.
    logging_pos = cpp.index('void Executive::loggingTick() const')
    head, tail = cpp[:logging_pos], cpp[logging_pos:]
    tail = tail.replace(old, new)
    cpp = head + tail

cpp = replace_once(
    cpp,
    '    const SchedulerDiagnostics& sd = scheduler_diagnostics_;\n    std::cout << "[SCHED]"\n',
    '    SchedulerDiagnostics sd;\n'
    '    {\n'
    '        std::lock_guard<std::mutex> lock(diagnostics_mutex_);\n'
    '        sd = scheduler_diagnostics_;\n'
    '    }\n\n'
    '    std::cout << "[SCHED]"\n'
    '              << " controlThread=1"\n',
    'logging diagnostics snapshot',
)
cpp_path.write_text(cpp)

ci = ci_path.read_text()
ci = replace_once(
    ci,
    "          for task in ('camera', 'yolo', 'perception', 'decision', 'planning', 'control', 'logging'):\n"
    "              needle = f'runPeriodicTask(scheduler_.{task}, scheduler_diagnostics_.{task}'\n"
    "              assert needle in run, f'missing diagnostics for {task}'\n\n",
    "          for task in ('camera', 'yolo', 'perception', 'decision', 'planning', 'logging'):\n"
    "              needle = f'runPeriodicTask(scheduler_.{task}, scheduler_diagnostics_.{task}'\n"
    "              assert needle in run, f'missing diagnostics for {task}'\n\n"
    "          assert 'runPeriodicTask(scheduler_.control' not in run, 'control must not run on cooperative vision loop'\n"
    "          assert 'telemetryTick();' not in run, 'telemetry must follow independent control timing'\n\n"
    "          worker_start = text.index('void Executive::controlWorkerLoop()')\n"
    "          worker_end = text.index('void Executive::setUserRunRequest', worker_start)\n"
    "          worker = text[worker_start:worker_end]\n"
    "          assert 'runPeriodicTask(' in worker and 'scheduler_.control' in worker\n"
    "          assert 'telemetryTick();' in worker\n"
    "          assert worker.index('parkingNetworkTick();') < worker.index('controlTick();') < worker.index('parkingBenchControlTick();')\n"
    "          assert 'control_state_mutex_' in worker\n"
    "          assert 'scheduler_diagnostics_.control = local_control_diagnostics' in worker\n\n",
    'ci timer assertions',
)
old_ci_control = '''          control_start = run.index('runPeriodicTask(scheduler_.control')
          control_end = run.index('runPeriodicTask(scheduler_.logging', control_start)
          control = run[control_start:control_end]
          assert control.index('parkingNetworkTick();') < control.index('controlTick();') < control.index('parkingBenchControlTick();')

'''
ci = replace_once(ci, old_ci_control, '', 'remove old ci control ordering')
ci = replace_once(
    ci,
    "          assert 'controlMissed=' in text\n",
    "          assert 'controlMissed=' in text\n"
    "          assert 'controlThread=1' in text\n"
    "          assert '[CONTROL_THREAD_SPLIT_V1]' in text\n"
    "          assert 'joinControlWorker();' in text\n"
    "          assert 'if (!user_run_request_.load())' in text\n",
    'ci control split markers',
)
ci = replace_once(
    ci,
    "          print('[PASS] scheduler jitter + main affinity OFF default + CPU2,3 AI + NO PARKING UART')\n",
    "          print('[PASS] threaded 20ms control isolation + jitter monitor + affinity policy + NO PARKING UART')\n",
    'ci pass label',
)
ci_path.write_text(ci)

print('[PATCH] control thread split applied')
