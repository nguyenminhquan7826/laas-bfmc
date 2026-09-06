#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


root = Path(__file__).resolve().parents[1]

hpp_path = root / "src/execution_control/Executive.hpp"
hpp = hpp_path.read_text()
hpp = replace_once(
    hpp,
    """    void controlWorkerLoop();\n    void joinControlWorker();\n\n    void cameraTick();\n""",
    """    void controlWorkerLoop();\n    void joinControlWorker();\n    void cameraWorkerLoop();\n    void joinCameraWorker();\n\n    void cameraTick();\n""",
    "Executive.hpp worker declarations",
)
hpp = replace_once(
    hpp,
    """    // [CONTROL_THREAD_SPLIT_V1]\n    // Vision stays on the Executive thread. The 20 ms control path runs\n    // independently so camera/perception overruns cannot block control.\n    std::thread control_thread_;\n""",
    """    // [CONTROL_THREAD_SPLIT_V1]\n    // [CAMERA_THREAD_SPLIT_V1]\n    // Control and camera capture have independent periodic workers. The\n    // remaining perception/decision/planning pipeline stays cooperative.\n    std::thread control_thread_;\n    std::thread camera_thread_;\n""",
    "Executive.hpp worker members",
)
hpp_path.write_text(hpp)

cpp_path = root / "src/execution_control/Executive.cpp"
cpp = cpp_path.read_text()
cpp = replace_once(
    cpp,
    """    running_.store(true);\n\n    // [CONTROL_THREAD_SPLIT_V1]\n    // Start the independent 20 ms control path before entering the\n    // cooperative vision/planning loop. Blackboard access is already\n    // synchronized; non-blackboard control state is protected separately.\n    try {\n        control_thread_ = std::thread(&Executive::controlWorkerLoop, this);\n    } catch (const std::exception& e) {\n        std::cerr << \"[CONTROL_THREAD] start failed: \" << e.what() << \"\\n\";\n        running_.store(false);\n        state_.store(RuntimeState::ERROR);\n        return;\n    }\n\n    std::cout << \"[CONTROL_THREAD] started periodMs=\"\n              << config_.runtime.control_period_ms << \"\\n\";\n""",
    """    running_.store(true);\n\n    // [CAMERA_THREAD_SPLIT_V1]\n    // Capture owns CameraInterface while run() is active and only publishes\n    // the latest FrameMsg to the thread-safe Blackboard. Perception may overrun\n    // without delaying acquisition of the newest camera frame.\n    try {\n        camera_thread_ = std::thread(&Executive::cameraWorkerLoop, this);\n    } catch (const std::exception& e) {\n        std::cerr << \"[CAMERA_THREAD] start failed: \" << e.what() << \"\\n\";\n        running_.store(false);\n        state_.store(RuntimeState::ERROR);\n        return;\n    }\n\n    std::cout << \"[CAMERA_THREAD] started periodMs=\"\n              << config_.runtime.camera_period_ms << \"\\n\";\n\n    // [CONTROL_THREAD_SPLIT_V1]\n    // Start the independent 20 ms control path before entering the remaining\n    // cooperative perception/planning loop.\n    try {\n        control_thread_ = std::thread(&Executive::controlWorkerLoop, this);\n    } catch (const std::exception& e) {\n        std::cerr << \"[CONTROL_THREAD] start failed: \" << e.what() << \"\\n\";\n        running_.store(false);\n        joinCameraWorker();\n        state_.store(RuntimeState::ERROR);\n        return;\n    }\n\n    std::cout << \"[CONTROL_THREAD] started periodMs=\"\n              << config_.runtime.control_period_ms << \"\\n\";\n""",
    "Executive.cpp worker startup",
)
cpp = replace_once(
    cpp,
    """        runPeriodicTask(scheduler_.camera, scheduler_diagnostics_.camera, [this]() {\n            cameraTick();\n        });\n\n        runPeriodicTask(scheduler_.yolo, scheduler_diagnostics_.yolo, [this]() {\n""",
    """        // Camera capture runs on cameraWorkerLoop(); tasks below always\n        // consume the newest frame available on the Blackboard.\n        runPeriodicTask(scheduler_.yolo, scheduler_diagnostics_.yolo, [this]() {\n""",
    "Executive.cpp remove main camera tick",
)
cpp = replace_once(
    cpp,
    """    running_.store(false);\n    joinControlWorker();\n\n    ControlCmdMsg stop_cmd;\n""",
    """    running_.store(false);\n    joinCameraWorker();\n    joinControlWorker();\n\n    ControlCmdMsg stop_cmd;\n""",
    "Executive.cpp run joins",
)
cpp = replace_once(
    cpp,
    """void Executive::stop()\n{\n    running_.store(false);\n    joinControlWorker();\n    yolo_.close();\n""",
    """void Executive::stop()\n{\n    running_.store(false);\n    joinCameraWorker();\n    joinControlWorker();\n    yolo_.close();\n""",
    "Executive.cpp stop joins",
)
cpp = replace_once(
    cpp,
    """void Executive::joinControlWorker()\n{\n""",
    """void Executive::joinCameraWorker()\n{\n    if (!camera_thread_.joinable()) {\n        return;\n    }\n\n    if (camera_thread_.get_id() == std::this_thread::get_id()) {\n        return;\n    }\n\n    camera_thread_.join();\n}\n\nvoid Executive::cameraWorkerLoop()\n{\n    PeriodicTaskDiagnostics local_camera_diagnostics;\n\n    while (running_.load()) {\n        const std::uint64_t runs_before = local_camera_diagnostics.runs;\n\n        runPeriodicTask(\n            scheduler_.camera,\n            local_camera_diagnostics,\n            [this]() {\n                cameraTick();\n            });\n\n        if (local_camera_diagnostics.runs != runs_before) {\n            std::lock_guard<std::mutex> lock(diagnostics_mutex_);\n            scheduler_diagnostics_.camera = local_camera_diagnostics;\n        }\n\n        std::this_thread::sleep_for(std::chrono::milliseconds(1));\n    }\n}\n\nvoid Executive::joinControlWorker()\n{\n""",
    "Executive.cpp camera worker implementation",
)
cpp = replace_once(
    cpp,
    """    std::cout << \"[SCHED]\"\n              << \" controlThread=1\"\n""",
    """    std::cout << \"[SCHED]\"\n              << \" controlThread=1\"\n              << \" cameraThread=1\"\n""",
    "Executive.cpp scheduler log marker",
)
cpp_path.write_text(cpp)

print("[PASS] applied camera capture thread split")
