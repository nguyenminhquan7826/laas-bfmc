#!/usr/bin/env python3
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one match, got {count}")
    return text.replace(old, new, 1)


root = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# Config: make bird-eye debug streaming an explicit optional path.
# ---------------------------------------------------------------------------
path = root / "src/laas_core/Config.hpp"
text = path.read_text()
text = replace_once(
    text,
    '    // Change this to the current IPv4 address of the monitoring laptop.\n'
    '    std::string monitor_ip = "192.168.1.253";\n',
    '    // Bird-eye debug streaming is observational only and can be disabled\n'
    '    // independently from the local YOLO safety path.\n'
    '    bool enable_debug_stream = true;\n\n'
    '    // Change this to the current IPv4 address of the monitoring laptop.\n'
    '    std::string monitor_ip = "192.168.1.253";\n',
    "Config.hpp debug flag",
)
path.write_text(text)

# ---------------------------------------------------------------------------
# Parking bench: keep local YOLO available while disabling debug JPEG/UDP by
# default. Explicit opt-in remains available for diagnostics.
# ---------------------------------------------------------------------------
path = root / "src/app/main.cpp"
text = path.read_text()
old = '''        config.runtime.enable_yolo_udp = bench_yolo_enabled;\n\n        config.parking.enable = true;\n'''
new = '''        config.runtime.enable_yolo_udp = bench_yolo_enabled;\n\n        // Bird-eye monitor traffic is not safety-critical and adds JPEG encode\n        // work to perception. Keep it OFF for bench/load tests unless explicitly\n        // requested. This never changes the UART safety gate above.\n        const char* bench_debug_env =\n            std::getenv("LAAS_PARKING_BENCH_DEBUG");\n        const bool bench_debug_enabled =\n            bench_yolo_enabled && bench_debug_env &&\n            std::string(bench_debug_env) == "1";\n        config.udp.enable_debug_stream = bench_debug_enabled;\n\n        config.parking.enable = true;\n'''
text = replace_once(text, old, new, "main.cpp bench debug flag")
text = replace_once(
    text,
    '            << " YOLO=" << (bench_yolo_enabled ? "ON" : "OFF")\n'
    '            << " server="\n',
    '            << " YOLO=" << (bench_yolo_enabled ? "ON" : "OFF")\n'
    '            << " DEBUG=" << (bench_debug_enabled ? "ON" : "OFF")\n'
    '            << " server="\n',
    "main.cpp startup debug status",
)
path.write_text(text)

# ---------------------------------------------------------------------------
# UDP interface: do not create/validate/use the monitor path when disabled.
# ---------------------------------------------------------------------------
path = root / "src/logical_robot/UdpYoloInterface.cpp"
text = path.read_text()
text = replace_once(
    text,
    '''    if (!makeAddress(udp.monitor_ip, udp.debug_send_port, impl_->debug_address)) {\n        std::cerr << "[UDP-YOLO] Invalid monitor address: "\n                  << udp.monitor_ip << ":" << udp.debug_send_port << "\\n";\n        return false;\n    }\n\n    impl_->yolo_send_sock = makeSendSocket();\n    impl_->debug_send_sock = makeSendSocket();\n    impl_->distance_recv_sock = makeReceiveSocket(\n        udp.local_ai_ip, udp.distance_recv_port, 1000);\n\n    if (impl_->yolo_send_sock < 0 ||\n        impl_->debug_send_sock < 0 ||\n        impl_->distance_recv_sock < 0) {\n''',
    '''    if (udp.enable_debug_stream &&\n        !makeAddress(udp.monitor_ip, udp.debug_send_port, impl_->debug_address)) {\n        std::cerr << "[UDP-YOLO] Invalid monitor address: "\n                  << udp.monitor_ip << ":" << udp.debug_send_port << "\\n";\n        return false;\n    }\n\n    impl_->yolo_send_sock = makeSendSocket();\n    if (udp.enable_debug_stream) {\n        impl_->debug_send_sock = makeSendSocket();\n    }\n    impl_->distance_recv_sock = makeReceiveSocket(\n        udp.local_ai_ip, udp.distance_recv_port, 1000);\n\n    if (impl_->yolo_send_sock < 0 ||\n        (udp.enable_debug_stream && impl_->debug_send_sock < 0) ||\n        impl_->distance_recv_sock < 0) {\n''',
    "UdpYoloInterface.cpp conditional debug socket",
)
text = replace_once(
    text,
    '''    std::cout << "[UDP-YOLO] Raw frame -> "\n              << udp.local_ai_ip << ":" << udp.yolo_send_port\n              << ", distance <- " << udp.local_ai_ip << ":"\n              << udp.distance_recv_port\n              << ", bird-eye -> " << udp.monitor_ip << ":"\n              << udp.debug_send_port << "\\n";\n''',
    '''    std::cout << "[UDP-YOLO] Raw frame -> "\n              << udp.local_ai_ip << ":" << udp.yolo_send_port\n              << ", distance <- " << udp.local_ai_ip << ":"\n              << udp.distance_recv_port;\n    if (udp.enable_debug_stream) {\n        std::cout << ", bird-eye -> " << udp.monitor_ip << ":"\n                  << udp.debug_send_port;\n    } else {\n        std::cout << ", bird-eye=OFF";\n    }\n    std::cout << "\\n";\n''',
    "UdpYoloInterface.cpp startup debug status",
)
text = replace_once(
    text,
    '''    if (!impl_->config.runtime.enable_yolo_udp || !impl_->initialized) {\n        return false;\n    }\n\n    const std::uint64_t now = nowMs();\n''',
    '''    if (!impl_->config.runtime.enable_yolo_udp || !impl_->initialized) {\n        return false;\n    }\n    if (!impl_->config.udp.enable_debug_stream) {\n        return true;\n    }\n\n    const std::uint64_t now = nowMs();\n''',
    "UdpYoloInterface.cpp debug no-op",
)
path.write_text(text)

# ---------------------------------------------------------------------------
# Lane detector: cache constant homography and keep the input frame const.
# Preserve the current no-crop/no-stretch IPM geometry exactly.
# ---------------------------------------------------------------------------
path = root / "src/functional/perception/LaneDetectorCore.hpp"
text = path.read_text()
text = replace_once(
    text,
    '    void processFrame(cv::Mat& frame_resize);\n',
    '    void processFrame(const cv::Mat& frame_resize);\n',
    "LaneDetectorCore.hpp const processFrame",
)
text = replace_once(
    text,
    '    cv::Mat bird_eye_view;\n    cv::Mat mask;\n',
    '    cv::Mat bird_eye_view;\n    cv::Mat mask;\n    cv::Mat ipm_matrix_;\n',
    "LaneDetectorCore.hpp cached homography",
)
path.write_text(text)

path = root / "src/functional/perception/LaneDetectorCore.cpp"
text = path.read_text()
text = replace_once(
    text,
    '''    lane_width_px_ = expected_lane_width_px_;\n}\n''',
    '''    lane_width_px_ = expected_lane_width_px_;\n\n    // Source/destination geometry is constant for a configured detector, so\n    // compute the homography once instead of rebuilding it for every frame.\n    const float margin_x = 150.0f *\n                           static_cast<float>(width) / 640.0f;\n    const std::vector<cv::Point2f> src_points = {\n        {235.0f, 285.0f},\n        {405.0f, 285.0f},\n        {560.0f, 470.0f},\n        { 95.0f, 470.0f}\n    };\n    const std::vector<cv::Point2f> dst_points = {\n        {margin_x, 0.0f},\n        {width - margin_x, 0.0f},\n        {width - margin_x, height - 1.0f},\n        {margin_x, height - 1.0f}\n    };\n    ipm_matrix_ = cv::getPerspectiveTransform(src_points, dst_points);\n}\n''',
    "LaneDetectorCore.cpp cache homography",
)
text = replace_once(
    text,
    'void LaneDetectorCore::processFrame(cv::Mat& frame_resize) {\n',
    'void LaneDetectorCore::processFrame(const cv::Mat& frame_resize) {\n',
    "LaneDetectorCore.cpp const processFrame",
)
start = text.index('cv::Mat LaneDetectorCore::applyIPM(const cv::Mat& frame)\n{')
end = text.index('\nvoid LaneDetectorCore::slidingWindow(', start)
replacement = '''cv::Mat LaneDetectorCore::applyIPM(const cv::Mat& frame)\n{\n    if (frame.empty()) {\n        return cv::Mat::zeros(height, width, CV_8UC3);\n    }\n\n    cv::Mat warped;\n    cv::warpPerspective(\n        frame,\n        warped,\n        ipm_matrix_,\n        cv::Size(width, height),\n        cv::INTER_LINEAR,\n        cv::BORDER_CONSTANT,\n        cv::Scalar(0, 0, 0));\n\n    // Preserve the currently validated A/B geometry: use the full warped\n    // 640x480 image without the legacy crop/stretch stage.\n    return warped;\n}\n'''
text = text[:start] + replacement + text[end:]
path.write_text(text)

# ---------------------------------------------------------------------------
# Perception wrapper: the detector does not modify the camera image. Avoid a
# full input clone and rely on cv::Mat reference counting for immutable outputs.
# ---------------------------------------------------------------------------
path = root / "src/functional/perception/LanePerceptionModule.cpp"
text = path.read_text()
text = replace_once(
    text,
    '''    cv::Mat frame = input.frame_bgr.clone();\n    detector_.processFrame(frame);\n\n    output.bird_eye_view = detector_.getBirdEyeView().clone();\n    output.mask = detector_.getMask().clone();\n''',
    '''    detector_.processFrame(input.frame_bgr);\n\n    // cv::Mat copies are reference-counted. Detector processing allocates new\n    // result buffers on the next frame, so these immutable snapshots do not\n    // need full-frame clones here.\n    output.bird_eye_view = detector_.getBirdEyeView();\n    output.mask = detector_.getMask();\n''',
    "LanePerceptionModule.cpp remove full-frame clones",
)
path.write_text(text)

# ---------------------------------------------------------------------------
# Executive: debug stream is explicitly gated; expose full perception timing.
# ---------------------------------------------------------------------------
path = root / "src/execution_control/Executive.cpp"
text = path.read_text()
text = replace_once(
    text,
    '''        if (config_.runtime.enable_yolo_udp && !lane.bird_eye_view.empty()) {\n            yolo_.sendDebugFrame(lane.bird_eye_view, 80);\n        }\n''',
    '''        if (config_.runtime.enable_yolo_udp &&\n            config_.udp.enable_debug_stream &&\n            !lane.bird_eye_view.empty()) {\n            yolo_.sendDebugFrame(lane.bird_eye_view, 80);\n        }\n''',
    "Executive.cpp debug stream gate",
)
text = replace_once(
    text,
    '''              << " perceptionExecUs=" << sd.perception.last_exec_us\n              << " planningExecUs=" << sd.planning.last_exec_us\n''',
    '''              << " perceptionExecUs=" << sd.perception.last_exec_us\n              << " perceptionMaxExecUs=" << sd.perception.max_exec_us\n              << " perceptionLateMs=" << sd.perception.last_lateness_ms\n              << " perceptionMissed=" << sd.perception.missed_periods\n              << " planningExecUs=" << sd.planning.last_exec_us\n''',
    "Executive.cpp perception diagnostics",
)
path.write_text(text)

# ---------------------------------------------------------------------------
# CI: permanently guard the low-overhead bench debug policy and cached IPM.
# ---------------------------------------------------------------------------
path = root / ".github/workflows/scheduler-audit-tests.yml"
text = path.read_text()
text = replace_once(
    text,
    '''          assert 'controlMissed=' in text\n          assert 'controlMaxLateMs=' in text\n          assert 'if (!user_run_request_.load())' in text\n''',
    '''          assert 'controlMissed=' in text\n          assert 'controlMaxLateMs=' in text\n          assert 'perceptionMaxExecUs=' in text\n          assert 'perceptionMissed=' in text\n          assert 'config_.udp.enable_debug_stream' in text\n          assert 'if (!user_run_request_.load())' in text\n''',
    "scheduler audit perception diagnostics",
)
text = replace_once(
    text,
    '''          assert 'config.runtime.enable_yolo_udp = bench_yolo_enabled;' in bench_profile\n          assert '\" UART=OFF\"' in bench_profile\n\n          assert 'configure_ai_affinity()' in ai_launcher\n''',
    '''          assert 'config.runtime.enable_yolo_udp = bench_yolo_enabled;' in bench_profile\n          assert 'LAAS_PARKING_BENCH_DEBUG' in bench_profile\n          assert 'config.udp.enable_debug_stream = bench_debug_enabled;' in bench_profile\n          assert '\" UART=OFF\"' in bench_profile\n          assert '\" DEBUG=\"' in bench_profile\n\n          lane_core_hpp = Path('src/functional/perception/LaneDetectorCore.hpp').read_text()\n          lane_core_cpp = Path('src/functional/perception/LaneDetectorCore.cpp').read_text()\n          lane_wrapper = Path('src/functional/perception/LanePerceptionModule.cpp').read_text()\n          assert 'cv::Mat ipm_matrix_' in lane_core_hpp\n          assert 'ipm_matrix_ = cv::getPerspectiveTransform' in lane_core_cpp\n          apply_start = lane_core_cpp.index('cv::Mat LaneDetectorCore::applyIPM')\n          apply_end = lane_core_cpp.index('void LaneDetectorCore::slidingWindow', apply_start)\n          apply_ipm = lane_core_cpp[apply_start:apply_end]\n          assert 'cv::getPerspectiveTransform' not in apply_ipm\n          assert 'return warped;' in apply_ipm\n          assert 'input.frame_bgr.clone()' not in lane_wrapper\n\n          assert 'configure_ai_affinity()' in ai_launcher\n''',
    "scheduler audit bench debug and cached IPM",
)
text = text.replace(
    "print('[PASS] independent camera/control workers + cached remap + safe bench invariants')",
    "print('[PASS] camera/control isolation + cached camera/IPM transforms + low-overhead bench debug policy + safety invariants')",
)
path.write_text(text)

print('[OK] perception cleanup applied')
