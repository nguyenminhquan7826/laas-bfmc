# Changelog

## 0.6.0-p0-batch - 2026-07-18

- Synchronized Config, Executive, SafetyFilter and Pure Pursuit APIs.
- Restored a buildable CMake project and enforced C++14.
- Added stale frame, behavior, trajectory and command timeouts.
- Made lane-change direction persist through temporary obstacle dropouts.
- Rejected lane-change paths beyond the physical steering-curvature limit.
- Extended the default lane-change horizon and trigger distance.
- Made the BEV scale fixed and physically configurable; removed dynamic crop.
- Removed IPM debug graphics from the perception input.
- Added V4L2 and CSI camera selection.
- Replaced single-datagram JPEG transfer with chunked UDP transport.
- Fixed the Python distance filter so sudden closer obstacles are accepted.
- Made the ONNX path portable and added Python dependencies.
- Kept P1a watchdog changes out by design.
