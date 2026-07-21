#include "LanePerceptionModule.hpp"

#include "../../laas_core/Time.hpp"

namespace laas {

LanePerceptionModule::LanePerceptionModule(const Config& config)
    : config_(config),
      detector_(config.camera.width,
                config.camera.height,
                config.planner.lane_width_m,
                config.camera.bev_forward_range_m)
{
}

bool LanePerceptionModule::process(const FrameMsg& input, LanePerceptionMsg& output)
{
    output = LanePerceptionMsg{};
    // Preserve the acquisition time. Stamping this with nowMs() would make an
    // old camera frame look fresh after a delayed perception pass.
    output.header.timestamp_ms = input.header.timestamp_ms;

    if (!input.header.valid || input.frame_bgr.empty()) {
        output.header.valid = false;
        return false;
    }

    cv::Mat frame = input.frame_bgr.clone();
    detector_.processFrame(frame);

    output.bird_eye_view = detector_.getBirdEyeView().clone();
    output.mask = detector_.getMask().clone();
    output.centerline = detector_.getCenterline();

    output.left_coeffs = detector_.getLeftCoeffs();
    output.right_coeffs = detector_.getRightCoeffs();
    output.has_left_lane = detector_.hasLeftLane();
    output.has_right_lane = detector_.hasRightLane();
    output.left_type = detector_.left_type;
    output.right_type = detector_.right_type;
    output.lane_width_px = detector_.getLaneWidthPx();

    output.header.valid = detector_.hasValidLane() && output.centerline.size() >= 3;
    return output.header.valid;
}

void LanePerceptionModule::reset()
{
    detector_.resetTracking();
}

}  // namespace laas
