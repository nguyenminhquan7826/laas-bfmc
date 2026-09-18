#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "../../laas_core/Config.hpp"
#include "../../laas_core/Messages.hpp"

namespace laas {

// Converts Pi-local YOLO boxes into map-frame parking occupancy. It is
// deliberately fail-closed: no sign, stale pose, missing calibration, or an
// unobservable slot yields UNKNOWN rather than FREE.
class ParkingPerceptionModule {
public:
    explicit ParkingPerceptionModule(const Config& config);

    bool ready() const { return ready_; }
    const std::string& reason() const { return reason_; }

    bool process(const YoloPerceptionMsg& perception,
                 const VehiclePoseMsg& pose,
                 std::uint64_t now_ms,
                 ParkingStatusMsg& output);

    // Observational debug view only. Warps the undistorted camera frame onto
    // the rear-axle-centred parking ground plane and overlays metric grid/slot
    // geometry. It is never consumed by planning or actuation.
    bool renderBirdEye(const cv::Mat& frame_bgr,
                       const VehiclePoseMsg& pose,
                       cv::Mat& output) const;

private:
    struct Slot {
        std::string id;
        std::vector<cv::Point2d> polygon_map;
    };

    struct SlotTrack {
        ParkingSlotState state{ParkingSlotState::UNKNOWN};
        int occupied_hits{0};
        int free_hits{0};
        float confidence{0.0F};
        std::uint64_t last_evidence_ms{0};
    };

    bool loadSlots(const std::string& path);
    bool imageToGround(double u, double v,
                       double& forward_m, double& left_m) const;
    bool groundToImage(double forward_m, double left_m,
                       cv::Point2f& pixel) const;
    bool slotVisible(const Slot& slot,
                     const VehiclePoseMsg& pose,
                     int image_width,
                     int image_height) const;
    static cv::Point2d groundToMap(double forward_m, double left_m,
                                   const VehiclePoseMsg& pose);
    static cv::Point2d mapToGround(const cv::Point2d& point,
                                   const VehiclePoseMsg& pose);
    static bool contains(const Slot& slot, const cv::Point2d& point);

    const Config& config_;
    cv::Matx33d image_to_ground_{};
    cv::Matx33d ground_to_image_{};
    std::vector<Slot> slots_;
    std::vector<SlotTrack> tracks_;
    bool ready_{false};
    std::string reason_{"DISABLED"};
    int sign_hits_{0};
    std::uint64_t sign_active_until_ms_{0};
    std::uint64_t sequence_{1};
    std::uint32_t last_frame_id_{0};
    bool have_frame_id_{false};
};

}  // namespace laas
