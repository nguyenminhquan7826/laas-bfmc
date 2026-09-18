#include "ParkingPerceptionModule.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "../../laas_core/Time.hpp"

namespace laas {
namespace {

std::string trim(const std::string& value)
{
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1U);
}

bool parseDouble(const std::string& text, double& value)
{
    try {
        std::size_t consumed = 0U;
        value = std::stod(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    } catch (const std::exception&) {
        return false;
    }
}

bool knownSlotId(const std::string& id)
{
    return id == "P_B1" || id == "P_B2" ||
           id == "P_T1" || id == "P_T2";
}

constexpr int kDebugBevWidthPx = 640;
constexpr int kDebugBevHeightPx = 480;
constexpr double kDebugBevPixelsPerMeter = 160.0;
constexpr double kDebugBevGridStepM = 0.5;

}  // namespace

ParkingPerceptionModule::ParkingPerceptionModule(const Config& config)
    : config_(config)
{
    if (!config_.parking.enable_camera_parking_perception) {
        return;
    }
    if (!config_.parking.image_to_ground_homography_valid) {
        reason_ = "IMAGE_TO_GROUND_HOMOGRAPHY_MISSING";
        return;
    }
    if (config_.parking.parking_sign_confirm_frames <= 0 ||
        config_.parking.occupied_confirm_frames <= 0 ||
        config_.parking.free_confirm_frames <= 0 ||
        config_.parking.parking_perception_timeout_ms <= 0 ||
        config_.parking.max_pose_frame_skew_ms < 0 ||
        config_.parking.slot_state_timeout_ms <= 0 ||
        config_.parking.min_visible_slot_fraction < 0.0 ||
        config_.parking.min_visible_slot_fraction > 1.0 ||
        config_.parking.min_projected_slot_area_px2 <= 0.0) {
        reason_ = "INVALID_PARKING_PERCEPTION_CONFIG";
        return;
    }

    const auto& h = config_.parking.image_to_ground_homography;
    image_to_ground_ = cv::Matx33d(
        h[0], h[1], h[2],
        h[3], h[4], h[5],
        h[6], h[7], h[8]);
    const double determinant = cv::determinant(cv::Mat(image_to_ground_));
    if (!std::isfinite(determinant) || std::fabs(determinant) < 1.0e-12) {
        reason_ = "IMAGE_TO_GROUND_HOMOGRAPHY_SINGULAR";
        return;
    }
    ground_to_image_ = image_to_ground_.inv();

    if (!loadSlots(config_.parking.slot_map_file)) {
        return;
    }
    tracks_.resize(slots_.size());
    ready_ = true;
    reason_ = "READY";
}

bool ParkingPerceptionModule::loadSlots(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        reason_ = "SLOT_MAP_UNREADABLE:" + path;
        return false;
    }

    bool in_slots = false;
    bool in_polygon = false;
    Slot* current = nullptr;
    double pending_x = 0.0;
    bool have_x = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string value = trim(line);
        if (value == "slots:") {
            in_slots = true;
            continue;
        }
        if (!in_slots) {
            continue;
        }
        if (!line.empty() && line.front() != ' ' &&
            value.compare(0U, 6U, "- id: ") != 0) {
            break;
        }
        if (value.compare(0U, 6U, "- id: ") == 0) {
            slots_.push_back(Slot{});
            current = &slots_.back();
            current->id = trim(value.substr(6U));
            in_polygon = false;
            have_x = false;
            continue;
        }
        if (value == "polygon_m:") {
            in_polygon = current != nullptr;
            have_x = false;
            continue;
        }
        if (in_polygon && value.compare(0U, 4U, "- - ") == 0) {
            have_x = parseDouble(trim(value.substr(4U)), pending_x);
            if (!have_x) {
                reason_ = "SLOT_MAP_INVALID_X";
                return false;
            }
            continue;
        }
        if (in_polygon && have_x && value.compare(0U, 2U, "- ") == 0) {
            double y = 0.0;
            if (!parseDouble(trim(value.substr(2U)), y)) {
                reason_ = "SLOT_MAP_INVALID_Y";
                return false;
            }
            current->polygon_map.emplace_back(pending_x, y);
            have_x = false;
            continue;
        }
        if (in_polygon && value != "polygon_m:" &&
            value.compare(0U, 2U, "- ") != 0) {
            in_polygon = false;
            have_x = false;
        }
    }

    if (slots_.size() != 4U) {
        reason_ = "SLOT_MAP_REQUIRES_FOUR_SLOTS";
        return false;
    }
    for (const Slot& slot : slots_) {
        if (!knownSlotId(slot.id) || slot.polygon_map.size() != 4U) {
            reason_ = "SLOT_MAP_INVALID_GEOMETRY:" + slot.id;
            return false;
        }
    }
    return true;
}

bool ParkingPerceptionModule::imageToGround(double u, double v,
                                             double& forward_m,
                                             double& left_m) const
{
    const double scale = image_to_ground_(2, 0) * u +
                         image_to_ground_(2, 1) * v +
                         image_to_ground_(2, 2);
    if (!std::isfinite(scale) || std::fabs(scale) < 1.0e-9) {
        return false;
    }
    forward_m = (image_to_ground_(0, 0) * u +
                 image_to_ground_(0, 1) * v +
                 image_to_ground_(0, 2)) / scale;
    left_m = (image_to_ground_(1, 0) * u +
              image_to_ground_(1, 1) * v +
              image_to_ground_(1, 2)) / scale;
    return std::isfinite(forward_m) && std::isfinite(left_m) &&
           forward_m > 0.0;
}

bool ParkingPerceptionModule::groundToImage(double forward_m, double left_m,
                                             cv::Point2f& pixel) const
{
    const double scale = ground_to_image_(2, 0) * forward_m +
                         ground_to_image_(2, 1) * left_m +
                         ground_to_image_(2, 2);
    if (!std::isfinite(scale) || std::fabs(scale) < 1.0e-9) {
        return false;
    }
    const double u = (ground_to_image_(0, 0) * forward_m +
                      ground_to_image_(0, 1) * left_m +
                      ground_to_image_(0, 2)) / scale;
    const double v = (ground_to_image_(1, 0) * forward_m +
                      ground_to_image_(1, 1) * left_m +
                      ground_to_image_(1, 2)) / scale;
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return false;
    }
    pixel = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
}

bool ParkingPerceptionModule::renderBirdEye(
    const cv::Mat& frame_bgr,
    const VehiclePoseMsg& pose,
    cv::Mat& output) const
{
    output.release();
    if (!ready_ || frame_bgr.empty()) {
        return false;
    }

    const double centre_x = 0.5 * static_cast<double>(kDebugBevWidthPx - 1);
    const double bottom_y = static_cast<double>(kDebugBevHeightPx - 1);
    const cv::Matx33d ground_to_bev(
        0.0, -kDebugBevPixelsPerMeter, centre_x,
        -kDebugBevPixelsPerMeter, 0.0, bottom_y,
        0.0, 0.0, 1.0);
    const cv::Matx33d image_to_bev = ground_to_bev * image_to_ground_;
    cv::warpPerspective(
        frame_bgr,
        output,
        cv::Mat(image_to_bev),
        cv::Size(kDebugBevWidthPx, kDebugBevHeightPx),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(8, 12, 18));

    const cv::Scalar grid_colour(75, 82, 92);
    const cv::Scalar axis_colour(40, 210, 255);
    const double max_forward_m = bottom_y / kDebugBevPixelsPerMeter;
    for (double forward = kDebugBevGridStepM;
         forward <= max_forward_m + 1.0e-9;
         forward += kDebugBevGridStepM) {
        const int y = static_cast<int>(std::lround(
            bottom_y - forward * kDebugBevPixelsPerMeter));
        cv::line(output, cv::Point(0, y),
                 cv::Point(kDebugBevWidthPx - 1, y),
                 grid_colour, 1, cv::LINE_AA);
        cv::putText(output, std::to_string(forward).substr(0, 3) + " m",
                    cv::Point(8, std::max(14, y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38,
                    grid_colour, 1, cv::LINE_AA);
    }
    const double max_left_m = centre_x / kDebugBevPixelsPerMeter;
    for (double left = -max_left_m; left <= max_left_m + 1.0e-9;
         left += kDebugBevGridStepM) {
        const int x = static_cast<int>(std::lround(
            centre_x - left * kDebugBevPixelsPerMeter));
        cv::line(output, cv::Point(x, 0),
                 cv::Point(x, kDebugBevHeightPx - 1),
                 std::fabs(left) < 1.0e-9 ? axis_colour : grid_colour,
                 std::fabs(left) < 1.0e-9 ? 2 : 1, cv::LINE_AA);
    }

    if (pose.header.valid) {
        for (const Slot& slot : slots_) {
            std::vector<cv::Point> polygon;
            polygon.reserve(slot.polygon_map.size());
            for (const cv::Point2d& map_point : slot.polygon_map) {
                const cv::Point2d ground = mapToGround(map_point, pose);
                polygon.emplace_back(
                    static_cast<int>(std::lround(
                        centre_x - ground.y * kDebugBevPixelsPerMeter)),
                    static_cast<int>(std::lround(
                        bottom_y - ground.x * kDebugBevPixelsPerMeter)));
            }
            if (polygon.size() >= 3U) {
                cv::polylines(output, polygon, true,
                              cv::Scalar(71, 223, 159), 2, cv::LINE_AA);
                const cv::Point label = polygon.front() + cv::Point(4, -5);
                cv::putText(output, slot.id, label,
                            cv::FONT_HERSHEY_SIMPLEX, 0.42,
                            cv::Scalar(71, 223, 159), 1, cv::LINE_AA);
            }
        }
    }

    const cv::Point rear_axle(
        static_cast<int>(std::lround(centre_x)),
        static_cast<int>(std::lround(bottom_y)));
    cv::circle(output, rear_axle, 6, cv::Scalar(40, 210, 255), -1,
               cv::LINE_AA);
    cv::arrowedLine(output, rear_axle,
                    rear_axle + cv::Point(0, -55),
                    axis_colour, 2, cv::LINE_AA, 0, 0.25);
    cv::putText(output, "rear axle / +forward",
                cv::Point(rear_axle.x + 10, rear_axle.y - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.42,
                axis_colour, 1, cv::LINE_AA);
    return true;
}

cv::Point2d ParkingPerceptionModule::groundToMap(
    double forward_m, double left_m, const VehiclePoseMsg& pose)
{
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    return cv::Point2d(
        pose.x_m + c * forward_m - s * left_m,
        pose.y_m + s * forward_m + c * left_m);
}

cv::Point2d ParkingPerceptionModule::mapToGround(
    const cv::Point2d& point, const VehiclePoseMsg& pose)
{
    const double dx = point.x - pose.x_m;
    const double dy = point.y - pose.y_m;
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    return cv::Point2d(c * dx + s * dy, -s * dx + c * dy);
}

bool ParkingPerceptionModule::contains(const Slot& slot,
                                        const cv::Point2d& point)
{
    bool inside = false;
    constexpr double kBoundaryEpsilon = 1.0e-9;
    for (std::size_t i = 0U, j = slot.polygon_map.size() - 1U;
         i < slot.polygon_map.size(); j = i++) {
        const cv::Point2d& a = slot.polygon_map[j];
        const cv::Point2d& b = slot.polygon_map[i];
        const double cross = (point.x - a.x) * (b.y - a.y) -
                             (point.y - a.y) * (b.x - a.x);
        if (std::fabs(cross) <= kBoundaryEpsilon &&
            point.x >= std::min(a.x, b.x) - kBoundaryEpsilon &&
            point.x <= std::max(a.x, b.x) + kBoundaryEpsilon &&
            point.y >= std::min(a.y, b.y) - kBoundaryEpsilon &&
            point.y <= std::max(a.y, b.y) + kBoundaryEpsilon) {
            return true;
        }
        const bool crosses = (a.y > point.y) != (b.y > point.y);
        if (crosses) {
            const double x_at_y = a.x +
                (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (point.x < x_at_y) {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool ParkingPerceptionModule::slotVisible(const Slot& slot,
                                           const VehiclePoseMsg& pose,
                                           int image_width,
                                           int image_height) const
{
    std::vector<cv::Point2f> projected;
    projected.reserve(slot.polygon_map.size());
    for (const cv::Point2d& map_point : slot.polygon_map) {
        const cv::Point2d ground = mapToGround(map_point, pose);
        if (ground.x <= 0.05) {
            return false;
        }
        cv::Point2f pixel;
        if (!groundToImage(ground.x, ground.y, pixel)) {
            return false;
        }
        projected.push_back(pixel);
    }

    std::vector<cv::Point2f> projected_hull;
    cv::convexHull(projected, projected_hull, false, true);
    const double projected_area =
        std::fabs(cv::contourArea(projected_hull));
    if (!std::isfinite(projected_area) ||
        projected_area < config_.parking.min_projected_slot_area_px2) {
        return false;
    }

    const std::vector<cv::Point2f> image_polygon{
        {0.0F, 0.0F},
        {static_cast<float>(image_width), 0.0F},
        {static_cast<float>(image_width), static_cast<float>(image_height)},
        {0.0F, static_cast<float>(image_height)},
    };
    std::vector<cv::Point2f> intersection;
    const float intersection_area = cv::intersectConvexConvex(
        projected_hull, image_polygon, intersection, true);
    return std::isfinite(intersection_area) &&
           intersection_area / projected_area >=
               config_.parking.min_visible_slot_fraction;
}

bool ParkingPerceptionModule::process(const YoloPerceptionMsg& perception,
                                       const VehiclePoseMsg& pose,
                                       std::uint64_t now_ms,
                                       ParkingStatusMsg& output)
{
    output = ParkingStatusMsg{};
    const std::uint64_t pose_frame_skew =
        pose.header.timestamp_ms >= perception.header.timestamp_ms
            ? pose.header.timestamp_ms - perception.header.timestamp_ms
            : perception.header.timestamp_ms - pose.header.timestamp_ms;
    if (!ready_ || !perception.header.valid ||
        !perception.has_detection_payload || !pose.header.valid ||
        perception.image_width <= 0 || perception.image_height <= 0 ||
        pose.map_id != config_.parking.map_id ||
        pose_frame_skew > static_cast<std::uint64_t>(
            config_.parking.max_pose_frame_skew_ms) ||
        !isFresh(now_ms, perception.header.timestamp_ms,
                 config_.parking.parking_perception_timeout_ms) ||
        !isFresh(now_ms, pose.header.timestamp_ms,
                 config_.parking.parking_perception_timeout_ms)) {
        return false;
    }
    if (have_frame_id_ && perception.frame_id == last_frame_id_) {
        return false;
    }
    have_frame_id_ = true;
    last_frame_id_ = perception.frame_id;

    float sign_confidence = 0.0F;
    for (const YoloDetection& detection : perception.detections) {
        if (detection.class_id == 0) {
            sign_confidence = std::max(sign_confidence, detection.confidence);
        }
    }
    if (sign_confidence >= config_.parking.parking_sign_min_confidence) {
        ++sign_hits_;
        if (sign_hits_ >= config_.parking.parking_sign_confirm_frames) {
            sign_active_until_ms_ = now_ms + static_cast<std::uint64_t>(
                std::max(0, config_.parking.parking_sign_hold_ms));
        }
    } else {
        sign_hits_ = 0;
    }
    const bool sign_active = sign_active_until_ms_ != 0U &&
                             now_ms <= sign_active_until_ms_;

    std::vector<bool> occupied(slots_.size(), false);
    std::vector<float> occupied_confidence(slots_.size(), 0.0F);
    for (const YoloDetection& detection : perception.detections) {
        if (detection.class_id == 0) {
            ParkingObjectEvidence evidence;
            evidence.class_name = "parking_sign";
            evidence.confidence = detection.confidence;
            output.objects.push_back(evidence);
            continue;
        }
        if (detection.class_id != 1 ||
            detection.confidence < config_.parking.vehicle_min_confidence) {
            continue;
        }

        const double foot_u = 0.5 * (detection.x1_px + detection.x2_px);
        const double foot_v = detection.y2_px;
        double forward_m = 0.0;
        double left_m = 0.0;
        if (!imageToGround(foot_u, foot_v, forward_m, left_m)) {
            continue;
        }
        const cv::Point2d map_point = groundToMap(forward_m, left_m, pose);
        ParkingObjectEvidence evidence;
        evidence.class_name = "vehicle";
        evidence.confidence = detection.confidence;
        evidence.relative_x_m = forward_m;
        evidence.relative_y_m = left_m;
        for (std::size_t slot_index = 0U;
             slot_index < slots_.size(); ++slot_index) {
            if (contains(slots_[slot_index], map_point)) {
                occupied[slot_index] = true;
                occupied_confidence[slot_index] = std::max(
                    occupied_confidence[slot_index], detection.confidence);
                evidence.associated_slot = slots_[slot_index].id;
                break;
            }
        }
        output.objects.push_back(evidence);
    }

    output.slots.reserve(slots_.size());
    for (std::size_t i = 0U; i < slots_.size(); ++i) {
        SlotTrack& track = tracks_[i];
        const bool visible = sign_active && slotVisible(
            slots_[i], pose, perception.image_width, perception.image_height);
        if (visible && occupied[i]) {
            ++track.occupied_hits;
            track.free_hits = 0;
            track.last_evidence_ms = now_ms;
            // A real vehicle observation invalidates a previously FREE slot
            // immediately. OCCUPIED still requires temporal confirmation.
            if (track.state == ParkingSlotState::FREE) {
                track.state = ParkingSlotState::UNKNOWN;
                track.confidence = 0.0F;
            }
            if (track.occupied_hits >= config_.parking.occupied_confirm_frames) {
                track.state = ParkingSlotState::OCCUPIED;
                track.confidence = occupied_confidence[i];
            }
        } else if (visible) {
            ++track.free_hits;
            track.occupied_hits = 0;
            track.last_evidence_ms = now_ms;
            if (track.free_hits >= config_.parking.free_confirm_frames) {
                track.state = ParkingSlotState::FREE;
                track.confidence = std::min(
                    1.0F,
                    static_cast<float>(track.free_hits) /
                        std::max(1, config_.parking.free_confirm_frames));
            }
        }

        if (!sign_active || track.last_evidence_ms == 0U ||
            !isFresh(now_ms, track.last_evidence_ms,
                     config_.parking.slot_state_timeout_ms)) {
            track.state = ParkingSlotState::UNKNOWN;
            track.confidence = 0.0F;
            if (!sign_active) {
                track.occupied_hits = 0;
                track.free_hits = 0;
            }
        }
        output.slots.push_back(
            {slots_[i].id, track.state, track.confidence});
    }

    output.header.valid = true;
    output.header.timestamp_ms = now_ms;
    output.sequence = sequence_++;
    output.map_id = config_.parking.map_id;
    return true;
}

}  // namespace laas
