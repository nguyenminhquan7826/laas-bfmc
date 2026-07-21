#include "PurePursuitController.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace laas {

namespace {
constexpr float kPi = 3.14159265358979323846f;
}

PurePursuitController::PurePursuitController()
    : wheelbase_m_(0.2515f),
      lookahead_m_(0.35f),
      pixel_per_meter_(250.0f),
      rear_axle_offset_px_(40.0f),
      max_steering_deg_(25.0f)
{
}

void PurePursuitController::setWheelbase(float wheelbase_m)
{
    if (wheelbase_m > 0.01f) {
        wheelbase_m_ = wheelbase_m;
    }
}

void PurePursuitController::setLookahead(float lookahead_m)
{
    if (lookahead_m > 0.01f) {
        lookahead_m_ = lookahead_m;
    }
}

void PurePursuitController::setPixelPerMeter(float pixel_per_meter)
{
    if (pixel_per_meter > 1e-3f) {
        pixel_per_meter_ = pixel_per_meter;
    }
}

void PurePursuitController::setRearAxleOffsetPx(float offset_px)
{
    rear_axle_offset_px_ = offset_px;
}

void PurePursuitController::setMaxSteeringDeg(float max_deg)
{
    if (max_deg > 0.0f) {
        max_steering_deg_ = max_deg;
    }
}

bool PurePursuitController::findTargetPoint(const std::vector<cv::Point>& path,
                                            const cv::Point2f& rear_axle,
                                            float lookahead_px,
                                            cv::Point2f& target) const
{
    if (path.size() < 2 || lookahead_px <= 0.0f) {
        return false;
    }

    bool found = false;
    float best_error_px = std::numeric_limits<float>::infinity();
    float best_forward_px = -std::numeric_limits<float>::infinity();
    float best_distance_px = 0.0f;

    for (const auto& point : path) {
        cv::Point2f p(static_cast<float>(point.x), static_cast<float>(point.y));
        const float dx = p.x - rear_axle.x;
        const float dy_forward = rear_axle.y - p.y;

        // BEV convention: smaller image y is in front of the rear axle.
        // Points on or behind the axle cannot be Pure Pursuit targets.
        if (!std::isfinite(dx) || !std::isfinite(dy_forward) || dy_forward <= 0.0f)
            continue;

        const float distance_px = std::sqrt(dx * dx + dy_forward * dy_forward);
        const float error_px = std::fabs(distance_px - lookahead_px);

        // This is independent of whether the vector is stored far-to-near or
        // near-to-far. In an equal-error tie, prefer the point farther ahead.
        if (!found ||
            error_px < best_error_px - 1e-4f ||
            (std::fabs(error_px - best_error_px) <= 1e-4f &&
             dy_forward > best_forward_px)) {
            target = p;
            best_error_px = error_px;
            best_forward_px = dy_forward;
            best_distance_px = distance_px;
            found = true;
        }
    }

    // A very short forward path produces unstable steering and must be
    // rejected so the integration layer can stop the vehicle safely.
    return found && best_distance_px >= 0.5f * lookahead_px;
}

bool PurePursuitController::computeSteeringAngle(const std::vector<cv::Point>& path,
                                                 const cv::Size& bev_size,
                                                 float vehicle_speed_mps,
                                                 float& steering_deg)
{
    steering_deg = 0.0f;

    if (path.size() < 2 || pixel_per_meter_ < 1e-3f || bev_size.width <= 0 || bev_size.height <= 0) {
        return false;
    }

    const cv::Point2f rear_axle(
        bev_size.width * 0.5f,
        bev_size.height - rear_axle_offset_px_);

    // Dynamic lookahead: low-speed car still gets a minimum lookahead, while
    // faster runs become smoother. This keeps the original default behavior
    // close to 0.35 m at vx ~= 0.15 m/s.
    const float min_lookahead_m = 0.25f;
    const float max_lookahead_m = std::max(lookahead_m_, min_lookahead_m);
    const float requested_ld_m = 0.25f + 0.65f * std::max(vehicle_speed_mps, 0.0f);
    const float dynamic_ld_m = std::min(
        std::max(requested_ld_m, min_lookahead_m),
        max_lookahead_m);
    const float ld_m = std::max(dynamic_ld_m, 0.01f);
    const float ld_px = ld_m * pixel_per_meter_;

    cv::Point2f target;
    if (!findTargetPoint(path, rear_axle, ld_px, target)) {
        return false;
    }

    const float dx = target.x - rear_axle.x;
    const float dy_forward = rear_axle.y - target.y;
    const float actual_ld_px = std::sqrt(dx * dx + dy_forward * dy_forward);
    const float actual_ld_m = actual_ld_px / pixel_per_meter_;

    if (!std::isfinite(actual_ld_m) || actual_ld_m < 0.01f)
        return false;

    const float alpha = std::atan2(dx, dy_forward);
    const float delta_rad = std::atan2(
        2.0f * wheelbase_m_ * std::sin(alpha),
        actual_ld_m);
    float delta_deg = delta_rad * 180.0f / kPi;

    if (!std::isfinite(delta_deg))
        return false;

    steering_deg = std::min(
        std::max(delta_deg, -max_steering_deg_),
        max_steering_deg_);
    return true;
}

}  // namespace laas
