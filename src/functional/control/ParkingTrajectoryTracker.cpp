#include "ParkingTrajectoryTracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../../laas_core/Time.hpp"

namespace laas {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFiniteEpsilon = 1e-9;
}

ParkingTrajectoryTracker::ParkingTrajectoryTracker(const Config& config)
    : config_(config)
{
}

void ParkingTrajectoryTracker::reset()
{
    active_trajectory_id_ = 0;
    progress_index_ = 0;
}

double ParkingTrajectoryTracker::normalizeAngle(double angle_rad)
{
    while (angle_rad > kPi) angle_rad -= 2.0 * kPi;
    while (angle_rad < -kPi) angle_rad += 2.0 * kPi;
    return angle_rad;
}

double ParkingTrajectoryTracker::distance(double x0, double y0,
                                           double x1, double y1)
{
    return std::hypot(x1 - x0, y1 - y0);
}

std::size_t ParkingTrajectoryTracker::findNearestIndex(
    const VehiclePoseMsg& pose,
    const ParkingTrajectoryMsg& trajectory)
{
    if (trajectory.points.empty()) return 0;

    if (trajectory.trajectory_id != active_trajectory_id_) {
        active_trajectory_id_ = trajectory.trajectory_id;
        progress_index_ = 0;
    }

    // Search forward from a small window behind the last progress point. This
    // prevents a self-crossing parking path from jumping to a later branch.
    const std::size_t begin = progress_index_ > 2 ? progress_index_ - 2 : 0;
    const std::size_t search_ahead = static_cast<std::size_t>(
        std::max(config_.parking.tracker_nearest_search_points, 4));
    const std::size_t end = std::min(trajectory.points.size(),
                                     progress_index_ + search_ahead + 1);

    std::size_t best = begin;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = begin; i < end; ++i) {
        const auto& p = trajectory.points[i];
        const double d = distance(pose.x_m, pose.y_m, p.x_m, p.y_m);
        if (d < best_distance) {
            best_distance = d;
            best = i;
        }
    }

    // Progress is monotonic within one trajectory. Allow the nearest search to
    // look slightly behind for noise, but never roll the tracker backward.
    if (best > progress_index_) progress_index_ = best;
    return progress_index_;
}

std::size_t ParkingTrajectoryTracker::findLookaheadIndex(
    std::size_t nearest_index,
    const ParkingTrajectoryMsg& trajectory,
    MotionDirection direction) const
{
    if (trajectory.points.empty()) return 0;

    const double lookahead_m = std::max(config_.parking.tracker_lookahead_m, 0.03);
    double accumulated = 0.0;
    std::size_t target = nearest_index;

    for (std::size_t i = nearest_index + 1; i < trajectory.points.size(); ++i) {
        // A direction change is a cusp. Never make Pure Pursuit look through a
        // cusp into the next motion primitive; the vehicle must reach the cusp
        // first, then the tracker can switch direction on a later tick.
        if (trajectory.points[i].direction != direction) break;

        accumulated += distance(trajectory.points[i - 1].x_m,
                                trajectory.points[i - 1].y_m,
                                trajectory.points[i].x_m,
                                trajectory.points[i].y_m);
        target = i;
        if (accumulated >= lookahead_m) break;
    }

    return target;
}

bool ParkingTrajectoryTracker::computeSteering(
    const VehiclePoseMsg& pose,
    const ParkingTrajectoryPoint& target,
    MotionDirection direction,
    float& steering_deg,
    double& target_distance_m) const
{
    steering_deg = 0.0f;

    const double dx = target.x_m - pose.x_m;
    const double dy = target.y_m - pose.y_m;
    target_distance_m = std::hypot(dx, dy);
    if (!std::isfinite(target_distance_m) || target_distance_m < kFiniteEpsilon)
        return false;

    // Pure Pursuit geometry is expressed in the direction of travel. For
    // reverse motion the travel heading is vehicle yaw + pi. Bicycle steering
    // must then be sign-inverted because v is negative in yaw_dot=v/L*tan(delta).
    const double motion_yaw = normalizeAngle(
        pose.yaw_rad + (direction == MotionDirection::REVERSE ? kPi : 0.0));
    const double lateral_left = -std::sin(motion_yaw) * dx +
                                 std::cos(motion_yaw) * dy;

    const double wheelbase = static_cast<double>(config_.vehicle.wheelbase_m);
    if (!std::isfinite(wheelbase) || wheelbase <= 0.01) return false;

    double curvature = 2.0 * lateral_left /
                       (target_distance_m * target_distance_m);
    if (direction == MotionDirection::REVERSE) curvature = -curvature;

    double delta_rad = std::atan(wheelbase * curvature);
    double delta_deg = delta_rad * 180.0 / kPi;
    if (!std::isfinite(delta_deg)) return false;

    const double limit = std::max(
        static_cast<double>(config_.vehicle.steering_limit_deg), 0.0);
    delta_deg = std::max(-limit, std::min(delta_deg, limit));
    steering_deg = static_cast<float>(delta_deg);
    return true;
}

bool ParkingTrajectoryTracker::process(const VehiclePoseMsg& pose,
                                       const ParkingTrajectoryMsg& trajectory,
                                       ControlCmdMsg& command,
                                       ParkingTrackerDebug* debug)
{
    command = ControlCmdMsg{};
    command.header.timestamp_ms = nowMs();
    if (debug) *debug = ParkingTrackerDebug{};

    if (!config_.parking.enable || !config_.parking.bench_mode ||
        !config_.parking.enable_bench_tracker || config_.runtime.enable_uart) {
        return false;
    }
    if (!pose.header.valid ||
        !isFresh(nowMs(), pose.header.timestamp_ms,
                 config_.parking.tracker_pose_timeout_ms) ||
        pose.map_id != config_.parking.map_id ||
        !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
        !std::isfinite(pose.yaw_rad)) {
        return false;
    }
    if (!trajectory.header.valid || trajectory.trajectory_id == 0 ||
        trajectory.map_id != config_.parking.map_id ||
        trajectory.reference_point != "rear_axle_center" ||
        trajectory.validation != "PASS" || trajectory.points.size() < 2) {
        return false;
    }

    const std::size_t nearest = findNearestIndex(pose, trajectory);
    const auto& nearest_point = trajectory.points[nearest];
    const double nearest_distance = distance(pose.x_m, pose.y_m,
                                             nearest_point.x_m,
                                             nearest_point.y_m);
    if (!std::isfinite(nearest_distance) ||
        nearest_distance > config_.parking.tracker_max_cross_track_error_m) {
        return false;
    }

    const auto& goal = trajectory.points.back();
    const double goal_distance = distance(pose.x_m, pose.y_m, goal.x_m, goal.y_m);
    const double goal_yaw_error = std::fabs(normalizeAngle(goal.yaw_rad - pose.yaw_rad));
    const bool goal_reached =
        nearest >= trajectory.points.size() - 2 &&
        goal_distance <= config_.parking.tracker_goal_position_tolerance_m &&
        goal_yaw_error <= config_.parking.tracker_goal_yaw_tolerance_rad;

    if (goal_reached) {
        command.header.valid = true;
        command.speed_mps = 0.0f;
        command.steering_deg = 0.0f;
        if (debug) {
            debug->valid = true;
            debug->goal_reached = true;
            debug->trajectory_id = trajectory.trajectory_id;
            debug->nearest_index = nearest;
            debug->target_index = trajectory.points.size() - 1;
            debug->nearest_distance_m = nearest_distance;
            debug->target_distance_m = goal_distance;
            debug->direction = nearest_point.direction;
        }
        return true;
    }

    const MotionDirection direction = nearest_point.direction;
    const std::size_t target_index = findLookaheadIndex(nearest, trajectory, direction);
    if (target_index == nearest && nearest + 1 < trajectory.points.size() &&
        trajectory.points[nearest + 1].direction != direction) {
        // We are at a direction-change cusp. Require proximity before allowing
        // the next tick to progress into the new primitive; otherwise command a
        // deliberate stop rather than steering across the cusp.
        if (nearest_distance <= config_.parking.tracker_cusp_position_tolerance_m) {
            progress_index_ = nearest + 1;
        }
        command.header.valid = true;
        command.speed_mps = 0.0f;
        command.steering_deg = 0.0f;
        if (debug) {
            debug->valid = true;
            debug->trajectory_id = trajectory.trajectory_id;
            debug->nearest_index = nearest;
            debug->target_index = nearest;
            debug->nearest_distance_m = nearest_distance;
            debug->direction = direction;
        }
        return true;
    }

    if (target_index == nearest) return false;

    float steering_deg = 0.0f;
    double target_distance = 0.0;
    if (!computeSteering(pose, trajectory.points[target_index], direction,
                         steering_deg, target_distance)) {
        return false;
    }

    float speed = nearest_point.v_ref_mps;
    const float max_speed = std::max(config_.parking.max_parking_speed_mps, 0.0f);
    speed = std::max(-max_speed, std::min(speed, max_speed));

    // Keep the sign contract explicit even if an upstream bug bypassed the
    // trajectory validator.
    if ((direction == MotionDirection::FORWARD && speed < 0.0f) ||
        (direction == MotionDirection::REVERSE && speed > 0.0f)) {
        return false;
    }

    command.header.valid = true;
    command.speed_mps = speed;
    command.steering_deg = steering_deg;
    command.servo_cmd = 0;  // bench tracker does not own servo conversion/UART

    if (debug) {
        debug->valid = true;
        debug->trajectory_id = trajectory.trajectory_id;
        debug->nearest_index = nearest;
        debug->target_index = target_index;
        debug->nearest_distance_m = nearest_distance;
        debug->target_distance_m = target_distance;
        debug->direction = direction;
    }
    return true;
}

}  // namespace laas
