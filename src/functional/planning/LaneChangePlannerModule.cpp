#include "LaneChangePlannerModule.hpp"

#include <algorithm>
#include <cmath>

#include "../../laas_core/Time.hpp"

namespace laas {

LaneChangePlannerModule::LaneChangePlannerModule(const Config& config)
    : config_(config)
{
    planner_.setLaneWidthMeters(config_.planner.lane_width_m);
    planner_.setVehicleSize(config_.planner.vehicle_width_m,
                            config_.planner.vehicle_length_m);
    planner_.setObstacleSize(config_.planner.obstacle_width_m,
                             config_.planner.obstacle_length_m);
    planner_.setSafeMargin(config_.planner.safe_margin_m);
    planner_.setTriggerDistance(config_.planner.trigger_distance_m);
    planner_.setSpeed(config_.vehicle.desired_speed_mps);

    const int planning_period_ms = std::max(1, config_.runtime.planning_period_ms);
    const int commit_frames = std::max(
        1,
        static_cast<int>(std::ceil(
            config_.planner.lane_change_commit_time_s * 1000.0f /
            static_cast<float>(planning_period_ms))));
    planner_.setLaneChangeCommitFrames(commit_frames);

    const float pi = 3.14159265358979323846f;
    const float steering_rad = config_.vehicle.steering_limit_deg * pi / 180.0f;
    if (config_.vehicle.wheelbase_m > 0.01f) {
        planner_.setMaxCurvature(
            std::tan(steering_rad) / config_.vehicle.wheelbase_m);
    }
}

float LaneChangePlannerModule::obstacleDistanceForPlanner(
    const ObstacleMsg& obstacle,
    const BehaviorRequest& behavior) const
{
    if (!behavior.allow_lane_change || behavior.mode != BehaviorMode::AVOID_OBSTACLE) {
        return -1.0f;
    }

    if (!obstacle.header.valid || !obstacle.has_obstacle) {
        return -1.0f;
    }

    if (obstacle.distance_m <= 0.05f) {
        return -1.0f;
    }

    return obstacle.distance_m;
}

bool LaneChangePlannerModule::process(const LanePerceptionMsg& lane,
                                      const ObstacleMsg& obstacle,
                                      const BehaviorRequest& behavior,
                                      TrajectoryMsg& trajectory)
{
    trajectory = TrajectoryMsg{};
    trajectory.header.timestamp_ms = nowMs();
    trajectory.behavior = behavior.mode;
    trajectory.base_centerline = lane.centerline;
    trajectory.target_centerline.clear();
    trajectory.collision_free = false;

    if (!lane.header.valid || lane.centerline.size() < 3) {
        trajectory.header.valid = false;
        return false;
    }

    if (behavior.mode == BehaviorMode::STOP ||
        behavior.mode == BehaviorMode::EMERGENCY_STOP) {
        planner_.reset();
        trajectory.header.valid = true;
        trajectory.collision_free = true;
        trajectory.target_centerline = lane.centerline;
        trajectory.planner_state = planner_.getState();
        trajectory.direction = planner_.getLastDirection();
        trajectory.min_distance_m = planner_.getLastMinDistance();
        trajectory.min_ttc_s = planner_.getLastMinTTC();
        trajectory.cost = planner_.getLastCost();
        return true;
    }

    planner_.setSpeed(std::max(behavior.desired_speed_mps, 0.0f));

    const float obstacle_distance = obstacleDistanceForPlanner(obstacle, behavior);

    const int img_width = !lane.bird_eye_view.empty()
        ? lane.bird_eye_view.cols
        : config_.camera.width;
    const int img_height = !lane.bird_eye_view.empty()
        ? lane.bird_eye_view.rows
        : config_.camera.height;

    std::vector<cv::Point> target = planner_.update(
        lane.centerline,
        lane.left_coeffs,
        lane.right_coeffs,
        lane.has_left_lane,
        lane.has_right_lane,
        lane.lane_width_px,
        lane.left_type,
        lane.right_type,
        obstacle_distance,
        img_width,
        img_height);

    trajectory.planner_state = planner_.getState();
    trajectory.direction = planner_.getLastDirection();
    trajectory.min_distance_m = planner_.getLastMinDistance();
    trajectory.min_ttc_s = planner_.getLastMinTTC();
    trajectory.cost = planner_.getLastCost();

    if (!planner_.hasCollisionFreeSolution() || target.size() < 3) {
        trajectory.header.valid = false;
        trajectory.collision_free = false;
        trajectory.target_centerline.clear();
        return false;
    }

    trajectory.target_centerline = std::move(target);
    trajectory.collision_free = true;
    trajectory.header.valid = true;

    return true;
}

}  // namespace laas
