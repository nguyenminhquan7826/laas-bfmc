#include "MissionSupervisor.hpp"

#include <cmath>

namespace laas {

MissionSupervisor::MissionSupervisor(const Config& config)
    : config_(config)
{
}

BehaviorRequest MissionSupervisor::update(const LanePerceptionMsg& lane,
                                          const ObstacleMsg& obstacle,
                                          bool user_run_request)
{
    const uint64_t now = nowMs();

    BehaviorRequest req;
    req.header.timestamp_ms = now;
    req.header.valid = true;
    req.control_mode = config_.runtime.control_mode;

    if (!user_run_request) {
        // A deliberate STOP acts as the acknowledgement/reset for a latched
        // emergency. The operator must press S and then R before motion can
        // resume after any emergency-stop condition.
        emergency_stop_latched_ = false;
        req.mode = BehaviorMode::STOP;
        req.allow_lane_change = false;
        req.desired_speed_mps = 0.0f;
        return req;
    }

    if (emergency_stop_latched_) {
        req.mode = BehaviorMode::EMERGENCY_STOP;
        req.allow_lane_change = false;
        req.desired_speed_mps = 0.0f;
        return req;
    }

    const bool lane_fresh = lane.header.valid &&
                            isFresh(now, lane.header.timestamp_ms, config_.runtime.lane_timeout_ms) &&
                            lane.centerline.size() >= 3;

    if (!lane_fresh) {
        emergency_stop_latched_ = true;
        req.mode = BehaviorMode::EMERGENCY_STOP;
        req.allow_lane_change = false;
        req.desired_speed_mps = 0.0f;
        return req;
    }

    const bool obstacle_fresh = obstacle.header.valid &&
                                isFresh(now, obstacle.header.timestamp_ms, config_.runtime.obstacle_timeout_ms);

    // A fresh packet must also have a self-consistent payload. The Python
    // detector currently uses a non-positive distance to mean "no obstacle".
    const bool obstacle_payload_valid =
        std::isfinite(obstacle.distance_m) &&
        ((obstacle.has_obstacle && obstacle.distance_m > 0.05f) ||
         (!obstacle.has_obstacle && obstacle.distance_m <= 0.05f));

    // When obstacle detection is enabled, loss of the detector/UDP stream or
    // a malformed payload is safety-critical. Do not silently treat it as an
    // empty road.
    if (config_.runtime.enable_yolo_udp &&
        (!obstacle_fresh || !obstacle_payload_valid)) {
        emergency_stop_latched_ = true;
        req.mode = BehaviorMode::EMERGENCY_STOP;
        req.allow_lane_change = false;
        req.desired_speed_mps = 0.0f;
        return req;
    }

    const bool obstacle_detected = obstacle_fresh &&
                                   obstacle_payload_valid &&
                                   obstacle.has_obstacle;

    if (obstacle_detected &&
        obstacle.distance_m <= config_.planner.emergency_stop_distance_m) {
        emergency_stop_latched_ = true;
        req.mode = BehaviorMode::EMERGENCY_STOP;
        req.allow_lane_change = false;
        req.desired_speed_mps = 0.0f;
        return req;
    }

    const bool obstacle_in_trigger = obstacle_detected &&
                                     obstacle.distance_m <= config_.planner.trigger_distance_m;

    if (obstacle_in_trigger) {
        req.mode = BehaviorMode::AVOID_OBSTACLE;
        req.allow_lane_change = true;
        req.desired_speed_mps = config_.vehicle.desired_speed_mps;
        return req;
    }

    req.mode = BehaviorMode::KEEP_LANE;
    req.allow_lane_change = false;
    req.desired_speed_mps = config_.vehicle.desired_speed_mps;
    return req;
}

}  // namespace laas
