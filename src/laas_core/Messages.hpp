#pragma once

#include <cstdint>
#include <vector>
#include <opencv2/opencv.hpp>

namespace laas {

enum class LaneLineType { UNKNOWN = 0, SOLID, DASHED };
enum class ControlMode { MPC = 0, PURE_PURSUIT };
enum class BehaviorMode {
    STOP = 0,
    KEEP_LANE,
    FOLLOW_LANE,
    AVOID_OBSTACLE,
    EMERGENCY_STOP
};
enum class PlannerState { KEEP_LANE = 0, CHANGE_USING_DASHED, FOLLOW_LANE };
enum class ChangeDirection { NONE = 0, LEFT, RIGHT };

struct Header {
    std::uint64_t timestamp_ms = 0;
    bool valid = false;
};

struct FrameMsg {
    Header header;
    cv::Mat frame_bgr;
};

struct LanePerceptionMsg {
    Header header;
    cv::Mat bird_eye_view;
    cv::Mat mask;
    std::vector<cv::Point> centerline;
    cv::Vec3f left_coeffs = cv::Vec3f(0.0F, 0.0F, 0.0F);
    cv::Vec3f right_coeffs = cv::Vec3f(0.0F, 0.0F, 0.0F);
    bool has_left_lane = false;
    bool has_right_lane = false;
    LaneLineType left_type = LaneLineType::UNKNOWN;
    LaneLineType right_type = LaneLineType::UNKNOWN;
    float lane_width_px = 0.0F;
};

struct ObstacleMsg {
    Header header;
    bool has_obstacle = false;
    float distance_m = -1.0F;
    float confidence = 0.0F;
};

struct BehaviorRequest {
    Header header;
    BehaviorMode mode = BehaviorMode::STOP;
    ControlMode control_mode = ControlMode::MPC;
    bool allow_lane_change = false;
    float desired_speed_mps = 0.0F;
};

struct TrajectoryMsg {
    Header header;
    bool collision_free = false;
    std::vector<cv::Point> base_centerline;
    std::vector<cv::Point> target_centerline;
    BehaviorMode behavior = BehaviorMode::KEEP_LANE;
    PlannerState planner_state = PlannerState::KEEP_LANE;
    ChangeDirection direction = ChangeDirection::NONE;
    float min_distance_m = 999.0F;
    float min_ttc_s = 999.0F;
    float cost = 0.0F;
};

struct VehicleStateMsg {
    Header header;
    bool valid = false;
    float lateral_error_m = 0.0F;
    float yaw_error_rad = 0.0F;
    std::vector<float> curvature;
};

struct ControlCmdMsg {
    Header header;
    float speed_mps = 0.0F;
    float steering_deg = 0.0F;
    int servo_cmd = 0;
};

}  // namespace laas
