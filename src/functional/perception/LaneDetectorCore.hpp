#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "../../laas_core/Messages.hpp"

namespace laas {

class LaneDetectorCore {
public:
    LaneDetectorCore(int width = 640,
                     int height = 480,
                     float lane_width_m = 0.40f,
                     float bev_forward_range_m = 2.0f);
    ~LaneDetectorCore() = default;

    void processFrame(cv::Mat& frame_resize);

    cv::Mat getMask() const { return mask; }
    cv::Mat getBirdEyeView() const { return bird_eye_view; }
    std::vector<cv::Point> getCenterline() const { return centerline; }
    bool hasValidLane() const { return has_valid_lane_; }

    cv::Vec3f getLeftCoeffs() const { return left_coeffs_; }
    cv::Vec3f getRightCoeffs() const { return right_coeffs_; }
    bool hasLeftLane() const { return has_left_lane_; }
    bool hasRightLane() const { return has_right_lane_; }
    float getLaneWidthPx();

    LaneLineType left_type = LaneLineType::UNKNOWN;
    LaneLineType right_type = LaneLineType::UNKNOWN;

    void resetTracking();

private:
    int width;
    int height;
    float expected_lane_width_px_ = 96.0f;
    bool initialized = false;

    cv::Vec3f prev_left_{0.0f, 0.0f, 0.0f};
    cv::Vec3f prev_right_{0.0f, 0.0f, 0.0f};
    bool has_prev_left_ = false;
    bool has_prev_right_ = false;

    std::vector<cv::Point> centerline;
    cv::Mat bird_eye_view;
    cv::Mat mask;
    bool has_valid_lane_ = false;

    cv::Vec3f left_coeffs_{0.0f, 0.0f, 0.0f};
    cv::Vec3f right_coeffs_{0.0f, 0.0f, 0.0f};
    bool has_left_lane_ = false;
    bool has_right_lane_ = false;
    float lane_width_px_ = 96.0f;

    cv::Mat applyIPM(const cv::Mat& frame);
    cv::Mat processMask(const cv::Mat& bird_eye_view);

    void slidingWindow(const cv::Mat& mask,
                       std::vector<cv::Point>& left_points,
                       std::vector<cv::Point>& right_points,
                       cv::Mat& outImg,
                       int minpix);

    void slidingWindowAdaptive(const cv::Mat& mask,
                               std::vector<cv::Point>& lane_points,
                               cv::Mat& outImg,
                               cv::Vec3f prev_poly,
                               bool isLeft);

    cv::Vec3f fitPoly(const std::vector<cv::Point>& points,
                      cv::Mat& outImg,
                      bool isLeft);

    std::vector<cv::Point> computeCenterline(cv::Vec3f coeff_left,
                                             cv::Vec3f coeff_right,
                                             bool has_left,
                                             bool has_right,
                                             cv::Mat& outImg);

    float computeLaneSlope(const cv::Vec3f& coeffs, float y);

    LaneLineType classifyLaneMarking(const cv::Mat& mask,
                                     const cv::Vec3f& coeff,
                                     int band_half_width = 12,
                                     int y_step = 4,
                                     int min_segment_len = 4,
                                     int min_gap_len = 4);
};

}  // namespace laas
