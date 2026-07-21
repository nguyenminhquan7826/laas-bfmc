#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace laas {

struct MpcState {
    std::vector<float> curvature;
    float lateral_deviation = 0.0f;
    float yaw_angle = 0.0f;
    bool is_valid = false;

    explicit MpcState(int horizon = 10)
        : curvature(static_cast<size_t>(horizon), 0.0f)
    {
    }
};

class MpcController {
public:
    MpcController();

    void init(float q_lateral, float q_yaw, float r_steering);
    void debugMatrices() const;

    MpcState computeMpcParameters(const std::vector<cv::Point>& centerline,
                                  const cv::Mat& bird_eye_view);
    float computeSteeringAngle(const MpcState& state, float velocity_mps);

    void setVehicleParams(float wheelbase,
                          float mass,
                          float lf,
                          float lr,
                          float caf,
                          float car,
                          float iz);
    void setPredictionHorizon(int horizon);
    void setVehiclePosition(float x, float y);
    void setMeterPerPixel(float meter_per_pixel);

private:
    void buildMpcMatrices(float vx_mps);
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& a, int power) const;
    float solveQP(const Eigen::VectorXd& x0, const Eigen::VectorXd& curvature_feedforward);

    cv::Vec3f fitCenterlinePoly(const std::vector<cv::Point>& centerline) const;
    std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int horizon) const;
    float computeLateralDeviation(const cv::Vec3f& coeffs, const cv::Mat& bird_eye_view);
    float computeYawAngle(const cv::Vec3f& coeffs, float y) const;

private:
    float wheelbase_;
    float mass_;
    float lf_;
    float lr_;
    float caf_;
    float car_;
    float iz_;

    int horizon_;
    float q_lateral_;
    float q_yaw_;
    float r_steering_;
    float sample_time_s_;
    bool initialized_;

    Eigen::MatrixXd a_d_;
    Eigen::MatrixXd b1_d_;
    Eigen::MatrixXd b2_d_;
    Eigen::MatrixXd ax_;
    Eigen::MatrixXd bu_;
    Eigen::MatrixXd bv_;
    Eigen::MatrixXd hessian_;

    double steering_min_rad_;
    double steering_max_rad_;
    std::unique_ptr<OsqpEigen::Solver> solver_;
    bool solver_initialized_;
    int previous_z_dim_;
    int previous_constraint_dim_;
    float cached_velocity_mps_;

    // Historical code used the name pixel_per_meter_ while storing meter/pixel.
    // The refactored controller keeps the behavior but names the value correctly.
    float meter_per_pixel_;
    float vehicle_x_px_;
    float vehicle_y_px_;
};

}  // namespace laas
