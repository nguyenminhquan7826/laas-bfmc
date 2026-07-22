#include "MpcController.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

namespace laas {

namespace {
constexpr float kDistanceCameraToAxleM = 0.15f;
constexpr float kDefaultBirdEyeWidth = 640.0f;
constexpr float kDefaultBirdEyeHeight = 480.0f;
constexpr float kMinVelocityForModel = 0.05f;
constexpr double kPi = 3.14159265358979323846;
}

MpcController::MpcController()
    : wheelbase_(0.2515f),
      mass_(2.3f),
      lf_(0.132f),
      lr_(0.12f),
      caf_(0.04f),
      car_(0.02f),
      iz_(0.04f),
      horizon_(10),
      q_lateral_(1500.0f),
      q_yaw_(120.0f),
      r_steering_(5.0f),
      sample_time_s_(0.071f),
      initialized_(false),
      steering_min_rad_(-28.0 * kPi / 180.0),
      steering_max_rad_(28.0 * kPi / 180.0),
      solver_(nullptr),
      solver_initialized_(false),
      previous_z_dim_(-1),
      previous_constraint_dim_(-1),
      cached_velocity_mps_(-1.0f),
      meter_per_pixel_(0.001f),
      vehicle_x_px_(0.0f),
      vehicle_y_px_(0.0f)
{
}

void MpcController::setVehicleParams(float wheelbase,
                                     float mass,
                                     float lf,
                                     float lr,
                                     float caf,
                                     float car,
                                     float iz)
{
    if (wheelbase > 0.01f) wheelbase_ = wheelbase;
    if (mass > 0.01f) mass_ = mass;
    if (lf > 0.0f) lf_ = lf;
    if (lr > 0.0f) lr_ = lr;
    if (caf > 0.0f) caf_ = caf;
    if (car > 0.0f) car_ = car;
    if (iz > 0.0f) iz_ = iz;

    initialized_ = false;
    solver_initialized_ = false;
}

void MpcController::setPredictionHorizon(int horizon)
{
    if (horizon >= 2) {
        horizon_ = horizon;
        initialized_ = false;
        solver_initialized_ = false;
        previous_z_dim_ = -1;
        previous_constraint_dim_ = -1;
    }
}

void MpcController::setVehiclePosition(float x, float y)
{
    vehicle_x_px_ = x;
    vehicle_y_px_ = y;
}

void MpcController::setMeterPerPixel(float meter_per_pixel)
{
    if (meter_per_pixel > 1e-6f && std::isfinite(meter_per_pixel)) {
        meter_per_pixel_ = meter_per_pixel;
    }
}

Eigen::MatrixXd MpcController::matrixPower(const Eigen::MatrixXd& a, int power) const
{
    if (power == 0) {
        return Eigen::MatrixXd::Identity(a.rows(), a.cols());
    }
    if (power == 1) {
        return a;
    }

    Eigen::MatrixXd result = a;
    for (int i = 1; i < power; ++i) {
        result = result * a;
    }
    return result;
}

void MpcController::init(float q_lateral, float q_yaw, float r_steering)
{
    q_lateral_ = q_lateral;
    q_yaw_ = q_yaw;
    r_steering_ = r_steering;

    buildMpcMatrices(kMinVelocityForModel);

    if (!solver_) {
        solver_ = std::make_unique<OsqpEigen::Solver>();
    }
    solver_->settings()->setVerbosity(false);
    solver_->settings()->setWarmStart(true);
    solver_->settings()->setMaxIteration(4000);
    solver_->settings()->setAbsoluteTolerance(1e-4);
    solver_->settings()->setRelativeTolerance(1e-4);

    initialized_ = true;
}

void MpcController::buildMpcMatrices(float vx_mps)
{
    const float vx = std::max(std::abs(vx_mps), kMinVelocityForModel);

    Eigen::MatrixXd a_c(4, 4);
    a_c << 0.0, 1.0, 0.0, 0.0,
           0.0, -(2.0 * caf_ + 2.0 * car_) / (mass_ * vx),
                (2.0 * caf_ + 2.0 * car_) / mass_,
                (-2.0 * caf_ * lf_ + 2.0 * car_ * lr_) / (mass_ * vx),
           0.0, 0.0, 0.0, 1.0,
           0.0, (-2.0 * caf_ * lf_ + 2.0 * car_ * lr_) / (iz_ * vx),
                (2.0 * caf_ * lf_ - 2.0 * car_ * lr_) / iz_,
                (-2.0 * caf_ * lf_ * lf_ - 2.0 * car_ * lr_ * lr_) / (iz_ * vx);

    Eigen::MatrixXd b_c(4, 2);
    b_c << 0.0, 0.0,
           2.0 * caf_ / mass_, (-2.0 * caf_ * lf_ + 2.0 * car_ * lr_) / (mass_ * vx) - vx,
           0.0, 0.0,
           2.0 * caf_ * lf_ / iz_, (-2.0 * caf_ * lf_ * lf_ - 2.0 * car_ * lr_ * lr_) / (iz_ * vx);

    Eigen::MatrixXd augmented(6, 6);
    augmented.setZero();
    augmented.block(0, 0, 4, 4) = a_c;
    augmented.block(0, 4, 4, 2) = b_c;
    augmented *= sample_time_s_;

    const Eigen::MatrixXd augmented_d = augmented.exp();
    a_d_ = augmented_d.block(0, 0, 4, 4);
    const Eigen::MatrixXd b_d = augmented_d.block(0, 4, 4, 2);
    b1_d_ = b_d.col(0);
    b2_d_ = b_d.col(1);

    const int state_dim = 4;
    const int input_dim = 1;

    ax_ = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim, state_dim);
    for (int i = 0; i <= horizon_; ++i) {
        ax_.block(i * state_dim, 0, state_dim, state_dim) = matrixPower(a_d_, i);
    }

    bu_ = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim, horizon_ * input_dim);
    for (int i = 1; i <= horizon_; ++i) {
        for (int j = 0; j < i; ++j) {
            bu_.block(i * state_dim, j * input_dim, state_dim, input_dim) =
                matrixPower(a_d_, i - j - 1) * b1_d_;
        }
    }

    bv_ = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim, horizon_ * input_dim);
    for (int i = 1; i <= horizon_; ++i) {
        for (int j = 0; j < i; ++j) {
            bv_.block(i * state_dim, j * input_dim, state_dim, input_dim) =
                matrixPower(a_d_, i - j - 1) * b2_d_;
        }
    }

    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(4, 4);
    q(0, 0) = q_lateral_;
    q(2, 2) = q_yaw_;
    const Eigen::MatrixXd q_terminal = q;
    const Eigen::MatrixXd r = Eigen::MatrixXd::Identity(1, 1) * r_steering_;

    Eigen::MatrixXd qx = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim,
                                               (horizon_ + 1) * state_dim);
    for (int i = 0; i < horizon_; ++i) {
        qx.block(i * state_dim, i * state_dim, state_dim, state_dim) = q;
    }
    qx.block(horizon_ * state_dim, horizon_ * state_dim, state_dim, state_dim) = q_terminal;

    Eigen::MatrixXd ru = Eigen::MatrixXd::Zero(horizon_ * input_dim, horizon_ * input_dim);
    for (int i = 0; i < horizon_; ++i) {
        ru.block(i * input_dim, i * input_dim, input_dim, input_dim) = r;
    }

    hessian_ = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim + horizon_ * input_dim,
                                     (horizon_ + 1) * state_dim + horizon_ * input_dim);
    hessian_.block(0, 0, (horizon_ + 1) * state_dim, (horizon_ + 1) * state_dim) = qx;
    hessian_.block((horizon_ + 1) * state_dim,
                   (horizon_ + 1) * state_dim,
                   horizon_ * input_dim,
                   horizon_ * input_dim) = ru;
    hessian_ += Eigen::MatrixXd::Identity(hessian_.rows(), hessian_.cols()) * 1e-6;
}

void MpcController::debugMatrices() const
{
    std::cout << "[MPC DEBUG] A_d: " << a_d_.rows() << "x" << a_d_.cols() << std::endl;
    std::cout << "[MPC DEBUG] B1_d: " << b1_d_.rows() << "x" << b1_d_.cols() << std::endl;
    std::cout << "[MPC DEBUG] B2_d: " << b2_d_.rows() << "x" << b2_d_.cols() << std::endl;
    std::cout << "[MPC DEBUG] H: " << hessian_.rows() << "x" << hessian_.cols() << std::endl;
}

float MpcController::solveQP(const Eigen::VectorXd& x0,
                             const Eigen::VectorXd& curvature_feedforward)
{
    const int state_dim = 4;
    const int input_dim = 1;
    const int z_dim = (horizon_ + 1) * state_dim + horizon_ * input_dim;
    const int constraint_dim = (horizon_ + 1) * state_dim;

    if (x0.size() != state_dim || curvature_feedforward.size() != horizon_) {
        std::cerr << "[MPC] Invalid input sizes." << std::endl;
        return 0.0f;
    }

    const Eigen::MatrixXd g = hessian_;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(z_dim);

    Eigen::MatrixXd equality = Eigen::MatrixXd::Zero((horizon_ + 1) * state_dim, z_dim);
    equality.block(0, 0,
                   (horizon_ + 1) * state_dim,
                   (horizon_ + 1) * state_dim) =
        Eigen::MatrixXd::Identity((horizon_ + 1) * state_dim,
                                  (horizon_ + 1) * state_dim);
    equality.block(0,
                   (horizon_ + 1) * state_dim,
                   (horizon_ + 1) * state_dim,
                   horizon_ * input_dim) = -bu_;

    const Eigen::VectorXd equality_rhs = ax_ * x0 + bv_ * curvature_feedforward;

    if (!g.allFinite() || !equality.allFinite() || !equality_rhs.allFinite()) {
        std::cerr << "[MPC] Matrix contains NaN/Inf." << std::endl;
        return 0.0f;
    }

    const Eigen::SparseMatrix<double> hessian_sparse = g.sparseView();
    const Eigen::SparseMatrix<double> equality_sparse = equality.sparseView();

    Eigen::VectorXd lower_bound = equality_rhs;
    Eigen::VectorXd upper_bound = equality_rhs;

    if (!solver_initialized_ ||
        previous_z_dim_ != z_dim ||
        previous_constraint_dim_ != constraint_dim) {
        solver_ = std::make_unique<OsqpEigen::Solver>();
        solver_->settings()->setVerbosity(false);
        solver_->settings()->setWarmStart(true);
        solver_->settings()->setMaxIteration(4000);
        solver_->settings()->setAbsoluteTolerance(1e-4);
        solver_->settings()->setRelativeTolerance(1e-4);

        solver_->data()->setNumberOfVariables(z_dim);
        solver_->data()->setNumberOfConstraints(constraint_dim);

        if (!solver_->data()->setHessianMatrix(hessian_sparse) ||
            !solver_->data()->setGradient(gradient) ||
            !solver_->data()->setLinearConstraintsMatrix(equality_sparse) ||
            !solver_->data()->setLowerBound(lower_bound) ||
            !solver_->data()->setUpperBound(upper_bound)) {
            std::cerr << "[MPC] Failed to set solver data." << std::endl;
            return 0.0f;
        }

        if (!solver_->initSolver()) {
            std::cerr << "[MPC] Failed to initialize solver." << std::endl;
            return 0.0f;
        }

        solver_initialized_ = true;
        previous_z_dim_ = z_dim;
        previous_constraint_dim_ = constraint_dim;
    } else {
        if (!solver_->updateHessianMatrix(hessian_sparse) ||
            !solver_->updateGradient(gradient) ||
            !solver_->updateBounds(lower_bound, upper_bound)) {
            std::cerr << "[MPC] Failed to update solver." << std::endl;
            return 0.0f;
        }
    }

    if (solver_->solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "[MPC] Solve failed." << std::endl;
        return 0.0f;
    }

    const Eigen::VectorXd z_opt = solver_->getSolution();
    if (z_opt.size() != z_dim || !z_opt.allFinite()) {
        std::cerr << "[MPC] Invalid solution." << std::endl;
        return 0.0f;
    }

    double u_cmd = z_opt((horizon_ + 1) * state_dim);
    u_cmd = std::min(std::max(u_cmd, steering_min_rad_), steering_max_rad_);
    return static_cast<float>(u_cmd * 180.0 / kPi);
}

float MpcController::computeSteeringAngle(const MpcState& state, float velocity_mps)
{
    if (!initialized_) {
        std::cerr << "[MPC] Not initialized." << std::endl;
        return 0.0f;
    }

    const float vx = std::max(std::abs(velocity_mps), kMinVelocityForModel);
    if (std::abs(vx - cached_velocity_mps_) > 0.01f) {
        buildMpcMatrices(vx);
        cached_velocity_mps_ = vx;
        solver_initialized_ = false;
        previous_z_dim_ = -1;
        previous_constraint_dim_ = -1;
    }

    if (!state.is_valid || state.curvature.size() < static_cast<size_t>(horizon_)) {
        return 0.0f;
    }

    Eigen::VectorXd x0(4);
    x0(0) = state.lateral_deviation;
    x0(1) = 0.0;
    x0(2) = state.yaw_angle;
    x0(3) = 0.0;

    Eigen::VectorXd curvature_feedforward(horizon_);
    for (int i = 0; i < horizon_; ++i) {
        curvature_feedforward(i) = state.curvature[static_cast<size_t>(i)] * vx;
    }

    return solveQP(x0, curvature_feedforward);
}

cv::Vec3f MpcController::fitCenterlinePoly(const std::vector<cv::Point>& centerline) const
{
    if (centerline.size() < 3) {
        return cv::Vec3f(0.0f, 0.0f, 0.0f);
    }

    std::vector<float> x_values;
    std::vector<float> y_values;
    x_values.reserve(centerline.size());
    y_values.reserve(centerline.size());

    for (const auto& point : centerline) {
        x_values.push_back(static_cast<float>(point.x));
        y_values.push_back(static_cast<float>(point.y));
    }

    cv::Mat y_mat(static_cast<int>(y_values.size()), 1, CV_32F, y_values.data());
    cv::Mat x_mat(static_cast<int>(x_values.size()), 1, CV_32F, x_values.data());

    cv::Mat a(y_mat.rows, 3, CV_32F);
    for (int i = 0; i < y_mat.rows; ++i) {
        const float y = y_mat.at<float>(i, 0);
        a.at<float>(i, 0) = y * y;
        a.at<float>(i, 1) = y;
        a.at<float>(i, 2) = 1.0f;
    }

    cv::Mat coeffs;
    const bool ok = cv::solve(a, x_mat, coeffs, cv::DECOMP_SVD);
    if (!ok) {
        return cv::Vec3f(0.0f, 0.0f, 0.0f);
    }

    return cv::Vec3f(coeffs.at<float>(0),
                     coeffs.at<float>(1),
                     coeffs.at<float>(2));
}

std::vector<float> MpcController::computeMultipleCurvatures(const cv::Vec3f& coeffs,
                                                            int horizon) const
{
    std::vector<float> curvatures;
    curvatures.reserve(static_cast<size_t>(horizon));

    const float a = coeffs[0];
    const float b = coeffs[1];
    const float y_base = (vehicle_y_px_ > 0.0f) ? vehicle_y_px_ : (kDefaultBirdEyeHeight - 1.0f);

    for (int i = 0; i < horizon; ++i) {
        const float y = y_base - i * 30.0f;
        const float dx_dy = 2.0f * a * y + b;
        const float d2x_dy2 = 2.0f * a;

        const float numerator = std::abs(d2x_dy2);
        const float denominator = std::pow(1.0f + dx_dy * dx_dy, 1.5f);
        const float kappa_pixel = (denominator > 1e-6f) ? numerator / denominator : 0.0f;
        const float kappa_meter = kappa_pixel / std::max(meter_per_pixel_, 1e-6f);

        curvatures.push_back(kappa_meter);
    }

    return curvatures;
}

float MpcController::computeLateralDeviation(const cv::Vec3f& coeffs,
                                             const cv::Mat& bird_eye_view)
{
    if (vehicle_x_px_ == 0.0f && vehicle_y_px_ == 0.0f) {
        vehicle_x_px_ = bird_eye_view.empty() ? kDefaultBirdEyeWidth * 0.5f
                                              : bird_eye_view.cols * 0.5f;
        vehicle_y_px_ = bird_eye_view.empty() ? kDefaultBirdEyeHeight - 1.0f
                                              : bird_eye_view.rows - 1.0f;
    }

    const float centerline_x = coeffs[0] * vehicle_y_px_ * vehicle_y_px_ +
                               coeffs[1] * vehicle_y_px_ +
                               coeffs[2];

    const float lateral_deviation_pixel = vehicle_x_px_ - centerline_x;
    return lateral_deviation_pixel * meter_per_pixel_;
}

float MpcController::computeYawAngle(const cv::Vec3f& coeffs, float y) const
{
    const float dx_dy = 2.0f * coeffs[0] * y + coeffs[1];
    return std::atan(dx_dy);
}

MpcState MpcController::computeMpcParameters(const std::vector<cv::Point>& centerline,
                                             const cv::Mat& bird_eye_view)
{
    MpcState result(horizon_);

    if (centerline.size() < 3) {
        result.is_valid = false;
        return result;
    }

    const cv::Vec3f coeffs = fitCenterlinePoly(centerline);

    if (vehicle_x_px_ == 0.0f && vehicle_y_px_ == 0.0f) {
        vehicle_x_px_ = bird_eye_view.empty() ? kDefaultBirdEyeWidth * 0.5f
                                              : bird_eye_view.cols * 0.5f;
        vehicle_y_px_ = bird_eye_view.empty() ? kDefaultBirdEyeHeight - 1.0f
                                              : bird_eye_view.rows - 1.0f;
    }

    result.curvature = computeMultipleCurvatures(coeffs, horizon_);
    result.yaw_angle = computeYawAngle(coeffs, vehicle_y_px_);

    const float raw_deviation = computeLateralDeviation(coeffs, bird_eye_view);
    result.lateral_deviation = raw_deviation - kDistanceCameraToAxleM * std::sin(result.yaw_angle);
    result.is_valid = true;
    return result;
}

}  // namespace laas
