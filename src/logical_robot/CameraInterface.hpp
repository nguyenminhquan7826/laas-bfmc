#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"

namespace laas {

class CameraInterface {
public:
    explicit CameraInterface(const Config& config);
    ~CameraInterface();

    bool init();
    bool grab(FrameMsg& output);
    bool isOpened() const;
    void close();

private:
    Config config_;
    cv::VideoCapture cap_;
    bool initialized_ = false;

    // Camera calibration is fixed for the configured capture size. Build the
    // remap tables once during init instead of recomputing camera matrices and
    // distortion maps on every frame.
    cv::Mat undistort_map1_;
    cv::Mat undistort_map2_;
    cv::Rect undistort_valid_roi_;
    bool undistort_ready_ = false;

    bool openCameraDevice();
    bool prepareUndistortMaps();
    cv::Mat undistortAndResize(const cv::Mat& input) const;
};

}  // namespace laas
