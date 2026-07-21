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
    int saved_raw_count_ = 0;

    bool openCameraDevice();
    cv::Mat undistortAndResize(const cv::Mat& input) const;
};

}  // namespace laas
