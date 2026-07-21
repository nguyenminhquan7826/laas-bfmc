#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "../laas_core/Config.hpp"
#include "../laas_core/Messages.hpp"

namespace laas {

class UdpYoloInterface {
public:
    explicit UdpYoloInterface(const Config& config);
    ~UdpYoloInterface();

    bool init();
    bool sendFrame(const FrameMsg& frame, int quality = 85);
    bool sendDebugFrame(const cv::Mat& frame, int quality = 75);
    bool receiveObstacle(ObstacleMsg& obstacle);
    void close();
    bool isInitialized() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace laas
