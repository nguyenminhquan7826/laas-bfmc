#include "CameraInterface.hpp"

#include <iostream>
#include <stdexcept>

#include "../laas_core/Time.hpp"

namespace laas {
namespace {

bool tryOpenGstreamer(cv::VideoCapture& capture, const std::string& pipeline)
{
    std::cout << "[CAMERA] Try pipeline:\n" << pipeline << "\n";
    if (!capture.open(pipeline, cv::CAP_GSTREAMER)) {
        std::cerr << "[CAMERA] Open failed.\n";
        return false;
    }
    std::cout << "[CAMERA] Opened OK.\n";
    return true;
}

}  // namespace

CameraInterface::CameraInterface(const Config& config)
    : config_(config)
{
}

CameraInterface::~CameraInterface()
{
    close();
}

bool CameraInterface::init()
{
    initialized_ = openCameraDevice();
    return initialized_;
}

bool CameraInterface::openCameraDevice()
{
    const int capture_width = config_.camera.width;
    const int capture_height = config_.camera.height;
    const int output_width = config_.camera.width;
    const int output_height = config_.camera.height;
    const int fps = config_.camera.fps;
    const std::string& device = config_.camera.device;

    if (device.find("/dev/video") == 0) {
        const std::string v4l2_pipeline =
            "v4l2src device=" + device + " ! "
            "video/x-raw,width=" + std::to_string(capture_width) +
            ",height=" + std::to_string(capture_height) +
            ",framerate=" + std::to_string(fps) + "/1 ! "
            "videoconvert ! videoscale ! "
            "video/x-raw,format=BGR,width=" + std::to_string(output_width) +
            ",height=" + std::to_string(output_height) + " ! "
            "appsink max-buffers=1 drop=true sync=false";

        if (tryOpenGstreamer(cap_, v4l2_pipeline)) {
            return true;
        }

        if (cap_.open(device, cv::CAP_V4L2)) {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height);
            cap_.set(cv::CAP_PROP_FPS, fps);
            return true;
        }

        std::cerr << "[CAMERA] Cannot open V4L2 camera: " << device << "\n";
        return false;
    }

    if (device == "libcamera" || device == "csi") {
        // This exact NV12 caps negotiation was verified with OV5647 on Pi 5.
        const std::string libcamera_pipeline =
            "libcamerasrc ! "
            "video/x-raw,format=NV12,width=" + std::to_string(capture_width) +
            ",height=" + std::to_string(capture_height) +
            ",framerate=" + std::to_string(fps) +
            "/1,colorimetry=bt709 ! "
            "queue ! videoconvert ! videoscale ! "
            "video/x-raw,format=BGR,width=" + std::to_string(output_width) +
            ",height=" + std::to_string(output_height) + " ! "
            "appsink max-buffers=1 drop=true sync=false";

        if (!tryOpenGstreamer(cap_, libcamera_pipeline)) {
            std::cerr << "[CAMERA] Cannot open CSI camera with libcamerasrc.\n";
            return false;
        }
        return true;
    }

    if (!cap_.open(device)) {
        std::cerr << "[CAMERA] Cannot open file/device: " << device << "\n";
        return false;
    }
    return true;
}

bool CameraInterface::grab(FrameMsg& output)
{
    output = FrameMsg{};

    if (!initialized_ || !cap_.isOpened()) {
        return false;
    }

    cv::Mat raw;
    cap_ >> raw;
    if (raw.empty()) {
        return false;
    }

    if (saved_raw_count_ == 0) {
        cv::imwrite("pi_raw.jpg", raw);
        std::cout << "[CAMERA] Saved pi_raw.jpg" << std::endl;
        saved_raw_count_ = 1;
    }

    output.frame_bgr = undistortAndResize(raw);
    output.header.timestamp_ms = nowMs();
    output.header.valid = !output.frame_bgr.empty();
    return output.header.valid;
}

cv::Mat CameraInterface::undistortAndResize(const cv::Mat& input) const
{
    if (input.empty()) {
        return cv::Mat{};
    }

    // These values must belong to the installed camera at 640x480.
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        262.08953333143063, 0.0, 330.77574325128484,
        0.0, 263.57901348164575, 250.50298224489268,
        0.0, 0.0, 1.0);

    cv::Mat distortion = (cv::Mat_<double>(1, 5) <<
        -0.27166331922859776, 0.09924985737514846,
        -0.0002707688044880526, 0.0006724194580262318,
        -0.01935517123682299);

    cv::Rect valid_roi;
    const cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix,
        distortion,
        input.size(),
        0.0,
        input.size(),
        &valid_roi);

    cv::Mat undistorted;
    cv::undistort(
        input, undistorted, camera_matrix, distortion, new_camera_matrix);

    if (valid_roi.width > 0 && valid_roi.height > 0) {
        undistorted = undistorted(valid_roi).clone();
    }

    cv::Mat resized;
    cv::resize(
        undistorted,
        resized,
        cv::Size(config_.camera.width, config_.camera.height));
    return resized;
}

bool CameraInterface::isOpened() const
{
    return initialized_ && cap_.isOpened();
}

void CameraInterface::close()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
    initialized_ = false;
}

}  // namespace laas
