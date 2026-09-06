#include "CameraInterface.hpp"

#include <iostream>

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
    if (!openCameraDevice()) {
        return false;
    }

    if (!prepareUndistortMaps()) {
        std::cerr << "[CAMERA] Failed to prepare undistort maps.\n";
        close();
        return false;
    }

    initialized_ = true;
    return true;
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
        // max-buffers=1 + drop=true keeps only the newest sample; the capture
        // path therefore never builds a latency-producing frame backlog.
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

bool CameraInterface::prepareUndistortMaps()
{
    const cv::Size image_size(config_.camera.width, config_.camera.height);
    if (image_size.width <= 0 || image_size.height <= 0) {
        return false;
    }

    // Calibration for the installed camera at 640x480.
    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        262.08953333143063, 0.0, 330.77574325128484,
        0.0, 263.57901348164575, 250.50298224489268,
        0.0, 0.0, 1.0);

    const cv::Mat distortion = (cv::Mat_<double>(1, 5) <<
        -0.27166331922859776, 0.09924985737514846,
        -0.0002707688044880526, 0.0006724194580262318,
        -0.01935517123682299);

    const cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix,
        distortion,
        image_size,
        0.0,
        image_size,
        &undistort_valid_roi_);

    cv::initUndistortRectifyMap(
        camera_matrix,
        distortion,
        cv::Mat(),
        new_camera_matrix,
        image_size,
        CV_16SC2,
        undistort_map1_,
        undistort_map2_);

    if (undistort_map1_.empty() || undistort_map2_.empty()) {
        return false;
    }

    if (undistort_valid_roi_.width <= 0 ||
        undistort_valid_roi_.height <= 0) {
        undistort_valid_roi_ = cv::Rect(0, 0, image_size.width, image_size.height);
    }

    undistort_ready_ = true;
    std::cout << "[CAMERA] Undistort maps cached for "
              << image_size.width << "x" << image_size.height << "\n";
    return true;
}

bool CameraInterface::grab(FrameMsg& output)
{
    output = FrameMsg{};

    if (!initialized_ || !cap_.isOpened() || !undistort_ready_) {
        return false;
    }

    cv::Mat raw;
    cap_ >> raw;
    if (raw.empty()) {
        return false;
    }

    // Automatic diagnostic image writes used to run here at frame 60. Disk I/O
    // does not belong on the production capture path; calibration snapshots
    // should be taken by a dedicated diagnostic tool instead.
    output.frame_bgr = undistortAndResize(raw);
    output.header.timestamp_ms = nowMs();
    output.header.valid = !output.frame_bgr.empty();
    return output.header.valid;
}

cv::Mat CameraInterface::undistortAndResize(const cv::Mat& input) const
{
    if (input.empty() || !undistort_ready_) {
        return cv::Mat{};
    }

    const cv::Size target_size(config_.camera.width, config_.camera.height);

    cv::Mat source;
    if (input.size() == target_size) {
        source = input;
    } else {
        cv::resize(input, source, target_size);
    }

    cv::Mat undistorted;
    cv::remap(
        source,
        undistorted,
        undistort_map1_,
        undistort_map2_,
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT);

    cv::Mat cropped;
    const cv::Rect frame_rect(0, 0, undistorted.cols, undistorted.rows);
    const cv::Rect roi = undistort_valid_roi_ & frame_rect;
    if (roi.width > 0 && roi.height > 0) {
        cropped = undistorted(roi);
    } else {
        cropped = undistorted;
    }

    if (cropped.size() == target_size) {
        return cropped.clone();
    }

    cv::Mat resized;
    cv::resize(cropped, resized, target_size);
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
    undistort_map1_.release();
    undistort_map2_.release();
    undistort_ready_ = false;
    initialized_ = false;
}

}  // namespace laas
