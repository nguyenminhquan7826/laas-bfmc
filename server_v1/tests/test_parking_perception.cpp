#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "functional/perception/ParkingPerceptionModule.hpp"

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

laas::Config makeConfig()
{
    laas::Config config;
    config.parking.enable_camera_parking_perception = true;
    config.parking.slot_map_file = "server_v1/map_v1.yaml";
    config.parking.image_to_ground_homography_valid = true;
    // Synthetic camera: u=320-200*left, v=480-150*forward.
    config.parking.image_to_ground_homography = {{
        0.0, -1.0 / 150.0, 3.2,
        -1.0 / 200.0, 0.0, 1.6,
        0.0, 0.0, 1.0,
    }};
    config.parking.parking_sign_confirm_frames = 3;
    config.parking.occupied_confirm_frames = 3;
    config.parking.free_confirm_frames = 5;
    config.parking.parking_perception_timeout_ms = 1000;
    return config;
}

laas::YoloDetection detection(int class_id, float confidence,
                              float x1, float y1, float x2, float y2)
{
    laas::YoloDetection value;
    value.class_id = class_id;
    value.confidence = confidence;
    value.x1_px = x1;
    value.y1_px = y1;
    value.x2_px = x2;
    value.y2_px = y2;
    return value;
}

const laas::ParkingSlotObservation& findSlot(
    const laas::ParkingStatusMsg& status, const std::string& id)
{
    for (const auto& slot : status.slots) {
        if (slot.id == id) {
            return slot;
        }
    }
    std::cerr << "FAIL: missing slot " << id << "\n";
    std::exit(1);
}

}  // namespace

int main()
{
    laas::Config missing = makeConfig();
    missing.parking.image_to_ground_homography_valid = false;
    laas::ParkingPerceptionModule not_ready(missing);
    require(!not_ready.ready(), "missing homography must fail closed");

    laas::Config config = makeConfig();
    laas::ParkingPerceptionModule module(config);
    require(module.ready(), module.reason());

    laas::VehiclePoseMsg pose;
    pose.header.valid = true;
    pose.header.timestamp_ms = 990;
    pose.map_id = "map_v1";
    pose.x_m = 1.300;
    pose.y_m = 0.751;
    pose.yaw_rad = 0.0;

    laas::ParkingStatusMsg result;
    for (std::uint32_t frame_id = 1; frame_id <= 7; ++frame_id) {
        laas::YoloPerceptionMsg perception;
        perception.header.valid = true;
        perception.header.timestamp_ms = 990;
        perception.has_detection_payload = true;
        perception.frame_id = frame_id;
        perception.image_width = 640;
        perception.image_height = 480;
        perception.detections.push_back(
            detection(0, 0.90F, 20.0F, 20.0F, 70.0F, 100.0F));
        // Footpoint (390,232) maps inside P_B2.
        perception.detections.push_back(
            detection(1, 0.85F, 370.0F, 180.0F, 410.0F, 232.0F));
        require(module.process(perception, pose, 1000, result),
                "valid perception should publish");
    }

    require(findSlot(result, "P_B2").state ==
                laas::ParkingSlotState::OCCUPIED,
            "vehicle footpoint must associate with P_B2");
    require(findSlot(result, "P_B1").state ==
                laas::ParkingSlotState::FREE,
            "visible empty P_B1 must become FREE after confirmation");
    require(findSlot(result, "P_T1").state ==
                laas::ParkingSlotState::FREE,
            "visible empty P_T1 must become FREE after confirmation");
    require(findSlot(result, "P_T2").state ==
                laas::ParkingSlotState::FREE,
            "visible empty P_T2 must become FREE after confirmation");
    require(!result.objects.empty(), "object evidence should be retained");

    laas::ParkingPerceptionModule no_sign(config);
    laas::YoloPerceptionMsg empty;
    empty.header.valid = true;
    empty.header.timestamp_ms = 990;
    empty.has_detection_payload = true;
    empty.frame_id = 1;
    empty.image_width = 640;
    empty.image_height = 480;
    require(no_sign.process(empty, pose, 1000, result),
            "no-sign frame should publish UNKNOWN states");
    for (const auto& slot : result.slots) {
        require(slot.state == laas::ParkingSlotState::UNKNOWN,
                "absence of confirmed parking sign must never imply FREE");
    }

    std::cout << "parking perception tests passed\n";
    return 0;
}
