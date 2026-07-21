#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "../laas_core/Blackboard.hpp"
#include "../laas_core/Config.hpp"
#include "RuntimeState.hpp"
#include "Scheduler.hpp"

#include "../decision/MissionSupervisor.hpp"
#include "../functional/perception/LanePerceptionModule.hpp"
#include "../functional/planning/LaneChangePlannerModule.hpp"
#include "../functional/control/PurePursuitControlModule.hpp"
#ifdef LAAS_ENABLE_MPC
#include "../functional/control/MpcControlModule.hpp"
#endif
#include "../functional/safety/SafetyFilterModule.hpp"
#include "../logical_robot/CameraInterface.hpp"
#include "../logical_robot/UdpYoloInterface.hpp"
#include "../logical_robot/UartVehicleInterface.hpp"

namespace laas {

class Executive {
public:
    explicit Executive(const Config& config);
    ~Executive();

    bool init();
    void run();
    void stop();

    void setUserRunRequest(bool enabled);
    RuntimeState state() const { return state_.load(); }
    const Blackboard& blackboard() const { return blackboard_; }

private:
    void configureScheduler();
    void handleKeyboardTick();

    void cameraTick();
    void yoloTick();
    void perceptionTick();
    void decisionTick();
    void planningTick();
    void controlTick();
    void loggingTick() const;

    ControlCmdMsg computeRawCommand(const TrajectoryMsg& trajectory,
                                    const LanePerceptionMsg& lane,
                                    const BehaviorRequest& behavior);

private:
    Config config_;
    Blackboard blackboard_;
    Scheduler scheduler_;

    CameraInterface camera_;
    UdpYoloInterface yolo_;
    UartVehicleInterface vehicle_;

    LanePerceptionModule lane_perception_;
    MissionSupervisor mission_;
    LaneChangePlannerModule planner_;
    PurePursuitControlModule pure_pursuit_;
#ifdef LAAS_ENABLE_MPC
    MpcControlModule mpc_;
#endif
    SafetyFilterModule safety_;

    std::atomic<bool> running_{false};
    std::atomic<bool> user_run_request_{false};
    std::atomic<RuntimeState> state_{RuntimeState::INIT};

    // A camera frame may remain on the blackboard after a failed grab. These
    // timestamps prevent that same frame from being sent/processed repeatedly.
    uint64_t last_yolo_frame_timestamp_ms_{0};
    uint64_t last_perception_frame_timestamp_ms_{0};
};

}  // namespace laas
