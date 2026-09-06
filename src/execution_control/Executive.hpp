#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "../laas_core/Blackboard.hpp"
#include "../laas_core/Config.hpp"
#include "RuntimeState.hpp"
#include "Scheduler.hpp"
#include "SchedulerDiagnostics.hpp"

#include "../decision/MissionSupervisor.hpp"
#include "../functional/perception/LanePerceptionModule.hpp"
#include "../functional/perception/ParkingStatusBenchSource.hpp"
#include "../functional/localization/VehiclePoseEstimator.hpp"
#include "../functional/planning/LaneChangePlannerModule.hpp"
#include "../functional/control/PurePursuitControlModule.hpp"
#include "../functional/control/ParkingTrajectoryTracker.hpp"
#ifdef LAAS_ENABLE_MPC
#include "../functional/control/MpcControlModule.hpp"
#endif
#include "../functional/safety/SafetyFilterModule.hpp"
#include "../functional/safety/ParkingTrajectoryValidator.hpp"
#include "../functional/safety/ParkingSafetyFilter.hpp"
#include "../logical_robot/CameraInterface.hpp"
#include "../logical_robot/UdpYoloInterface.hpp"
#include "../logical_robot/UartVehicleInterface.hpp"
#ifdef LAAS_ENABLE_PARKING_CLIENT
#include "../logical_robot/ParkingServerClient.hpp"
#include "../logical_robot/ParkingSessionSyncPolicy.hpp"
#include "../logical_robot/ParkingTrajectoryStatusPolicy.hpp"
#endif

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
    OperatingMode operatingMode() const { return operating_mode_.load(); }
    const Blackboard& blackboard() const { return blackboard_; }

private:
    void configureScheduler();
    void handleKeyboardTick();

    void cameraTick();
    void yoloTick();
    void perceptionTick();
    void decisionTick();
    void planningTick();
    void telemetryTick();
    void parkingNetworkTick();
    void parkingBenchControlTick();

    void parkingSafetyEventSyncTick(
        const ParkingTrajectoryMsg& trajectory);

    void queueParkingSafetyEvent(
        const std::string& event,
        std::uint64_t trajectory_id,
        bool has_trajectory_id);

    void flushParkingSafetyEvents();

#ifdef LAAS_ENABLE_PARKING_CLIENT
    void applyParkingSessionSnapshot(
        const ParkingSessionSnapshot& session);

    void clearLocalParkingTrajectory(
        const std::string& reason);

    void parkingTrajectoryStatusSyncTick(
        const ParkingTrajectoryMsg& trajectory);
#endif

    void controlTick();
    void loggingTick() const;

    ControlCmdMsg computeRawCommand(const TrajectoryMsg& trajectory,
                                    const LanePerceptionMsg& lane,
                                    const BehaviorRequest& behavior);

private:
    struct PendingParkingSafetyEvent {
        std::string event;
        std::uint64_t trajectory_id{0};
        bool has_trajectory_id{false};
    };

    Config config_;
    Blackboard blackboard_;
    Scheduler scheduler_;
    SchedulerDiagnostics scheduler_diagnostics_;

    CameraInterface camera_;
    UdpYoloInterface yolo_;
    UartVehicleInterface vehicle_;
#ifdef LAAS_ENABLE_PARKING_CLIENT
    ParkingServerClient parking_server_;
#endif

    LanePerceptionModule lane_perception_;
    ParkingStatusBenchSource parking_status_bench_source_;
    VehiclePoseEstimator vehicle_pose_estimator_;
    MissionSupervisor mission_;
    LaneChangePlannerModule planner_;
    PurePursuitControlModule pure_pursuit_;
    ParkingTrajectoryTracker parking_trajectory_tracker_;
#ifdef LAAS_ENABLE_MPC
    MpcControlModule mpc_;
#endif
    SafetyFilterModule safety_;
    ParkingTrajectoryValidator parking_trajectory_validator_;
    ParkingSafetyFilter parking_safety_filter_;

    std::atomic<bool> running_{false};
    std::atomic<bool> user_run_request_{false};
    std::atomic<RuntimeState> state_{RuntimeState::INIT};
    std::atomic<OperatingMode> operating_mode_{OperatingMode::LANE_DRIVING};

    // A camera frame may remain on the blackboard after a failed grab. These
    // timestamps prevent that same frame from being sent/processed repeatedly.
    uint64_t last_yolo_frame_timestamp_ms_{0};
    uint64_t last_perception_frame_timestamp_ms_{0};

    VehicleTelemetryMsg latest_telemetry_;
    ControlCmdMsg parking_bench_raw_command_;
    ControlCmdMsg parking_bench_safe_command_;
    ParkingTrackerDebug parking_tracker_debug_;
    ParkingSafetyResult parking_safety_result_;

    bool have_telemetry_sequence_{false};
    std::uint32_t last_telemetry_sequence_{0};

    std::uint64_t telemetry_received_frames_{0};
    std::uint64_t telemetry_sequence_gaps_{0};
    std::uint64_t telemetry_duplicate_frames_{0};
    std::uint64_t telemetry_sequence_resets_{0};

    std::uint64_t telemetry_window_start_ms_{0};
    std::uint64_t telemetry_window_frames_{0};
    double telemetry_rx_hz_{0.0};

    std::uint64_t last_sent_pose_sequence_{0};
    bool have_sent_pose_sequence_{false};
    std::uint64_t last_sent_parking_status_sequence_{0};
    bool have_sent_parking_status_sequence_{false};
    std::uint64_t parking_status_tx_sequence_{1};

    // Step-11: transition-based parking safety synchronization.
    // Events generated during a TCP outage remain queued and are
    // transmitted after the connection is restored.
    std::deque<PendingParkingSafetyEvent>
        parking_safety_event_queue_;

    std::string last_parking_safety_reason_{
        "NOT_EVALUATED"};

    bool parking_blocking_safety_active_{false};

#ifdef LAAS_ENABLE_PARKING_CLIENT
    bool parking_server_connected_{false};

    // Step-12: a TCP connection alone does not authorize parking motion.
    // Executive holds parking until the Server session snapshot is reconciled
    // with the local trajectory. Reconnect always re-enters this hold.
    bool parking_session_query_pending_{false};
    bool parking_session_sync_hold_{true};
    std::string parking_session_sync_reason_{"NOT_CONNECTED"};

    ParkingTrajectoryStatusPolicy parking_trajectory_status_policy_;
#endif
};

}  // namespace laas