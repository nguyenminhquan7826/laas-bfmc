#!/usr/bin/env python3
from pathlib import Path

path = Path("src/execution_control/Executive.cpp")
text = path.read_text()


def replace_once(old: str, new: str, label: str) -> None:
    global text
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"[FAIL] {label}: expected exactly 1 marker, found {count}")
    text = text.replace(old, new, 1)
    print(f"[PASS] {label}")


replace_once(
'''    if (config_.parking.enable) {
        parking_server_.init();
    }
''',
'''    if (config_.parking.enable) {
        parking_server_.init();
        parking_trajectory_status_policy_.reset();
        parking_session_query_pending_ = false;
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "NOT_CONNECTED";
    }
''',
"init parking sync state")

replace_once(
'''    parking_server_.service();
    parking_server_connected_ = parking_server_.connected();

    // A safety event may have been generated while TCP was down.
    // Flush it immediately after reconnect.
    flushParkingSafetyEvents();

    ParkingServerMessage server_message;
''',
'''    const bool was_connected = parking_server_connected_;
    parking_server_.service();
    parking_server_connected_ = parking_server_.connected();

    if (was_connected && !parking_server_connected_) {
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "SERVER_DISCONNECTED";
        parking_session_query_pending_ = false;
        std::cout << "[PARKING][SYNC] disconnected -> HOLD\\n";
    }

    if (!was_connected && parking_server_connected_) {
        // A TCP connection alone never authorizes parking motion.
        // Reconcile Server session state before releasing the hold.
        parking_session_sync_hold_ = true;
        parking_session_sync_reason_ = "AWAITING_SESSION_SYNC";
        parking_session_query_pending_ = true;

        // Force the newest bench/local inputs to be republished on a
        // fresh TCP session even if their sequence did not change.
        have_sent_pose_sequence_ = false;
        have_sent_parking_status_sequence_ = false;

        std::cout << "[PARKING][SYNC] connected -> HOLD awaiting session\\n";
    }

    // A safety event may have been generated while TCP was down.
    // Flush it before session_query so SERVER_TIMEOUT is ordered
    // ahead of the snapshot request on the same TCP stream.
    flushParkingSafetyEvents();

    if (parking_server_connected_ && parking_session_query_pending_) {
        if (parking_server_.sendSessionQuery()) {
            parking_session_query_pending_ = false;
            std::cout << "[PARKING][SYNC] session_query sent\\n";
        }
    }

    ParkingServerMessage server_message;
''',
"reconnect session query and hold")

replace_once(
'''            if (validation.accepted) {
                blackboard_.setParkingTrajectory(server_message.trajectory);
                parking_server_.sendTrajectoryStatus(
''',
'''            if (validation.accepted) {
                blackboard_.setParkingTrajectory(server_message.trajectory);
                parking_trajectory_status_policy_.onTrajectoryReceived(
                    server_message.trajectory.trajectory_id);

                // The Server attaches its TRAJECTORY_READY snapshot to
                // trajectory messages. Reconcile only after the Pi has
                // validated/stored the same trajectory ID.
                if (server_message.has_session) {
                    applyParkingSessionSnapshot(server_message.session);
                }

                parking_server_.sendTrajectoryStatus(
''',
"accepted trajectory sync epoch")

replace_once(
'''            } else {
                parking_server_.sendTrajectoryStatus(
                    parking_status_tx_sequence_++,
                    server_message.trajectory.trajectory_id,
                    "REJECTED", validation.reason);
''',
'''            } else {
                parking_session_sync_hold_ = true;
                parking_session_sync_reason_ =
                    "TRAJECTORY_REJECTED_" + validation.reason;
                clearLocalParkingTrajectory(parking_session_sync_reason_);

                parking_server_.sendTrajectoryStatus(
                    parking_status_tx_sequence_++,
                    server_message.trajectory.trajectory_id,
                    "REJECTED", validation.reason);
''',
"rejected trajectory fails closed")

replace_once(
'''        } else if (server_message.type == ParkingServerMessageType::PLANNING_RESULT) {
            std::cout << "[PARKING] planning_result=" << server_message.status
                      << " reason=" << server_message.reason << "\\n";
        } else if (server_message.type == ParkingServerMessageType::ERROR) {
''',
'''        } else if (server_message.type == ParkingServerMessageType::ACK) {
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            }
            if (!server_message.accepted) {
                std::cerr << "[PARKING][ACK] rejected reason="
                          << server_message.reason << "\\n";
            }
        } else if (server_message.type == ParkingServerMessageType::SESSION_STATUS) {
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            } else {
                parking_session_sync_hold_ = true;
                parking_session_sync_reason_ = "SESSION_STATUS_MISSING_SNAPSHOT";
                std::cerr << "[PARKING][SYNC] invalid session_status -> HOLD\\n";
            }
        } else if (server_message.type == ParkingServerMessageType::PLANNING_RESULT) {
            std::cout << "[PARKING] planning_result=" << server_message.status
                      << " reason=" << server_message.reason << "\\n";
            if (server_message.has_session) {
                applyParkingSessionSnapshot(server_message.session);
            }
        } else if (server_message.type == ParkingServerMessageType::ERROR) {
''',
"consume server session snapshots")

marker = "void Executive::queueParkingSafetyEvent(\n"
if text.count(marker) != 1:
    raise SystemExit("[FAIL] helper insertion marker not unique")

helpers = r'''#ifdef LAAS_ENABLE_PARKING_CLIENT
void Executive::clearLocalParkingTrajectory(
    const std::string& reason)
{
    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();
    if (local.header.valid && local.trajectory_id > 0U) {
        std::cout << "[PARKING][SYNC] clear local tid="
                  << local.trajectory_id
                  << " reason=" << reason << "\n";
    }

    blackboard_.setParkingTrajectory(ParkingTrajectoryMsg{});
    parking_trajectory_tracker_.reset();
    parking_trajectory_status_policy_.reset();
}

void Executive::applyParkingSessionSnapshot(
    const ParkingSessionSnapshot& session)
{
    const ParkingTrajectoryMsg local = blackboard_.parkingTrajectory();
    const ParkingSessionSyncDecision decision =
        ParkingSessionSyncPolicy::evaluate(session, local);

    parking_session_sync_hold_ = decision.hold_motion;
    parking_session_sync_reason_ = decision.reason;

    if (decision.clear_local_trajectory) {
        clearLocalParkingTrajectory(decision.reason);
    }

    if (decision.request_replan && parking_server_connected_) {
        const bool sent = parking_server_.sendTrajectoryStatus(
            parking_status_tx_sequence_++,
            decision.replan_trajectory_id,
            "REPLAN_REQUESTED",
            "STEP12_SESSION_RESYNC_" + decision.reason);

        if (!sent) {
            // Re-query on a later tick so the fail-closed decision can be
            // retried without assuming delivery.
            parking_session_query_pending_ = true;
        }
    }

    std::cout << "[PARKING][SYNC] serverState=" << session.state
              << " serverTid="
              << (session.has_active_trajectory
                      ? std::to_string(session.active_trajectory_id)
                      : std::string("none"))
              << " localTid="
              << ((local.header.valid && local.trajectory_id > 0U)
                      ? std::to_string(local.trajectory_id)
                      : std::string("none"))
              << " hold=" << (parking_session_sync_hold_ ? 1 : 0)
              << " reason=" << parking_session_sync_reason_ << "\n";
}

void Executive::parkingTrajectoryStatusSyncTick(
    const ParkingTrajectoryMsg& trajectory)
{
    // Step-12B remains bench-only. Hard-lock physical execution authorization
    // to false: static/bench tracking cannot emit EXECUTING or COMPLETED.
    constexpr bool kActuationAuthorized = false;

    const ParkingTrajectoryStatusUpdate update =
        parking_trajectory_status_policy_.evaluate(
            trajectory,
            parking_tracker_debug_,
            parking_safety_result_,
            kActuationAuthorized);

    if (!update.emit || !parking_server_connected_) {
        return;
    }

    if (parking_server_.sendTrajectoryStatus(
            parking_status_tx_sequence_++,
            update.trajectory_id,
            update.status,
            update.reason)) {
        std::cout << "[PARKING][STATUS_TX] tid="
                  << update.trajectory_id
                  << " status=" << update.status
                  << " reason=" << update.reason << "\n";
    }
}
#endif

'''
text = text.replace(marker, helpers + marker, 1)
print("[PASS] insert Step-12 helper methods")

replace_once(
'''    parking_bench_safe_command_ = parking_safety_filter_.filter(
        parking_bench_raw_command_, parking_tracker_debug_, pose, trajectory,
        latest_telemetry_, obstacle, server_connected, &parking_safety_result_);

    // Step-11 sends safety state to the Server only.
''',
'''    parking_bench_safe_command_ = parking_safety_filter_.filter(
        parking_bench_raw_command_, parking_tracker_debug_, pose, trajectory,
        latest_telemetry_, obstacle, server_connected, &parking_safety_result_);

#ifdef LAAS_ENABLE_PARKING_CLIENT
    // Session synchronization is an independent fail-closed gate. It never
    // sends UART; it only prevents a locally computed bench command from being
    // treated as runnable while Pi and Server disagree.
    if (parking_session_sync_hold_) {
        parking_bench_safe_command_.header.valid = true;
        parking_bench_safe_command_.header.timestamp_ms = nowMs();
        parking_bench_safe_command_.speed_mps = 0.0F;
        parking_bench_safe_command_.steering_deg = 0.0F;
    }
#endif

    // Step-11 sends safety state to the Server only.
''',
"apply session sync hold to bench command")

replace_once(
'''    parkingSafetyEventSyncTick(trajectory);

    // Deliberately no vehicle_.send(...) here.
''',
'''    parkingSafetyEventSyncTick(trajectory);

#ifdef LAAS_ENABLE_PARKING_CLIENT
    parkingTrajectoryStatusSyncTick(trajectory);
#endif

    // Deliberately no vehicle_.send(...) here.
''',
"run trajectory status policy")

replace_once(
'''                  << " cte=" << parking_tracker_debug_.nearest_distance_m
                  << " safety=" << parking_safety_result_.reason
                  << " [NO_UART]\\n";
''',
'''                  << " cte=" << parking_tracker_debug_.nearest_distance_m
                  << " safety=" << parking_safety_result_.reason
#ifdef LAAS_ENABLE_PARKING_CLIENT
                  << " sync=" << (parking_session_sync_hold_ ? "HOLD" : "READY")
                  << " syncReason=" << parking_session_sync_reason_
#endif
                  << " [NO_UART]\\n";
''',
"add sync diagnostics")

path.write_text(text)
print("[SUMMARY] Step-12B Executive patch applied")
