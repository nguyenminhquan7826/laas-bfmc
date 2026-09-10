#!/usr/bin/env python3
"""Protocol validation tests for LAAS Parking Server V1."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from server_stub import (
    MAP_ID,
    normalize_source_seq,
    validate_navigation_decision_status,
    validate_common,
    validate_parking_status,
    validate_plan_request,
    validate_runtime_status,
    validate_safety_event,
    validate_serialized_trajectory,
    validate_trajectory_status,
    validate_vehicle_pose,
)


class ServerProtocolTests(unittest.TestCase):
    @staticmethod
    def runtime_status() -> dict:
        return {
            "type": "runtime_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 12,
            "timestamp_ms": 1236,
            "map_id": MAP_ID,
            "operating_mode": "PARKING",
            "tracker": {
                "valid": True,
                "goal_reached": False,
                "trajectory_id": 42,
                "nearest_index": 5,
                "target_index": 9,
                "cross_track_error_m": 0.021,
            },
            "safety": {
                "evaluated": True,
                "motion_allowed": False,
                "reason": "PASS_BENCH_ONLY",
            },
            "uart": {
                "rx_enabled": True,
                "tx_enabled": False,
                "telemetry_valid": True,
                "telemetry_age_ms": 18,
            },
            "session_sync": {
                "hold": False,
                "reason": "SYNC_READY",
            },
        }

    def test_vehicle_pose_accepts_valid_message(self) -> None:
        msg = {
            "type": "vehicle_pose",
            "version": 1,
            "seq": 10,
            "timestamp_ms": 1234,
            "map_id": MAP_ID,
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
        }
        self.assertEqual(validate_vehicle_pose(msg), (True, "ok"))

    def test_vehicle_pose_rejects_nonfinite_value(self) -> None:
        msg = {
            "type": "vehicle_pose",
            "version": 1,
            "seq": 10,
            "timestamp_ms": 1234,
            "map_id": MAP_ID,
            "pose": {"x_m": float("nan"), "y_m": 0.751, "yaw_rad": 0.0},
        }
        self.assertEqual(validate_vehicle_pose(msg), (False, "invalid_pose:x_m"))

    def test_runtime_status_accepts_read_only_telemetry(self) -> None:
        self.assertEqual(validate_runtime_status(self.runtime_status()), (True, "ok"))

    def test_runtime_status_rejects_valid_tracker_without_trajectory(self) -> None:
        msg = self.runtime_status()
        msg["tracker"]["trajectory_id"] = 0
        self.assertEqual(
            validate_runtime_status(msg),
            (False, "tracker_valid_without_trajectory"),
        )

    def test_runtime_status_requires_boolean_uart_policy(self) -> None:
        msg = self.runtime_status()
        msg["uart"]["tx_enabled"] = 0
        self.assertEqual(
            validate_runtime_status(msg),
            (False, "invalid_bool:uart.tx_enabled"),
        )

    def test_navigation_decision_status_accepts_client_ack(self) -> None:
        msg = {
            "type": "navigation_decision_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 13,
            "timestamp_ms": 1237,
            "map_id": MAP_ID,
            "decision_id": 7,
            "status": "ACCEPTED",
            "reason": "MAP_PACKAGE_ID_MATCH_LOCAL_VERIFY_PENDING",
        }
        self.assertEqual(validate_navigation_decision_status(msg), (True, "ok"))

    def test_navigation_decision_status_rejects_unknown_status(self) -> None:
        msg = {
            "type": "navigation_decision_status",
            "version": 1,
            "seq": 13,
            "timestamp_ms": 1237,
            "map_id": MAP_ID,
            "decision_id": 7,
            "status": "RUNNING",
            "reason": "not_in_protocol_v1",
        }
        self.assertEqual(
            validate_navigation_decision_status(msg),
            (False, "invalid_navigation_decision_status"),
        )

    def test_parking_status_requires_exactly_all_four_slots(self) -> None:
        msg = {
            "type": "parking_status",
            "version": 1,
            "seq": 11,
            "timestamp_ms": 1235,
            "map_id": MAP_ID,
            "slots": [
                {"id": "P_B1", "state": "OCCUPIED", "confidence": 1.0},
                {"id": "P_B2", "state": "FREE", "confidence": 1.0},
                {"id": "P_T1", "state": "OCCUPIED", "confidence": 1.0},
            ],
        }
        self.assertEqual(validate_parking_status(msg), (False, "all_four_slots_required"))

    def test_parking_status_accepts_bench_layout(self) -> None:
        msg = {
            "type": "parking_status",
            "version": 1,
            "seq": 11,
            "timestamp_ms": 1235,
            "map_id": MAP_ID,
            "slots": [
                {"id": "P_B1", "state": "OCCUPIED", "confidence": 1.0},
                {"id": "P_B2", "state": "FREE", "confidence": 1.0},
                {"id": "P_T1", "state": "OCCUPIED", "confidence": 1.0},
                {"id": "P_T2", "state": "OCCUPIED", "confidence": 1.0},
            ],
        }
        self.assertEqual(validate_parking_status(msg), (True, "ok"))

    def test_safety_event_accepts_supported_event_and_tid(self) -> None:
        msg = {
            "type": "safety_event",
            "version": 1,
            "map_id": MAP_ID,
            "event": "SERVER_TIMEOUT",
            "trajectory_id": 42,
        }
        self.assertEqual(validate_safety_event(msg), (True, "ok"))

    def test_safety_event_rejects_unknown_event(self) -> None:
        msg = {
            "type": "safety_event",
            "version": 1,
            "map_id": MAP_ID,
            "event": "UNKNOWN_EVENT",
            "trajectory_id": 42,
        }
        self.assertEqual(
            validate_safety_event(msg),
            (False, "invalid_safety_event:UNKNOWN_EVENT"),
        )

    def test_trajectory_status_rejects_unknown_status(self) -> None:
        msg = {
            "type": "trajectory_status",
            "version": 1,
            "map_id": MAP_ID,
            "trajectory_id": 42,
            "status": "RESUMED",
        }
        self.assertEqual(
            validate_trajectory_status(msg),
            (False, "invalid_trajectory_status:RESUMED"),
        )

    def test_plan_request_requires_valid_seq_and_timestamp(self) -> None:
        msg = {
            "type": "plan_request",
            "version": 1,
            "seq": -1,
            "timestamp_ms": 100,
            "map_id": MAP_ID,
        }
        self.assertEqual(validate_plan_request(msg), (False, "invalid_seq"))

    def test_common_rejects_wrong_map(self) -> None:
        msg = {"version": 1, "map_id": "map_v2"}
        self.assertEqual(validate_common(msg), (False, "map_id_mismatch"))

    def test_safety_replan_source_seq_falls_back_to_latest_pose(self) -> None:
        snap = {
            "pose_seq": 27,
            "parking_status": {"seq": 26},
        }
        self.assertEqual(normalize_source_seq(None, snap), 27)

    def test_source_seq_prefers_explicit_trigger_sequence(self) -> None:
        snap = {
            "pose_seq": 27,
            "parking_status": {"seq": 26},
        }
        self.assertEqual(normalize_source_seq(31, snap), 31)

    def test_source_seq_falls_back_to_parking_sequence(self) -> None:
        snap = {
            "pose_seq": None,
            "parking_status": {"seq": 19},
        }
        self.assertEqual(normalize_source_seq(None, snap), 19)

    def test_serialized_trajectory_rejects_null_source_seq_before_send(self) -> None:
        response = {
            "type": "trajectory",
            "version": 1,
            "source_seq": None,
            "map_id": MAP_ID,
            "reference_point": "rear_axle_center",
            "target_slot": "P_B2",
            "points": [],
        }

        # Invalid source_seq is checked before planner geometry is touched, so
        # a context object is deliberately unnecessary for this regression.
        self.assertEqual(
            validate_serialized_trajectory(None, response, {"P_B2": "FREE"}),
            (False, "trajectory_source_seq_invalid"),
        )


if __name__ == "__main__":
    unittest.main()
