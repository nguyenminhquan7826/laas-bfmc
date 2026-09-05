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
    validate_common,
    validate_parking_status,
    validate_plan_request,
    validate_safety_event,
    validate_trajectory_status,
    validate_vehicle_pose,
)


class ServerProtocolTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
