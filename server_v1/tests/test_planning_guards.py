#!/usr/bin/env python3
"""Regression tests for Server V1 planning-time freshness guards."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from parking_session_v1 import ParkingSession
from server_stub import Handler, MAP_ID


class FakePlanningContext:
    def __init__(self) -> None:
        self.planning_enabled = True
        self.pose_max_age_ms = 2000
        self.parking_max_age_ms = 3000
        self.planning_lock = threading.Lock()
        self.session = ParkingSession()
        self.map_cfg = {}
        self.vehicle_cfg = {}
        self.planner = object()
        self._snapshot_calls = 0

        self._parking_status = {
            "type": "parking_status",
            "version": 1,
            "seq": 44,
            "timestamp_ms": 100,
            "map_id": MAP_ID,
            "slots": [
                {"id": "P_B1", "state": "OCCUPIED", "confidence": 1.0},
                {"id": "P_B2", "state": "FREE", "confidence": 1.0},
                {"id": "P_T1", "state": "OCCUPIED", "confidence": 1.0},
                {"id": "P_T2", "state": "OCCUPIED", "confidence": 1.0},
            ],
        }

    def snapshot(self):
        self._snapshot_calls += 1
        # First snapshot is fresh. The second is deliberately stale after the
        # mocked planner returns, exercising the post-planning guard branch.
        parking_age_ms = 10.0 if self._snapshot_calls == 1 else 5000.0
        return {
            "pose": object(),
            "pose_seq": 45,
            "pose_age_ms": 10.0,
            "pose_generation": 1,
            "parking_status": self._parking_status,
            "parking_age_ms": parking_age_ms,
            "parking_generation": 1,
        }

    def generations_unchanged(self, pose_generation, parking_generation):
        return pose_generation == 1 and parking_generation == 1


class FakeHandler:
    def __init__(self) -> None:
        self.ctx = FakePlanningContext()
        self.planning_results = []

    def send_planning_result(self, source_seq, status, reason, **extra):
        self.planning_results.append(
            {
                "source_seq": source_seq,
                "status": status,
                "reason": reason,
                **extra,
            }
        )


class PlanningGuardTests(unittest.TestCase):
    def test_parking_stale_after_planning_returns_guard_result(self) -> None:
        handler = FakeHandler()

        with patch("server_stub.choose_best_free_slot", return_value=(object(), [])):
            Handler.plan_with_latest_state(
                handler,
                source_seq=None,
                trigger="test_post_planning_stale",
            )

        self.assertEqual(len(handler.planning_results), 1)
        result = handler.planning_results[0]
        self.assertEqual(result["source_seq"], 45)
        self.assertEqual(result["status"], "STALE_INPUT")
        self.assertEqual(result["reason"], "parking_status_stale")
        self.assertEqual(result["pose_age_ms"], 10.0)
        self.assertEqual(result["parking_age_ms"], 5000.0)
        self.assertEqual(handler.ctx.session.snapshot()["state"], "WAITING_INPUT")
        self.assertTrue(handler.ctx.session.snapshot()["replan_pending"])


if __name__ == "__main__":
    unittest.main()
