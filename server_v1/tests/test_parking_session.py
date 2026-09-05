#!/usr/bin/env python3
"""Unit tests for the Server V1 parking session state machine."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from parking_session_v1 import ParkingSession


class ParkingSessionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.session = ParkingSession()
        self.session.start_new_session("test_start")
        self.session.mark_trajectory_ready(101, "P_B2")

    def test_received_keeps_trajectory_ready(self) -> None:
        ok, reason = self.session.mark_received(101)

        self.assertTrue(ok)
        self.assertEqual(reason, "ok")
        snap = self.session.snapshot()
        self.assertEqual(snap["state"], "TRAJECTORY_READY")
        self.assertEqual(snap["active_trajectory_id"], 101)
        self.assertEqual(snap["target_slot"], "P_B2")

    def test_executing_then_completed(self) -> None:
        ok, reason = self.session.mark_executing(101)
        self.assertTrue(ok, reason)
        self.assertEqual(self.session.snapshot()["state"], "EXECUTING")

        ok, reason = self.session.mark_completed(101)
        self.assertTrue(ok, reason)
        snap = self.session.snapshot()
        self.assertEqual(snap["state"], "COMPLETED")
        self.assertFalse(snap["replan_pending"])
        self.assertIsNone(snap["pause_reason"])

    def test_pause_then_safety_clear_requests_replan(self) -> None:
        ok, reason = self.session.mark_paused(
            101, "CRITICAL_OBSTACLE", replan_pending=True
        )
        self.assertTrue(ok, reason)

        paused = self.session.snapshot()
        self.assertEqual(paused["state"], "PAUSED")
        self.assertTrue(paused["replan_pending"])
        self.assertEqual(paused["pause_reason"], "CRITICAL_OBSTACLE")
        self.assertEqual(paused["active_trajectory_id"], 101)

        ok, reason = self.session.clear_safety_and_request_replan("safety_cleared")
        self.assertTrue(ok, reason)

        replanning = self.session.snapshot()
        self.assertEqual(replanning["state"], "REPLAN")
        self.assertTrue(replanning["replan_pending"])
        self.assertEqual(replanning["replan_count"], 1)
        self.assertIsNone(replanning["active_trajectory_id"])
        self.assertIsNone(replanning["target_slot"])
        self.assertEqual(replanning["last_target_slot"], "P_B2")
        self.assertIsNone(replanning["pause_reason"])

    def test_safety_clear_is_rejected_when_not_paused(self) -> None:
        ok, reason = self.session.clear_safety_and_request_replan("unexpected_clear")

        self.assertFalse(ok)
        self.assertEqual(reason, "invalid_state_for_SAFETY_CLEARED:TRAJECTORY_READY")
        self.assertEqual(self.session.snapshot()["state"], "TRAJECTORY_READY")

    def test_wrong_trajectory_id_is_rejected(self) -> None:
        ok, reason = self.session.mark_paused(
            999, "SERVER_TIMEOUT", replan_pending=True
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "trajectory_id_mismatch")
        snap = self.session.snapshot()
        self.assertEqual(snap["state"], "TRAJECTORY_READY")
        self.assertEqual(snap["active_trajectory_id"], 101)

    def test_target_invalidation_is_deferred_while_paused(self) -> None:
        ok, reason = self.session.mark_paused(
            101, "CRITICAL_OBSTACLE", replan_pending=True
        )
        self.assertTrue(ok, reason)

        action = self.session.target_became_invalid("OCCUPIED")

        self.assertEqual(action, "DEFERRED_WHILE_PAUSED")
        snap = self.session.snapshot()
        self.assertEqual(snap["state"], "PAUSED")
        self.assertTrue(snap["replan_pending"])
        self.assertEqual(snap["active_trajectory_id"], 101)
        self.assertEqual(snap["target_slot"], "P_B2")


if __name__ == "__main__":
    unittest.main()
