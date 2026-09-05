#!/usr/bin/env python3
"""TCP/NDJSON integration tests for Server V1 safety-event handling."""

from __future__ import annotations

import json
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from server_stub import MAP_ID, Handler, ReusableTCPServer, ServerContext


class ServerHarness:
    def __init__(self) -> None:
        self.ctx = ServerContext(SERVER_DIR, planning_enabled=False)
        self.server = ReusableTCPServer(("127.0.0.1", 0), Handler, self.ctx)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.host, self.port = self.server.server_address

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)


class JsonLineClient:
    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(2.0)
        self.file = self.sock.makefile("rwb")

    def close(self) -> None:
        try:
            self.file.close()
        finally:
            self.sock.close()

    def send(self, msg: dict) -> dict:
        self.file.write((json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8"))
        self.file.flush()
        raw = self.file.readline()
        if not raw:
            raise RuntimeError("server closed connection before response")
        return json.loads(raw.decode("utf-8"))

    def send_fragmented(self, msg: dict, split_at: int) -> dict:
        payload = (json.dumps(msg, separators=(",", ":")) + "\n").encode("utf-8")
        split_at = max(1, min(split_at, len(payload) - 1))
        self.file.write(payload[:split_at])
        self.file.flush()
        time.sleep(0.01)
        self.file.write(payload[split_at:])
        self.file.flush()
        raw = self.file.readline()
        if not raw:
            raise RuntimeError("server closed connection before fragmented response")
        return json.loads(raw.decode("utf-8"))


class SafetyEventTcpTests(unittest.TestCase):
    def setUp(self) -> None:
        self.harness = ServerHarness()
        self.harness.ctx.session.start_new_session("test_start")
        self.harness.ctx.session.mark_trajectory_ready(77, "P_B2")
        self.client = JsonLineClient(self.harness.host, self.harness.port)

    def tearDown(self) -> None:
        self.client.close()
        self.harness.close()

    def safety_event(self, event: str, trajectory_id: int = 77, seq: int = 1) -> dict:
        return {
            "type": "safety_event",
            "version": 1,
            "seq": seq,
            "timestamp_ms": 1000 + seq,
            "map_id": MAP_ID,
            "trajectory_id": trajectory_id,
            "event": event,
        }

    def test_critical_obstacle_then_clear_replans(self) -> None:
        ack = self.client.send(self.safety_event("CRITICAL_OBSTACLE", seq=1))
        self.assertEqual(ack["type"], "ack")
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "PAUSED")
        self.assertEqual(ack["session"]["pause_reason"], "CRITICAL_OBSTACLE")

        ack = self.client.send(self.safety_event("SAFETY_CLEARED", seq=2))
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "REPLAN")
        self.assertTrue(ack["session"]["replan_pending"])
        self.assertIsNone(ack["session"]["active_trajectory_id"])
        self.assertEqual(ack["session"]["last_target_slot"], "P_B2")

    def test_server_timeout_after_reconnect_then_clear_replans(self) -> None:
        self.client.close()
        time.sleep(0.02)

        self.client = JsonLineClient(self.harness.host, self.harness.port)
        ack = self.client.send(self.safety_event("SERVER_TIMEOUT", seq=3))
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "PAUSED")
        self.assertEqual(ack["session"]["pause_reason"], "SERVER_TIMEOUT")

        ack = self.client.send(self.safety_event("SAFETY_CLEARED", seq=4))
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "REPLAN")
        self.assertTrue(ack["session"]["replan_pending"])

    def test_wrong_trajectory_id_is_rejected(self) -> None:
        ack = self.client.send(self.safety_event("CRITICAL_OBSTACLE", trajectory_id=999, seq=5))
        self.assertFalse(ack["accepted"])
        self.assertEqual(ack["reason"], "trajectory_id_mismatch")
        self.assertEqual(ack["session"]["state"], "TRAJECTORY_READY")

    def test_safety_clear_without_pause_is_rejected(self) -> None:
        ack = self.client.send(self.safety_event("SAFETY_CLEARED", seq=6))
        self.assertFalse(ack["accepted"])
        self.assertEqual(
            ack["reason"],
            "invalid_state_for_SAFETY_CLEARED:TRAJECTORY_READY",
        )
        self.assertEqual(ack["session"]["state"], "TRAJECTORY_READY")

    def test_trajectory_invalid_requests_replan_without_clear(self) -> None:
        ack = self.client.send(self.safety_event("TRAJECTORY_INVALID", seq=7))
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "REPLAN")
        self.assertTrue(ack["session"]["replan_pending"])
        self.assertIsNone(ack["session"]["active_trajectory_id"])

    def test_fragmented_ndjson_message_is_accepted(self) -> None:
        msg = self.safety_event("CRITICAL_OBSTACLE", seq=8)
        ack = self.client.send_fragmented(msg, split_at=13)
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "PAUSED")

    def test_invalid_json_returns_error_without_killing_connection(self) -> None:
        self.client.file.write(b'{"type":"safety_event"\n')
        self.client.file.flush()
        first = json.loads(self.client.file.readline().decode("utf-8"))
        self.assertEqual(first["type"], "error")
        self.assertTrue(first["reason"].startswith("invalid_json:"))

        ack = self.client.send(self.safety_event("CRITICAL_OBSTACLE", seq=9))
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["session"]["state"], "PAUSED")


if __name__ == "__main__":
    unittest.main()
