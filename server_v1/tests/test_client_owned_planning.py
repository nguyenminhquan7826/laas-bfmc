#!/usr/bin/env python3
"""Step 3 map-package and client-owned Hybrid A* tests."""

from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
import unittest
from pathlib import Path

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from local_parking_planner_v1 import plan_local_request
from map_package_v1 import MAP_PACKAGE_FILES, load_and_verify_manifest
from server_stub import Handler, ReusableTCPServer, ServerContext


def slots() -> list[dict]:
    return [
        {"id": "P_B1", "state": "OCCUPIED", "confidence": 1.0},
        {"id": "P_B2", "state": "FREE", "confidence": 1.0},
        {"id": "P_T1", "state": "OCCUPIED", "confidence": 1.0},
        {"id": "P_T2", "state": "OCCUPIED", "confidence": 1.0},
    ]


class MapPackageAndLocalPlannerTests(unittest.TestCase):
    def test_checked_in_map_manifest_matches_operational_files(self) -> None:
        manifest = load_and_verify_manifest(SERVER_DIR)
        self.assertEqual(manifest["map_id"], "map_v1")
        self.assertEqual(len(manifest["package_sha256"]), 64)
        self.assertEqual(len(manifest["files"]), 4)

    def test_map_manifest_is_identical_for_windows_crlf_checkout(self) -> None:
        expected = load_and_verify_manifest(SERVER_DIR)
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for name in MAP_PACKAGE_FILES:
                content = (SERVER_DIR / name).read_bytes()
                content = content.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
                (root / name).write_bytes(content)
            (root / "map_manifest_v1.json").write_bytes(
                (SERVER_DIR / "map_manifest_v1.json").read_bytes()
            )

            actual = load_and_verify_manifest(root)

        self.assertEqual(actual, expected)

    def test_local_hybrid_astar_rejects_map_hash_mismatch(self) -> None:
        request = {
            "source_seq": 10,
            "decision": {
                "decision_id": 1,
                "map_id": "map_v1",
                "map_package_sha256": "0" * 64,
                "maneuver": "PARK_AT_SLOT",
                "target_slot": "P_B2",
            },
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
            "slots": slots(),
        }
        result = plan_local_request(SERVER_DIR, request)
        self.assertEqual(result["status"], "REJECTED")
        self.assertEqual(result["reason"], "map_package_sha256_mismatch")

    def test_local_hybrid_astar_builds_valid_targeted_trajectory(self) -> None:
        manifest = load_and_verify_manifest(SERVER_DIR)
        request = {
            "source_seq": 10,
            "decision": {
                "decision_id": 7,
                "map_id": "map_v1",
                "map_package_sha256": manifest["package_sha256"],
                "maneuver": "PARK_AT_SLOT",
                "target_slot": "P_B2",
            },
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
            "slots": slots(),
        }
        result = plan_local_request(SERVER_DIR, request)
        self.assertEqual(result["status"], "READY", result)
        self.assertEqual(result["decision_id"], 7)
        self.assertEqual(result["trajectory"]["trajectory_id"], 7)
        self.assertEqual(result["trajectory"]["target_slot"], "P_B2")
        self.assertEqual(result["trajectory"]["validation"], "PASS")
        self.assertGreater(len(result["trajectory"]["points"]), 1)


class ClientOwnedServerModeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ctx = ServerContext(
            SERVER_DIR,
            planning_enabled=True,
            planning_owner="client",
        )
        self.server = ReusableTCPServer(("127.0.0.1", 0), Handler, self.ctx)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.sock = socket.create_connection(self.server.server_address, timeout=2.0)
        self.file = self.sock.makefile("rwb")

    def tearDown(self) -> None:
        self.file.close()
        self.sock.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def send(self, message: dict) -> None:
        self.file.write((json.dumps(message) + "\n").encode("utf-8"))
        self.file.flush()

    def receive(self) -> dict:
        return json.loads(self.file.readline().decode("utf-8"))

    def test_server_sends_decision_not_trajectory(self) -> None:
        self.send({
            "type": "vehicle_pose", "version": 1, "seq": 10,
            "timestamp_ms": 1000, "map_id": "map_v1",
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
        })
        self.assertEqual(self.receive()["type"], "ack")
        self.send({
            "type": "parking_status", "version": 1, "seq": 11,
            "timestamp_ms": 1001, "map_id": "map_v1", "slots": slots(),
        })
        self.assertEqual(self.receive()["type"], "ack")
        decision = self.receive()
        self.assertEqual(decision["type"], "navigation_decision")
        self.assertEqual(decision["planning_owner"], "client")
        self.assertEqual(decision["maneuver"], "PARK_AT_SLOT")
        self.assertEqual(decision["target_slot"], "P_B2")
        self.assertEqual(
            decision["map_package_sha256"],
            self.ctx.map_manifest["package_sha256"],
        )

        self.send({
            "type": "navigation_decision_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 12,
            "timestamp_ms": 1002,
            "map_id": "map_v1",
            "decision_id": decision["decision_id"],
            "status": "ACCEPTED",
            "reason": "MAP_PACKAGE_ID_MATCH_LOCAL_VERIFY_PENDING",
        })
        ack = self.receive()
        self.assertEqual(ack["type"], "ack")
        self.assertTrue(ack["accepted"], ack)
        monitored = self.ctx.monitoring.vehicle_snapshot("car_01")
        self.assertEqual(monitored["navigation"]["decision_id"], decision["decision_id"])
        self.assertEqual(monitored["navigation_status"]["status"], "ACCEPTED")


if __name__ == "__main__":
    unittest.main()
