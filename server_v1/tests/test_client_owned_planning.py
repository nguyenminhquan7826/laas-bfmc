#!/usr/bin/env python3
"""Step 3 map-package and client-owned Hybrid A* tests."""

from __future__ import annotations

import json
import math
import socket
import subprocess
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
from slot_selector_v1 import desired_body_center_for_slot


def slots() -> list[dict]:
    return [
        {"id": "P_B1", "state": "OCCUPIED", "confidence": 1.0},
        {"id": "P_B2", "state": "FREE", "confidence": 1.0},
        {"id": "P_T1", "state": "OCCUPIED", "confidence": 1.0},
        {"id": "P_T2", "state": "OCCUPIED", "confidence": 1.0},
    ]


class MapPackageAndLocalPlannerTests(unittest.TestCase):
    def test_all_map_slots_have_parallel_body_goal_yaw(self) -> None:
        ctx = ServerContext(SERVER_DIR, planning_enabled=True)
        for slot in ctx.map_cfg["slots"]:
            body = desired_body_center_for_slot(slot)
            self.assertAlmostEqual(math.sin(body.yaw), 0.0, places=7)
            expected_cos = 1.0 if slot["row"] == "bottom" else -1.0
            self.assertAlmostEqual(math.cos(body.yaw), expected_cos, places=7)

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
        final_yaw = result["trajectory"]["points"][-1]["yaw_rad"]
        self.assertLessEqual(abs(final_yaw), math.radians(12.0))

    def test_protocol_output_emits_standard_trajectory_for_cpp_bridge(self) -> None:
        manifest = load_and_verify_manifest(SERVER_DIR)
        request = {
            "source_seq": 10,
            "decision": {
                "decision_id": 9,
                "map_id": "map_v1",
                "map_package_sha256": manifest["package_sha256"],
                "maneuver": "PARK_AT_SLOT",
                "target_slot": "P_B2",
            },
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
            "slots": slots(),
        }
        completed = subprocess.run(
            [
                sys.executable,
                str(SERVER_DIR / "local_parking_planner_v1.py"),
                "--root",
                str(SERVER_DIR),
                "--protocol-output",
            ],
            input=json.dumps(request) + "\n",
            text=True,
            capture_output=True,
            timeout=20,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        trajectory = json.loads(completed.stdout)
        self.assertEqual(trajectory["type"], "trajectory")
        self.assertEqual(trajectory["trajectory_id"], 9)
        self.assertEqual(trajectory["decision_id"], 9)
        self.assertEqual(trajectory["planning_owner"], "client")
        self.assertEqual(
            trajectory["map_package_sha256"], manifest["package_sha256"]
        )


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

        self.send({
            "type": "navigation_decision_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 13,
            "timestamp_ms": 1013,
            "map_id": "map_v1",
            "decision_id": decision["decision_id"],
            "status": "PLANNING",
            "reason": "LOCAL_HYBRID_A_STAR_RUNNING",
        })
        self.assertTrue(self.receive()["accepted"])

        # READY is not authoritative by itself: the Server must have received
        # and validated the Pi-owned trajectory first.
        self.send({
            "type": "navigation_decision_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 14,
            "timestamp_ms": 1014,
            "map_id": "map_v1",
            "decision_id": decision["decision_id"],
            "status": "READY",
            "reason": "LOCAL_HYBRID_A_STAR_PASS_PI_VALIDATED",
        })
        premature_ready = self.receive()
        self.assertFalse(premature_ready["accepted"], premature_ready)
        self.assertEqual(
            premature_ready["reason"],
            "local_trajectory_missing_before_ready",
        )

        local_request = {
            "source_seq": decision["source_seq"],
            "decision": decision,
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.0},
            "slots": slots(),
        }
        local_result = plan_local_request(SERVER_DIR, local_request)
        self.assertEqual(local_result["status"], "READY", local_result)
        local_trajectory = dict(local_result["trajectory"])
        local_trajectory.update({
            "type": "local_trajectory",
            "seq": 15,
            "timestamp_ms": 1015,
            "decision_id": decision["decision_id"],
            "map_package_sha256": decision["map_package_sha256"],
            "planning_owner": "client",
        })

        wrong_map_trajectory = dict(local_trajectory)
        wrong_map_trajectory["map_package_sha256"] = "0" * 64
        self.send(wrong_map_trajectory)
        wrong_map_ack = self.receive()
        self.assertFalse(wrong_map_ack["accepted"], wrong_map_ack)
        self.assertEqual(
            wrong_map_ack["reason"],
            "local_trajectory_map_package_mismatch",
        )

        local_trajectory["seq"] = 16
        local_trajectory["timestamp_ms"] = 1016
        self.send(local_trajectory)
        trajectory_ack = self.receive()
        self.assertTrue(trajectory_ack["accepted"], trajectory_ack)

        self.send({
            "type": "navigation_decision_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 17,
            "timestamp_ms": 1017,
            "map_id": "map_v1",
            "decision_id": decision["decision_id"],
            "status": "READY",
            "reason": "LOCAL_HYBRID_A_STAR_PASS_PI_VALIDATED",
        })
        ack = self.receive()
        self.assertTrue(ack["accepted"], ack)

        self.assertEqual(ack["session"]["state"], "TRAJECTORY_READY")
        self.assertEqual(
            ack["session"]["active_trajectory_id"], decision["decision_id"]
        )
        self.assertEqual(ack["session"]["target_slot"], "P_B2")
        monitored = self.ctx.monitoring.vehicle_snapshot("car_01")
        self.assertEqual(monitored["navigation_status"]["status"], "READY")
        self.assertEqual(
            monitored["local_trajectory"]["trajectory_id"],
            decision["decision_id"],
        )
        self.assertEqual(monitored["session"]["state"], "TRAJECTORY_READY")


if __name__ == "__main__":
    unittest.main()
