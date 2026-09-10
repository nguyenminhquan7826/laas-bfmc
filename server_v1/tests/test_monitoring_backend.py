#!/usr/bin/env python3
"""Read-only monitoring API and Parking TCP integration tests."""

from __future__ import annotations

import json
import socket
import sys
import threading
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

SERVER_DIR = Path(__file__).resolve().parents[1]
if str(SERVER_DIR) not in sys.path:
    sys.path.insert(0, str(SERVER_DIR))

from monitoring_v1 import MonitoringHTTPServer, VehicleStateStore
from server_stub import MAP_ID, PROTOCOL_VERSION, Handler, ReusableTCPServer, ServerContext


class MonitoringHarness:
    def __init__(self) -> None:
        self.ctx = ServerContext(
            SERVER_DIR,
            planning_enabled=False,
            vehicle_offline_after_ms=500,
        )
        self.parking_server = ReusableTCPServer(
            ("127.0.0.1", 0), Handler, self.ctx
        )
        self.parking_thread = threading.Thread(
            target=self.parking_server.serve_forever, daemon=True
        )
        self.parking_thread.start()
        self.parking_host, self.parking_port = self.parking_server.server_address

        self.http_server = MonitoringHTTPServer(
            ("127.0.0.1", 0),
            self.ctx.monitoring,
            PROTOCOL_VERSION,
            MAP_ID,
            map_metadata={
                "map_id": MAP_ID,
                "width_x_m": 4.469,
                "height_y_m": 4.41,
                "slots": [{"id": "P_B1", "center_m": [2.1, 0.4]}],
            },
            map_reference_path=SERVER_DIR / "map_reference.png",
        )
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, daemon=True
        )
        self.http_thread.start()
        http_host, http_port = self.http_server.server_address
        self.base_url = f"http://{http_host}:{http_port}"

    def close(self) -> None:
        self.parking_server.shutdown()
        self.parking_server.server_close()
        self.parking_thread.join(timeout=2.0)
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)

    def get(self, path: str) -> tuple[int, dict, dict]:
        request = Request(self.base_url + path, method="GET")
        try:
            with urlopen(request, timeout=2.0) as response:
                return (
                    response.status,
                    json.loads(response.read().decode("utf-8")),
                    dict(response.headers),
                )
        except HTTPError as exc:
            return (
                exc.code,
                json.loads(exc.read().decode("utf-8")),
                dict(exc.headers),
            )


class JsonLineClient:
    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(2.0)
        self.file = self.sock.makefile("rwb")

    def send(self, message: dict) -> dict:
        self.file.write(
            (json.dumps(message, separators=(",", ":")) + "\n").encode("utf-8")
        )
        self.file.flush()
        return json.loads(self.file.readline().decode("utf-8"))

    def close(self) -> None:
        try:
            self.file.close()
        finally:
            self.sock.close()


class VehicleStateStoreTests(unittest.TestCase):
    def test_connection_and_validated_message_snapshot(self) -> None:
        store = VehicleStateStore(offline_after_ms=1000)
        store.connection_opened("c1", "127.0.0.1:1234")
        store.accept_message(
            "c1",
            {
                "type": "vehicle_pose",
                "version": 1,
                "seq": 4,
                "timestamp_ms": 100,
                "map_id": MAP_ID,
                "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.1},
            },
            {"session_id": 1, "state": "WAITING_INPUT"},
        )

        status = store.vehicle_snapshot("car_01")
        self.assertIsNotNone(status)
        assert status is not None
        self.assertTrue(status["connected"])
        self.assertFalse(status["stale"])
        self.assertEqual(status["pose"]["seq"], 4)
        self.assertEqual(status["session"]["state"], "WAITING_INPUT")

        store.connection_closed("c1")
        status = store.vehicle_snapshot("car_01")
        assert status is not None
        self.assertFalse(status["connected"])


class MonitoringApiIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.harness = MonitoringHarness()
        self.client = JsonLineClient(
            self.harness.parking_host, self.harness.parking_port
        )

    def tearDown(self) -> None:
        self.client.close()
        self.harness.close()

    def test_health_and_unknown_vehicle(self) -> None:
        status, payload, headers = self.harness.get("/api/health")
        self.assertEqual(status, 200)
        self.assertEqual(payload["status"], "ok")
        self.assertTrue(payload["read_only"])
        self.assertEqual(payload["protocol_version"], 1)
        self.assertEqual(headers.get("Access-Control-Allow-Origin"), "*")

        status, payload, _ = self.harness.get(
            "/api/vehicles/unknown/status"
        )
        self.assertEqual(status, 404)
        self.assertEqual(payload["error"], "vehicle_not_found")

    def test_map_metadata_and_unbuilt_dashboard(self) -> None:
        status, payload, _ = self.harness.get("/api/map")
        self.assertEqual(status, 200)
        self.assertEqual(payload["map_id"], MAP_ID)
        self.assertEqual(payload["width_x_m"], 4.469)
        self.assertEqual(payload["slots"][0]["id"], "P_B1")

        status, payload, _ = self.harness.get("/")
        self.assertEqual(status, 503)
        self.assertEqual(payload["error"], "dashboard_not_built")

    def test_tcp_pose_is_visible_through_http(self) -> None:
        pose = {
            "type": "vehicle_pose",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 10,
            "timestamp_ms": 1234,
            "map_id": MAP_ID,
            "source": "ENCODER_IMU",
            "pose": {"x_m": 1.3, "y_m": 0.751, "yaw_rad": 0.2},
        }
        ack = self.client.send(pose)
        self.assertTrue(ack["accepted"])

        status, payload, _ = self.harness.get(
            "/api/vehicles/car_01/status"
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["connected"])
        self.assertFalse(payload["stale"])
        self.assertEqual(payload["pose"]["source"], "ENCODER_IMU")
        self.assertEqual(payload["pose"]["pose"]["yaw_rad"], 0.2)
        self.assertEqual(payload["session"]["state"], "WAITING_INPUT")

        status, listing, _ = self.harness.get("/api/vehicles")
        self.assertEqual(status, 200)
        self.assertEqual(len(listing["vehicles"]), 1)
        self.assertEqual(listing["vehicles"][0]["vehicle_id"], "car_01")

    def test_invalid_pose_is_not_published(self) -> None:
        invalid = {
            "type": "vehicle_pose",
            "version": 1,
            "seq": 11,
            "timestamp_ms": 1235,
            "map_id": MAP_ID,
            "pose": {"x_m": "bad", "y_m": 0.751, "yaw_rad": 0.0},
        }
        ack = self.client.send(invalid)
        self.assertFalse(ack["accepted"])

        status, payload, _ = self.harness.get(
            "/api/vehicles/car_01/status"
        )
        self.assertEqual(status, 200)
        self.assertIsNone(payload["pose"])
        self.assertEqual(payload["message_count"], 0)

    def test_trajectory_and_safety_state_are_visible(self) -> None:
        self.harness.ctx.session.start_new_session("monitor_test")
        self.harness.ctx.session.mark_trajectory_ready(77, "P_B2")

        trajectory_status = {
            "type": "trajectory_status",
            "version": 1,
            "seq": 20,
            "timestamp_ms": 2000,
            "map_id": MAP_ID,
            "trajectory_id": 77,
            "status": "EXECUTING",
            "reason": "TRACKER_ACTIVE",
        }
        ack = self.client.send(trajectory_status)
        self.assertTrue(ack["accepted"])

        safety_event = {
            "type": "safety_event",
            "version": 1,
            "seq": 21,
            "timestamp_ms": 2001,
            "map_id": MAP_ID,
            "trajectory_id": 77,
            "event": "CRITICAL_OBSTACLE",
        }
        ack = self.client.send(safety_event)
        self.assertTrue(ack["accepted"])

        status, payload, _ = self.harness.get(
            "/api/vehicles/car_01/status"
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["trajectory"]["status"], "EXECUTING")
        self.assertEqual(payload["safety"]["event"], "CRITICAL_OBSTACLE")
        self.assertEqual(payload["session"]["state"], "PAUSED")
        self.assertEqual(payload["session"]["pause_reason"], "CRITICAL_OBSTACLE")

    def test_runtime_tracker_uart_and_sync_are_visible(self) -> None:
        runtime_status = {
            "type": "runtime_status",
            "version": 1,
            "vehicle_id": "car_01",
            "seq": 30,
            "timestamp_ms": 3000,
            "map_id": MAP_ID,
            "operating_mode": "PARKING",
            "tracker": {
                "valid": True,
                "goal_reached": False,
                "trajectory_id": 88,
                "nearest_index": 12,
                "target_index": 17,
                "cross_track_error_m": 0.031,
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
                "telemetry_age_ms": 14,
            },
            "session_sync": {
                "hold": False,
                "reason": "SYNC_READY",
            },
        }
        ack = self.client.send(runtime_status)
        self.assertTrue(ack["accepted"])

        status, payload, _ = self.harness.get(
            "/api/vehicles/car_01/status"
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["runtime"]["tracker"]["nearest_index"], 12)
        self.assertEqual(
            payload["runtime"]["tracker"]["cross_track_error_m"], 0.031
        )
        self.assertTrue(payload["runtime"]["uart"]["rx_enabled"])
        self.assertFalse(payload["runtime"]["uart"]["tx_enabled"])
        self.assertEqual(
            payload["runtime"]["session_sync"]["reason"], "SYNC_READY"
        )


if __name__ == "__main__":
    unittest.main()
