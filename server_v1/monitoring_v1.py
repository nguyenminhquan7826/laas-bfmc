#!/usr/bin/env python3
"""Read-only monitoring state and HTTP API for LAAS Parking Server V1.

This module deliberately has no actuator or mission-command endpoint.  It is
the transport-neutral state layer that the BFMC dashboard/Socket.IO adapter can
reuse in the next step.
"""

from __future__ import annotations

import copy
import json
import mimetypes
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import unquote, urlsplit


DEFAULT_VEHICLE_ID = "car_01"
DEFAULT_OFFLINE_AFTER_MS = 3000


def _valid_vehicle_id(value: Any) -> bool:
    return (
        isinstance(value, str)
        and 0 < len(value) <= 64
        and all(ch.isalnum() or ch in "_-" for ch in value)
    )


class VehicleStateStore:
    """Thread-safe latest-value store for read-only dashboard consumers."""

    def __init__(
        self,
        default_vehicle_id: str = DEFAULT_VEHICLE_ID,
        offline_after_ms: int = DEFAULT_OFFLINE_AFTER_MS,
    ) -> None:
        if not _valid_vehicle_id(default_vehicle_id):
            raise ValueError("invalid default_vehicle_id")
        if offline_after_ms <= 0:
            raise ValueError("offline_after_ms must be positive")

        self.default_vehicle_id = default_vehicle_id
        self.offline_after_ms = int(offline_after_ms)
        self._lock = threading.Lock()
        self._started_mono = time.monotonic()
        self._vehicles: dict[str, dict[str, Any]] = {}
        self._connections: dict[str, str] = {}

    @staticmethod
    def _new_vehicle(vehicle_id: str) -> dict[str, Any]:
        return {
            "vehicle_id": vehicle_id,
            "connections": set(),
            "peers": {},
            "last_seen_mono": None,
            "last_seen_utc_ms": None,
            "last_message_type": None,
            "message_count": 0,
            "map_id": None,
            "pose": None,
            "parking": None,
            "trajectory": None,
            "safety": None,
            "runtime": None,
            "session": None,
        }

    def _vehicle_locked(self, vehicle_id: str) -> dict[str, Any]:
        vehicle = self._vehicles.get(vehicle_id)
        if vehicle is None:
            vehicle = self._new_vehicle(vehicle_id)
            self._vehicles[vehicle_id] = vehicle
        return vehicle

    def connection_opened(self, connection_id: str, peer: str) -> None:
        now_mono = time.monotonic()
        now_utc_ms = int(time.time() * 1000.0)
        with self._lock:
            vehicle_id = self.default_vehicle_id
            self._connections[connection_id] = vehicle_id
            vehicle = self._vehicle_locked(vehicle_id)
            vehicle["connections"].add(connection_id)
            vehicle["peers"][connection_id] = peer
            vehicle["last_seen_mono"] = now_mono
            vehicle["last_seen_utc_ms"] = now_utc_ms

    def connection_closed(self, connection_id: str) -> None:
        with self._lock:
            vehicle_id = self._connections.pop(connection_id, None)
            if vehicle_id is None:
                return
            vehicle = self._vehicles.get(vehicle_id)
            if vehicle is not None:
                vehicle["connections"].discard(connection_id)
                vehicle["peers"].pop(connection_id, None)

    def _bind_connection_locked(
        self,
        connection_id: str,
        requested_vehicle_id: Any,
    ) -> str:
        current = self._connections.get(connection_id, self.default_vehicle_id)
        vehicle_id = requested_vehicle_id if _valid_vehicle_id(requested_vehicle_id) else current
        if vehicle_id == current:
            return vehicle_id

        previous = self._vehicles.get(current)
        peer = None
        if previous is not None:
            previous["connections"].discard(connection_id)
            peer = previous["peers"].pop(connection_id, None)

        self._connections[connection_id] = vehicle_id
        vehicle = self._vehicle_locked(vehicle_id)
        vehicle["connections"].add(connection_id)
        if peer is not None:
            vehicle["peers"][connection_id] = peer
        return vehicle_id

    def accept_message(
        self,
        connection_id: str,
        message: dict[str, Any],
        session: Optional[dict[str, Any]] = None,
    ) -> None:
        """Record one protocol-validated Pi message."""

        now_mono = time.monotonic()
        now_utc_ms = int(time.time() * 1000.0)
        msg = copy.deepcopy(message)
        msg_type = str(msg.get("type") or "unknown")

        with self._lock:
            vehicle_id = self._bind_connection_locked(
                connection_id, msg.get("vehicle_id")
            )
            vehicle = self._vehicle_locked(vehicle_id)
            vehicle["last_seen_mono"] = now_mono
            vehicle["last_seen_utc_ms"] = now_utc_ms
            vehicle["last_message_type"] = msg_type
            vehicle["message_count"] += 1
            if isinstance(msg.get("map_id"), str):
                vehicle["map_id"] = msg["map_id"]

            received = {
                "received_utc_ms": now_utc_ms,
                "received_mono": now_mono,
                "message": msg,
            }
            if msg_type == "vehicle_pose":
                vehicle["pose"] = received
            elif msg_type == "parking_status":
                vehicle["parking"] = received
            elif msg_type == "trajectory_status":
                vehicle["trajectory"] = received
            elif msg_type == "safety_event":
                vehicle["safety"] = received
            elif msg_type == "runtime_status":
                vehicle["runtime"] = received

            if session is not None:
                vehicle["session"] = copy.deepcopy(session)

    def update_session(
        self,
        connection_id: str,
        session: dict[str, Any],
    ) -> None:
        with self._lock:
            vehicle_id = self._connections.get(
                connection_id, self.default_vehicle_id
            )
            self._vehicle_locked(vehicle_id)["session"] = copy.deepcopy(session)

    @staticmethod
    def _message_snapshot(
        record: Optional[dict[str, Any]],
        now_mono: float,
    ) -> Optional[dict[str, Any]]:
        if record is None:
            return None
        result = copy.deepcopy(record["message"])
        result["server_received_utc_ms"] = record["received_utc_ms"]
        result["server_receive_age_ms"] = round(
            max(0.0, now_mono - record["received_mono"]) * 1000.0, 1
        )
        return result

    def _snapshot_locked(
        self,
        vehicle: dict[str, Any],
        now_mono: float,
    ) -> dict[str, Any]:
        last_seen_mono = vehicle["last_seen_mono"]
        last_seen_age_ms = None
        if last_seen_mono is not None:
            last_seen_age_ms = round(
                max(0.0, now_mono - last_seen_mono) * 1000.0, 1
            )
        connection_count = len(vehicle["connections"])
        connected = connection_count > 0
        stale = (
            last_seen_age_ms is None
            or last_seen_age_ms > self.offline_after_ms
        )
        return {
            "vehicle_id": vehicle["vehicle_id"],
            "connected": connected,
            "stale": stale,
            "connection_count": connection_count,
            "peers": sorted(vehicle["peers"].values()),
            "last_seen_utc_ms": vehicle["last_seen_utc_ms"],
            "last_seen_age_ms": last_seen_age_ms,
            "offline_after_ms": self.offline_after_ms,
            "last_message_type": vehicle["last_message_type"],
            "message_count": vehicle["message_count"],
            "map_id": vehicle["map_id"],
            "pose": self._message_snapshot(vehicle["pose"], now_mono),
            "parking": self._message_snapshot(vehicle["parking"], now_mono),
            "trajectory": self._message_snapshot(vehicle["trajectory"], now_mono),
            "safety": self._message_snapshot(vehicle["safety"], now_mono),
            "runtime": self._message_snapshot(vehicle["runtime"], now_mono),
            "session": copy.deepcopy(vehicle["session"]),
        }

    def vehicle_snapshot(self, vehicle_id: str) -> Optional[dict[str, Any]]:
        now_mono = time.monotonic()
        with self._lock:
            vehicle = self._vehicles.get(vehicle_id)
            if vehicle is None:
                return None
            return self._snapshot_locked(vehicle, now_mono)

    def vehicles_snapshot(self) -> list[dict[str, Any]]:
        now_mono = time.monotonic()
        with self._lock:
            return [
                self._snapshot_locked(self._vehicles[key], now_mono)
                for key in sorted(self._vehicles)
            ]

    def health_snapshot(self, protocol_version: int, map_id: str) -> dict[str, Any]:
        vehicles = self.vehicles_snapshot()
        return {
            "status": "ok",
            "service": "laas-parking-monitor",
            "read_only": True,
            "protocol_version": protocol_version,
            "map_id": map_id,
            "uptime_ms": round((time.monotonic() - self._started_mono) * 1000.0, 1),
            "vehicles_total": len(vehicles),
            "vehicles_connected": sum(1 for item in vehicles if item["connected"]),
            "vehicles_fresh": sum(1 for item in vehicles if not item["stale"]),
        }


class MonitoringHTTPHandler(BaseHTTPRequestHandler):
    server_version = "LAASMonitoring/1"
    protocol_version = "HTTP/1.1"

    @property
    def store(self) -> VehicleStateStore:
        return self.server.state_store  # type: ignore[attr-defined]

    def _send_json(self, status: int, payload: Any) -> None:
        body = json.dumps(
            payload, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:  # noqa: N802 - stdlib callback name
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802 - stdlib callback name
        path = urlsplit(self.path).path
        if path == "/api/health":
            self._send_json(
                200,
                self.store.health_snapshot(
                    self.server.protocol_version,  # type: ignore[attr-defined]
                    self.server.map_id,  # type: ignore[attr-defined]
                ),
            )
            return

        if path == "/api/vehicles":
            self._send_json(200, {"vehicles": self.store.vehicles_snapshot()})
            return

        if path == "/api/map":
            metadata = self.server.map_metadata  # type: ignore[attr-defined]
            if metadata is None:
                self._send_json(404, {"error": "map_metadata_unavailable"})
            else:
                self._send_json(200, metadata)
            return

        if path == "/api/map/reference":
            reference = self.server.map_reference_path  # type: ignore[attr-defined]
            if reference is None or not reference.is_file():
                self._send_json(404, {"error": "map_reference_unavailable"})
            else:
                self._send_file(reference, "image/png", "no-cache", cors=True)
            return

        if path == "/api/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(
                        {"vehicles": self.store.vehicles_snapshot()},
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                    self.wfile.write(
                        f"event: vehicles\ndata: {payload}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
                    time.sleep(0.5)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return

        prefix = "/api/vehicles/"
        suffix = "/status"
        if path.startswith(prefix) and path.endswith(suffix):
            encoded_id = path[len(prefix):-len(suffix)]
            vehicle_id = unquote(encoded_id).strip("/")
            if not _valid_vehicle_id(vehicle_id):
                self._send_json(400, {"error": "invalid_vehicle_id"})
                return
            status = self.store.vehicle_snapshot(vehicle_id)
            if status is None:
                self._send_json(404, {"error": "vehicle_not_found"})
                return
            self._send_json(200, status)
            return

        if self._serve_dashboard(path):
            return
        self._send_json(404, {"error": "not_found"})

    def _serve_dashboard(self, request_path: str) -> bool:
        root = self.server.dashboard_root  # type: ignore[attr-defined]
        if root is None or not root.is_dir():
            if request_path == "/":
                self._send_json(
                    503,
                    {
                        "error": "dashboard_not_built",
                        "build": "cd server_v1/dashboard/frontend && npm install && npm run build",
                    },
                )
                return True
            return False

        relative = unquote(request_path).lstrip("/") or "index.html"
        candidate = (root / relative).resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            self._send_json(400, {"error": "invalid_dashboard_path"})
            return True

        # Angular client-side routes fall back to index.html. Paths with a file
        # suffix must still return 404 so missing JS/CSS is visible.
        if not candidate.is_file():
            if Path(relative).suffix:
                return False
            candidate = root / "index.html"
        if not candidate.is_file():
            return False

        content_type = mimetypes.guess_type(candidate.name)[0]
        self._send_file(
            candidate,
            content_type or "application/octet-stream",
            "no-cache" if candidate.name == "index.html" else "public, max-age=3600",
        )
        return True

    def _send_file(
        self,
        path: Path,
        content_type: str,
        cache_control: str,
        cors: bool = False,
    ) -> None:
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", cache_control)
        if cors:
            self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: Any) -> None:
        # Keep high-rate dashboard polling out of the planning console.
        return


class MonitoringHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        state_store: VehicleStateStore,
        protocol_version: int,
        map_id: str,
        dashboard_root: Optional[Path] = None,
        map_metadata: Optional[dict[str, Any]] = None,
        map_reference_path: Optional[Path] = None,
    ) -> None:
        self.state_store = state_store
        self.protocol_version = int(protocol_version)
        self.map_id = map_id
        self.dashboard_root = (
            dashboard_root.resolve() if dashboard_root is not None else None
        )
        self.map_metadata = copy.deepcopy(map_metadata)
        self.map_reference_path = (
            map_reference_path.resolve()
            if map_reference_path is not None else None
        )
        super().__init__(server_address, MonitoringHTTPHandler)
