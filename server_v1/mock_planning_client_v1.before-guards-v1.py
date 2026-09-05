#!/usr/bin/env python3
"""Mock Pi/localization client for server_v1.

Sends one vehicle_pose, then one parking_status, and expects an offline
trajectory or an explicit planning_result. No actuator code is present.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from pathlib import Path


def send_line(sock: socket.socket, obj: dict) -> None:
    sock.sendall((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))


def read_line(file_obj) -> dict:
    raw = file_obj.readline()
    if not raw:
        raise RuntimeError("server_closed_connection")
    return json.loads(raw)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--x", type=float, default=1.30)
    parser.add_argument("--y", type=float, default=0.7511)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    args = parser.parse_args()

    now = int(time.time() * 1000)
    pose_msg = {
        "type": "vehicle_pose",
        "version": 1,
        "seq": 1,
        "timestamp_ms": now,
        "map_id": "map_v1",
        "source": "MOCK_LOCALIZATION",
        "pose": {
            "x_m": args.x,
            "y_m": args.y,
            "yaw_rad": math.radians(args.yaw_deg),
        },
    }
    parking_msg = {
        "type": "parking_status",
        "version": 1,
        "seq": 2,
        "timestamp_ms": now + 1,
        "map_id": "map_v1",
        "slots": [
            {"id": "P_B1", "state": "OCCUPIED", "confidence": 0.96},
            {"id": "P_B2", "state": "FREE", "confidence": 0.91},
            {"id": "P_T1", "state": "UNKNOWN", "confidence": 0.45},
            {"id": "P_T2", "state": "FREE", "confidence": 0.95},
        ],
        "objects": [],
    }

    print(f"[MOCK] connect {args.host}:{args.port}")
    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.settimeout(15.0)
        f = sock.makefile("r", encoding="utf-8", newline="\n")

        send_line(sock, pose_msg)
        pose_ack = read_line(f)
        print(f"[MOCK] pose ack: {pose_ack}")
        if pose_ack.get("type") != "ack" or not pose_ack.get("accepted"):
            raise SystemExit("pose rejected")

        send_line(sock, parking_msg)
        parking_ack = read_line(f)
        print(f"[MOCK] parking ack: {parking_ack}")
        if parking_ack.get("type") != "ack" or not parking_ack.get("accepted"):
            raise SystemExit("parking status rejected")

        result = read_line(f)
        if result.get("type") == "trajectory":
            points = result.get("points", [])
            print(
                f"[MOCK] TRAJECTORY PASS tid={result.get('trajectory_id')} "
                f"slot={result.get('target_slot')} points={len(points)} "
                f"cost={result.get('cost')} expansions={result.get('expansions')}"
            )
            if not points:
                raise SystemExit("trajectory has no points")
            allowed = {"FORWARD", "REVERSE"}
            if any(p.get("direction") not in allowed for p in points):
                raise SystemExit("invalid direction in trajectory")
            Path("mock_trajectory_response.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            print("[MOCK] wrote mock_trajectory_response.json")
            print("[RESULT] PASS")
            return

        if result.get("type") == "planning_result":
            print(f"[MOCK] planning result: {result}")
            raise SystemExit(2)

        raise SystemExit(f"unexpected server response: {result}")


if __name__ == "__main__":
    main()
