#!/usr/bin/env python3
"""Mock Pi/localization client for server_v1.

Scenarios:
- basic: one pose + parking status -> one validated trajectory.
- replan: exercise session state machine, target invalidation, safety pause/clear,
  replanning, and completion.

No actuator code is present.
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


def expect_ack(file_obj, label: str, state: str | None = None) -> dict:
    msg = read_line(file_obj)
    print(f"[MOCK] {label} ack: {msg}")
    if msg.get("type") != "ack" or not msg.get("accepted"):
        raise SystemExit(f"{label} rejected: {msg}")
    if state is not None:
        actual = (msg.get("session") or {}).get("state")
        if actual != state:
            raise SystemExit(f"{label}: expected session state {state}, got {actual}")
    return msg


def expect_trajectory(file_obj, label: str, target_slot: str | None = None) -> dict:
    result = read_line(file_obj)
    if result.get("type") == "planning_result":
        raise SystemExit(f"{label}: planning_result instead of trajectory: {result}")
    if result.get("type") != "trajectory":
        raise SystemExit(f"{label}: unexpected response: {result}")
    points = result.get("points", [])
    print(
        f"[MOCK] {label} TRAJECTORY PASS tid={result.get('trajectory_id')} "
        f"slot={result.get('target_slot')} points={len(points)} "
        f"cost={result.get('cost')} expansions={result.get('expansions')}"
    )
    if not points:
        raise SystemExit(f"{label}: trajectory has no points")
    if result.get("validation") != "PASS":
        raise SystemExit(f"{label}: validation not PASS")
    if target_slot is not None and result.get("target_slot") != target_slot:
        raise SystemExit(f"{label}: expected target {target_slot}, got {result.get('target_slot')}")
    sess = result.get("session") or {}
    if sess.get("state") != "TRAJECTORY_READY":
        raise SystemExit(f"{label}: expected TRAJECTORY_READY, got {sess.get('state')}")
    allowed = {"FORWARD", "REVERSE"}
    if any(p.get("direction") not in allowed for p in points):
        raise SystemExit(f"{label}: invalid direction")
    return result


def pose_msg(seq: int, x: float, y: float, yaw_deg: float) -> dict:
    return {
        "type": "vehicle_pose",
        "version": 1,
        "seq": seq,
        "timestamp_ms": int(time.time() * 1000),
        "map_id": "map_v1",
        "source": "MOCK_LOCALIZATION",
        "pose": {"x_m": x, "y_m": y, "yaw_rad": math.radians(yaw_deg)},
    }


def parking_msg(seq: int, b2_state: str = "FREE", t2_state: str = "FREE") -> dict:
    return {
        "type": "parking_status",
        "version": 1,
        "seq": seq,
        "timestamp_ms": int(time.time() * 1000),
        "map_id": "map_v1",
        "slots": [
            {"id": "P_B1", "state": "OCCUPIED", "confidence": 0.96},
            {"id": "P_B2", "state": b2_state, "confidence": 0.95},
            {"id": "P_T1", "state": "UNKNOWN", "confidence": 0.45},
            {"id": "P_T2", "state": t2_state, "confidence": 0.95},
        ],
        "objects": [],
    }


def status_msg(seq: int, tid: int, status: str, reason: str | None = None) -> dict:
    msg = {
        "type": "trajectory_status",
        "version": 1,
        "seq": seq,
        "timestamp_ms": int(time.time() * 1000),
        "map_id": "map_v1",
        "trajectory_id": tid,
        "status": status,
    }
    if reason:
        msg["reason"] = reason
    return msg


def safety_msg(seq: int, tid: int, event: str) -> dict:
    return {
        "type": "safety_event",
        "version": 1,
        "seq": seq,
        "timestamp_ms": int(time.time() * 1000),
        "map_id": "map_v1",
        "trajectory_id": tid,
        "event": event,
    }


def run_basic(sock: socket.socket, f, x: float, y: float, yaw_deg: float) -> None:
    send_line(sock, pose_msg(1, x, y, yaw_deg))
    expect_ack(f, "pose", "WAITING_INPUT")

    send_line(sock, parking_msg(2))
    expect_ack(f, "parking", "WAITING_INPUT")
    result = expect_trajectory(f, "basic", "P_B2")

    ages = result.get("input_age_ms_at_plan_start", {})
    print(f"[MOCK] validation=PASS inputAge={ages}")
    Path("mock_trajectory_response.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("[MOCK] wrote mock_trajectory_response.json")
    print("[RESULT] PASS")


def run_replan(sock: socket.socket, f, x: float, y: float, yaw_deg: float) -> None:
    # Initial plan -> P_B2.
    send_line(sock, pose_msg(1, x, y, yaw_deg))
    expect_ack(f, "pose", "WAITING_INPUT")
    send_line(sock, parking_msg(2, "FREE", "FREE"))
    expect_ack(f, "parking-initial", "WAITING_INPUT")
    t1 = expect_trajectory(f, "initial", "P_B2")
    tid1 = int(t1["trajectory_id"])

    # Pi starts executing the first trajectory.
    send_line(sock, status_msg(3, tid1, "EXECUTING"))
    expect_ack(f, "executing-1", "EXECUTING")

    # P_B2 becomes occupied. Server must invalidate tid1 and replan to P_T2.
    send_line(sock, parking_msg(4, "OCCUPIED", "FREE"))
    ack = expect_ack(f, "parking-target-invalidated", "REPLAN")
    if (ack.get("session") or {}).get("replan_count") != 1:
        raise SystemExit("expected replan_count=1 after target invalidation")
    t2 = expect_trajectory(f, "replan-target", "P_T2")
    tid2 = int(t2["trajectory_id"])
    if tid2 == tid1:
        raise SystemExit("replan reused trajectory_id")

    send_line(sock, status_msg(5, tid2, "EXECUTING"))
    expect_ack(f, "executing-2", "EXECUTING")

    # Local Pi safety stop is authoritative; server session only mirrors PAUSED.
    send_line(sock, safety_msg(6, tid2, "PEDESTRIAN_BLOCKING"))
    expect_ack(f, "pedestrian-blocking", "PAUSED")

    # A fresh pose may arrive while paused; it must NOT auto-resume the old path.
    send_line(sock, pose_msg(7, x, y, yaw_deg))
    expect_ack(f, "fresh-pose-while-paused", "PAUSED")

    # Once safety is cleared, old trajectory is not resumed; server replans.
    send_line(sock, safety_msg(8, tid2, "SAFETY_CLEARED"))
    ack = expect_ack(f, "safety-cleared", "REPLAN")
    if (ack.get("session") or {}).get("replan_count") != 2:
        raise SystemExit("expected replan_count=2 after safety clear")
    t3 = expect_trajectory(f, "replan-after-safety", "P_T2")
    tid3 = int(t3["trajectory_id"])
    if tid3 in {tid1, tid2}:
        raise SystemExit("safety replan reused trajectory_id")

    send_line(sock, status_msg(9, tid3, "EXECUTING"))
    expect_ack(f, "executing-3", "EXECUTING")
    send_line(sock, status_msg(10, tid3, "COMPLETED"))
    expect_ack(f, "completed", "COMPLETED")

    send_line(sock, {"type": "session_query", "version": 1, "map_id": "map_v1"})
    session_status = read_line(f)
    print(f"[MOCK] session status: {session_status}")
    sess = session_status.get("session") or {}
    if session_status.get("type") != "session_status" or sess.get("state") != "COMPLETED":
        raise SystemExit("session did not finish COMPLETED")
    if sess.get("replan_count") != 2:
        raise SystemExit(f"expected final replan_count=2, got {sess.get('replan_count')}")

    Path("mock_replan_response.json").write_text(
        json.dumps({"initial": t1, "target_replan": t2, "safety_replan": t3, "session": session_status}, indent=2),
        encoding="utf-8",
    )
    print("[MOCK] wrote mock_replan_response.json")
    print("[RESULT] REPLAN PASS")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--x", type=float, default=1.30)
    parser.add_argument("--y", type=float, default=0.7511)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--scenario", choices=("basic", "replan"), default="basic")
    args = parser.parse_args()

    print(f"[MOCK] connect {args.host}:{args.port} scenario={args.scenario}")
    with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
        sock.settimeout(20.0)
        f = sock.makefile("r", encoding="utf-8", newline="\n")
        if args.scenario == "basic":
            run_basic(sock, f, args.x, args.y, args.yaw_deg)
        else:
            run_replan(sock, f, args.x, args.y, args.yaw_deg)


if __name__ == "__main__":
    main()
