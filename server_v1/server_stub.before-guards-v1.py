#!/usr/bin/env python3
"""LAAS Parking Server V1: TCP/NDJSON + offline Hybrid A* integration.

Safety scope:
- Planning/simulation only.
- Does not talk to STM32, UART, motors, or steering actuators.
- Full vehicle footprint is still NOT verified.
"""

from __future__ import annotations

import argparse
import json
import math
import socketserver
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from hybrid_astar_v1 import HybridAStarPlanner, Pose, load_yaml
from slot_selector_v1 import SlotPlan, choose_best_free_slot

PROTOCOL_VERSION = 1
MAP_ID = "map_v1"
SLOT_IDS = {"P_B1", "P_B2", "P_T1", "P_T2"}
SLOT_STATES = {"UNKNOWN", "FREE", "OCCUPIED"}


def validate_common(msg: dict[str, Any]) -> tuple[bool, str]:
    if msg.get("version") != PROTOCOL_VERSION:
        return False, "unsupported_version"
    if msg.get("map_id") not in (None, MAP_ID):
        return False, "map_id_mismatch"
    return True, "ok"


def validate_vehicle_pose(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    if msg.get("map_id") != MAP_ID:
        return False, "map_id_required"
    pose = msg.get("pose")
    if not isinstance(pose, dict):
        return False, "pose_not_object"
    for key in ("x_m", "y_m", "yaw_rad"):
        value = pose.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return False, f"invalid_pose:{key}"
    return True, "ok"


def validate_parking_status(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    if msg.get("map_id") != MAP_ID:
        return False, "map_id_required"
    slots = msg.get("slots")
    if not isinstance(slots, list):
        return False, "slots_not_list"
    seen: set[str] = set()
    for slot in slots:
        if not isinstance(slot, dict):
            return False, "slot_not_object"
        slot_id = slot.get("id")
        state = slot.get("state")
        conf = slot.get("confidence")
        if slot_id not in SLOT_IDS:
            return False, f"invalid_slot:{slot_id}"
        if slot_id in seen:
            return False, f"duplicate_slot:{slot_id}"
        seen.add(slot_id)
        if state not in SLOT_STATES:
            return False, f"invalid_state:{state}"
        if not isinstance(conf, (int, float)) or not 0.0 <= float(conf) <= 1.0:
            return False, f"invalid_confidence:{slot_id}"
    if seen != SLOT_IDS:
        return False, "all_four_slots_required"
    return True, "ok"


def slot_states_from_status(msg: dict[str, Any]) -> Dict[str, str]:
    return {str(s["id"]): str(s["state"]).upper() for s in msg["slots"]}


def pose_from_message(msg: dict[str, Any]) -> Pose:
    p = msg["pose"]
    return Pose(float(p["x_m"]), float(p["y_m"]), float(p["yaw_rad"]))


def trajectory_points(selected: SlotPlan, nominal_speed: float) -> list[dict[str, Any]]:
    path = selected.result.path
    points: list[dict[str, Any]] = []
    for i, node in enumerate(path):
        direction = node.direction
        if direction == 0:
            if i + 1 < len(path):
                direction = path[i + 1].direction
            else:
                direction = +1
        direction_text = "FORWARD" if direction >= 0 else "REVERSE"
        speed = abs(nominal_speed) if direction >= 0 else -abs(nominal_speed)
        points.append({
            "x_m": round(float(node.x), 6),
            "y_m": round(float(node.y), 6),
            "yaw_rad": round(float(node.yaw), 7),
            "v_ref_mps": round(speed, 4),
            "direction": direction_text,
        })
    return points


class ServerContext:
    def __init__(self, root: Path, planning_enabled: bool):
        self.root = root
        self.planning_enabled = planning_enabled
        self.map_cfg = load_yaml(root / "map_v1.yaml")
        self.vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
        self.planner_cfg = load_yaml(root / "planner_v1.yaml")
        self.planner = HybridAStarPlanner(self.map_cfg, self.vehicle_cfg, self.planner_cfg)
        self.nominal_speed = float(self.vehicle_cfg.get("motion", {}).get("parking_nominal_speed_mps", 0.10))

        self.state_lock = threading.Lock()
        self.planning_lock = threading.Lock()
        self.latest_pose: Optional[Pose] = None
        self.latest_pose_seq: Optional[int] = None
        self.latest_parking_status: Optional[dict[str, Any]] = None
        self.trajectory_id = 0

    def next_trajectory_id(self) -> int:
        with self.state_lock:
            self.trajectory_id += 1
            return self.trajectory_id

    def set_pose(self, pose: Pose, seq: Any) -> None:
        with self.state_lock:
            self.latest_pose = pose
            self.latest_pose_seq = seq if isinstance(seq, int) else None

    def set_parking_status(self, msg: dict[str, Any]) -> None:
        with self.state_lock:
            self.latest_parking_status = msg

    def get_pose(self) -> Optional[Pose]:
        with self.state_lock:
            return self.latest_pose


class Handler(socketserver.StreamRequestHandler):
    @property
    def ctx(self) -> ServerContext:
        return self.server.context  # type: ignore[attr-defined]

    def send_json(self, obj: dict[str, Any]) -> None:
        self.wfile.write((json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8"))
        self.wfile.flush()

    def send_ack(self, msg: dict[str, Any], ok: bool, reason: str) -> None:
        self.send_json({
            "type": "ack",
            "version": PROTOCOL_VERSION,
            "seq": msg.get("seq"),
            "accepted": ok,
            "reason": reason,
        })

    def plan_and_send(self, parking_msg: dict[str, Any]) -> None:
        if not self.ctx.planning_enabled:
            return

        start = self.ctx.get_pose()
        if start is None:
            self.send_json({
                "type": "planning_result",
                "version": PROTOCOL_VERSION,
                "map_id": MAP_ID,
                "source_seq": parking_msg.get("seq"),
                "status": "WAITING_FOR_POSE",
                "reason": "no_vehicle_pose",
            })
            return

        states = slot_states_from_status(parking_msg)
        free_slots = [sid for sid, state in states.items() if state == "FREE"]
        if not free_slots:
            self.send_json({
                "type": "planning_result",
                "version": PROTOCOL_VERSION,
                "map_id": MAP_ID,
                "source_seq": parking_msg.get("seq"),
                "status": "NO_FREE_SLOT",
                "reason": "no_slot_in_FREE_state",
            })
            return

        with self.ctx.planning_lock:
            selected, candidates = choose_best_free_slot(
                self.ctx.planner,
                self.ctx.map_cfg,
                self.ctx.vehicle_cfg,
                start,
                states,
            )

        if selected is None:
            self.send_json({
                "type": "planning_result",
                "version": PROTOCOL_VERSION,
                "map_id": MAP_ID,
                "source_seq": parking_msg.get("seq"),
                "status": "NO_FEASIBLE_TRAJECTORY",
                "reason": "hybrid_astar_rejected_all_free_slots",
                "candidates": [
                    {
                        "slot_id": c.slot_id,
                        "success": c.result.success,
                        "reason": c.result.reason,
                        "expansions": c.result.expansions,
                    }
                    for c in candidates
                ],
            })
            print(f"[PLAN] seq={parking_msg.get('seq')} no feasible trajectory")
            return

        tid = self.ctx.next_trajectory_id()
        points = trajectory_points(selected, self.ctx.nominal_speed)
        response = {
            "type": "trajectory",
            "version": PROTOCOL_VERSION,
            "trajectory_id": tid,
            "source_seq": parking_msg.get("seq"),
            "map_id": MAP_ID,
            "target_slot": selected.slot_id,
            "reference_point": "rear_axle_center",
            "goal_mode": selected.goal_mode,
            "cost": round(float(selected.result.cost), 6),
            "expansions": int(selected.result.expansions),
            "prototype_warning": "OFFLINE_ONLY_FULL_VEHICLE_FOOTPRINT_NOT_VERIFIED",
            "points": points,
        }
        self.send_json(response)
        print(
            f"[PLAN] tid={tid} seq={parking_msg.get('seq')} slot={selected.slot_id} "
            f"cost={selected.result.cost:.3f} expansions={selected.result.expansions} points={len(points)}"
        )

    def handle(self) -> None:
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        print(f"[SERVER] connected {peer}")
        try:
            for raw in self.rfile:
                try:
                    msg = json.loads(raw.decode("utf-8"))
                except Exception as exc:
                    self.send_json({"type": "error", "version": PROTOCOL_VERSION, "reason": f"invalid_json:{exc}"})
                    continue
                if not isinstance(msg, dict):
                    self.send_json({"type": "error", "version": PROTOCOL_VERSION, "reason": "json_root_not_object"})
                    continue

                msg_type = msg.get("type")
                if msg_type == "vehicle_pose":
                    ok, reason = validate_vehicle_pose(msg)
                    if ok:
                        pose = pose_from_message(msg)
                        self.ctx.set_pose(pose, msg.get("seq"))
                        print(
                            f"[POSE] seq={msg.get('seq')} x={pose.x:.3f} y={pose.y:.3f} "
                            f"yaw={math.degrees(pose.yaw):.1f}deg"
                        )
                    self.send_ack(msg, ok, reason)

                elif msg_type == "parking_status":
                    ok, reason = validate_parking_status(msg)
                    if ok:
                        self.ctx.set_parking_status(msg)
                        free_slots = [s["id"] for s in msg["slots"] if s["state"] == "FREE"]
                        print(f"[PARKING] seq={msg.get('seq')} free={free_slots}")
                    self.send_ack(msg, ok, reason)
                    if ok:
                        self.plan_and_send(msg)

                elif msg_type in {"safety_event", "trajectory_status"}:
                    ok, reason = validate_common(msg)
                    if ok:
                        print(f"[{str(msg_type).upper()}] {msg}")
                    self.send_ack(msg, ok, reason)

                else:
                    self.send_json({
                        "type": "error",
                        "version": PROTOCOL_VERSION,
                        "reason": "unsupported_message_type",
                    })
        finally:
            print(f"[SERVER] disconnected {peer}")


class ReusableTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, request_handler_class, context: ServerContext):
        self.context = context
        super().__init__(server_address, request_handler_class)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--transport-only", action="store_true", help="disable Hybrid A* and behave as transport/validation server only")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    ctx = ServerContext(root, planning_enabled=not args.transport_only)

    with ReusableTCPServer((args.host, args.port), Handler, ctx) as server:
        mode = "transport-only" if args.transport_only else "hybrid-a*-offline"
        print(f"[SERVER] listening {args.host}:{args.port} protocol=v1 map={MAP_ID} mode={mode}")
        print("[SAFETY] planning only; no actuator interface is present in this server")
        server.serve_forever()


if __name__ == "__main__":
    main()
