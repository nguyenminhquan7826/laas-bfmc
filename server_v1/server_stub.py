#!/usr/bin/env python3
"""LAAS Parking Server V1: TCP/NDJSON + offline Hybrid A* + safety guards.

Safety scope:
- Planning/simulation only.
- Does not talk to STM32, UART, motors, or steering actuators.
- Full vehicle footprint is still NOT verified.
- Staleness thresholds are configurable prototype guards, not frozen safety limits.
"""

from __future__ import annotations

import argparse
import json
import math
import socketserver
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from hybrid_astar_v1 import HybridAStarPlanner, Node, Pose, build_slot_obstacles, load_yaml
from slot_selector_v1 import SlotPlan, choose_best_free_slot
from parking_session_v1 import ParkingSession

PROTOCOL_VERSION = 1
MAP_ID = "map_v1"
SLOT_IDS = {"P_B1", "P_B2", "P_T1", "P_T2"}
SLOT_STATES = {"UNKNOWN", "FREE", "OCCUPIED"}
ALLOWED_DIRECTIONS = {"FORWARD", "REVERSE"}
TRAJECTORY_STATUSES = {"RECEIVED", "EXECUTING", "PAUSED", "COMPLETED", "REJECTED", "REPLAN_REQUESTED"}
SAFETY_EVENTS = {"PEDESTRIAN_BLOCKING", "CRITICAL_OBSTACLE", "TRAJECTORY_INVALID", "SERVER_TIMEOUT", "SAFETY_CLEARED"}

DEFAULT_POSE_MAX_AGE_MS = 2000
DEFAULT_PARKING_MAX_AGE_MS = 3000


def validate_common(msg: dict[str, Any]) -> tuple[bool, str]:
    if msg.get("version") != PROTOCOL_VERSION:
        return False, "unsupported_version"
    if msg.get("map_id") not in (None, MAP_ID):
        return False, "map_id_mismatch"
    return True, "ok"


def validate_seq_and_timestamp(msg: dict[str, Any]) -> tuple[bool, str]:
    seq = msg.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        return False, "invalid_seq"
    timestamp_ms = msg.get("timestamp_ms")
    if not isinstance(timestamp_ms, int) or isinstance(timestamp_ms, bool) or timestamp_ms < 0:
        return False, "invalid_timestamp_ms"
    return True, "ok"


def validate_vehicle_pose(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    ok, reason = validate_seq_and_timestamp(msg)
    if not ok:
        return ok, reason
    if msg.get("map_id") != MAP_ID:
        return False, "map_id_required"
    pose = msg.get("pose")
    if not isinstance(pose, dict):
        return False, "pose_not_object"
    for key in ("x_m", "y_m", "yaw_rad"):
        value = pose.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            return False, f"invalid_pose:{key}"
    return True, "ok"


def validate_parking_status(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    ok, reason = validate_seq_and_timestamp(msg)
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
        if not isinstance(conf, (int, float)) or isinstance(conf, bool) or not 0.0 <= float(conf) <= 1.0:
            return False, f"invalid_confidence:{slot_id}"
    if seen != SLOT_IDS:
        return False, "all_four_slots_required"
    return True, "ok"


def validate_plan_request(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    ok, reason = validate_seq_and_timestamp(msg)
    if not ok:
        return ok, reason
    if msg.get("map_id") != MAP_ID:
        return False, "map_id_required"
    return True, "ok"


def validate_trajectory_status(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    trajectory_id = msg.get("trajectory_id")
    if not isinstance(trajectory_id, int) or isinstance(trajectory_id, bool) or trajectory_id <= 0:
        return False, "invalid_trajectory_id"
    status = msg.get("status")
    if status not in TRAJECTORY_STATUSES:
        return False, f"invalid_trajectory_status:{status}"
    return True, "ok"


def validate_safety_event(msg: dict[str, Any]) -> tuple[bool, str]:
    ok, reason = validate_common(msg)
    if not ok:
        return ok, reason
    event = msg.get("event")
    if event not in SAFETY_EVENTS:
        return False, f"invalid_safety_event:{event}"
    trajectory_id = msg.get("trajectory_id")
    if trajectory_id is not None and (not isinstance(trajectory_id, int) or isinstance(trajectory_id, bool) or trajectory_id <= 0):
        return False, "invalid_trajectory_id"
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
    def __init__(
        self,
        root: Path,
        planning_enabled: bool,
        pose_max_age_ms: int = DEFAULT_POSE_MAX_AGE_MS,
        parking_max_age_ms: int = DEFAULT_PARKING_MAX_AGE_MS,
    ):
        self.root = root
        self.planning_enabled = planning_enabled
        self.pose_max_age_ms = int(pose_max_age_ms)
        self.parking_max_age_ms = int(parking_max_age_ms)

        self.map_cfg = load_yaml(root / "map_v1.yaml")
        self.vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
        self.planner_cfg = load_yaml(root / "planner_v1.yaml")
        self.planner = HybridAStarPlanner(self.map_cfg, self.vehicle_cfg, self.planner_cfg)
        self.nominal_speed = float(self.vehicle_cfg.get("motion", {}).get("parking_nominal_speed_mps", 0.10))

        self.state_lock = threading.Lock()
        self.planning_lock = threading.Lock()
        self.latest_pose: Optional[Pose] = None
        self.latest_pose_seq: Optional[int] = None
        self.latest_pose_received_mono: Optional[float] = None
        self.pose_generation = 0

        self.latest_parking_status: Optional[dict[str, Any]] = None
        self.latest_parking_received_mono: Optional[float] = None
        self.parking_generation = 0

        self.trajectory_id = 0
        self.session = ParkingSession()

    def next_trajectory_id(self) -> int:
        with self.state_lock:
            self.trajectory_id += 1
            return self.trajectory_id

    def set_pose(self, pose: Pose, seq: Any) -> None:
        with self.state_lock:
            self.latest_pose = pose
            self.latest_pose_seq = seq if isinstance(seq, int) and not isinstance(seq, bool) else None
            self.latest_pose_received_mono = time.monotonic()
            self.pose_generation += 1

    def set_parking_status(self, msg: dict[str, Any]) -> None:
        with self.state_lock:
            self.latest_parking_status = msg
            self.latest_parking_received_mono = time.monotonic()
            self.parking_generation += 1

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.state_lock:
            pose_age_ms = None
            if self.latest_pose_received_mono is not None:
                pose_age_ms = (now - self.latest_pose_received_mono) * 1000.0
            parking_age_ms = None
            if self.latest_parking_received_mono is not None:
                parking_age_ms = (now - self.latest_parking_received_mono) * 1000.0
            return {
                "pose": self.latest_pose,
                "pose_seq": self.latest_pose_seq,
                "pose_age_ms": pose_age_ms,
                "pose_generation": self.pose_generation,
                "parking_status": self.latest_parking_status,
                "parking_age_ms": parking_age_ms,
                "parking_generation": self.parking_generation,
            }

    def generations_unchanged(self, pose_generation: int, parking_generation: int) -> bool:
        with self.state_lock:
            return self.pose_generation == pose_generation and self.parking_generation == parking_generation


def freshness_reason(ctx: ServerContext, snap: dict[str, Any]) -> tuple[bool, str]:
    if snap["pose"] is None:
        return False, "no_vehicle_pose"
    if snap["parking_status"] is None:
        return False, "no_parking_status"
    pose_age = snap["pose_age_ms"]
    parking_age = snap["parking_age_ms"]
    if pose_age is None or pose_age > ctx.pose_max_age_ms:
        return False, "pose_stale"
    if parking_age is None or parking_age > ctx.parking_max_age_ms:
        return False, "parking_status_stale"
    return True, "ok"


def normalize_source_seq(source_seq: Any, snap: dict[str, Any]) -> int:
    """Return a protocol-valid source sequence for every trajectory.

    Safety events do not currently carry a `seq`, but safety clear/invalid can
    trigger replanning. In that case, anchor the trajectory to the freshest
    accepted Pi input sequence instead of serializing `null`, which the C++
    protocol decoder correctly rejects.
    """
    if isinstance(source_seq, int) and not isinstance(source_seq, bool) and source_seq >= 0:
        return source_seq

    pose_seq = snap.get("pose_seq")
    if isinstance(pose_seq, int) and not isinstance(pose_seq, bool) and pose_seq >= 0:
        return pose_seq

    parking_status = snap.get("parking_status")
    if isinstance(parking_status, dict):
        parking_seq = parking_status.get("seq")
        if isinstance(parking_seq, int) and not isinstance(parking_seq, bool) and parking_seq >= 0:
            return parking_seq

    return 0


def validate_plan_candidate(
    ctx: ServerContext,
    selected: SlotPlan,
    states: Dict[str, str],
) -> tuple[bool, str]:
    path = selected.result.path
    if not selected.result.success:
        return False, "planner_result_not_success"
    if len(path) < 2:
        return False, "path_too_short"
    if selected.slot_id not in SLOT_IDS or states.get(selected.slot_id) != "FREE":
        return False, "target_slot_not_free"

    obstacles = build_slot_obstacles(ctx.map_cfg, states, target_slot=selected.slot_id)
    max_steer = max((abs(v) for v in ctx.planner.steering_samples), default=0.0)

    for i, node in enumerate(path):
        for value_name, value in (("x", node.x), ("y", node.y), ("yaw", node.yaw), ("g", node.g), ("steer", node.steer_rad)):
            if not math.isfinite(float(value)):
                return False, f"nonfinite_node_{value_name}:{i}"
        if ctx.planner.point_collision(float(node.x), float(node.y), obstacles):
            return False, f"node_collision:{i}"
        if i == 0:
            if node.direction != 0:
                return False, "start_direction_not_zero"
            continue
        if node.direction not in (-1, +1):
            return False, f"invalid_node_direction:{i}"
        if abs(float(node.steer_rad)) > max_steer + 1e-9:
            return False, f"steering_out_of_range:{i}"

        prev = path[i - 1]
        replay = ctx.planner.simulate_primitive(prev, node.direction, node.steer_rad, obstacles)
        if replay is None:
            return False, f"primitive_replay_collision:{i}"
        pos_err = math.hypot(replay.x - node.x, replay.y - node.y)
        yaw_err = abs(ctx.planner.normalize_angle(replay.yaw - node.yaw))
        if pos_err > 1e-5 or yaw_err > 1e-5:
            return False, f"primitive_replay_mismatch:{i}"

    return True, "ok"


def validate_serialized_trajectory(
    ctx: ServerContext,
    response: dict[str, Any],
    states: Dict[str, str],
) -> tuple[bool, str]:
    if response.get("type") != "trajectory":
        return False, "wrong_response_type"
    if response.get("map_id") != MAP_ID:
        return False, "trajectory_map_id_mismatch"
    if response.get("reference_point") != "rear_axle_center":
        return False, "trajectory_reference_point_mismatch"

    source_seq = response.get("source_seq")
    if not isinstance(source_seq, int) or isinstance(source_seq, bool) or source_seq < 0:
        return False, "trajectory_source_seq_invalid"

    target_slot = response.get("target_slot")
    if target_slot not in SLOT_IDS or states.get(str(target_slot)) != "FREE":
        return False, "trajectory_target_not_free"

    points = response.get("points")
    if not isinstance(points, list) or len(points) < 2:
        return False, "trajectory_points_invalid"
    if len(points) > 2000:
        return False, "trajectory_too_many_points"

    obstacles = build_slot_obstacles(ctx.map_cfg, states, target_slot=str(target_slot))
    max_speed = abs(ctx.nominal_speed) * 1.05 + 1e-9
    max_spacing = max(0.15, ctx.planner.motion_step * 1.25)
    max_yaw_step = max(0.35, 2.0 * ctx.planner.motion_step / ctx.planner.wheelbase * math.tan(ctx.planner.max_steer))

    prev: Optional[dict[str, Any]] = None
    for i, point in enumerate(points):
        if not isinstance(point, dict):
            return False, f"trajectory_point_not_object:{i}"
        direction = point.get("direction")
        if direction not in ALLOWED_DIRECTIONS:
            return False, f"trajectory_direction_invalid:{i}"
        vals: dict[str, float] = {}
        for key in ("x_m", "y_m", "yaw_rad", "v_ref_mps"):
            value = point.get(key)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
                return False, f"trajectory_value_invalid:{i}:{key}"
            vals[key] = float(value)
        if abs(vals["v_ref_mps"]) > max_speed:
            return False, f"trajectory_speed_exceeds_limit:{i}"
        if direction == "FORWARD" and vals["v_ref_mps"] < -1e-9:
            return False, f"trajectory_speed_direction_mismatch:{i}"
        if direction == "REVERSE" and vals["v_ref_mps"] > 1e-9:
            return False, f"trajectory_speed_direction_mismatch:{i}"
        if ctx.planner.point_collision(vals["x_m"], vals["y_m"], obstacles):
            return False, f"trajectory_point_collision:{i}"

        if prev is not None:
            dx = vals["x_m"] - float(prev["x_m"])
            dy = vals["y_m"] - float(prev["y_m"])
            spacing = math.hypot(dx, dy)
            if spacing > max_spacing:
                return False, f"trajectory_spacing_too_large:{i}"
            yaw_step = abs(ctx.planner.normalize_angle(vals["yaw_rad"] - float(prev["yaw_rad"])))
            if yaw_step > max_yaw_step:
                return False, f"trajectory_yaw_jump_too_large:{i}"
        prev = point

    return True, "ok"


def build_trajectory_response(
    ctx: ServerContext,
    selected: SlotPlan,
    source_seq: Any,
    states: Dict[str, str],
    pose_age_ms: float,
    parking_age_ms: float,
) -> tuple[Optional[dict[str, Any]], str]:
    ok, reason = validate_plan_candidate(ctx, selected, states)
    if not ok:
        return None, f"candidate_validation_failed:{reason}"

    points = trajectory_points(selected, ctx.nominal_speed)
    response = {
        "type": "trajectory",
        "version": PROTOCOL_VERSION,
        "trajectory_id": 0,  # assigned only after all validation passes
        "source_seq": source_seq,
        "map_id": MAP_ID,
        "target_slot": selected.slot_id,
        "reference_point": "rear_axle_center",
        "goal_mode": selected.goal_mode,
        "cost": round(float(selected.result.cost), 6),
        "expansions": int(selected.result.expansions),
        "input_age_ms_at_plan_start": {
            "pose": round(float(pose_age_ms), 1),
            "parking_status": round(float(parking_age_ms), 1),
        },
        "validation": "PASS",
        "prototype_warning": "OFFLINE_ONLY_FULL_VEHICLE_FOOTPRINT_NOT_VERIFIED",
        "points": points,
    }
    ok, reason = validate_serialized_trajectory(ctx, response, states)
    if not ok:
        return None, f"serialized_validation_failed:{reason}"
    response["trajectory_id"] = ctx.next_trajectory_id()
    return response, "ok"


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
            "session": self.ctx.session.snapshot(),
        })

    def send_planning_result(self, source_seq: Any, status: str, reason: str, **extra: Any) -> None:
        payload: dict[str, Any] = {
            "type": "planning_result",
            "version": PROTOCOL_VERSION,
            "map_id": MAP_ID,
            "source_seq": source_seq,
            "status": status,
            "reason": reason,
            "session": self.ctx.session.snapshot(),
        }
        payload.update(extra)
        self.send_json(payload)

    def plan_with_latest_state(self, source_seq: Any, trigger: str = "auto") -> None:
        if not self.ctx.planning_enabled:
            return

        snap = self.ctx.snapshot()
        source_seq = normalize_source_seq(source_seq, snap)
        fresh, reason = freshness_reason(self.ctx, snap)
        if not fresh:
            status = "WAITING_FOR_INPUT" if reason.startswith("no_") else "STALE_INPUT"
            self.ctx.session.set_waiting(reason, replan_pending=True)
            self.send_planning_result(
                source_seq,
                status,
                reason,
                pose_age_ms=None if snap["pose_age_ms"] is None else round(float(snap["pose_age_ms"]), 1),
                parking_age_ms=None if snap["parking_age_ms"] is None else round(float(snap["parking_age_ms"]), 1),
            )
            print(f"[GUARD] seq={source_seq} reject={reason}")
            return

        start: Pose = snap["pose"]
        parking_msg: dict[str, Any] = snap["parking_status"]
        states = slot_states_from_status(parking_msg)
        free_slots = [sid for sid, state in states.items() if state == "FREE"]
        if not free_slots:
            self.ctx.session.set_waiting("no_slot_in_FREE_state", replan_pending=True)
            self.send_planning_result(source_seq, "NO_FREE_SLOT", "no_slot_in_FREE_state")
            return

        self.ctx.session.start_planning(trigger)
        print(f"[SESSION] state=PLANNING trigger={trigger} session={self.ctx.session.snapshot()['session_id']}")

        with self.ctx.planning_lock:
            selected, candidates = choose_best_free_slot(
                self.ctx.planner,
                self.ctx.map_cfg,
                self.ctx.vehicle_cfg,
                start,
                states,
            )

        # Do not send a plan based on state that changed while Hybrid A* was running.
        if not self.ctx.generations_unchanged(snap["pose_generation"], snap["parking_generation"]):
            self.ctx.session.request_replan("input_changed_during_planning")
            self.send_planning_result(source_seq, "INPUT_CHANGED_DURING_PLANNING", "newer_pose_or_parking_status_received")
            print(f"[GUARD] seq={source_seq} reject=input_changed_during_planning")
            return

        post = self.ctx.snapshot()
        fresh, reason = freshness_reason(self.ctx, post)
        if not fresh:
            self.ctx.session.set_waiting(reason, replan_pending=True)
            self.send_planning_result(
                source_seq,
                "STALE_INPUT",
                reason,
                pose_age_ms=None if post["pose_age_ms"] is None else round(float(post["pose_age_ms"]), 1),
                parking_age_ms=None if post["parking_age_age_ms"] is None else round(float(post["parking_age_ms"]), 1),
            )
            print(f"[GUARD] seq={source_seq} reject={reason}_after_planning")
            return

        if selected is None:
            self.ctx.session.set_waiting("no_feasible_trajectory", replan_pending=True)
            self.send_planning_result(
                source_seq,
                "NO_FEASIBLE_TRAJECTORY",
                "hybrid_astar_rejected_all_free_slots",
                candidates=[
                    {
                        "slot_id": c.slot_id,
                        "success": c.result.success,
                        "reason": c.result.reason,
                        "expansions": c.result.expansions,
                    }
                    for c in candidates
                ],
            )
            print(f"[PLAN] seq={source_seq} no feasible trajectory")
            return

        response, validation_reason = build_trajectory_response(
            self.ctx,
            selected,
            source_seq,
            states,
            float(snap["pose_age_ms"]),
            float(snap["parking_age_ms"]),
        )
        if response is None:
            self.ctx.session.set_waiting("trajectory_rejected", replan_pending=True)
            self.send_planning_result(source_seq, "TRAJECTORY_REJECTED", validation_reason)
            print(f"[VALIDATE] seq={source_seq} REJECT {validation_reason}")
            return

        self.ctx.session.mark_trajectory_ready(int(response["trajectory_id"]), selected.slot_id)
        response["session"] = self.ctx.session.snapshot()
        self.send_json(response)
        print(
            f"[VALIDATE] tid={response['trajectory_id']} PASS "
            f"poseAge={post['pose_age_ms']:.0f}ms parkingAge={post['parking_age_ms']:.0f}ms"
        )
        print(
            f"[PLAN] tid={response['trajectory_id']} seq={source_seq} slot={selected.slot_id} "
            f"cost={selected.result.cost:.3f} expansions={selected.result.expansions} points={len(response['points'])}"
        )
        sess = self.ctx.session.snapshot()
        print(f"[SESSION] state={sess['state']} session={sess['session_id']} replanCount={sess['replan_count']}")

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
                        self.ctx.session.ensure_started("vehicle_pose_received")
                        print(
                            f"[POSE] seq={msg.get('seq')} x={pose.x:.3f} y={pose.y:.3f} "
                            f"yaw={math.degrees(pose.yaw):.1f}deg"
                        )
                    self.send_ack(msg, ok, reason)
                    if ok:
                        sess = self.ctx.session.snapshot()
                        if sess["state"] == "WAITING_INPUT" and sess["replan_pending"] and self.ctx.snapshot()["parking_status"] is not None:
                            self.plan_with_latest_state(msg.get("seq"), trigger="fresh_pose_for_pending_replan")

                elif msg_type == "parking_status":
                    ok, reason = validate_parking_status(msg)
                    should_plan = False
                    trigger = "parking_status_initial"
                    if ok:
                        self.ctx.set_parking_status(msg)
                        self.ctx.session.ensure_started("parking_status_received")
                        states = slot_states_from_status(msg)
                        free_slots = [s["id"] for s in msg["slots"] if s["state"] == "FREE"]
                        print(f"[PARKING] seq={msg.get('seq')} free={free_slots}")

                        sess = self.ctx.session.snapshot()
                        target = sess.get("target_slot")
                        if target is not None and states.get(str(target)) != "FREE":
                            action = self.ctx.session.target_became_invalid(states.get(str(target), "UNKNOWN"))
                            print(f"[REPLAN] target={target} state={states.get(str(target))} action={action}")
                            if action == "REPLAN":
                                should_plan = True
                                trigger = "target_slot_invalidated"
                        else:
                            sess = self.ctx.session.snapshot()
                            if sess["state"] in {"WAITING_INPUT", "REPLAN"}:
                                should_plan = True
                                trigger = "parking_status_for_pending_plan" if sess["replan_pending"] else "parking_status_initial"
                    self.send_ack(msg, ok, reason)
                    if ok and should_plan:
                        self.plan_with_latest_state(msg.get("seq"), trigger=trigger)

                elif msg_type == "plan_request":
                    ok, reason = validate_plan_request(msg)
                    if ok:
                        sess = self.ctx.session.snapshot()
                        if bool(msg.get("new_session")) or sess["state"] in {"IDLE", "COMPLETED"}:
                            self.ctx.session.start_new_session("plan_request_new_session")
                        elif sess["state"] == "PAUSED":
                            ok, reason = False, "session_paused_wait_for_safety_clear"
                        elif sess["state"] in {"TRAJECTORY_READY", "EXECUTING"}:
                            self.ctx.session.request_replan("explicit_plan_request")
                    self.send_ack(msg, ok, reason)
                    if ok:
                        self.plan_with_latest_state(msg.get("seq"), trigger="explicit_plan_request")

                elif msg_type == "trajectory_status":
                    ok, reason = validate_trajectory_status(msg)
                    trigger_replan = False
                    if ok:
                        tid = int(msg["trajectory_id"])
                        status = str(msg["status"])
                        if status == "RECEIVED":
                            ok, reason = self.ctx.session.mark_received(tid)
                        elif status == "EXECUTING":
                            ok, reason = self.ctx.session.mark_executing(tid)
                        elif status == "PAUSED":
                            ok, reason = self.ctx.session.mark_paused(tid, str(msg.get("reason") or "trajectory_status_PAUSED"))
                        elif status == "COMPLETED":
                            ok, reason = self.ctx.session.mark_completed(tid)
                        elif status in {"REJECTED", "REPLAN_REQUESTED"}:
                            if not self.ctx.session.active_matches(tid):
                                ok, reason = False, "trajectory_id_mismatch"
                            else:
                                self.ctx.session.request_replan(f"trajectory_status_{status}")
                                trigger_replan = True
                        if ok:
                            print(f"[TRAJECTORY_STATUS] tid={tid} status={status} session={self.ctx.session.snapshot()['state']}")
                    self.send_ack(msg, ok, reason)
                    if ok and trigger_replan:
                        self.plan_with_latest_state(msg.get("seq"), trigger="trajectory_status_replan")

                elif msg_type == "safety_event":
                    ok, reason = validate_safety_event(msg)
                    trigger_replan = False
                    if ok:
                        event = str(msg["event"])
                        tid_raw = msg.get("trajectory_id")
                        tid = int(tid_raw) if isinstance(tid_raw, int) and not isinstance(tid_raw, bool) else None
                        if event in {"PEDESTRIAN_BLOCKING", "CRITICAL_OBSTACLE", "SERVER_TIMEOUT"}:
                            ok, reason = self.ctx.session.mark_paused(tid, event, replan_pending=True)
                        elif event == "TRAJECTORY_INVALID":
                            if tid is not None and not self.ctx.session.active_matches(tid):
                                ok, reason = False, "trajectory_id_mismatch"
                            else:
                                self.ctx.session.request_replan("safety_TRAJECTORY_INVALID")
                                trigger_replan = True
                        elif event == "SAFETY_CLEARED":
                            if tid is not None and not self.ctx.session.active_matches(tid):
                                ok, reason = False, "trajectory_id_mismatch"
                            else:
                                ok, reason = self.ctx.session.clear_safety_and_request_replan("safety_cleared")
                                trigger_replan = ok
                        if ok:
                            print(f"[SAFETY_EVENT] event={event} tid={tid} session={self.ctx.session.snapshot()['state']}")
                    self.send_ack(msg, ok, reason)
                    if ok and trigger_replan:
                        self.plan_with_latest_state(msg.get("seq"), trigger="safety_clear_or_invalid")

                elif msg_type == "session_query":
                    ok, reason = validate_common(msg)
                    if not ok:
                        self.send_ack(msg, ok, reason)
                    else:
                        self.send_json({
                            "type": "session_status",
                            "version": PROTOCOL_VERSION,
                            "map_id": MAP_ID,
                            "session": self.ctx.session.snapshot(),
                        })

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
    parser.add_argument("--pose-max-age-ms", type=int, default=DEFAULT_POSE_MAX_AGE_MS)
    parser.add_argument("--parking-max-age-ms", type=int, default=DEFAULT_PARKING_MAX_AGE_MS)
    args = parser.parse_args()

    if args.pose_max_age_ms <= 0 or args.parking_max_age_ms <= 0:
        raise SystemExit("staleness thresholds must be positive")

    root = Path(__file__).resolve().parent
    ctx = ServerContext(
        root,
        planning_enabled=not args.transport_only,
        pose_max_age_ms=args.pose_max_age_ms,
        parking_max_age_ms=args.parking_max_age_ms,
    )

    with ReusableTCPServer((args.host, args.port), Handler, ctx) as server:
        mode = "transport-only" if args.transport_only else "hybrid-a*-offline"
        print(f"[SERVER] listening {args.host}:{args.port} protocol=v1 map={MAP_ID} mode={mode}")
        print(
            f"[GUARD] poseMaxAge={ctx.pose_max_age_ms}ms parkingMaxAge={ctx.parking_max_age_ms}ms "
            "clock=server_monotonic_receive_time"
        )
        print("[SESSION] state-machine=IDLE>WAITING_INPUT>PLANNING>TRAJECTORY_READY>EXECUTING/PAUSED>REPLAN/COMPLETED")
        print("[SAFETY] planning only; no actuator interface is present in this server")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n[SERVER] stopped by user")


if __name__ == "__main__":
    main()
