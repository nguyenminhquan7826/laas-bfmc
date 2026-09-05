from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from hybrid_astar_v1 import HybridAStarPlanner, PlanResult, Pose, build_slot_obstacles


@dataclass
class SlotPlan:
    slot_id: str
    goal: Pose                 # Hybrid A* reference pose: rear axle center
    desired_body_center: Pose  # physical parking target
    goal_mode: str
    result: PlanResult


def desired_body_center_for_slot(slot: dict) -> Pose:
    """Physical parking target.

    Success means:
      1) vehicle geometric body center coincides with slot center;
      2) vehicle longitudinal axis is parallel to the two side boundaries.

    Bottom row: nose points toward center road (+Y).
    Top row:    nose points toward center road (-Y).
    The final motion primitive may be FORWARD or REVERSE.
    """
    cx, cy = map(float, slot["center_m"])
    sid = str(slot["id"])
    yaw = math.pi / 2.0 if sid.startswith("P_B") else -math.pi / 2.0
    return Pose(cx, cy, yaw)


def rear_axle_goal_for_slot(slot: dict, vehicle_cfg: dict) -> Tuple[Pose, Pose, str]:
    """Convert desired body-center pose to Hybrid A* rear-axle pose.

    Exact conversion requires rear_overhang_m:
        d = vehicle_length/2 - rear_overhang
        rear_axle = body_center - d * [cos(yaw), sin(yaw)]

    Until that measurement exists, use slot center as a clearly-labelled
    rear-axle proxy for OFFLINE algorithm testing only.
    """
    body = desired_body_center_for_slot(slot)
    geom = vehicle_cfg["geometry"]
    rear_overhang = geom.get("rear_overhang_m")

    if rear_overhang is None:
        return Pose(body.x, body.y, body.yaw), body, "REAR_AXLE_AT_SLOT_CENTER_PROXY"

    length = float(geom["length_m"])
    d = 0.5 * length - float(rear_overhang)
    rear = Pose(
        body.x - d * math.cos(body.yaw),
        body.y - d * math.sin(body.yaw),
        body.yaw,
    )
    return rear, body, "BODY_CENTER_ALIGNED_EXACT_FROM_REAR_OVERHANG"


def choose_best_free_slot(
    planner: HybridAStarPlanner,
    map_cfg: dict,
    vehicle_cfg: dict,
    start: Pose,
    slot_states: Dict[str, str],
) -> Tuple[Optional[SlotPlan], List[SlotPlan]]:
    candidates: List[SlotPlan] = []
    for slot in map_cfg["slots"]:
        sid = slot["id"]
        if slot_states.get(sid, "UNKNOWN").upper() != "FREE":
            continue
        goal, body_target, goal_mode = rear_axle_goal_for_slot(slot, vehicle_cfg)
        obstacles = build_slot_obstacles(map_cfg, slot_states, target_slot=sid)
        result = planner.plan(start, goal, obstacles)
        candidates.append(SlotPlan(sid, goal, body_target, goal_mode, result))

    feasible = [c for c in candidates if c.result.success]
    if not feasible:
        return None, candidates
    feasible.sort(key=lambda c: c.result.cost)
    return feasible[0], candidates
