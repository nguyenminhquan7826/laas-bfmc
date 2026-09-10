#!/usr/bin/env python3
"""Raspberry Pi local Hybrid A* worker for one navigation decision.

The worker owns no UART interface. It verifies the local operational-map
package, plans one parking trajectory, validates it, and writes one NDJSON
result to stdout. The C++ runtime integration can keep this process isolated
from the 20 ms control loop.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from hybrid_astar_v1 import Pose, build_slot_obstacles
from map_package_v1 import load_and_verify_manifest
from server_stub import ServerContext, build_trajectory_response
from slot_selector_v1 import SlotPlan, rear_axle_goal_for_slot


def _failure(reason: str, decision_id: Any = None) -> dict[str, Any]:
    return {
        "type": "local_planning_result",
        "version": 1,
        "decision_id": decision_id,
        "status": "REJECTED",
        "reason": reason,
    }


def plan_local_request(root: Path, request: dict[str, Any]) -> dict[str, Any]:
    decision = request.get("decision")
    pose_value = request.get("pose")
    slots_value = request.get("slots")
    if not isinstance(decision, dict):
        return _failure("decision_not_object")
    decision_id = decision.get("decision_id")
    if not isinstance(decision_id, int) or isinstance(decision_id, bool) or decision_id <= 0:
        return _failure("invalid_decision_id", decision_id)

    try:
        manifest = load_and_verify_manifest(root)
    except (ValueError, FileNotFoundError) as exc:
        return _failure(str(exc), decision_id)
    if decision.get("map_id") != manifest["map_id"]:
        return _failure("map_id_mismatch", decision_id)
    if decision.get("map_package_sha256") != manifest["package_sha256"]:
        return _failure("map_package_sha256_mismatch", decision_id)
    if decision.get("maneuver") != "PARK_AT_SLOT":
        return _failure("unsupported_maneuver", decision_id)

    if not isinstance(pose_value, dict):
        return _failure("pose_not_object", decision_id)
    try:
        pose = Pose(
            float(pose_value["x_m"]),
            float(pose_value["y_m"]),
            float(pose_value["yaw_rad"]),
        )
    except (KeyError, TypeError, ValueError):
        return _failure("invalid_pose", decision_id)
    if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        return _failure("invalid_pose", decision_id)

    if not isinstance(slots_value, list):
        return _failure("slots_not_list", decision_id)
    states: dict[str, str] = {}
    for slot_state in slots_value:
        if not isinstance(slot_state, dict):
            return _failure("slot_not_object", decision_id)
        slot_id = slot_state.get("id")
        state = slot_state.get("state")
        if not isinstance(slot_id, str) or state not in {"UNKNOWN", "FREE", "OCCUPIED"}:
            return _failure("invalid_slot_state", decision_id)
        states[slot_id] = state

    target_slot = decision.get("target_slot")
    if not isinstance(target_slot, str) or states.get(target_slot) != "FREE":
        return _failure("target_slot_not_free", decision_id)

    ctx = ServerContext(root, planning_enabled=True)
    slot_cfg = next(
        (slot for slot in ctx.map_cfg.get("slots", []) if slot.get("id") == target_slot),
        None,
    )
    if slot_cfg is None:
        return _failure("target_slot_not_in_local_map", decision_id)

    goal, body_target, goal_mode = rear_axle_goal_for_slot(slot_cfg, ctx.vehicle_cfg)
    obstacles = build_slot_obstacles(ctx.map_cfg, states, target_slot=target_slot)
    result = ctx.planner.plan(pose, goal, obstacles)
    if not result.success:
        return _failure(f"hybrid_astar:{result.reason}", decision_id)

    selected = SlotPlan(target_slot, goal, body_target, goal_mode, result)
    response, reason = build_trajectory_response(
        ctx, selected, request.get("source_seq", 0), states, 0.0, 0.0
    )
    if response is None:
        return _failure(reason, decision_id)

    # The worker may be launched once per decision, so its in-process counter
    # always starts at one. Tie the local trajectory to the server decision
    # instead; this remains stable across worker restarts and is directly
    # traceable on the monitoring dashboard.
    response["trajectory_id"] = decision_id

    return {
        "type": "local_planning_result",
        "version": 1,
        "decision_id": decision_id,
        "map_id": manifest["map_id"],
        "map_package_sha256": manifest["package_sha256"],
        "status": "READY",
        "reason": "local_hybrid_astar_pass",
        "trajectory": response,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="directory containing map_manifest_v1.json and map package files",
    )
    parser.add_argument(
        "--protocol-output",
        action="store_true",
        help="emit a standard trajectory/planning_result for the C++ bridge",
    )
    args = parser.parse_args()
    raw = sys.stdin.readline()
    try:
        request = json.loads(raw)
    except json.JSONDecodeError as exc:
        print(json.dumps(_failure(f"invalid_json:{exc.msg}"), separators=(",", ":")))
        raise SystemExit(2)
    if not isinstance(request, dict):
        result = _failure("request_not_object")
    else:
        result = plan_local_request(args.root.resolve(), request)
    output = result
    if args.protocol_output:
        if result.get("status") == "READY":
            output = dict(result["trajectory"])
            output["decision_id"] = result["decision_id"]
            output["map_package_sha256"] = result["map_package_sha256"]
            output["planning_owner"] = "client"
        else:
            decision = request.get("decision", {}) if isinstance(request, dict) else {}
            output = {
                "type": "planning_result",
                "version": 1,
                "map_id": decision.get("map_id", "map_v1"),
                "source_seq": request.get("source_seq") if isinstance(request, dict) else None,
                "decision_id": result.get("decision_id"),
                "status": "LOCAL_PLANNING_REJECTED",
                "reason": result.get("reason", "local_planning_rejected"),
            }
    print(json.dumps(output, separators=(",", ":"), allow_nan=False), flush=True)
    raise SystemExit(0 if result.get("status") == "READY" else 1)


if __name__ == "__main__":
    main()
