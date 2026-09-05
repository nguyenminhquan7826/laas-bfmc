#!/usr/bin/env python3
"""Offline tests for Server V1 staleness and trajectory sanity guards."""

from __future__ import annotations

import copy
import time
from pathlib import Path

from hybrid_astar_v1 import Pose
from server_stub import (
    ServerContext,
    build_trajectory_response,
    freshness_reason,
    slot_states_from_status,
    validate_serialized_trajectory,
)
from slot_selector_v1 import choose_best_free_slot


def make_parking_status() -> dict:
    return {
        "type": "parking_status",
        "version": 1,
        "seq": 2,
        "timestamp_ms": int(time.time() * 1000),
        "map_id": "map_v1",
        "slots": [
            {"id": "P_B1", "state": "OCCUPIED", "confidence": 0.96},
            {"id": "P_B2", "state": "FREE", "confidence": 0.91},
            {"id": "P_T1", "state": "UNKNOWN", "confidence": 0.45},
            {"id": "P_T2", "state": "FREE", "confidence": 0.95},
        ],
        "objects": [],
    }


def expect(label: str, condition: bool, detail: str = "") -> None:
    print(f"{label:36s} -> {'PASS' if condition else 'FAIL'} {detail}")
    if not condition:
        raise SystemExit(1)


def main() -> None:
    root = Path(__file__).resolve().parent
    ctx = ServerContext(root, planning_enabled=True, pose_max_age_ms=50, parking_max_age_ms=70)
    pose = Pose(1.30, 0.7511, 0.0)
    parking = make_parking_status()
    states = slot_states_from_status(parking)

    ctx.set_pose(pose, 1)
    ctx.set_parking_status(parking)
    snap = ctx.snapshot()
    fresh, reason = freshness_reason(ctx, snap)
    expect("fresh input accepted", fresh, reason)

    selected, _ = choose_best_free_slot(ctx.planner, ctx.map_cfg, ctx.vehicle_cfg, pose, states)
    expect("Hybrid A* candidate exists", selected is not None)
    assert selected is not None

    response, reason = build_trajectory_response(
        ctx,
        selected,
        parking["seq"],
        states,
        float(snap["pose_age_ms"]),
        float(snap["parking_age_ms"]),
    )
    expect("valid trajectory accepted", response is not None, reason)
    assert response is not None

    bad = copy.deepcopy(response)
    bad["points"][1]["x_m"] = -99.0
    ok, reason = validate_serialized_trajectory(ctx, bad, states)
    expect("out-of-map trajectory rejected", not ok, reason)

    bad = copy.deepcopy(response)
    bad["points"][1]["direction"] = "SIDEWAYS"
    ok, reason = validate_serialized_trajectory(ctx, bad, states)
    expect("invalid direction rejected", not ok, reason)

    bad = copy.deepcopy(response)
    bad["points"][1]["v_ref_mps"] = 9.0
    ok, reason = validate_serialized_trajectory(ctx, bad, states)
    expect("excessive speed rejected", not ok, reason)

    time.sleep(0.06)
    snap = ctx.snapshot()
    fresh, reason = freshness_reason(ctx, snap)
    expect("stale pose rejected", (not fresh and reason == "pose_stale"), reason)

    # Refresh only pose; parking status should now age beyond its longer threshold.
    ctx.set_pose(pose, 3)
    time.sleep(0.02)
    snap = ctx.snapshot()
    fresh, reason = freshness_reason(ctx, snap)
    expect("stale parking rejected", (not fresh and reason == "parking_status_stale"), reason)

    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
