from __future__ import annotations

import json
import math
import time
from pathlib import Path

from hybrid_astar_v1 import HybridAStarPlanner, Pose, load_yaml
from slot_selector_v1 import choose_best_free_slot


def main() -> None:
    root = Path(__file__).resolve().parent
    map_cfg = load_yaml(root / "map_v1.yaml")
    vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
    planner_cfg = load_yaml(root / "planner_v1.yaml")
    planner = HybridAStarPlanner(map_cfg, vehicle_cfg, planner_cfg)

    slot_states = {
        "P_B1": "OCCUPIED",
        "P_B2": "FREE",
        "P_T1": "UNKNOWN",
        "P_T2": "FREE",
    }

    cases = [
        {"id": "local_left", "start": [1.30, 0.7511, 0.0], "expect_feasible": True},
        {"id": "lower_left", "start": [0.30, 0.70, 0.0], "expect_feasible": True},
        {"id": "right_vertical", "start": [4.00, 1.50, -90.0], "expect_feasible": True},
        {"id": "lower_right", "start": [3.80, 0.75, 180.0], "expect_feasible": True},
        {"id": "central_island_negative", "start": [1.30, 2.00, 0.0], "expect_feasible": False},
    ]

    report = {
        "schema_version": 1,
        "map_id": map_cfg["map_id"],
        "planner_id": planner_cfg["planner_id"],
        "collision_mode": planner_cfg["collision"]["mode"],
        "drivable_area": map_cfg["semantic_map"]["status"],
        "slot_states": slot_states,
        "cases": [],
    }

    failures = []
    print("[HYBRID-A* V1] BATCH / DRIVABLE-AREA TEST")
    for case in cases:
        x, y, yaw_deg = case["start"]
        start = Pose(float(x), float(y), math.radians(float(yaw_deg)))
        t0 = time.perf_counter()
        selected, candidates = choose_best_free_slot(planner, map_cfg, vehicle_cfg, start, slot_states)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        feasible = selected is not None
        passed = feasible == bool(case["expect_feasible"])
        if not passed:
            failures.append(case["id"])

        candidate_rows = []
        for c in candidates:
            candidate_rows.append({
                "slot_id": c.slot_id,
                "success": c.result.success,
                "reason": c.result.reason,
                "cost": None if not c.result.success else round(c.result.cost, 6),
                "expansions": c.result.expansions,
                "nodes": len(c.result.path),
            })

        row = {
            "id": case["id"],
            "start": {"x": x, "y": y, "yaw_deg": yaw_deg},
            "expect_feasible": case["expect_feasible"],
            "actual_feasible": feasible,
            "selected_slot": None if selected is None else selected.slot_id,
            "elapsed_ms": round(elapsed_ms, 3),
            "pass": passed,
            "candidates": candidate_rows,
        }
        report["cases"].append(row)
        print(
            f"{case['id']:24s} expected={'PASS' if case['expect_feasible'] else 'REJECT':6s} "
            f"actual={'PASS' if feasible else 'REJECT':6s} selected={row['selected_slot']} "
            f"time={elapsed_ms:.1f}ms -> {'OK' if passed else 'FAIL'}"
        )

    out = root / "batch_results_v1.json"
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"wrote {out.name}")
    if failures:
        print(f"[RESULT] FAIL: {', '.join(failures)}")
        raise SystemExit(2)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
