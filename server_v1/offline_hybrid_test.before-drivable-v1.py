from __future__ import annotations

import argparse
import base64
import html
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List

from hybrid_astar_v1 import HybridAStarPlanner, Node, Pose, load_yaml
from slot_selector_v1 import SlotPlan, choose_best_free_slot


def write_trajectory_json(path: Path, selected: SlotPlan) -> None:
    points = []
    for n in selected.result.path:
        points.append({
            "x": round(n.x, 5),
            "y": round(n.y, 5),
            "yaw": round(n.yaw, 6),
            "v_ref": 0.10 if n.direction >= 0 else -0.10,
            "direction": "FORWARD" if n.direction >= 0 else "REVERSE",
            "steering_deg_debug": round(math.degrees(n.steer_rad), 3),
        })
    payload = {
        "type": "trajectory",
        "version": 1,
        "trajectory_id": 1,
        "target_slot": selected.slot_id,
        "map_id": "map_v1",
        "prototype_warning": "OFFLINE_ONLY_REAR_AXLE_POINT_PLUS_DRIVABLE_AREA; FULL_FOOTPRINT_PENDING",
        "points": points,
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def svg_xy(x: float, y: float, width_m: float, height_m: float, px_per_m: float, margin: float):
    sx = margin + x * px_per_m
    sy = margin + (height_m - y) * px_per_m
    return sx, sy


def write_debug_svg(path: Path, map_cfg: dict, slot_states: Dict[str, str], selected: SlotPlan, start: Pose) -> None:
    width_m = float(map_cfg["map"]["width_x_m"])
    height_m = float(map_cfg["map"]["height_y_m"])
    scale = 180.0
    margin = 30.0
    w = width_m * scale + 2 * margin
    h = height_m * scale + 2 * margin

    chunks: List[str] = []
    chunks.append(f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="{w:.0f}" height="{h:.0f}" viewBox="0 0 {w:.1f} {h:.1f}">')
    chunks.append('<rect width="100%" height="100%" fill="white"/>')

    reference_file = map_cfg.get("geometry", {}).get("render_reference")
    reference_path = path.parent / str(reference_file) if reference_file else None
    if reference_path and reference_path.exists():
        encoded = base64.b64encode(reference_path.read_bytes()).decode("ascii")
        chunks.append(
            f'<image href="data:image/png;base64,{encoded}" x="{margin:.1f}" y="{margin:.1f}" '
            f'width="{width_m*scale:.1f}" height="{height_m*scale:.1f}" preserveAspectRatio="none"/>'
        )
    else:
        chunks.append(f'<rect x="{margin}" y="{margin}" width="{width_m*scale:.1f}" height="{height_m*scale:.1f}" fill="#111" stroke="#111" stroke-width="2"/>')

    # Overlay the active CAD-derived drivable polygon used by Hybrid A*.
    for area in map_cfg.get("drivable_areas", []):
        poly = area.get("planner_polygon_m") or area.get("raw_polygon_m")
        if not poly:
            continue
        pts = [svg_xy(float(p[0]), float(p[1]), width_m, height_m, scale, margin) for p in poly]
        pts_s = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        chunks.append(
            f'<polygon points="{pts_s}" fill="#00ff88" fill-opacity="0.10" '
            f'stroke="#00ff88" stroke-opacity="0.75" stroke-width="2" stroke-dasharray="7 5"/>'
        )

    for slot in map_cfg["slots"]:
        pts = [svg_xy(float(p[0]), float(p[1]), width_m, height_m, scale, margin) for p in slot["polygon_m"]]
        pts_s = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        sid = slot["id"]
        state = slot_states.get(sid, "UNKNOWN").upper()
        fill = {"FREE": "#54d66b", "OCCUPIED": "#e55b5b", "UNKNOWN": "#e5cf59"}.get(state, "#888")
        stroke = "#006400" if sid == selected.slot_id else "#333"
        sw = 4 if sid == selected.slot_id else 2
        chunks.append(f'<polygon points="{pts_s}" fill="{fill}" fill-opacity="0.30" stroke="{stroke}" stroke-width="{sw}"/>')
        cx, cy = svg_xy(float(slot["center_m"][0]), float(slot["center_m"][1]), width_m, height_m, scale, margin)
        chunks.append(f'<text x="{cx:.1f}" y="{cy:.1f}" font-size="16" text-anchor="middle" fill="#ffffff" stroke="#000000" stroke-width="0.5">{html.escape(sid)} {state}</text>')

    path_pts = [svg_xy(n.x, n.y, width_m, height_m, scale, margin) for n in selected.result.path]
    if path_pts:
        pts_s = " ".join(f"{x:.1f},{y:.1f}" for x, y in path_pts)
        chunks.append(f'<polyline points="{pts_s}" fill="none" stroke="#00b7ff" stroke-width="5"/>')

    sx, sy = svg_xy(start.x, start.y, width_m, height_m, scale, margin)
    gx, gy = svg_xy(selected.goal.x, selected.goal.y, width_m, height_m, scale, margin)
    chunks.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="#ff9f1a"/>')
    chunks.append(f'<text x="{sx+10:.1f}" y="{sy-8:.1f}" font-size="16" fill="#ffffff">START</text>')
    chunks.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="8" fill="#00ff77"/>')
    chunks.append(f'<text x="{gx+10:.1f}" y="{gy-8:.1f}" font-size="16" fill="#ffffff">GOAL(debug)</text>')
    chunks.append(f'<text x="{margin}" y="{h-8:.1f}" font-size="15" fill="#a00">CAD map active. DRIVABLE constraint ACTIVE for parking_sector. FULL VEHICLE FOOTPRINT still pending.</text>')
    chunks.append('</svg>')
    path.write_text("\n".join(chunks), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-x", type=float, default=1.30)
    parser.add_argument("--start-y", type=float, default=0.7511)
    parser.add_argument("--start-yaw-deg", type=float, default=0.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    map_cfg = load_yaml(root / "map_v1.yaml")
    vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
    planner_cfg = load_yaml(root / "planner_v1.yaml")

    planner = HybridAStarPlanner(map_cfg, vehicle_cfg, planner_cfg)
    start = Pose(args.start_x, args.start_y, math.radians(args.start_yaw_deg))

    # Same states as the TCP mock test.
    slot_states = {
        "P_B1": "OCCUPIED",
        "P_B2": "FREE",
        "P_T1": "UNKNOWN",
        "P_T2": "FREE",
    }

    selected, candidates = choose_best_free_slot(planner, map_cfg, vehicle_cfg, start, slot_states)

    print("[HYBRID-A* V1] OFFLINE PROTOTYPE")
    print(f"start=({start.x:.3f}, {start.y:.3f}, {math.degrees(start.yaw):.1f}deg)")
    for c in candidates:
        if c.result.success:
            print(f"candidate={c.slot_id} PASS cost={c.result.cost:.3f} expansions={c.result.expansions} nodes={len(c.result.path)}")
        else:
            print(f"candidate={c.slot_id} FAIL reason={c.result.reason} expansions={c.result.expansions}")

    if selected is None:
        print("[RESULT] no feasible FREE slot")
        raise SystemExit(2)

    print(f"[RESULT] selected={selected.slot_id} cost={selected.result.cost:.3f}")
    violations = sum(1 for n in selected.result.path if not planner.point_is_drivable(n.x, n.y))
    active_areas = [str(a.get("id")) for a in map_cfg.get("drivable_areas", []) if a.get("planner_polygon_m") or a.get("raw_polygon_m")]
    print(f"[DRIVABLE] active={active_areas} path_violations={violations}")
    if violations != 0:
        raise RuntimeError("planner returned a path outside the active drivable area")
    trajectory_path = root / "trajectory_debug.json"
    svg_path = root / "plan_debug.svg"
    write_trajectory_json(trajectory_path, selected)
    write_debug_svg(svg_path, map_cfg, slot_states, selected, start)
    print(f"wrote {trajectory_path.name}")
    print(f"wrote {svg_path.name}")
    print("WARNING: do not send this trajectory to the vehicle; footprint and final goal geometry are not verified.")


if __name__ == "__main__":
    main()
