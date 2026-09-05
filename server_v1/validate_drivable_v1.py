from pathlib import Path

from hybrid_astar_v1 import HybridAStarPlanner, load_yaml


def main() -> None:
    root = Path(__file__).resolve().parent
    map_cfg = load_yaml(root / "map_v1.yaml")
    vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
    planner_cfg = load_yaml(root / "planner_v1.yaml")
    planner = HybridAStarPlanner(map_cfg, vehicle_cfg, planner_cfg)

    should_be_drivable = {
        "parking_approach_start": (1.30, 0.7511),
        "P_B1_center": (2.137512, 0.398974),
        "P_B2_center": (2.953155, 0.398974),
        "P_T1_center": (2.140588, 1.161744),
        "P_T2_center": (2.955993, 1.161744),
        "lower_left_road": (0.30, 0.70),
        "lower_right_road": (4.00, 1.00),
        "upper_left_road": (0.30, 2.70),
        "upper_main_road": (2.00, 3.00),
    }
    should_be_non_drivable = {
        "central_island": (1.30, 2.00),
        "top_outside": (1.00, 3.80),
        "inside_upper_curve": (4.10, 2.90),
        "bottom_outside": (1.20, 0.05),
    }

    failures = []
    print("[DRIVABLE AREA V1] semantic sanity check")
    for name, (x, y) in should_be_drivable.items():
        got = planner.point_in_drivable_area(x, y)
        print(f"DRIVABLE     {name:24s} ({x:.3f}, {y:.3f}) -> {got}")
        if not got:
            failures.append(name)
    for name, (x, y) in should_be_non_drivable.items():
        got = planner.point_in_drivable_area(x, y)
        print(f"NON_DRIVABLE {name:24s} ({x:.3f}, {y:.3f}) -> {got}")
        if got:
            failures.append(name)

    if failures:
        print(f"[RESULT] FAIL: {', '.join(failures)}")
        raise SystemExit(2)
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
