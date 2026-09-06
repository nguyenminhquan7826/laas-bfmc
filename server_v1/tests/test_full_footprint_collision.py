from __future__ import annotations

import copy
import math
import unittest
from pathlib import Path

from hybrid_astar_v1 import DrivableGrid, HybridAStarPlanner, Pose, RectObstacle, load_yaml


def make_map() -> dict:
    return {
        "map": {
            "width_x_m": 3.0,
            "height_y_m": 3.0,
        }
    }


def make_vehicle(*, verified: bool = True, rear=0.08, front=0.07) -> dict:
    return {
        "geometry": {
            "reference_point": "rear_axle_center",
            "length_m": 0.40,
            "width_m": 0.20,
            "wheelbase_m": 0.25,
            "track_width_m": 0.16,
            "rear_overhang_m": rear,
            "front_overhang_m": front,
            "body_center_from_rear_axle_m": 0.12 if rear is not None else None,
            "footprint_verified": verified,
        }
    }


def make_planner_cfg(mode: str = "FULL_FOOTPRINT") -> dict:
    return {
        "search": {
            "grid_resolution_m": 0.05,
            "yaw_resolution_deg": 10.0,
            "motion_step_m": 0.10,
            "integration_step_m": 0.02,
            "max_expansions": 1000,
        },
        "motion": {
            "steering_samples_deg": [-25.0, 0.0, 25.0],
            "allow_forward": True,
            "allow_reverse": True,
        },
        "cost": {
            "reverse_multiplier": 1.35,
            "direction_switch_penalty": 0.3,
            "steering_penalty": 0.04,
            "steering_change_penalty": 0.06,
            "yaw_heuristic_weight": 0.08,
        },
        "goal": {
            "position_tolerance_m": 0.08,
            "yaw_tolerance_deg": 12.0,
        },
        "collision": {
            "mode": mode,
            "obstacle_inflation_m": 0.0,
            "treat_unknown_slots_as_blocked": True,
            "require_drivable_area": False,
            "require_verified_geometry": True,
        },
    }


def packed_grid(width: int, height: int, blocked: tuple[int, int]) -> bytes:
    data = bytearray((width * height + 7) // 8)
    for iy in range(height):
        for ix in range(width):
            if (ix, iy) == blocked:
                continue
            idx = iy * width + ix
            data[idx >> 3] |= 1 << (idx & 7)
    return bytes(data)


class FullFootprintCollisionTests(unittest.TestCase):
    def test_current_project_config_remains_point_mode(self) -> None:
        root = Path(__file__).resolve().parents[1]
        map_cfg = load_yaml(root / "map_v1.yaml")
        vehicle_cfg = load_yaml(root / "vehicle_v1.yaml")
        planner_cfg = load_yaml(root / "planner_v1.yaml")
        planner = HybridAStarPlanner(map_cfg, vehicle_cfg, planner_cfg)
        self.assertEqual(planner.collision_mode, HybridAStarPlanner.POINT_COLLISION_MODE)

    def test_full_footprint_rejects_unverified_geometry(self) -> None:
        with self.assertRaisesRegex(ValueError, "footprint_verified"):
            HybridAStarPlanner(make_map(), make_vehicle(verified=False), make_planner_cfg())

    def test_full_footprint_rejects_missing_overhang(self) -> None:
        with self.assertRaisesRegex(ValueError, "rear_overhang_m"):
            HybridAStarPlanner(
                make_map(),
                make_vehicle(rear=None, front=0.07),
                make_planner_cfg(),
            )

    def test_full_footprint_rejects_inconsistent_geometry(self) -> None:
        vehicle = make_vehicle()
        vehicle["geometry"]["front_overhang_m"] = 0.20
        with self.assertRaisesRegex(ValueError, "geometry inconsistent"):
            HybridAStarPlanner(make_map(), vehicle, make_planner_cfg())

    def test_reference_point_clear_but_front_body_hits_slot(self) -> None:
        planner = HybridAStarPlanner(make_map(), make_vehicle(), make_planner_cfg())
        pose = Pose(1.0, 1.0, 0.0)
        obstacle = RectObstacle(1.30, 1.36, 0.95, 1.05, "front_overlap")

        self.assertFalse(planner.point_hits_slot_obstacle(pose.x, pose.y, [obstacle]))
        self.assertTrue(planner.footprint_hits_slot_obstacle(pose, [obstacle]))
        self.assertTrue(planner.pose_collision(pose, [obstacle]))

    def test_rotated_body_hits_slot_while_reference_point_is_clear(self) -> None:
        planner = HybridAStarPlanner(make_map(), make_vehicle(), make_planner_cfg())
        pose = Pose(1.0, 1.0, math.pi / 2.0)
        obstacle = RectObstacle(0.95, 1.05, 1.30, 1.36, "rotated_front_overlap")

        self.assertFalse(planner.point_hits_slot_obstacle(pose.x, pose.y, [obstacle]))
        self.assertTrue(planner.footprint_hits_slot_obstacle(pose, [obstacle]))

    def test_clear_pose_is_not_in_collision(self) -> None:
        planner = HybridAStarPlanner(make_map(), make_vehicle(), make_planner_cfg())
        pose = Pose(1.0, 1.0, 0.0)
        obstacle = RectObstacle(1.7, 1.8, 1.7, 1.8, "far")
        self.assertFalse(planner.pose_collision(pose, [obstacle]))

    def test_body_crossing_map_boundary_is_blocked(self) -> None:
        planner = HybridAStarPlanner(make_map(), make_vehicle(), make_planner_cfg())
        pose = Pose(0.05, 1.0, 0.0)
        self.assertFalse(planner.footprint_in_drivable_area(pose))
        self.assertTrue(planner.pose_collision(pose, []))

    def test_non_drivable_cad_cell_under_body_is_blocked(self) -> None:
        planner_cfg = make_planner_cfg()
        planner = HybridAStarPlanner(make_map(), make_vehicle(), planner_cfg)

        width = 30
        height = 30
        planner.drivable_grid = DrivableGrid(
            resolution_m=0.10,
            width_cells=width,
            height_cells=height,
            width_x_m=3.0,
            height_y_m=3.0,
            packed=packed_grid(width, height, blocked=(12, 10)),
        )
        planner.require_drivable_area = True

        pose = Pose(1.0, 1.0, 0.0)
        self.assertTrue(planner.point_in_drivable_area(pose.x, pose.y))
        self.assertFalse(planner.footprint_in_drivable_area(pose))
        self.assertTrue(planner.pose_collision(pose, []))

    def test_plan_rejects_start_when_only_body_is_in_collision(self) -> None:
        planner = HybridAStarPlanner(make_map(), make_vehicle(), make_planner_cfg())
        start = Pose(1.0, 1.0, 0.0)
        goal = Pose(2.0, 2.0, 0.0)
        obstacle = RectObstacle(1.30, 1.36, 0.95, 1.05, "front_overlap")
        result = planner.plan(start, goal, [obstacle])
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "start_footprint_in_slot_obstacle")

    def test_point_mode_preserves_legacy_reference_point_behavior(self) -> None:
        cfg = copy.deepcopy(make_planner_cfg(mode="REAR_AXLE_POINT_ONLY"))
        planner = HybridAStarPlanner(make_map(), make_vehicle(verified=False), cfg)
        pose = Pose(1.0, 1.0, 0.0)
        obstacle = RectObstacle(1.30, 1.36, 0.95, 1.05, "front_overlap")
        self.assertFalse(planner.pose_collision(pose, [obstacle]))


if __name__ == "__main__":
    unittest.main()
