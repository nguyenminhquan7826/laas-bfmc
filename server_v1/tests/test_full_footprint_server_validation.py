from __future__ import annotations

import math
import unittest

from hybrid_astar_v1 import HybridAStarPlanner, Pose
from server_stub import validate_serialized_trajectory


def planner_cfg() -> dict:
    return {
        "search": {"grid_resolution_m": 0.05, "yaw_resolution_deg": 10.0, "motion_step_m": 0.10, "integration_step_m": 0.02, "max_expansions": 1000},
        "motion": {"steering_samples_deg": [-25.0, 0.0, 25.0], "allow_forward": True, "allow_reverse": True},
        "cost": {"reverse_multiplier": 1.35, "direction_switch_penalty": 0.3, "steering_penalty": 0.04, "steering_change_penalty": 0.06, "yaw_heuristic_weight": 0.08},
        "goal": {"position_tolerance_m": 0.08, "yaw_tolerance_deg": 12.0},
        "collision": {"mode": "FULL_FOOTPRINT", "obstacle_inflation_m": 0.0, "treat_unknown_slots_as_blocked": True, "require_drivable_area": False, "require_verified_geometry": True},
    }


def vehicle_cfg() -> dict:
    return {"geometry": {
        "reference_point": "rear_axle_center",
        "length_m": 0.40, "width_m": 0.20, "wheelbase_m": 0.25,
        "rear_overhang_m": 0.08, "front_overhang_m": 0.07,
        "body_center_from_rear_axle_m": 0.12, "footprint_verified": True,
    }}


def map_cfg() -> dict:
    def slot(sid, x0, x1, y0, y1):
        return {"id": sid, "polygon_m": [[x0,y0],[x1,y0],[x1,y1],[x0,y1]], "center_m": [(x0+x1)/2,(y0+y1)/2]}
    return {
        "map": {"width_x_m": 3.0, "height_y_m": 3.0},
        "slots": [
            slot("P_B1", 1.28, 1.36, 0.95, 1.05),
            slot("P_B2", 2.00, 2.40, 0.50, 0.90),
            slot("P_T1", 2.00, 2.40, 1.50, 1.90),
            slot("P_T2", 2.50, 2.90, 1.50, 1.90),
        ],
    }


class FakeContext:
    def __init__(self):
        self.map_cfg = map_cfg()
        self.planner = HybridAStarPlanner(self.map_cfg, vehicle_cfg(), planner_cfg())
        self.nominal_speed = 0.10


class FullFootprintServerValidationTests(unittest.TestCase):
    def test_serialized_validator_rejects_body_collision_when_rear_axle_is_clear(self):
        ctx = FakeContext()
        states = {"P_B1":"OCCUPIED", "P_B2":"FREE", "P_T1":"OCCUPIED", "P_T2":"OCCUPIED"}
        self.assertFalse(ctx.planner.point_collision(1.0, 1.0, []))
        response = {
            "type":"trajectory", "version":1, "trajectory_id":1, "source_seq":1,
            "map_id":"map_v1", "target_slot":"P_B2", "reference_point":"rear_axle_center",
            "points":[
                {"x_m":1.0,"y_m":1.0,"yaw_rad":0.0,"v_ref_mps":0.1,"direction":"FORWARD"},
                {"x_m":1.05,"y_m":1.0,"yaw_rad":0.0,"v_ref_mps":0.1,"direction":"FORWARD"},
            ],
        }
        ok, reason = validate_serialized_trajectory(ctx, response, states)
        self.assertFalse(ok)
        self.assertEqual(reason, "trajectory_point_collision:0")


if __name__ == "__main__":
    unittest.main()
