from pathlib import Path

server = Path('server_v1/server_stub.py')
text = server.read_text(encoding='utf-8')
text = text.replace(
    '- Full vehicle footprint is still NOT verified.\n',
    '- Measured full-vehicle footprint collision is enabled for offline planning.\n'
    '- Parking actuation remains unauthorized; the server has no actuator interface.\n',
    1,
)
old = '        if ctx.planner.point_collision(float(node.x), float(node.y), obstacles):\n            return False, f"node_collision:{i}"\n'
new = '        node_pose = Pose(float(node.x), float(node.y), float(node.yaw))\n        if ctx.planner.pose_collision(node_pose, obstacles):\n            return False, f"node_collision:{i}"\n'
if old not in text:
    raise SystemExit('candidate point-collision anchor missing')
text = text.replace(old, new, 1)
old = '        if ctx.planner.point_collision(vals["x_m"], vals["y_m"], obstacles):\n            return False, f"trajectory_point_collision:{i}"\n'
new = '        serialized_pose = Pose(vals["x_m"], vals["y_m"], vals["yaw_rad"])\n        if ctx.planner.pose_collision(serialized_pose, obstacles):\n            return False, f"trajectory_point_collision:{i}"\n'
if old not in text:
    raise SystemExit('serialized point-collision anchor missing')
text = text.replace(old, new, 1)
server.write_text(text, encoding='utf-8')

map_path = Path('server_v1/map_v1.yaml')
text = map_path.read_text(encoding='utf-8')
text = text.replace(
    '- goal_pose remains unset until rear-axle-to-body geometry is physically verified.\n',
    '- goal_pose fields remain unset; slot_selector_v1 computes the exact rear-axle goal\n'
    '  from the measured rear-axle-to-body-center offset.\n',
    1,
)
text = text.replace('  planner_scope: FULL_MAP_POINT_REFERENCE_TEST\n', '  planner_scope: FULL_MAP_FULL_FOOTPRINT_TEST\n', 1)
text = text.replace('  scope: FULL_MAP_POINT_REFERENCE_V1\n', '  scope: FULL_MAP_FULL_FOOTPRINT_V1\n', 1)
text = text.replace(
    '  warning: Rear-axle point only; full vehicle footprint is still disabled until overhang\n    geometry is measured.\n',
    '  warning: FULL_FOOTPRINT uses measured vehicle geometry for offline planning; parking\n    actuation remains unauthorized.\n',
    1,
)
map_path.write_text(text, encoding='utf-8')

test = Path('server_v1/tests/test_full_footprint_server_validation.py')
test.write_text('''from __future__ import annotations\n\nimport math\nimport unittest\n\nfrom hybrid_astar_v1 import HybridAStarPlanner, Pose\nfrom server_stub import validate_serialized_trajectory\n\n\ndef planner_cfg() -> dict:\n    return {\n        "search": {"grid_resolution_m": 0.05, "yaw_resolution_deg": 10.0, "motion_step_m": 0.10, "integration_step_m": 0.02, "max_expansions": 1000},\n        "motion": {"steering_samples_deg": [-25.0, 0.0, 25.0], "allow_forward": True, "allow_reverse": True},\n        "cost": {"reverse_multiplier": 1.35, "direction_switch_penalty": 0.3, "steering_penalty": 0.04, "steering_change_penalty": 0.06, "yaw_heuristic_weight": 0.08},\n        "goal": {"position_tolerance_m": 0.08, "yaw_tolerance_deg": 12.0},\n        "collision": {"mode": "FULL_FOOTPRINT", "obstacle_inflation_m": 0.0, "treat_unknown_slots_as_blocked": True, "require_drivable_area": False, "require_verified_geometry": True},\n    }\n\n\ndef vehicle_cfg() -> dict:\n    return {"geometry": {\n        "reference_point": "rear_axle_center",\n        "length_m": 0.40, "width_m": 0.20, "wheelbase_m": 0.25,\n        "rear_overhang_m": 0.08, "front_overhang_m": 0.07,\n        "body_center_from_rear_axle_m": 0.12, "footprint_verified": True,\n    }}\n\n\ndef map_cfg() -> dict:\n    def slot(sid, x0, x1, y0, y1):\n        return {"id": sid, "polygon_m": [[x0,y0],[x1,y0],[x1,y1],[x0,y1]], "center_m": [(x0+x1)/2,(y0+y1)/2]}\n    return {\n        "map": {"width_x_m": 3.0, "height_y_m": 3.0},\n        "slots": [\n            slot("P_B1", 1.28, 1.36, 0.95, 1.05),\n            slot("P_B2", 2.00, 2.40, 0.50, 0.90),\n            slot("P_T1", 2.00, 2.40, 1.50, 1.90),\n            slot("P_T2", 2.50, 2.90, 1.50, 1.90),\n        ],\n    }\n\n\nclass FakeContext:\n    def __init__(self):\n        self.map_cfg = map_cfg()\n        self.planner = HybridAStarPlanner(self.map_cfg, vehicle_cfg(), planner_cfg())\n        self.nominal_speed = 0.10\n\n\nclass FullFootprintServerValidationTests(unittest.TestCase):\n    def test_serialized_validator_rejects_body_collision_when_rear_axle_is_clear(self):\n        ctx = FakeContext()\n        states = {"P_B1":"OCCUPIED", "P_B2":"FREE", "P_T1":"OCCUPIED", "P_T2":"OCCUPIED"}\n        self.assertFalse(ctx.planner.point_collision(1.0, 1.0, []))\n        response = {\n            "type":"trajectory", "version":1, "trajectory_id":1, "source_seq":1,\n            "map_id":"map_v1", "target_slot":"P_B2", "reference_point":"rear_axle_center",\n            "points":[\n                {"x_m":1.0,"y_m":1.0,"yaw_rad":0.0,"v_ref_mps":0.1,"direction":"FORWARD"},\n                {"x_m":1.05,"y_m":1.0,"yaw_rad":0.0,"v_ref_mps":0.1,"direction":"FORWARD"},\n            ],\n        }\n        ok, reason = validate_serialized_trajectory(ctx, response, states)\n        self.assertFalse(ok)\n        self.assertEqual(reason, "trajectory_point_collision:0")\n\n\nif __name__ == "__main__":\n    unittest.main()\n''', encoding='utf-8')
print('patched server validation, map docs, and regression test')
