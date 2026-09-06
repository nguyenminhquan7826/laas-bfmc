from pathlib import Path

path = Path('server_v1/hybrid_astar_v1.py')
text = path.read_text(encoding='utf-8')

# Add bisect for blocked-cell row narrowing.
old = 'import base64\nimport heapq\n'
new = 'import base64\nimport bisect\nimport heapq\n'
if old not in text:
    raise SystemExit('missing import anchor')
text = text.replace(old, new, 1)

# Add blocked-row cache alongside prefix cache.
old = '        self._drivable_prefix_stride: int = 0\n'
new = old + '        self._blocked_cells_by_row: Optional[List[List[int]]] = None\n'
if old not in text:
    raise SystemExit('missing prefix cache anchor')
text = text.replace(old, new, 1)

# Clear blocked-row cache when there is no grid.
old = '''            self._drivable_prefix_stride = 0\n            return\n'''
new = '''            self._drivable_prefix_stride = 0\n            self._blocked_cells_by_row = None\n            return\n'''
if old not in text:
    raise SystemExit('missing prefix clear anchor')
text = text.replace(old, new, 1)

# Build row lists while building exact prefix counts.
old = '''        stride = grid.width_cells + 1\n        prefix = [0] * ((grid.height_cells + 1) * stride)\n        for iy in range(grid.height_cells):\n            row_blocked = 0\n            prev_base = iy * stride\n            base = (iy + 1) * stride\n            for ix in range(grid.width_cells):\n                if not grid.is_cell_drivable(ix, iy):\n                    row_blocked += 1\n                prefix[base + ix + 1] = prefix[prev_base + ix + 1] + row_blocked\n\n        self._drivable_blocked_prefix = prefix\n        self._drivable_prefix_grid_id = grid_id\n        self._drivable_prefix_stride = stride\n'''
new = '''        stride = grid.width_cells + 1\n        prefix = [0] * ((grid.height_cells + 1) * stride)\n        blocked_cells_by_row: List[List[int]] = [[] for _ in range(grid.height_cells)]\n        for iy in range(grid.height_cells):\n            row_blocked = 0\n            blocked_row = blocked_cells_by_row[iy]\n            prev_base = iy * stride\n            base = (iy + 1) * stride\n            for ix in range(grid.width_cells):\n                if not grid.is_cell_drivable(ix, iy):\n                    row_blocked += 1\n                    blocked_row.append(ix)\n                prefix[base + ix + 1] = prefix[prev_base + ix + 1] + row_blocked\n\n        self._drivable_blocked_prefix = prefix\n        self._drivable_prefix_grid_id = grid_id\n        self._drivable_prefix_stride = stride\n        self._blocked_cells_by_row = blocked_cells_by_row\n'''
if old not in text:
    raise SystemExit('missing prefix build block')
text = text.replace(old, new, 1)

# Insert analytical footprint AABB helper after footprint OBB.
anchor = '''    @staticmethod\n    def _obb_intersects_aabb(\n'''
helper = '''    @staticmethod\n    def _footprint_aabb_from_obb(\n        obb: Tuple[float, float, float, float, float, float, float, float]\n    ) -> Tuple[float, float, float, float]:\n        cx, cy, ux, uy, vx, vy, half_l, half_w = obb\n        half_x = half_l * abs(ux) + half_w * abs(vx)\n        half_y = half_l * abs(uy) + half_w * abs(vy)\n        return cx - half_x, cx + half_x, cy - half_y, cy + half_y\n\n'''
if anchor not in text:
    raise SystemExit('missing SAT anchor')
text = text.replace(anchor, helper + anchor, 1)

# Replace drivable footprint kernel: one OBB, analytical AABB, enumerate only blocked cells.
start = text.index('    def footprint_in_drivable_area(self, pose: Pose) -> bool:\n')
end = text.index('    def footprint_hits_slot_obstacle(\n', start)
new_block = '''    def footprint_in_drivable_area(\n        self,\n        pose: Pose,\n        obb: Optional[Tuple[float, float, float, float, float, float, float, float]] = None,\n        aabb: Optional[Tuple[float, float, float, float]] = None,\n    ) -> bool:\n        if obb is None:\n            obb = self._footprint_obb(pose)\n        if aabb is None:\n            aabb = self._footprint_aabb_from_obb(obb)\n        min_x, max_x, min_y, max_y = aabb\n\n        if min_x < 0.0 or max_x > self.map_width or min_y < 0.0 or max_y > self.map_height:\n            return False\n\n        if self.drivable_grid is None:\n            return not self.require_drivable_area\n\n        grid = self.drivable_grid\n        ix0 = max(0, int(math.floor(min_x / grid.resolution_m)))\n        ix1 = min(grid.width_cells - 1, int(math.floor(max_x / grid.resolution_m)))\n        iy0 = max(0, int(math.floor(min_y / grid.resolution_m)))\n        iy1 = min(grid.height_cells - 1, int(math.floor(max_y / grid.resolution_m)))\n\n        if self._blocked_count_in_rect(ix0, iy0, ix1, iy1) == 0:\n            return True\n\n        blocked_rows = self._blocked_cells_by_row\n        if blocked_rows is None:\n            raise RuntimeError('blocked-cell row cache missing')\n\n        for iy in range(iy0, iy1 + 1):\n            blocked_row = blocked_rows[iy]\n            left = bisect.bisect_left(blocked_row, ix0)\n            right = bisect.bisect_right(blocked_row, ix1)\n            for ix in blocked_row[left:right]:\n                x_min, x_max, y_min, y_max = grid.cell_bounds(ix, iy)\n                if self._obb_intersects_aabb(obb, x_min, x_max, y_min, y_max):\n                    return False\n        return True\n\n'''
text = text[:start] + new_block + text[end:]

# Replace slot kernel and pose collision: shared OBB/AABB + exact AABB reject before SAT.
start = text.index('    def footprint_hits_slot_obstacle(\n')
end = text.index('    def simulate_primitive(\n', start)
new_block = '''    def footprint_hits_slot_obstacle(\n        self,\n        pose: Pose,\n        obstacles: Sequence[RectObstacle],\n        obb: Optional[Tuple[float, float, float, float, float, float, float, float]] = None,\n        aabb: Optional[Tuple[float, float, float, float]] = None,\n    ) -> bool:\n        if obb is None:\n            obb = self._footprint_obb(pose)\n        if aabb is None:\n            aabb = self._footprint_aabb_from_obb(obb)\n        min_x, max_x, min_y, max_y = aabb\n        m = self.obstacle_inflation\n        for obs in obstacles:\n            ox0 = obs.x_min - m\n            ox1 = obs.x_max + m\n            oy0 = obs.y_min - m\n            oy1 = obs.y_max + m\n            if max_x < ox0 or min_x > ox1 or max_y < oy0 or min_y > oy1:\n                continue\n            if self._obb_intersects_aabb(obb, ox0, ox1, oy0, oy1):\n                return True\n        return False\n\n    def pose_collision(self, pose: Pose, obstacles: Sequence[RectObstacle]) -> bool:\n        if self.collision_mode == self.POINT_COLLISION_MODE:\n            return self.point_collision(pose.x, pose.y, obstacles)\n        obb = self._footprint_obb(pose)\n        aabb = self._footprint_aabb_from_obb(obb)\n        if not self.footprint_in_drivable_area(pose, obb=obb, aabb=aabb):\n            return True\n        return self.footprint_hits_slot_obstacle(pose, obstacles, obb=obb, aabb=aabb)\n\n'''
text = text[:start] + new_block + text[end:]

# Precompute curvature once per primitive instead of tan() at every integration step.
old = '''        x, y, yaw = node.x, node.y, node.yaw\n        remaining = self.motion_step\n        while remaining > 1e-9:\n'''
new = '''        x, y, yaw = node.x, node.y, node.yaw\n        curvature = math.tan(steer_rad) / self.wheelbase\n        remaining = self.motion_step\n        while remaining > 1e-9:\n'''
if old not in text:
    raise SystemExit('missing simulate anchor')
text = text.replace(old, new, 1)
old = '''            yaw = self.normalize_angle(\n                yaw + signed_ds / self.wheelbase * math.tan(steer_rad)\n            )\n'''
new = '''            yaw = self.normalize_angle(yaw + signed_ds * curvature)\n'''
if old not in text:
    raise SystemExit('missing curvature anchor')
text = text.replace(old, new, 1)

path.write_text(text, encoding='utf-8')
print('patched', path)
