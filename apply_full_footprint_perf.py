from pathlib import Path

path = Path("server_v1/hybrid_astar_v1.py")
text = path.read_text(encoding="utf-8")

old = "        self.drivable_grid: Optional[DrivableGrid] = None\n"
new = old + (
    "        self._drivable_blocked_prefix: Optional[List[int]] = None\n"
    "        self._drivable_prefix_grid_id: Optional[int] = None\n"
    "        self._drivable_prefix_stride: int = 0\n"
)
if old not in text:
    raise SystemExit("missing drivable_grid anchor")
text = text.replace(old, new, 1)

old = "        self.max_steer = max((abs(v) for v in self.steering_samples), default=1.0)\n\n    def _load_verified_footprint_geometry"
new = (
    "        self._ensure_drivable_blocked_prefix()\n"
    "        self.max_steer = max((abs(v) for v in self.steering_samples), default=1.0)\n\n"
    "    def _load_verified_footprint_geometry"
)
if old not in text:
    raise SystemExit("missing max_steer anchor")
text = text.replace(old, new, 1)

anchor = "    def footprint_in_drivable_area(self, pose: Pose) -> bool:\n"
helpers = '''    def _ensure_drivable_blocked_prefix(self) -> None:\n        grid = self.drivable_grid\n        if grid is None:\n            self._drivable_blocked_prefix = None\n            self._drivable_prefix_grid_id = None\n            self._drivable_prefix_stride = 0\n            return\n\n        grid_id = id(grid)\n        if (\n            self._drivable_blocked_prefix is not None\n            and self._drivable_prefix_grid_id == grid_id\n        ):\n            return\n\n        stride = grid.width_cells + 1\n        prefix = [0] * ((grid.height_cells + 1) * stride)\n        for iy in range(grid.height_cells):\n            row_blocked = 0\n            prev_base = iy * stride\n            base = (iy + 1) * stride\n            for ix in range(grid.width_cells):\n                if not grid.is_cell_drivable(ix, iy):\n                    row_blocked += 1\n                prefix[base + ix + 1] = prefix[prev_base + ix + 1] + row_blocked\n\n        self._drivable_blocked_prefix = prefix\n        self._drivable_prefix_grid_id = grid_id\n        self._drivable_prefix_stride = stride\n\n    def _blocked_count_in_rect(self, ix0: int, iy0: int, ix1: int, iy1: int) -> int:\n        self._ensure_drivable_blocked_prefix()\n        grid = self.drivable_grid\n        prefix = self._drivable_blocked_prefix\n        if grid is None or prefix is None:\n            return 0\n\n        ix0 = max(0, min(grid.width_cells - 1, ix0))\n        ix1 = max(0, min(grid.width_cells - 1, ix1))\n        iy0 = max(0, min(grid.height_cells - 1, iy0))\n        iy1 = max(0, min(grid.height_cells - 1, iy1))\n        if ix0 > ix1 or iy0 > iy1:\n            return 0\n\n        stride = self._drivable_prefix_stride\n        x0 = ix0\n        x1 = ix1 + 1\n        y0 = iy0\n        y1 = iy1 + 1\n        return (\n            prefix[y1 * stride + x1]\n            - prefix[y0 * stride + x1]\n            - prefix[y1 * stride + x0]\n            + prefix[y0 * stride + x0]\n        )\n\n'''
if anchor not in text:
    raise SystemExit("missing footprint anchor")
text = text.replace(anchor, helpers + anchor, 1)

old = "        iy1 = min(grid.height_cells - 1, int(math.floor(max_y / grid.resolution_m)))\n\n        for iy in range(iy0, iy1 + 1):\n"
new = (
    "        iy1 = min(grid.height_cells - 1, int(math.floor(max_y / grid.resolution_m)))\n\n"
    "        # Exact fail-safe broad phase: if the footprint AABB contains no blocked\n"
    "        # CAD cells, the OBB cannot intersect a blocked cell. This removes the\n"
    "        # expensive Python cell scan for the common all-drivable case without\n"
    "        # relaxing collision semantics.\n"
    "        if self._blocked_count_in_rect(ix0, iy0, ix1, iy1) == 0:\n"
    "            return True\n\n"
    "        for iy in range(iy0, iy1 + 1):\n"
)
if old not in text:
    raise SystemExit("missing CAD loop anchor")
text = text.replace(old, new, 1)

path.write_text(text, encoding="utf-8")
print("patched", path)
