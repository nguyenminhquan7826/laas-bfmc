from __future__ import annotations

import base64
import bisect
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Node:
    x: float
    y: float
    yaw: float
    g: float
    parent: Optional["Node"]
    direction: int  # +1 forward, -1 reverse, 0 start
    steer_rad: float

    @property
    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.yaw)


@dataclass
class PlanResult:
    success: bool
    path: List[Node]
    cost: float
    expansions: int
    reason: str


@dataclass(frozen=True)
class RectObstacle:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    source_id: str


@dataclass(frozen=True)
class DrivableGrid:
    resolution_m: float
    width_cells: int
    height_cells: int
    width_x_m: float
    height_y_m: float
    packed: bytes

    @classmethod
    def load(cls, path: Path) -> "DrivableGrid":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if payload.get("representation") != "BASE64_PACKED_BIT_GRID":
            raise ValueError(f"unsupported drivable-grid representation: {payload.get('representation')}")
        if payload.get("bitorder") != "little":
            raise ValueError("drivable grid currently requires bitorder=little")
        return cls(
            resolution_m=float(payload["resolution_m"]),
            width_cells=int(payload["width_cells"]),
            height_cells=int(payload["height_cells"]),
            width_x_m=float(payload["width_x_m"]),
            height_y_m=float(payload["height_y_m"]),
            packed=base64.b64decode(payload["data_b64"]),
        )

    def is_cell_drivable(self, ix: int, iy: int) -> bool:
        if ix < 0 or ix >= self.width_cells or iy < 0 or iy >= self.height_cells:
            return False
        idx = iy * self.width_cells + ix
        byte = self.packed[idx >> 3]
        return bool((byte >> (idx & 7)) & 1)

    def cell_bounds(self, ix: int, iy: int) -> Tuple[float, float, float, float]:
        x_min = ix * self.resolution_m
        x_max = min(self.width_x_m, (ix + 1) * self.resolution_m)
        y_min = iy * self.resolution_m
        y_max = min(self.height_y_m, (iy + 1) * self.resolution_m)
        return x_min, x_max, y_min, y_max

    def is_drivable(self, x: float, y: float) -> bool:
        if x < 0.0 or x > self.width_x_m or y < 0.0 or y > self.height_y_m:
            return False
        ix = min(self.width_cells - 1, max(0, int(x / self.resolution_m)))
        iy = min(self.height_cells - 1, max(0, int(y / self.resolution_m)))
        return self.is_cell_drivable(ix, iy)


class HybridAStarPlanner:
    """Offline Hybrid A* prototype with fail-closed collision modes.

    REAR_AXLE_POINT_ONLY preserves the existing V1 behavior.
    FULL_FOOTPRINT checks an oriented rectangular vehicle body against both
    the CAD drivable grid and parking-slot obstacles. FULL_FOOTPRINT refuses
    to initialize unless the required physical geometry is explicitly marked
    verified; the planner never infers missing overhangs.
    """

    POINT_COLLISION_MODE = "REAR_AXLE_POINT_ONLY"
    FULL_FOOTPRINT_MODE = "FULL_FOOTPRINT"

    def __init__(self, map_cfg: dict, vehicle_cfg: dict, planner_cfg: dict):
        self.map_cfg = map_cfg
        self.vehicle_cfg = vehicle_cfg
        self.cfg = planner_cfg

        self.map_width = float(map_cfg["map"]["width_x_m"])
        self.map_height = float(map_cfg["map"]["height_y_m"])

        geometry = vehicle_cfg["geometry"]
        self.wheelbase = float(geometry["wheelbase_m"])
        self.vehicle_length = float(geometry["length_m"])
        self.vehicle_width = float(geometry["width_m"])

        search = planner_cfg["search"]
        motion = planner_cfg["motion"]
        cost = planner_cfg["cost"]
        goal = planner_cfg["goal"]
        collision = planner_cfg["collision"]

        self.grid_res = float(search["grid_resolution_m"])
        self.yaw_res = math.radians(float(search["yaw_resolution_deg"]))
        self.motion_step = float(search["motion_step_m"])
        self.integration_step = float(search["integration_step_m"])
        self.max_expansions = int(search["max_expansions"])

        self.steering_samples = [math.radians(float(v)) for v in motion["steering_samples_deg"]]
        self.directions = []
        if motion.get("allow_forward", True):
            self.directions.append(+1)
        if motion.get("allow_reverse", True):
            self.directions.append(-1)

        self.reverse_multiplier = float(cost["reverse_multiplier"])
        self.direction_switch_penalty = float(cost["direction_switch_penalty"])
        self.steering_penalty = float(cost["steering_penalty"])
        self.steering_change_penalty = float(cost["steering_change_penalty"])
        self.yaw_heuristic_weight = float(cost["yaw_heuristic_weight"])

        self.goal_pos_tol = float(goal["position_tolerance_m"])
        self.goal_yaw_tol = math.radians(float(goal["yaw_tolerance_deg"]))

        self.collision_mode = str(collision.get("mode", self.POINT_COLLISION_MODE)).upper()
        if self.collision_mode not in (self.POINT_COLLISION_MODE, self.FULL_FOOTPRINT_MODE):
            raise ValueError(f"unsupported collision.mode={self.collision_mode}")

        self.obstacle_inflation = float(collision["obstacle_inflation_m"])
        self.treat_unknown_as_blocked = bool(collision.get("treat_unknown_slots_as_blocked", True))
        self.require_drivable_area = bool(collision.get("require_drivable_area", False))
        self.require_verified_geometry = bool(collision.get("require_verified_geometry", True))

        self.rear_overhang: Optional[float] = None
        self.front_overhang: Optional[float] = None
        self.body_center_from_rear_axle: Optional[float] = None
        if self.collision_mode == self.FULL_FOOTPRINT_MODE:
            self._load_verified_footprint_geometry(geometry)

        self.drivable_grid: Optional[DrivableGrid] = None
        self._drivable_blocked_prefix: Optional[List[int]] = None
        self._drivable_prefix_grid_id: Optional[int] = None
        self._drivable_prefix_stride: int = 0
        self._blocked_cells_by_row: Optional[List[List[int]]] = None
        semantic = map_cfg.get("semantic_map", {})
        grid_file = semantic.get("drivable_grid_file")
        if grid_file:
            root = Path(str(map_cfg.get("_source_dir", ".")))
            grid_path = root / str(grid_file)
            if not grid_path.exists():
                if self.require_drivable_area:
                    raise FileNotFoundError(f"required drivable grid not found: {grid_path}")
            else:
                self.drivable_grid = DrivableGrid.load(grid_path)
                if (
                    abs(self.drivable_grid.width_x_m - self.map_width) > 1e-6
                    or abs(self.drivable_grid.height_y_m - self.map_height) > 1e-6
                ):
                    raise ValueError("drivable-grid physical extent does not match map_v1")
        elif self.require_drivable_area:
            raise ValueError(
                "collision.require_drivable_area=true but map semantic_map.drivable_grid_file is not configured"
            )

        self._ensure_drivable_blocked_prefix()
        self.max_steer = max((abs(v) for v in self.steering_samples), default=1.0)

    def _load_verified_footprint_geometry(self, geometry: dict) -> None:
        if self.require_verified_geometry and not bool(geometry.get("footprint_verified", False)):
            raise ValueError("FULL_FOOTPRINT requires geometry.footprint_verified=true")

        required = ("rear_overhang_m", "front_overhang_m")
        missing = [key for key in required if geometry.get(key) is None]
        if missing:
            raise ValueError(
                "FULL_FOOTPRINT requires measured geometry: " + ", ".join(missing)
            )

        rear = float(geometry["rear_overhang_m"])
        front = float(geometry["front_overhang_m"])
        if rear < 0.0 or front < 0.0 or self.vehicle_length <= 0.0 or self.vehicle_width <= 0.0:
            raise ValueError("FULL_FOOTPRINT geometry dimensions must be positive/non-negative")

        expected_length = rear + self.wheelbase + front
        if abs(expected_length - self.vehicle_length) > 0.01:
            raise ValueError(
                "FULL_FOOTPRINT geometry inconsistent: length_m must equal "
                "rear_overhang_m + wheelbase_m + front_overhang_m within 0.01 m"
            )

        center = self.vehicle_length / 2.0 - rear
        configured_center = geometry.get("body_center_from_rear_axle_m")
        if configured_center is not None and abs(float(configured_center) - center) > 0.01:
            raise ValueError(
                "FULL_FOOTPRINT geometry inconsistent: body_center_from_rear_axle_m"
            )

        self.rear_overhang = rear
        self.front_overhang = front
        self.body_center_from_rear_axle = center

    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def state_key(self, x: float, y: float, yaw: float) -> Tuple[int, int, int]:
        ix = int(round(x / self.grid_res))
        iy = int(round(y / self.grid_res))
        yaw_n = self.normalize_angle(yaw)
        iyaw = int(round((yaw_n + math.pi) / self.yaw_res))
        n_yaw = max(1, int(round(2.0 * math.pi / self.yaw_res)))
        return ix, iy, iyaw % n_yaw

    def heuristic(self, pose: Pose, goal: Pose) -> float:
        d = math.hypot(goal.x - pose.x, goal.y - pose.y)
        yaw_err = abs(self.normalize_angle(goal.yaw - pose.yaw))
        return d + self.yaw_heuristic_weight * yaw_err

    def goal_reached(self, node: Node, goal: Pose) -> bool:
        pos_err = math.hypot(goal.x - node.x, goal.y - node.y)
        yaw_err = abs(self.normalize_angle(goal.yaw - node.yaw))
        return pos_err <= self.goal_pos_tol and yaw_err <= self.goal_yaw_tol

    def point_in_drivable_area(self, x: float, y: float) -> bool:
        if x < 0.0 or x > self.map_width or y < 0.0 or y > self.map_height:
            return False
        if self.drivable_grid is None:
            return not self.require_drivable_area
        return self.drivable_grid.is_drivable(x, y)

    def point_hits_slot_obstacle(self, x: float, y: float, obstacles: Sequence[RectObstacle]) -> bool:
        m = self.obstacle_inflation
        for obs in obstacles:
            if (
                (obs.x_min - m) <= x <= (obs.x_max + m)
                and (obs.y_min - m) <= y <= (obs.y_max + m)
            ):
                return True
        return False

    def point_collision(self, x: float, y: float, obstacles: Sequence[RectObstacle]) -> bool:
        if not self.point_in_drivable_area(x, y):
            return True
        return self.point_hits_slot_obstacle(x, y, obstacles)

    def _footprint_obb(
        self, pose: Pose
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        if self.body_center_from_rear_axle is None:
            raise RuntimeError("footprint OBB requested without verified geometry")
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        center_offset = self.body_center_from_rear_axle
        cx = pose.x + center_offset * c
        cy = pose.y + center_offset * s
        ux, uy = c, s
        vx, vy = -s, c
        return (
            cx,
            cy,
            ux,
            uy,
            vx,
            vy,
            self.vehicle_length / 2.0,
            self.vehicle_width / 2.0,
        )

    @staticmethod
    def _footprint_aabb_from_obb(
        obb: Tuple[float, float, float, float, float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        cx, cy, ux, uy, vx, vy, half_l, half_w = obb
        half_x = half_l * abs(ux) + half_w * abs(vx)
        half_y = half_l * abs(uy) + half_w * abs(vy)
        return cx - half_x, cx + half_x, cy - half_y, cy + half_y

    @staticmethod
    def _obb_intersects_aabb(
        obb: Tuple[float, float, float, float, float, float, float, float],
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ) -> bool:
        cx, cy, ux, uy, vx, vy, half_l, half_w = obb
        acx = (x_min + x_max) * 0.5
        acy = (y_min + y_max) * 0.5
        ahx = max(0.0, (x_max - x_min) * 0.5)
        ahy = max(0.0, (y_max - y_min) * 0.5)
        dx = acx - cx
        dy = acy - cy

        for ax, ay in ((1.0, 0.0), (0.0, 1.0), (ux, uy), (vx, vy)):
            center_distance = abs(dx * ax + dy * ay)
            obb_radius = (
                half_l * abs(ux * ax + uy * ay)
                + half_w * abs(vx * ax + vy * ay)
            )
            aabb_radius = ahx * abs(ax) + ahy * abs(ay)
            if center_distance > obb_radius + aabb_radius + 1e-12:
                return False
        return True

    def footprint_corners(self, pose: Pose) -> List[Tuple[float, float]]:
        cx, cy, ux, uy, vx, vy, half_l, half_w = self._footprint_obb(pose)
        corners: List[Tuple[float, float]] = []
        for longitudinal in (-half_l, half_l):
            for lateral in (-half_w, half_w):
                corners.append(
                    (
                        cx + longitudinal * ux + lateral * vx,
                        cy + longitudinal * uy + lateral * vy,
                    )
                )
        return corners

    def _ensure_drivable_blocked_prefix(self) -> None:
        grid = self.drivable_grid
        if grid is None:
            self._drivable_blocked_prefix = None
            self._drivable_prefix_grid_id = None
            self._drivable_prefix_stride = 0
            self._blocked_cells_by_row = None
            return

        grid_id = id(grid)
        if (
            self._drivable_blocked_prefix is not None
            and self._drivable_prefix_grid_id == grid_id
        ):
            return

        stride = grid.width_cells + 1
        prefix = [0] * ((grid.height_cells + 1) * stride)
        blocked_cells_by_row: List[List[int]] = [[] for _ in range(grid.height_cells)]
        for iy in range(grid.height_cells):
            row_blocked = 0
            blocked_row = blocked_cells_by_row[iy]
            prev_base = iy * stride
            base = (iy + 1) * stride
            for ix in range(grid.width_cells):
                if not grid.is_cell_drivable(ix, iy):
                    row_blocked += 1
                    blocked_row.append(ix)
                prefix[base + ix + 1] = prefix[prev_base + ix + 1] + row_blocked

        self._drivable_blocked_prefix = prefix
        self._drivable_prefix_grid_id = grid_id
        self._drivable_prefix_stride = stride
        self._blocked_cells_by_row = blocked_cells_by_row

    def _blocked_count_in_rect(self, ix0: int, iy0: int, ix1: int, iy1: int) -> int:
        self._ensure_drivable_blocked_prefix()
        grid = self.drivable_grid
        prefix = self._drivable_blocked_prefix
        if grid is None or prefix is None:
            return 0

        ix0 = max(0, min(grid.width_cells - 1, ix0))
        ix1 = max(0, min(grid.width_cells - 1, ix1))
        iy0 = max(0, min(grid.height_cells - 1, iy0))
        iy1 = max(0, min(grid.height_cells - 1, iy1))
        if ix0 > ix1 or iy0 > iy1:
            return 0

        stride = self._drivable_prefix_stride
        x0 = ix0
        x1 = ix1 + 1
        y0 = iy0
        y1 = iy1 + 1
        return (
            prefix[y1 * stride + x1]
            - prefix[y0 * stride + x1]
            - prefix[y1 * stride + x0]
            + prefix[y0 * stride + x0]
        )

    def footprint_in_drivable_area(
        self,
        pose: Pose,
        obb: Optional[Tuple[float, float, float, float, float, float, float, float]] = None,
        aabb: Optional[Tuple[float, float, float, float]] = None,
    ) -> bool:
        if obb is None:
            obb = self._footprint_obb(pose)
        if aabb is None:
            aabb = self._footprint_aabb_from_obb(obb)
        min_x, max_x, min_y, max_y = aabb

        if min_x < 0.0 or max_x > self.map_width or min_y < 0.0 or max_y > self.map_height:
            return False

        if self.drivable_grid is None:
            return not self.require_drivable_area

        grid = self.drivable_grid
        ix0 = max(0, int(math.floor(min_x / grid.resolution_m)))
        ix1 = min(grid.width_cells - 1, int(math.floor(max_x / grid.resolution_m)))
        iy0 = max(0, int(math.floor(min_y / grid.resolution_m)))
        iy1 = min(grid.height_cells - 1, int(math.floor(max_y / grid.resolution_m)))

        if self._blocked_count_in_rect(ix0, iy0, ix1, iy1) == 0:
            return True

        blocked_rows = self._blocked_cells_by_row
        if blocked_rows is None:
            raise RuntimeError('blocked-cell row cache missing')

        for iy in range(iy0, iy1 + 1):
            blocked_row = blocked_rows[iy]
            left = bisect.bisect_left(blocked_row, ix0)
            right = bisect.bisect_right(blocked_row, ix1)
            for ix in blocked_row[left:right]:
                x_min, x_max, y_min, y_max = grid.cell_bounds(ix, iy)
                if self._obb_intersects_aabb(obb, x_min, x_max, y_min, y_max):
                    return False
        return True

    def footprint_hits_slot_obstacle(
        self,
        pose: Pose,
        obstacles: Sequence[RectObstacle],
        obb: Optional[Tuple[float, float, float, float, float, float, float, float]] = None,
        aabb: Optional[Tuple[float, float, float, float]] = None,
    ) -> bool:
        if obb is None:
            obb = self._footprint_obb(pose)
        if aabb is None:
            aabb = self._footprint_aabb_from_obb(obb)
        min_x, max_x, min_y, max_y = aabb
        m = self.obstacle_inflation
        for obs in obstacles:
            ox0 = obs.x_min - m
            ox1 = obs.x_max + m
            oy0 = obs.y_min - m
            oy1 = obs.y_max + m
            if max_x < ox0 or min_x > ox1 or max_y < oy0 or min_y > oy1:
                continue
            if self._obb_intersects_aabb(obb, ox0, ox1, oy0, oy1):
                return True
        return False

    def pose_collision(self, pose: Pose, obstacles: Sequence[RectObstacle]) -> bool:
        if self.collision_mode == self.POINT_COLLISION_MODE:
            return self.point_collision(pose.x, pose.y, obstacles)
        obb = self._footprint_obb(pose)
        aabb = self._footprint_aabb_from_obb(obb)
        if not self.footprint_in_drivable_area(pose, obb=obb, aabb=aabb):
            return True
        return self.footprint_hits_slot_obstacle(pose, obstacles, obb=obb, aabb=aabb)

    def simulate_primitive(
        self,
        node: Node,
        direction: int,
        steer_rad: float,
        obstacles: Sequence[RectObstacle],
    ) -> Optional[Pose]:
        x, y, yaw = node.x, node.y, node.yaw
        curvature = math.tan(steer_rad) / self.wheelbase
        remaining = self.motion_step
        while remaining > 1e-9:
            ds = min(self.integration_step, remaining)
            signed_ds = direction * ds
            x += signed_ds * math.cos(yaw)
            y += signed_ds * math.sin(yaw)
            yaw = self.normalize_angle(yaw + signed_ds * curvature)
            if self.pose_collision(Pose(x, y, yaw), obstacles):
                return None
            remaining -= ds
        return Pose(x, y, yaw)

    def transition_cost(self, parent: Node, direction: int, steer_rad: float) -> float:
        base = self.motion_step * (self.reverse_multiplier if direction < 0 else 1.0)
        if parent.direction != 0 and direction != parent.direction:
            base += self.direction_switch_penalty
        if self.max_steer > 1e-9:
            base += self.steering_penalty * abs(steer_rad) / self.max_steer
            base += (
                self.steering_change_penalty
                * abs(steer_rad - parent.steer_rad)
                / self.max_steer
            )
        return base

    def plan(self, start: Pose, goal: Pose, obstacles: Sequence[RectObstacle]) -> PlanResult:
        if self.collision_mode == self.POINT_COLLISION_MODE:
            if not self.point_in_drivable_area(start.x, start.y):
                return PlanResult(False, [], math.inf, 0, "start_outside_drivable_area")
            if not self.point_in_drivable_area(goal.x, goal.y):
                return PlanResult(False, [], math.inf, 0, "goal_outside_drivable_area")
            if self.point_hits_slot_obstacle(start.x, start.y, obstacles):
                return PlanResult(False, [], math.inf, 0, "start_in_slot_obstacle")
            if self.point_hits_slot_obstacle(goal.x, goal.y, obstacles):
                return PlanResult(False, [], math.inf, 0, "goal_in_slot_obstacle")
        else:
            if not self.footprint_in_drivable_area(start):
                return PlanResult(False, [], math.inf, 0, "start_footprint_outside_drivable_area")
            if not self.footprint_in_drivable_area(goal):
                return PlanResult(False, [], math.inf, 0, "goal_footprint_outside_drivable_area")
            if self.footprint_hits_slot_obstacle(start, obstacles):
                return PlanResult(False, [], math.inf, 0, "start_footprint_in_slot_obstacle")
            if self.footprint_hits_slot_obstacle(goal, obstacles):
                return PlanResult(False, [], math.inf, 0, "goal_footprint_in_slot_obstacle")

        start_node = Node(
            start.x,
            start.y,
            self.normalize_angle(start.yaw),
            0.0,
            None,
            0,
            0.0,
        )
        counter = 0
        open_heap: List[Tuple[float, int, Node]] = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), counter, start_node))
        best_g: Dict[Tuple[int, int, int], float] = {
            self.state_key(start.x, start.y, start.yaw): 0.0
        }

        expansions = 0
        while open_heap and expansions < self.max_expansions:
            _, _, current = heapq.heappop(open_heap)
            key = self.state_key(current.x, current.y, current.yaw)
            if current.g > best_g.get(key, math.inf) + 1e-9:
                continue

            expansions += 1
            if self.goal_reached(current, goal):
                path = self.reconstruct(current)
                return PlanResult(True, path, current.g, expansions, "goal_reached")

            for direction in self.directions:
                for steer in self.steering_samples:
                    nxt = self.simulate_primitive(current, direction, steer, obstacles)
                    if nxt is None:
                        continue
                    g2 = current.g + self.transition_cost(current, direction, steer)
                    k2 = self.state_key(nxt.x, nxt.y, nxt.yaw)
                    if g2 + 1e-9 >= best_g.get(k2, math.inf):
                        continue
                    best_g[k2] = g2
                    child = Node(nxt.x, nxt.y, nxt.yaw, g2, current, direction, steer)
                    counter += 1
                    f2 = g2 + self.heuristic(nxt, goal)
                    heapq.heappush(open_heap, (f2, counter, child))

        reason = "max_expansions" if expansions >= self.max_expansions else "open_set_exhausted"
        return PlanResult(False, [], math.inf, expansions, reason)

    @staticmethod
    def reconstruct(goal_node: Node) -> List[Node]:
        out: List[Node] = []
        n: Optional[Node] = goal_node
        while n is not None:
            out.append(n)
            n = n.parent
        out.reverse()
        return out


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if isinstance(data, dict):
        data["_source_dir"] = str(path.resolve().parent)
    return data


def slot_rect(slot: dict, source_id: str) -> RectObstacle:
    xs = [float(p[0]) for p in slot["polygon_m"]]
    ys = [float(p[1]) for p in slot["polygon_m"]]
    return RectObstacle(min(xs), max(xs), min(ys), max(ys), source_id)


def build_slot_obstacles(
    map_cfg: dict,
    slot_states: Dict[str, str],
    target_slot: Optional[str] = None,
) -> List[RectObstacle]:
    out: List[RectObstacle] = []
    for slot in map_cfg["slots"]:
        sid = slot["id"]
        if sid == target_slot:
            continue
        state = slot_states.get(sid, "UNKNOWN").upper()
        if state in ("OCCUPIED", "UNKNOWN"):
            out.append(slot_rect(slot, sid))
    return out
