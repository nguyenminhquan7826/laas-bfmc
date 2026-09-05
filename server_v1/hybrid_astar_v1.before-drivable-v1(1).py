from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

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


class HybridAStarPlanner:
    """Offline Hybrid A* prototype.

    Safety limitation:
      Collision checking currently protects only the rear-axle reference point.
      Full body footprint checking must not be enabled until rear/front overhang
      geometry has been physically verified.
    """

    def __init__(self, map_cfg: dict, vehicle_cfg: dict, planner_cfg: dict):
        self.map_cfg = map_cfg
        self.vehicle_cfg = vehicle_cfg
        self.cfg = planner_cfg

        self.map_width = float(map_cfg["map"]["width_x_m"])
        self.map_height = float(map_cfg["map"]["height_y_m"])
        self.wheelbase = float(vehicle_cfg["geometry"]["wheelbase_m"])

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

        self.obstacle_inflation = float(collision["obstacle_inflation_m"])
        self.treat_unknown_as_blocked = bool(collision.get("treat_unknown_slots_as_blocked", True))

        self.max_steer = max((abs(v) for v in self.steering_samples), default=1.0)

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

    def point_collision(self, x: float, y: float, obstacles: Sequence[RectObstacle]) -> bool:
        # Reference point must remain inside global map bounds.
        if x < 0.0 or x > self.map_width or y < 0.0 or y > self.map_height:
            return True
        m = self.obstacle_inflation
        for obs in obstacles:
            if (obs.x_min - m) <= x <= (obs.x_max + m) and (obs.y_min - m) <= y <= (obs.y_max + m):
                return True
        return False

    def simulate_primitive(
        self,
        node: Node,
        direction: int,
        steer_rad: float,
        obstacles: Sequence[RectObstacle],
    ) -> Optional[Pose]:
        x, y, yaw = node.x, node.y, node.yaw
        remaining = self.motion_step
        while remaining > 1e-9:
            ds = min(self.integration_step, remaining)
            signed_ds = direction * ds
            x += signed_ds * math.cos(yaw)
            y += signed_ds * math.sin(yaw)
            yaw = self.normalize_angle(yaw + signed_ds / self.wheelbase * math.tan(steer_rad))
            if self.point_collision(x, y, obstacles):
                return None
            remaining -= ds
        return Pose(x, y, yaw)

    def transition_cost(self, parent: Node, direction: int, steer_rad: float) -> float:
        base = self.motion_step * (self.reverse_multiplier if direction < 0 else 1.0)
        if parent.direction != 0 and direction != parent.direction:
            base += self.direction_switch_penalty
        if self.max_steer > 1e-9:
            base += self.steering_penalty * abs(steer_rad) / self.max_steer
            base += self.steering_change_penalty * abs(steer_rad - parent.steer_rad) / self.max_steer
        return base

    def plan(self, start: Pose, goal: Pose, obstacles: Sequence[RectObstacle]) -> PlanResult:
        if self.point_collision(start.x, start.y, obstacles):
            return PlanResult(False, [], math.inf, 0, "start_in_collision")
        if self.point_collision(goal.x, goal.y, obstacles):
            return PlanResult(False, [], math.inf, 0, "goal_in_collision")

        start_node = Node(start.x, start.y, self.normalize_angle(start.yaw), 0.0, None, 0, 0.0)
        counter = 0
        open_heap: List[Tuple[float, int, Node]] = []
        heapq.heappush(open_heap, (self.heuristic(start, goal), counter, start_node))
        best_g: Dict[Tuple[int, int, int], float] = {self.state_key(start.x, start.y, start.yaw): 0.0}

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
        return yaml.safe_load(f)


def slot_rect(slot: dict, source_id: str) -> RectObstacle:
    xs = [float(p[0]) for p in slot["polygon_m"]]
    ys = [float(p[1]) for p in slot["polygon_m"]]
    return RectObstacle(min(xs), max(xs), min(ys), max(ys), source_id)


def build_slot_obstacles(map_cfg: dict, slot_states: Dict[str, str], target_slot: Optional[str] = None) -> List[RectObstacle]:
    out: List[RectObstacle] = []
    for slot in map_cfg["slots"]:
        sid = slot["id"]
        if sid == target_slot:
            continue
        state = slot_states.get(sid, "UNKNOWN").upper()
        if state in ("OCCUPIED", "UNKNOWN"):
            out.append(slot_rect(slot, sid))
    return out
