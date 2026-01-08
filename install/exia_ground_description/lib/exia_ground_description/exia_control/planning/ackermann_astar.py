import math
import heapq
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict

import numpy as np
from nav_msgs.msg import OccupancyGrid

from .motion_primitives import MotionPrimitiveSet


@dataclass
class HybridState:
    x: float
    y: float
    theta: float
    g_cost: float = 0.0
    h_cost: float = 0.0
    parent: Optional['HybridState'] = None
    primitive_idx: int = -1

    @property
    def f_cost(self) -> float:
        return self.g_cost + self.h_cost

    def discretize(self, resolution: float, angle_bins: int) -> Tuple[int, int, int]:
        gx = int(self.x / resolution)
        gy = int(self.y / resolution)
        gt = int((self.theta + math.pi) / (2 * math.pi) * angle_bins) % angle_bins
        return (gx, gy, gt)

    def __lt__(self, other):
        return self.f_cost < other.f_cost


@dataclass(order=True)
class PriorityItem:
    priority: float
    counter: int = field(compare=True)
    state: HybridState = field(compare=False)


class AckermannAStar:
    def __init__(
        self,
        resolution: float = 0.1,
        angle_bins: int = 36,
        wheelbase: float = 1.3,
        max_steering: float = 0.6,
        robot_footprint: Optional[List[Tuple[float, float]]] = None,
        obstacle_threshold: int = 50,
        max_iterations: int = 50000,
        timeout_ms: float = 500.0,
        goal_tolerance: float = 1.0,
        goal_angle_tolerance: float = 0.5,
    ):
        self.resolution = resolution
        self.angle_bins = angle_bins
        self.wheelbase = wheelbase
        self.max_steering = max_steering
        self.obstacle_threshold = obstacle_threshold
        self.max_iterations = max_iterations
        self.timeout_ms = timeout_ms
        self.goal_tolerance = goal_tolerance
        self.goal_angle_tolerance = goal_angle_tolerance

        if robot_footprint is None:
            self.robot_footprint = [
                (1.1, 0.63),
                (1.1, -0.63),
                (-1.1, -0.63),
                (-1.1, 0.63),
            ]
        else:
            self.robot_footprint = robot_footprint

        self.motion_primitives = MotionPrimitiveSet(
            wheelbase=wheelbase,
            max_steering=max_steering,
            step_length=resolution * 3,
            num_angles=7
        )

        self._costmap: Optional[np.ndarray] = None
        self._costmap_info = None
        self._costmap_origin_x = 0.0
        self._costmap_origin_y = 0.0

    def set_costmap(self, costmap: OccupancyGrid):
        self._costmap = np.array(costmap.data, dtype=np.int8).reshape(
            costmap.info.height, costmap.info.width
        )
        self._costmap_info = costmap.info
        self._costmap_origin_x = costmap.info.origin.position.x
        self._costmap_origin_y = costmap.info.origin.position.y
        self._costmap_resolution = costmap.info.resolution

    def plan(
        self,
        start_x: float,
        start_y: float,
        start_theta: float,
        goal_x: float,
        goal_y: float,
        goal_theta: Optional[float] = None,
    ) -> Optional[List[HybridState]]:
        if self._costmap is None:
            return None

        start_time = time.time()
        counter = 0

        start_state = HybridState(
            x=start_x,
            y=start_y,
            theta=self._normalize_angle(start_theta),
            g_cost=0.0,
            h_cost=self._heuristic(start_x, start_y, goal_x, goal_y)
        )

        if self._is_in_collision(start_x, start_y, start_theta):
            return None

        open_set: List[PriorityItem] = []
        heapq.heappush(open_set, PriorityItem(start_state.f_cost, counter, start_state))
        counter += 1

        closed_set: Dict[Tuple[int, int, int], HybridState] = {}

        iterations = 0

        while open_set and iterations < self.max_iterations:
            elapsed_ms = (time.time() - start_time) * 1000
            if elapsed_ms > self.timeout_ms:
                break

            iterations += 1

            current_item = heapq.heappop(open_set)
            current = current_item.state

            dist_to_goal = math.sqrt(
                (current.x - goal_x)**2 + (current.y - goal_y)**2
            )

            if dist_to_goal < self.goal_tolerance:
                if goal_theta is None:
                    return self._reconstruct_path(current)
                else:
                    angle_diff = abs(self._normalize_angle(current.theta - goal_theta))
                    if angle_diff < self.goal_angle_tolerance:
                        return self._reconstruct_path(current)

            disc_state = current.discretize(self.resolution, self.angle_bins)
            if disc_state in closed_set:
                continue
            closed_set[disc_state] = current

            successors = self.motion_primitives.get_successors(
                current.x, current.y, current.theta
            )

            for idx, (new_x, new_y, new_theta, arc_length, prim_cost) in enumerate(successors):
                if not self._is_collision_free_arc(current, idx):
                    continue

                proximity_cost = self._get_proximity_cost(new_x, new_y)
                heading_cost = abs(self._normalize_angle(new_theta - current.theta)) * 0.3

                new_g_cost = current.g_cost + arc_length * prim_cost + proximity_cost + heading_cost

                new_state = HybridState(
                    x=new_x,
                    y=new_y,
                    theta=new_theta,
                    g_cost=new_g_cost,
                    h_cost=self._heuristic(new_x, new_y, goal_x, goal_y),
                    parent=current,
                    primitive_idx=idx
                )

                new_disc = new_state.discretize(self.resolution, self.angle_bins)
                if new_disc in closed_set:
                    continue

                heapq.heappush(open_set, PriorityItem(new_state.f_cost, counter, new_state))
                counter += 1

        return None

    def _heuristic(self, x: float, y: float, goal_x: float, goal_y: float) -> float:
        return math.sqrt((goal_x - x)**2 + (goal_y - y)**2)

    def _is_in_collision(self, x: float, y: float, theta: float) -> bool:
        footprint_world = self._transform_footprint(x, y, theta)

        for fx, fy in footprint_world:
            cost = self._get_cost_at(fx, fy)
            if cost >= self.obstacle_threshold or cost < 0:
                return True

        center_cost = self._get_cost_at(x, y)
        if center_cost >= self.obstacle_threshold or center_cost < 0:
            return True

        return False

    def _is_collision_free_arc(self, start_state: HybridState, primitive_idx: int) -> bool:
        samples = self.motion_primitives.interpolate_arc(
            start_state.x, start_state.y, start_state.theta,
            primitive_idx, num_samples=5
        )

        for sx, sy, stheta in samples:
            if self._is_in_collision(sx, sy, stheta):
                return False

        return True

    def _get_cost_at(self, x: float, y: float) -> int:
        if self._costmap is None:
            return 0

        grid_x = int((x - self._costmap_origin_x) / self._costmap_resolution)
        grid_y = int((y - self._costmap_origin_y) / self._costmap_resolution)

        if 0 <= grid_x < self._costmap.shape[1] and 0 <= grid_y < self._costmap.shape[0]:
            return int(self._costmap[grid_y, grid_x])

        return 255

    def _get_proximity_cost(self, x: float, y: float) -> float:
        cost = self._get_cost_at(x, y)
        if cost < 0:
            cost = 0
        return cost / 50.0

    def _transform_footprint(
        self,
        x: float,
        y: float,
        theta: float
    ) -> List[Tuple[float, float]]:
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        world_footprint = []
        for fx, fy in self.robot_footprint:
            wx = x + fx * cos_theta - fy * sin_theta
            wy = y + fx * sin_theta + fy * cos_theta
            world_footprint.append((wx, wy))

        return world_footprint

    def _reconstruct_path(self, goal_state: HybridState) -> List[HybridState]:
        path = []
        current = goal_state

        while current is not None:
            path.append(current)
            current = current.parent

        path.reverse()
        return path

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
