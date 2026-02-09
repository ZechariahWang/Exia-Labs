from dataclasses import dataclass
from typing import List, Tuple, Optional
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion


@dataclass
class ValidationResult:
    is_valid: bool
    blocked_index: int = -1
    blocked_position: Tuple[float, float] = (0.0, 0.0)
    total_cost: float = 0.0
    high_cost_segments: int = 0


class PathValidator:

    LETHAL_THRESHOLD = 252
    HIGH_COST_THRESHOLD = 180
    UNKNOWN_VALUE = 255

    def __init__(self, robot_footprint: Optional[List[Tuple[float, float]]] = None):
        if robot_footprint is None:
            self._footprint = self._generate_dense_footprint(
                length=2.1, width=1.2, edge_spacing=0.3
            )
        else:
            self._footprint = robot_footprint

        self._costmap: Optional[OccupancyGrid] = None
        self._costmap_data: Optional[np.ndarray] = None

    def _generate_dense_footprint(
        self, length: float, width: float, edge_spacing: float
    ) -> List[Tuple[float, float]]:
        points = []
        half_length = length / 2.0
        half_width = width / 2.0

        x = -half_length
        while x <= half_length:
            points.append((x, half_width))
            points.append((x, -half_width))
            x += edge_spacing

        y = -half_width + edge_spacing
        while y < half_width:
            points.append((half_length, y))
            points.append((-half_length, y))
            y += edge_spacing

        points.append((half_length, half_width))
        points.append((half_length, -half_width))
        points.append((-half_length, half_width))
        points.append((-half_length, -half_width))

        return list(set(points))

    def update_costmap(self, costmap: OccupancyGrid) -> None:
        self._costmap = costmap
        self._costmap_data = np.array(costmap.data, dtype=np.uint8).reshape(
            (costmap.info.height, costmap.info.width)
        )

    def validate_path(self, path: Path,
                     check_footprint: bool = True,
                     max_poses: int = -1) -> ValidationResult:
        if self._costmap is None or self._costmap_data is None:
            return ValidationResult(is_valid=True)

        if len(path.poses) == 0:
            return ValidationResult(is_valid=False, blocked_index=0)

        total_cost = 0.0
        high_cost_count = 0
        poses_to_check = path.poses if max_poses < 0 else path.poses[:max_poses]

        for i, pose_stamped in enumerate(poses_to_check):
            if check_footprint:
                in_collision, cost = self._check_footprint_collision(pose_stamped)
            else:
                cost = self._get_cost_at_pose(pose_stamped)
                in_collision = cost >= self.LETHAL_THRESHOLD

            total_cost += cost

            if cost >= self.HIGH_COST_THRESHOLD:
                high_cost_count += 1

            if in_collision:
                return ValidationResult(
                    is_valid=False,
                    blocked_index=i,
                    blocked_position=(
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y
                    ),
                    total_cost=total_cost,
                    high_cost_segments=high_cost_count
                )

        return ValidationResult(
            is_valid=True,
            total_cost=total_cost,
            high_cost_segments=high_cost_count
        )

    def is_pose_valid(self, pose: PoseStamped,
                     check_footprint: bool = True) -> bool:
        if self._costmap is None:
            return True

        if check_footprint:
            in_collision, _ = self._check_footprint_collision(pose)
            return not in_collision
        else:
            cost = self._get_cost_at_pose(pose)
            return cost < self.LETHAL_THRESHOLD

    def get_path_cost(self, path: Path) -> float:
        result = self.validate_path(path, check_footprint=False)
        return result.total_cost

    def find_clear_direction(self, x: float, y: float, yaw: float,
                            check_distance: float = 3.0,
                            angle_steps: int = 8) -> Optional[float]:
        if self._costmap is None:
            return 0.0

        angle_offsets = [0.0]
        for i in range(1, angle_steps):
            angle = (i * math.pi) / angle_steps
            angle_offsets.extend([angle, -angle])

        for offset in angle_offsets:
            check_angle = yaw + offset
            check_x = x + check_distance * math.cos(check_angle)
            check_y = y + check_distance * math.sin(check_angle)

            cost = self._get_cost_at_world(check_x, check_y)
            if cost < self.HIGH_COST_THRESHOLD:
                return offset

        return None

    def _check_footprint_collision(self, pose: PoseStamped) -> Tuple[bool, float]:
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = self._quaternion_to_yaw(pose.pose.orientation)

        world_footprint = self._transform_footprint(x, y, yaw)

        max_cost = 0
        for fx, fy in world_footprint:
            cost = self._get_cost_at_world(fx, fy)
            max_cost = max(max_cost, cost)
            if cost >= self.LETHAL_THRESHOLD:
                return True, cost

        center_cost = self._get_cost_at_world(x, y)
        max_cost = max(max_cost, center_cost)

        return center_cost >= self.LETHAL_THRESHOLD, max_cost

    def _get_cost_at_pose(self, pose: PoseStamped) -> int:
        return self._get_cost_at_world(
            pose.pose.position.x,
            pose.pose.position.y
        )

    def _get_cost_at_world(self, x: float, y: float) -> int:
        if self._costmap is None or self._costmap_data is None:
            return 0

        info = self._costmap.info

        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            return int(self._costmap_data[grid_y, grid_x])

        return 0

    def _transform_footprint(self, x: float, y: float,
                            yaw: float) -> List[Tuple[float, float]]:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_footprint = []
        for fx, fy in self._footprint:
            wx = x + fx * cos_yaw - fy * sin_yaw
            wy = y + fx * sin_yaw + fy * cos_yaw
            world_footprint.append((wx, wy))

        return world_footprint

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
