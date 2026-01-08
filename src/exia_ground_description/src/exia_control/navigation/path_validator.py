"""
Path Validator - Validates paths against costmap for collisions

Checks if a planned path is collision-free by testing each pose
against the costmap. Accounts for robot footprint geometry.

Author: Zechariah Wang
Date: December 2025
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import math

import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion


@dataclass
class ValidationResult:
    """Result of path validation."""
    is_valid: bool
    blocked_index: int = -1  # Index of first blocked pose (-1 if valid)
    blocked_position: Tuple[float, float] = (0.0, 0.0)
    total_cost: float = 0.0
    high_cost_segments: int = 0


class PathValidator:
    """
    Validates paths against costmap for collision detection.

    Uses robot footprint to check if poses would collide with obstacles.
    Can also compute path cost for comparing alternative routes.
    """

    # Costmap value thresholds
    # Nav2 trinary costmap: 0=free, 100=lethal, 255=unknown
    # In trinary mode, costs are: free(0), inscribed(99), lethal(100), unknown(255/-1)
    #
    # IMPORTANT: Cost 99 is INSCRIBED space (robot center can pass but footprint would
    # touch obstacle). This is NOT a collision - it's a warning zone.
    # Only cost >= 100 is actual LETHAL collision where robot center would hit obstacle.
    #
    # Using 99 as threshold causes FALSE POSITIVES because:
    # - Costmap inflation creates inscribed zones around all obstacles
    # - Robot can safely traverse inscribed space if careful
    # - Only LETHAL (100+) means guaranteed collision
    LETHAL_THRESHOLD = 100      # Only actual lethal obstacles (NOT inscribed space at 99)
    HIGH_COST_THRESHOLD = 50    # High inflation cost - should avoid if possible
    UNKNOWN_VALUE = 255         # Unknown space (treat as obstacle for safety)

    def __init__(self, robot_footprint: Optional[List[Tuple[float, float]]] = None):
        """
        Initialize path validator.

        Args:
            robot_footprint: Robot outline as [(x1, y1), (x2, y2), ...] in robot frame.
                           Defaults to Exia Ground footprint if None.
        """
        # Default footprint for Exia Ground (2.11m x 1.2m with margin)
        if robot_footprint is None:
            self._footprint = [
                (1.15, 0.65),   # Front right
                (1.15, -0.65),  # Front left
                (-1.15, -0.65), # Rear left
                (-1.15, 0.65),  # Rear right
            ]
        else:
            self._footprint = robot_footprint

        self._costmap: Optional[OccupancyGrid] = None
        self._costmap_data: Optional[np.ndarray] = None

    def update_costmap(self, costmap: OccupancyGrid) -> None:
        """
        Update internal costmap data.

        Args:
            costmap: New costmap message
        """
        self._costmap = costmap
        # Reshape to 2D array for efficient lookup
        self._costmap_data = np.array(costmap.data, dtype=np.int8).reshape(
            (costmap.info.height, costmap.info.width)
        )

    def validate_path(self, path: Path,
                     check_footprint: bool = True,
                     max_poses: int = -1) -> ValidationResult:
        """
        Validate entire path against costmap.

        Args:
            path: Path to validate
            check_footprint: If True, check full robot footprint; else just center
            max_poses: Maximum poses to check (-1 for all)

        Returns:
            ValidationResult with validity and details
        """
        if self._costmap is None or self._costmap_data is None:
            return ValidationResult(is_valid=True)  # No costmap, assume valid

        if len(path.poses) == 0:
            return ValidationResult(is_valid=False, blocked_index=0)

        total_cost = 0.0
        high_cost_count = 0
        poses_to_check = path.poses if max_poses < 0 else path.poses[:max_poses]

        for i, pose_stamped in enumerate(poses_to_check):
            # Check if pose is in collision
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
        """
        Check if a single pose is collision-free.

        Args:
            pose: Pose to check
            check_footprint: If True, check full robot footprint

        Returns:
            True if pose is valid (no collision)
        """
        if self._costmap is None:
            return True

        if check_footprint:
            in_collision, _ = self._check_footprint_collision(pose)
            return not in_collision
        else:
            cost = self._get_cost_at_pose(pose)
            return cost < self.LETHAL_THRESHOLD

    def get_path_cost(self, path: Path) -> float:
        """
        Compute total traversal cost of path.

        Higher cost = path goes near obstacles.

        Args:
            path: Path to evaluate

        Returns:
            Sum of costmap values along path
        """
        result = self.validate_path(path, check_footprint=False)
        return result.total_cost

    def find_clear_direction(self, x: float, y: float, yaw: float,
                            check_distance: float = 3.0,
                            angle_steps: int = 8) -> Optional[float]:
        """
        Find a clear direction from current position.

        Args:
            x, y: Current position
            yaw: Current heading (radians)
            check_distance: Distance to check ahead
            angle_steps: Number of directions to check

        Returns:
            Angle offset (radians) for clear direction, or None if all blocked
        """
        if self._costmap is None:
            return 0.0

        # Check directions in order of preference (straight first, then alternating)
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
        """
        Check if robot footprint at pose collides with obstacles.

        Returns:
            (in_collision, max_cost) tuple
        """
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = self._quaternion_to_yaw(pose.pose.orientation)

        # Transform footprint to world coordinates
        world_footprint = self._transform_footprint(x, y, yaw)

        max_cost = 0
        for fx, fy in world_footprint:
            cost = self._get_cost_at_world(fx, fy)
            max_cost = max(max_cost, cost)
            if cost >= self.LETHAL_THRESHOLD:
                return True, cost

        # Also check center and intermediate points
        center_cost = self._get_cost_at_world(x, y)
        max_cost = max(max_cost, center_cost)

        return center_cost >= self.LETHAL_THRESHOLD, max_cost

    def _get_cost_at_pose(self, pose: PoseStamped) -> int:
        """Get costmap value at pose position."""
        return self._get_cost_at_world(
            pose.pose.position.x,
            pose.pose.position.y
        )

    def _get_cost_at_world(self, x: float, y: float) -> int:
        """Get costmap value at world coordinates."""
        if self._costmap is None or self._costmap_data is None:
            return 0

        info = self._costmap.info

        # Convert world coordinates to grid indices
        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        if 0 <= grid_x < info.width and 0 <= grid_y < info.height:
            return int(self._costmap_data[grid_y, grid_x])

        return 0

    def _transform_footprint(self, x: float, y: float,
                            yaw: float) -> List[Tuple[float, float]]:
        """Transform footprint from robot frame to world frame."""
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_footprint = []
        for fx, fy in self._footprint:
            # Rotation + translation
            wx = x + fx * cos_yaw - fy * sin_yaw
            wy = y + fx * sin_yaw + fy * cos_yaw
            world_footprint.append((wx, wy))

        return world_footprint

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        """Extract yaw angle from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
