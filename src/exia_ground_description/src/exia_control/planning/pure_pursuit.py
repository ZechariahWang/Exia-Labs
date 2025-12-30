"""
Pure Pursuit Path Following Controller

A geometric path tracking controller that follows a path by
steering toward a lookahead point on the path.

Algorithm:
    1. Find the closest point on the path
    2. Find the lookahead point (distance ahead on path)
    3. Compute steering angle to reach lookahead point
    4. Apply Ackermann kinematics

Author: Zechariah Wang
Date: December 2025
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple, List

from geometry_msgs.msg import Twist


@dataclass
class PurePursuitConfig:
    """Configuration for Pure Pursuit controller."""
    # Lookahead parameters (scaled for larger vehicle)
    lookahead_distance: float = 2.0      # Base lookahead distance (m)
    min_lookahead: float = 1.0           # Minimum lookahead (m)
    max_lookahead: float = 5.0           # Maximum lookahead (m)
    lookahead_gain: float = 0.3          # Speed-dependent lookahead gain

    # Goal parameters
    goal_tolerance: float = 1.0          # Distance to consider goal reached (m)

    # Speed control
    max_linear_speed: float = 2.0        # Maximum speed (m/s)
    min_linear_speed: float = 0.3        # Minimum speed when moving (m/s)
    max_angular_speed: float = 1.0       # Maximum angular velocity (rad/s)

    # Vehicle parameters
    wheelbase: float = 1.3               # Front to rear axle (m)


@dataclass
class PathPoint:
    """A point on the path with optional speed."""
    x: float
    y: float
    speed: Optional[float] = None  # Desired speed at this point (m/s)


class Path:
    """A path consisting of waypoints."""

    def __init__(self, points: Optional[List[PathPoint]] = None):
        self.points: List[PathPoint] = points or []
        self._current_index = 0

    def add_point(self, x: float, y: float, speed: Optional[float] = None):
        """Add a point to the path."""
        self.points.append(PathPoint(x, y, speed))

    def clear(self):
        """Clear all points."""
        self.points = []
        self._current_index = 0

    def reset(self):
        """Reset to start of path."""
        self._current_index = 0

    def is_empty(self) -> bool:
        """Check if path is empty."""
        return len(self.points) == 0

    def __len__(self) -> int:
        return len(self.points)

    def __getitem__(self, index: int) -> PathPoint:
        return self.points[index]


class PurePursuitController:
    """
    Pure Pursuit path following controller.

    Uses proper lookahead point finding along the path rather than
    waypoint-by-waypoint targeting.
    """

    def __init__(self, config: Optional[PurePursuitConfig] = None):
        self.config = config or PurePursuitConfig()
        self._path: Optional[Path] = None
        self._closest_point_idx = 0  # Index of closest point on path
        self._is_active = False

    def set_path(self, path: Path):
        """Set a new path to follow."""
        self._path = path
        self._closest_point_idx = 0
        self._is_active = len(path) > 0

    def clear_path(self):
        """Clear the current path."""
        self._path = None
        self._closest_point_idx = 0
        self._is_active = False

    def start(self):
        """Start path following."""
        if self._path and len(self._path) > 0:
            self._is_active = True

    def stop(self):
        """Stop path following."""
        self._is_active = False

    def is_active(self) -> bool:
        """Check if controller is active."""
        return self._is_active

    def is_goal_reached(self) -> bool:
        """Check if goal is reached."""
        return not self._is_active

    def get_current_waypoint_index(self) -> int:
        """Get current closest point index for visualization."""
        return self._closest_point_idx

    def compute_velocity(self, x: float, y: float, yaw: float,
                        current_speed: float = 0.0) -> Tuple[Twist, bool]:
        """
        Compute velocity command to follow the path.

        Args:
            x: Robot x position in map frame
            y: Robot y position in map frame
            yaw: Robot heading in map frame (radians)
            current_speed: Current linear speed (m/s)

        Returns:
            Tuple of (Twist command, goal_reached boolean)
        """
        cmd = Twist()

        if not self._is_active or self._path is None or self._path.is_empty():
            return cmd, True

        # Step 1: Find closest point on path (starting from current index)
        closest_idx = self._find_closest_point(x, y)
        self._closest_point_idx = closest_idx

        # Step 2: Check if we've reached the final goal
        final_point = self._path[-1]
        dist_to_goal = math.sqrt((final_point.x - x)**2 + (final_point.y - y)**2)

        if dist_to_goal < self.config.goal_tolerance:
            self._is_active = False
            return cmd, True  # Goal reached, stop

        # Step 3: Compute adaptive lookahead distance
        lookahead = self.config.lookahead_distance + \
                    self.config.lookahead_gain * abs(current_speed)
        lookahead = max(self.config.min_lookahead,
                       min(self.config.max_lookahead, lookahead))

        # Step 4: Find lookahead point on path
        lookahead_point = self._find_lookahead_point(x, y, closest_idx, lookahead)

        if lookahead_point is None:
            # No valid lookahead point, target the goal
            lookahead_point = (final_point.x, final_point.y)

        lx, ly = lookahead_point

        # Step 5: Compute angle to lookahead point
        dx = lx - x
        dy = ly - y
        dist_to_lookahead = math.sqrt(dx*dx + dy*dy)

        if dist_to_lookahead < 0.1:
            # Already at lookahead point
            return cmd, False

        # Angle to lookahead point in world frame
        target_angle = math.atan2(dy, dx)

        # Angle error (alpha) in vehicle frame
        alpha = self._normalize_angle(target_angle - yaw)

        # Step 6: Pure Pursuit steering computation
        # Curvature: k = 2 * sin(alpha) / L_d
        # Steering angle: delta = atan(k * wheelbase)
        curvature = 2.0 * math.sin(alpha) / dist_to_lookahead
        steering_angle = math.atan(self.config.wheelbase * curvature)

        # Step 7: Compute linear velocity
        # Slow down for sharp turns and when approaching goal
        turn_factor = 1.0 - min(abs(steering_angle) / 0.6, 0.7)

        # Slow down when approaching goal
        approach_factor = min(dist_to_goal / 3.0, 1.0)

        # Get target speed from path or use max
        target_speed = self.config.max_linear_speed
        if closest_idx < len(self._path) and self._path[closest_idx].speed is not None:
            target_speed = self._path[closest_idx].speed

        linear_vel = target_speed * turn_factor * approach_factor
        linear_vel = max(self.config.min_linear_speed, linear_vel)

        # Step 8: Compute angular velocity using Ackermann kinematics
        # omega = v * tan(delta) / L
        angular_vel = linear_vel * math.tan(steering_angle) / self.config.wheelbase
        angular_vel = max(-self.config.max_angular_speed,
                         min(self.config.max_angular_speed, angular_vel))

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        return cmd, False

    def _find_closest_point(self, x: float, y: float) -> int:
        """
        Find the index of the closest point on the path.

        Only searches forward from current index to prevent backtracking.
        """
        if self._path is None or self._path.is_empty():
            return 0

        min_dist = float('inf')
        closest_idx = self._closest_point_idx

        # Search from current position forward (don't go backwards)
        # Also check a few points behind in case we overshot
        start_idx = max(0, self._closest_point_idx - 2)

        for i in range(start_idx, len(self._path)):
            point = self._path[i]
            dist = math.sqrt((point.x - x)**2 + (point.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def _find_lookahead_point(self, x: float, y: float, start_idx: int,
                              lookahead_dist: float) -> Optional[Tuple[float, float]]:
        """
        Find a point on the path at approximately lookahead_dist ahead.

        Interpolates between path points to find exact lookahead point.
        """
        if self._path is None or self._path.is_empty():
            return None

        # Accumulate distance along path from start_idx
        accumulated_dist = 0.0

        for i in range(start_idx, len(self._path) - 1):
            p1 = self._path[i]
            p2 = self._path[i + 1]

            segment_length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

            if accumulated_dist + segment_length >= lookahead_dist:
                # Lookahead point is on this segment
                # Interpolate to find exact point
                remaining = lookahead_dist - accumulated_dist
                if segment_length > 0.001:
                    ratio = remaining / segment_length
                else:
                    ratio = 0.0

                lx = p1.x + ratio * (p2.x - p1.x)
                ly = p1.y + ratio * (p2.y - p1.y)
                return (lx, ly)

            accumulated_dist += segment_length

        # Lookahead extends beyond path, return final point
        if len(self._path) > 0:
            final = self._path[-1]
            return (final.x, final.y)

        return None

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_lookahead_point(self, x: float, y: float, yaw: float) -> Optional[Tuple[float, float]]:
        """Get the current lookahead point for visualization."""
        if not self._is_active or self._path is None:
            return None

        lookahead = self.config.lookahead_distance
        return self._find_lookahead_point(x, y, self._closest_point_idx, lookahead)
