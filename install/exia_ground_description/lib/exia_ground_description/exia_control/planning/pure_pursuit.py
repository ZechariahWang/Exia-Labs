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
    lookahead_gain: float = 0.5          # Speed-dependent lookahead gain

    # Goal parameters
    goal_tolerance: float = 0.5          # Distance to consider goal reached (m)

    # Speed control
    max_linear_speed: float = 2.0        # Maximum speed (m/s)
    min_linear_speed: float = 0.3        # Minimum speed when moving (m/s) - ensures throttle > deadband
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

    def __init__(self, config: Optional[PurePursuitConfig] = None):
        self.config = config or PurePursuitConfig()
        self._path: Optional[Path] = None
        self._current_waypoint_idx = 0
        self._is_active = False

    def set_path(self, path: Path):
        self._path = path
        self._current_waypoint_idx = 0
        self._is_active = len(path) > 0

    def clear_path(self):
        self._path = None
        self._current_waypoint_idx = 0
        self._is_active = False

    def start(self):
        if self._path and len(self._path) > 0:
            self._is_active = True
            self._current_waypoint_idx = 0

    def stop(self):
        self._is_active = False

    def is_active(self) -> bool:
        return self._is_active

    def is_goal_reached(self) -> bool:
        if self._path is None or self._path.is_empty():
            return True
        return self._current_waypoint_idx >= len(self._path)

    def get_current_waypoint_index(self) -> int:
        return self._current_waypoint_idx

    def compute_velocity(self, x: float, y: float, yaw: float, current_speed: float = 0.0) -> Tuple[Twist, bool]:
        
        cmd = Twist()
        if not self._is_active or self._path is None or self._path.is_empty():
            return cmd, True

        if self._current_waypoint_idx >= len(self._path):
            self._is_active = False
            return cmd, True

        # Get target waypoint
        target = self._path[self._current_waypoint_idx]

        # Compute distance to target
        dx = target.x - x
        dy = target.y - y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if waypoint reached
        if distance < self.config.goal_tolerance:
            self._current_waypoint_idx += 1
            if self._current_waypoint_idx >= len(self._path):
                self._is_active = False
                return cmd, True
            # Get next target
            target = self._path[self._current_waypoint_idx]
            dx = target.x - x
            dy = target.y - y
            distance = math.sqrt(dx*dx + dy*dy)

        # Compute lookahead distance (speed-dependent)
        lookahead = self.config.lookahead_distance + \
                    self.config.lookahead_gain * abs(current_speed)
        lookahead = max(self.config.min_lookahead,
                       min(self.config.max_lookahead, lookahead))
        lookahead = min(lookahead, distance)  # Don't look past target

        # Compute angle to target in world frame
        target_angle = math.atan2(dy, dx)

        # Compute angle error (alpha) in vehicle frame
        alpha = target_angle - yaw
        alpha = self._normalize_angle(alpha)

        # Pure Pursuit steering computation
        # Curvature: k = 2 * sin(alpha) / L_d
        # Steering angle: delta = atan(k * wheelbase)
        if lookahead > 0.1:
            curvature = 2.0 * math.sin(alpha) / lookahead
            steering_angle = math.atan(self.config.wheelbase * curvature)
        else:
            steering_angle = 0.0

        # Compute linear velocity
        # Slow down for sharp turns and when approaching goal
        turn_factor = 1.0 - min(abs(steering_angle) / 0.6, 0.7)
        approach_factor = min(distance / self.config.lookahead_distance, 1.0)

        # Use waypoint speed if specified
        if target.speed is not None:
            target_speed = target.speed
        else:
            target_speed = self.config.max_linear_speed

        linear_vel = target_speed * turn_factor * approach_factor
        linear_vel = max(self.config.min_linear_speed, linear_vel)

        # Compute angular velocity using Ackermann kinematics
        # omega = v * tan(delta) / L
        if abs(linear_vel) > 0.01:
            angular_vel = linear_vel * math.tan(steering_angle) / self.config.wheelbase
            angular_vel = max(-self.config.max_angular_speed,
                            min(self.config.max_angular_speed, angular_vel))
        else:
            angular_vel = 0.0

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        return cmd, False

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_lookahead_point(self, x: float, y: float, yaw: float) -> Optional[Tuple[float, float]]:
        """
        Get the current lookahead point for visualization.

        Returns:
            Tuple of (lookahead_x, lookahead_y) or None
        """
        if not self._is_active or self._path is None:
            return None

        if self._current_waypoint_idx >= len(self._path):
            return None

        target = self._path[self._current_waypoint_idx]
        return (target.x, target.y)
