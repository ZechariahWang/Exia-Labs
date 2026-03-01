import math
from dataclasses import dataclass
from typing import Optional, Tuple, List

from geometry_msgs.msg import Twist


@dataclass
class PurePursuitConfig:
    lookahead_distance: float = 2.0
    min_lookahead: float = 1.0
    max_lookahead: float = 5.0
    lookahead_gain: float = 0.3

    goal_tolerance: float = 1.5

    max_linear_speed: float = 2.0
    min_linear_speed: float = 0.3
    max_angular_speed: float = 1.0
    reverse_speed: float = 0.5

    wheelbase: float = 1.3

    reverse_angle_threshold: float = 2.0


@dataclass
class PathPoint:
    x: float
    y: float
    speed: Optional[float] = None


class Path:

    def __init__(self, points: Optional[List[PathPoint]] = None):
        self.points: List[PathPoint] = points or []
        self._current_index = 0

    def add_point(self, x: float, y: float, speed: Optional[float] = None):
        self.points.append(PathPoint(x, y, speed))

    def clear(self):
        self.points = []
        self._current_index = 0

    def reset(self):
        self._current_index = 0

    def is_empty(self) -> bool:
        return len(self.points) == 0

    def __len__(self) -> int:
        return len(self.points)

    def __getitem__(self, index: int) -> PathPoint:
        return self.points[index]


class PurePursuitController:

    def __init__(self, config: Optional[PurePursuitConfig] = None):
        self.config = config or PurePursuitConfig()
        self._path: Optional[Path] = None
        self._closest_point_idx = 0
        self._is_active = False

    def set_path(self, path: Path):
        self._path = path
        self._closest_point_idx = 0
        self._is_active = len(path) > 0

    def clear_path(self):
        self._path = None
        self._closest_point_idx = 0
        self._is_active = False

    def start(self):
        if self._path and len(self._path) > 0:
            self._is_active = True

    def stop(self):
        self._is_active = False

    def is_active(self) -> bool:
        return self._is_active

    def is_goal_reached(self) -> bool:
        return not self._is_active

    def get_current_waypoint_index(self) -> int:
        return self._closest_point_idx

    def compute_velocity(self, x: float, y: float, yaw: float,
                        current_speed: float = 0.0) -> Tuple[Twist, bool]:
        cmd = Twist()

        if not self._is_active or self._path is None or self._path.is_empty():
            return cmd, True

        closest_idx = self._find_closest_point(x, y)
        self._closest_point_idx = closest_idx

        final_point = self._path[-1]
        dist_to_goal = math.sqrt((final_point.x - x)**2 + (final_point.y - y)**2)

        if dist_to_goal < self.config.goal_tolerance:
            self._is_active = False
            return cmd, True

        lookahead = self.config.lookahead_distance + \
                    self.config.lookahead_gain * abs(current_speed)
        lookahead = max(self.config.min_lookahead,
                       min(self.config.max_lookahead, lookahead))

        lookahead_point = self._find_lookahead_point(x, y, closest_idx, lookahead)

        if lookahead_point is None:
            lookahead_point = (final_point.x, final_point.y)

        lx, ly = lookahead_point

        dx = lx - x
        dy = ly - y
        dist_to_lookahead = math.sqrt(dx*dx + dy*dy)

        if dist_to_lookahead < 0.1:
            return cmd, False

        target_angle = math.atan2(dy, dx)

        alpha = self._normalize_angle(target_angle - yaw)

        should_reverse = abs(alpha) > self.config.reverse_angle_threshold

        if should_reverse:
            alpha_reverse = self._normalize_angle(alpha + math.pi if alpha > 0 else alpha - math.pi)

            curvature = 2.0 * math.sin(alpha_reverse) / dist_to_lookahead
            steering_angle = math.atan(self.config.wheelbase * curvature)
            steering_angle = max(-0.6, min(0.6, steering_angle))

            approach_factor = min(dist_to_goal / 2.0, 1.0)
            linear_vel = -self.config.reverse_speed * approach_factor
            linear_vel = max(-self.config.reverse_speed, min(-self.config.min_linear_speed, linear_vel))

            angular_vel = linear_vel * math.tan(steering_angle) / self.config.wheelbase
        else:
            curvature = 2.0 * math.sin(alpha) / dist_to_lookahead
            steering_angle = math.atan(self.config.wheelbase * curvature)
            steering_angle = max(-0.6, min(0.6, steering_angle))

            turn_factor = 1.0 - min(abs(steering_angle) / 0.6, 0.7)

            approach_factor = min(dist_to_goal / 3.0, 1.0)

            target_speed = self.config.max_linear_speed
            if closest_idx < len(self._path) and self._path[closest_idx].speed is not None:
                path_speed = self._path[closest_idx].speed
                if path_speed is not None and path_speed > 0:
                    target_speed = path_speed

            linear_vel = target_speed * turn_factor * approach_factor
            linear_vel = max(self.config.min_linear_speed, linear_vel)

            angular_vel = linear_vel * math.tan(steering_angle) / self.config.wheelbase

        angular_vel = max(-self.config.max_angular_speed,
                         min(self.config.max_angular_speed, angular_vel))

        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        return cmd, False

    def _find_closest_point(self, x: float, y: float) -> int:
        if self._path is None or self._path.is_empty():
            return 0

        min_dist = float('inf')
        closest_idx = 0

        for i in range(len(self._path)):
            point = self._path[i]
            dist = math.sqrt((point.x - x)**2 + (point.y - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def _find_lookahead_point(self, x: float, y: float, start_idx: int,
                              lookahead_dist: float) -> Optional[Tuple[float, float]]:
        if self._path is None or self._path.is_empty():
            return None

        accumulated_dist = 0.0

        for i in range(start_idx, len(self._path) - 1):
            p1 = self._path[i]
            p2 = self._path[i + 1]

            segment_length = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

            if accumulated_dist + segment_length >= lookahead_dist:
                remaining = lookahead_dist - accumulated_dist
                if segment_length > 0.001:
                    ratio = remaining / segment_length
                else:
                    ratio = 0.0

                lx = p1.x + ratio * (p2.x - p1.x)
                ly = p1.y + ratio * (p2.y - p1.y)
                return (lx, ly)

            accumulated_dist += segment_length

        if len(self._path) > 0:
            final = self._path[-1]
            return (final.x, final.y)

        return None

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_lookahead_point(self, x: float, y: float, yaw: float) -> Optional[Tuple[float, float]]:
        if not self._is_active or self._path is None:
            return None

        lookahead = self.config.lookahead_distance
        return self._find_lookahead_point(x, y, self._closest_point_idx, lookahead)
