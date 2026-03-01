import math
from typing import Optional
from dataclasses import dataclass

import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion


@dataclass
class PathSmootherConfig:
    control_point_spacing: float = 2.5
    output_spacing: float = 0.5
    direction_change_threshold: float = 0.52
    min_control_points: int = 4
    average_window: int = 5
    enabled: bool = True


class PathSmoother:

    def __init__(self, config: Optional[PathSmootherConfig] = None):
        self._config = config or PathSmootherConfig()

    def smooth(self, nav_path: Path) -> Path:
        if not self._config.enabled:
            return nav_path

        poses = nav_path.poses
        if len(poses) < self._config.min_control_points:
            return nav_path

        raw_x = np.array([p.pose.position.x for p in poses])
        raw_y = np.array([p.pose.position.y for p in poses])

        avg_x = self._moving_average(raw_x)
        avg_y = self._moving_average(raw_y)

        avg_x[0] = raw_x[0]
        avg_y[0] = raw_y[0]
        avg_x[-1] = raw_x[-1]
        avg_y[-1] = raw_y[-1]

        indices = self._subsample_indices(avg_x, avg_y)
        if len(indices) < self._config.min_control_points:
            return nav_path

        ctrl_x = avg_x[indices]
        ctrl_y = avg_y[indices]

        ctrl_x[0] = raw_x[0]
        ctrl_y[0] = raw_y[0]
        ctrl_x[-1] = raw_x[-1]
        ctrl_y[-1] = raw_y[-1]

        t = np.zeros(len(indices))
        for i in range(1, len(indices)):
            dx = ctrl_x[i] - ctrl_x[i - 1]
            dy = ctrl_y[i] - ctrl_y[i - 1]
            t[i] = t[i - 1] + math.sqrt(dx * dx + dy * dy)

        total_length = t[-1]
        if total_length < 1.0:
            return nav_path

        ax, bx, cx, dx_coeff = self._fit_cubic_spline(t, ctrl_x)
        ay, by, cy, dy_coeff = self._fit_cubic_spline(t, ctrl_y)

        points = self._resample_uniform(
            t, ax, bx, cx, dx_coeff, ay, by, cy, dy_coeff, total_length)

        if len(points) < 2:
            return nav_path

        result = Path()
        result.header = nav_path.header
        for i, (px, py) in enumerate(points):
            if i < len(points) - 1:
                hdx = points[i + 1][0] - px
                hdy = points[i + 1][1] - py
            else:
                hdx = px - points[i - 1][0]
                hdy = py - points[i - 1][1]
            yaw = math.atan2(hdy, hdx)

            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation = self._yaw_to_quaternion(yaw)
            result.poses.append(pose)

        return result

    def _moving_average(self, values: np.ndarray) -> np.ndarray:
        n = len(values)
        w = self._config.average_window
        result = np.empty(n)
        for i in range(n):
            lo = max(0, i - w)
            hi = min(n, i + w + 1)
            result[i] = np.mean(values[lo:hi])
        return result

    def _subsample_indices(self, avg_x: np.ndarray, avg_y: np.ndarray) -> np.ndarray:
        n = len(avg_x)
        if n <= self._config.min_control_points:
            return np.arange(n)

        indices = [0]
        accumulated = 0.0
        prev_x = avg_x[0]
        prev_y = avg_y[0]

        for i in range(1, n - 1):
            dx = avg_x[i] - prev_x
            dy = avg_y[i] - prev_y
            accumulated += math.sqrt(dx * dx + dy * dy)

            direction_change = False
            if i >= 2 and i < n - 1:
                ax_seg = avg_x[i] - avg_x[max(0, i - 3)]
                ay_seg = avg_y[i] - avg_y[max(0, i - 3)]
                bx_seg = avg_x[min(n - 1, i + 3)] - avg_x[i]
                by_seg = avg_y[min(n - 1, i + 3)] - avg_y[i]
                len_a = math.sqrt(ax_seg * ax_seg + ay_seg * ay_seg)
                len_b = math.sqrt(bx_seg * bx_seg + by_seg * by_seg)
                if len_a > 1e-6 and len_b > 1e-6:
                    dot = (ax_seg * bx_seg + ay_seg * by_seg) / (len_a * len_b)
                    dot = max(-1.0, min(1.0, dot))
                    angle = math.acos(dot)
                    if angle > self._config.direction_change_threshold:
                        direction_change = True

            if accumulated >= self._config.control_point_spacing or direction_change:
                indices.append(i)
                accumulated = 0.0
                prev_x = avg_x[i]
                prev_y = avg_y[i]
            else:
                prev_x = avg_x[i]
                prev_y = avg_y[i]

        if indices[-1] != n - 1:
            indices.append(n - 1)

        return np.array(indices)

    def _fit_cubic_spline(self, t: np.ndarray, values: np.ndarray):
        n = len(t)
        if n < 2:
            return (np.array([values[0]]), np.array([0.0]),
                    np.array([0.0]), np.array([0.0]))

        h = np.diff(t)
        h = np.maximum(h, 1e-10)

        m = np.zeros(n)

        if n > 2:
            diag_main = np.zeros(n - 2)
            diag_upper = np.zeros(n - 3)
            diag_lower = np.zeros(n - 3)
            rhs = np.zeros(n - 2)

            for i in range(n - 2):
                diag_main[i] = 2.0 * (h[i] + h[i + 1])
                rhs[i] = 6.0 * ((values[i + 2] - values[i + 1]) / h[i + 1] -
                                 (values[i + 1] - values[i]) / h[i])
            for i in range(n - 3):
                diag_upper[i] = h[i + 1]
                diag_lower[i] = h[i + 1]

            for i in range(1, n - 2):
                if abs(diag_main[i - 1]) < 1e-12:
                    break
                factor = diag_lower[i - 1] / diag_main[i - 1]
                diag_main[i] -= factor * diag_upper[i - 1]
                rhs[i] -= factor * rhs[i - 1]

            interior_m = np.zeros(n - 2)
            if abs(diag_main[-1]) > 1e-12:
                interior_m[-1] = rhs[-1] / diag_main[-1]
            for i in range(n - 4, -1, -1):
                if abs(diag_main[i]) > 1e-12:
                    interior_m[i] = (rhs[i] - diag_upper[i] * interior_m[i + 1]) / diag_main[i]

            m[1:-1] = interior_m

        a = values[:-1].copy()
        b = np.zeros(n - 1)
        c = np.zeros(n - 1)
        d = np.zeros(n - 1)

        for i in range(n - 1):
            b[i] = ((values[i + 1] - values[i]) / h[i] -
                     h[i] * (2.0 * m[i] + m[i + 1]) / 6.0)
            c[i] = m[i] / 2.0
            d[i] = (m[i + 1] - m[i]) / (6.0 * h[i])

        return a, b, c, d

    def _evaluate_spline(self, t_eval: float, t: np.ndarray,
                         a: np.ndarray, b: np.ndarray,
                         c: np.ndarray, d: np.ndarray) -> float:
        idx = np.searchsorted(t[1:], t_eval, side='right')
        idx = min(idx, len(a) - 1)
        dt = t_eval - t[idx]
        return float(a[idx] + b[idx] * dt + c[idx] * dt * dt + d[idx] * dt * dt * dt)

    def _resample_uniform(self, t, ax, bx, cx, dx_coeff,
                          ay, by, cy, dy_coeff, total_length):
        num_points = max(int(total_length / self._config.output_spacing), 2)
        points = []

        for i in range(num_points + 1):
            t_eval = (i / num_points) * total_length
            t_eval = min(t_eval, total_length)
            x = self._evaluate_spline(t_eval, t, ax, bx, cx, dx_coeff)
            y = self._evaluate_spline(t_eval, t, ay, by, cy, dy_coeff)
            points.append((x, y))

        return points

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q
