import numpy as np
from typing import Tuple, Dict, Optional


class PointCloudProcessor:
    def __init__(
        self,
        ground_height: float = 0.0,
        ground_tolerance: float = 0.15,
        min_obstacle_height: float = 0.2,
        max_obstacle_height: float = 1.5,
        negative_obstacle_threshold: float = 0.3,
        grid_resolution: float = 0.1,
        grid_size: float = 20.0,
        min_range: float = 0.5,
        max_range: float = 15.0,
    ):
        self.ground_height = ground_height
        self.ground_tolerance = ground_tolerance
        self.min_obstacle_height = min_obstacle_height
        self.max_obstacle_height = max_obstacle_height
        self.negative_obstacle_threshold = negative_obstacle_threshold
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size
        self.min_range = min_range
        self.max_range = max_range

        self.grid_cells = int(grid_size / grid_resolution)
        self.grid_origin = -grid_size / 2.0

    def process(self, points: np.ndarray) -> Dict:
        if points.shape[0] == 0:
            return self._empty_result()

        obstacle_points = self._detect_positive_obstacles(points)
        negative_obstacle_cells = self._detect_negative_obstacles(points)
        obstacle_grid = self._create_obstacle_grid(obstacle_points, negative_obstacle_cells)

        return {
            'obstacle_points': obstacle_points,
            'negative_obstacle_cells': negative_obstacle_cells,
            'obstacle_grid': obstacle_grid,
        }

    def _detect_positive_obstacles(self, points: np.ndarray) -> np.ndarray:
        z = points[:, 2]
        min_z = self.ground_height + self.min_obstacle_height
        max_z = self.ground_height + self.max_obstacle_height

        obstacle_mask = (z > min_z) & (z < max_z)

        xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        range_mask = (xy_dist > self.min_range) & (xy_dist < self.max_range)

        return points[obstacle_mask & range_mask]

    def _detect_negative_obstacles(self, points: np.ndarray) -> np.ndarray:
        num_angular_bins = 360
        num_distance_bins = int((self.max_range - self.min_range) / 0.5)
        distance_bin_size = 0.5

        negative_cells = []

        xy = points[:, :2]
        z = points[:, 2]

        angles = np.arctan2(xy[:, 1], xy[:, 0])
        distances = np.sqrt(xy[:, 0]**2 + xy[:, 1]**2)

        angle_bins = ((angles + np.pi) / (2 * np.pi) * num_angular_bins).astype(int) % num_angular_bins
        distance_bins = ((distances - self.min_range) / distance_bin_size).astype(int)

        ground_min = self.ground_height - self.ground_tolerance
        ground_max = self.ground_height + self.ground_tolerance
        ground_mask = (z >= ground_min) & (z <= ground_max)

        for angle_idx in range(num_angular_bins):
            angle_mask = angle_bins == angle_idx

            last_ground_dist = self.min_range

            for dist_idx in range(num_distance_bins):
                dist_mask = distance_bins == dist_idx
                bin_mask = angle_mask & dist_mask

                if not np.any(bin_mask):
                    continue

                bin_points_z = z[bin_mask]
                has_ground = np.any((bin_points_z >= ground_min) & (bin_points_z <= ground_max))
                has_below_ground = np.any(bin_points_z < (self.ground_height - self.negative_obstacle_threshold))

                current_dist = self.min_range + (dist_idx + 0.5) * distance_bin_size

                if has_ground:
                    last_ground_dist = current_dist
                elif has_below_ground or (not has_ground and current_dist - last_ground_dist > distance_bin_size * 2):
                    angle_rad = (angle_idx / num_angular_bins) * 2 * np.pi - np.pi
                    x = current_dist * np.cos(angle_rad)
                    y = current_dist * np.sin(angle_rad)
                    negative_cells.append([x, y])

        if len(negative_cells) == 0:
            return np.array([]).reshape(0, 2)

        return np.array(negative_cells)

    def _create_obstacle_grid(
        self,
        obstacle_points: np.ndarray,
        negative_cells: np.ndarray
    ) -> np.ndarray:
        grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.int8)

        if obstacle_points.shape[0] > 0:
            grid_x = ((obstacle_points[:, 0] - self.grid_origin) / self.grid_resolution).astype(int)
            grid_y = ((obstacle_points[:, 1] - self.grid_origin) / self.grid_resolution).astype(int)

            valid = (grid_x >= 0) & (grid_x < self.grid_cells) & \
                    (grid_y >= 0) & (grid_y < self.grid_cells)

            grid[grid_y[valid], grid_x[valid]] = 100

        if negative_cells.shape[0] > 0:
            grid_x = ((negative_cells[:, 0] - self.grid_origin) / self.grid_resolution).astype(int)
            grid_y = ((negative_cells[:, 1] - self.grid_origin) / self.grid_resolution).astype(int)

            valid = (grid_x >= 0) & (grid_x < self.grid_cells) & \
                    (grid_y >= 0) & (grid_y < self.grid_cells)

            grid[grid_y[valid], grid_x[valid]] = 100

        return grid

    def _empty_result(self) -> Dict:
        return {
            'obstacle_points': np.array([]).reshape(0, 3),
            'negative_obstacle_cells': np.array([]).reshape(0, 2),
            'obstacle_grid': np.zeros((self.grid_cells, self.grid_cells), dtype=np.int8),
        }

    def inflate_obstacles(self, grid: np.ndarray, inflation_radius: float) -> np.ndarray:
        inflation_cells = int(inflation_radius / self.grid_resolution)
        if inflation_cells <= 0:
            return grid.copy()

        inflated = grid.copy()
        obstacle_y, obstacle_x = np.where(grid == 100)

        for oy, ox in zip(obstacle_y, obstacle_x):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dy*dy + dx*dx <= inflation_cells*inflation_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < self.grid_cells and 0 <= nx < self.grid_cells:
                            if inflated[ny, nx] == 0:
                                inflated[ny, nx] = 50

        return inflated
