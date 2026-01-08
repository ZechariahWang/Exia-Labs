import math
import struct
import threading
import time
from typing import Optional, Tuple

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.duration import Duration


class PointCloudCostmapGenerator:
    def __init__(
        self,
        node: Node,
        map_size: float = 80.0,
        resolution: float = 0.1,
        min_obstacle_height: float = 0.2,
        max_obstacle_height: float = 1.5,
        ground_height: float = 0.0,
        inflation_radius: float = 1.3,
        min_range: float = 0.5,
        max_range: float = 20.0,
        decay_time: float = 2.0,
    ):
        self._node = node
        self._logger = node.get_logger()

        self.map_size = map_size
        self.resolution = resolution
        self.min_obstacle_height = min_obstacle_height
        self.max_obstacle_height = max_obstacle_height
        self.ground_height = ground_height
        self.inflation_radius = inflation_radius
        self.min_range = min_range
        self.max_range = max_range
        self.decay_time = decay_time

        self.grid_cells = int(map_size / resolution)
        self._grid_lock = threading.Lock()

        self._obstacle_grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.float32)
        self._timestamp_grid = np.zeros((self.grid_cells, self.grid_cells), dtype=np.float64)
        self._inflated_grid: Optional[np.ndarray] = None

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._last_update_time = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        pointcloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pointcloud_sub = node.create_subscription(
            PointCloud2,
            '/points',
            self._pointcloud_callback,
            pointcloud_qos
        )

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self._costmap_pub = node.create_publisher(
            OccupancyGrid,
            '/ackermann_costmap',
            costmap_qos
        )

        self._publish_timer = node.create_timer(0.1, self._publish_costmap)
        self._initialized = False

    def _pointcloud_callback(self, msg: PointCloud2):
        try:
            transform = self._tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            if not self._initialized:
                return
            self._logger.warn(f'TF lookup failed: {e}')
            return

        self._initialized = True

        points = self._pointcloud2_to_numpy(msg)
        if points is None or points.shape[0] == 0:
            return

        points_map = self._transform_points(points, transform)

        obstacle_mask = (
            (points_map[:, 2] > self.ground_height + self.min_obstacle_height) &
            (points_map[:, 2] < self.ground_height + self.max_obstacle_height)
        )

        xy_dist = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        range_mask = (xy_dist > self.min_range) & (xy_dist < self.max_range)

        obstacle_points = points_map[obstacle_mask & range_mask]

        try:
            robot_transform = self._tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                msg.header.stamp,
                timeout=Duration(seconds=0.05)
            )
            self._robot_x = robot_transform.transform.translation.x
            self._robot_y = robot_transform.transform.translation.y
            q = robot_transform.transform.rotation
            self._robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except TransformException:
            pass

        current_time = time.time()

        with self._grid_lock:
            if current_time - self._last_update_time > 0.5:
                age = current_time - self._timestamp_grid
                decay_mask = age > self.decay_time
                self._obstacle_grid[decay_mask] = 0
                self._timestamp_grid[decay_mask] = 0

            self._last_update_time = current_time

            if obstacle_points.shape[0] > 0:
                origin_x = self._robot_x - self.map_size / 2.0
                origin_y = self._robot_y - self.map_size / 2.0

                grid_x = ((obstacle_points[:, 0] - origin_x) / self.resolution).astype(int)
                grid_y = ((obstacle_points[:, 1] - origin_y) / self.resolution).astype(int)

                valid = (
                    (grid_x >= 0) & (grid_x < self.grid_cells) &
                    (grid_y >= 0) & (grid_y < self.grid_cells)
                )

                self._obstacle_grid[grid_y[valid], grid_x[valid]] = 100
                self._timestamp_grid[grid_y[valid], grid_x[valid]] = current_time

            self._inflated_grid = self._inflate_obstacles(self._obstacle_grid)

    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        if msg.width * msg.height == 0:
            return None

        point_step = msg.point_step
        x_offset = y_offset = z_offset = -1

        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if x_offset < 0 or y_offset < 0 or z_offset < 0:
            return None

        num_points = msg.width * msg.height
        points = np.zeros((num_points, 3), dtype=np.float32)

        data = np.frombuffer(msg.data, dtype=np.uint8)

        for i in range(num_points):
            base = i * point_step
            points[i, 0] = struct.unpack_from('f', data, base + x_offset)[0]
            points[i, 1] = struct.unpack_from('f', data, base + y_offset)[0]
            points[i, 2] = struct.unpack_from('f', data, base + z_offset)[0]

        valid = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
        return points[valid]

    def _transform_points(self, points: np.ndarray, transform: TransformStamped) -> np.ndarray:
        t = transform.transform.translation
        q = transform.transform.rotation

        qw, qx, qy, qz = q.w, q.x, q.y, q.z

        rot = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])

        points_rotated = points @ rot.T
        points_rotated[:, 0] += t.x
        points_rotated[:, 1] += t.y
        points_rotated[:, 2] += t.z

        return points_rotated

    def _inflate_obstacles(self, grid: np.ndarray) -> np.ndarray:
        inflation_cells = int(self.inflation_radius / self.resolution)
        if inflation_cells <= 0:
            return grid.copy()

        inflated = grid.copy()
        obstacle_y, obstacle_x = np.where(grid >= 100)

        if len(obstacle_y) == 0:
            return inflated

        y_coords, x_coords = np.meshgrid(
            np.arange(-inflation_cells, inflation_cells + 1),
            np.arange(-inflation_cells, inflation_cells + 1),
            indexing='ij'
        )

        dist_sq = x_coords**2 + y_coords**2
        disk_mask = dist_sq <= inflation_cells**2

        for oy, ox in zip(obstacle_y, obstacle_x):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dy*dy + dx*dx <= inflation_cells*inflation_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < self.grid_cells and 0 <= nx < self.grid_cells:
                            if inflated[ny, nx] < 100:
                                dist = math.sqrt(dy*dy + dx*dx) * self.resolution
                                cost = int(50 * (1.0 - dist / self.inflation_radius))
                                inflated[ny, nx] = max(inflated[ny, nx], cost)

        return inflated

    def _publish_costmap(self):
        with self._grid_lock:
            if self._inflated_grid is None:
                return

            grid_to_publish = self._inflated_grid.copy()

        msg = OccupancyGrid()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_cells
        msg.info.height = self.grid_cells
        msg.info.origin.position.x = self._robot_x - self.map_size / 2.0
        msg.info.origin.position.y = self._robot_y - self.map_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid_to_publish.astype(np.int8).flatten().tolist()

        self._costmap_pub.publish(msg)

    def get_costmap(self) -> Optional[OccupancyGrid]:
        with self._grid_lock:
            if self._inflated_grid is None:
                return None

            grid = self._inflated_grid.copy()

        msg = OccupancyGrid()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_cells
        msg.info.height = self.grid_cells
        msg.info.origin.position.x = self._robot_x - self.map_size / 2.0
        msg.info.origin.position.y = self._robot_y - self.map_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.astype(np.int8).flatten().tolist()

        return msg

    def get_cost_at(self, x: float, y: float) -> int:
        with self._grid_lock:
            if self._inflated_grid is None:
                return 0

            origin_x = self._robot_x - self.map_size / 2.0
            origin_y = self._robot_y - self.map_size / 2.0

            grid_x = int((x - origin_x) / self.resolution)
            grid_y = int((y - origin_y) / self.resolution)

            if 0 <= grid_x < self.grid_cells and 0 <= grid_y < self.grid_cells:
                return int(self._inflated_grid[grid_y, grid_x])

            return 255

    def is_ready(self) -> bool:
        return self._initialized and self._inflated_grid is not None
