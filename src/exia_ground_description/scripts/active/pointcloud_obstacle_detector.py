#!/usr/bin/env python3

import os
import sys
import struct
import numpy as np

def _setup_module_path():
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('exia_ground_description')
        lib_path = os.path.join(pkg_prefix, 'lib', 'exia_ground_description')
        if os.path.isdir(lib_path) and lib_path not in sys.path:
            sys.path.insert(0, lib_path)
        return
    except Exception:
        pass
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(script_dir, '..', '..', 'src')
    if os.path.isdir(os.path.join(src_path, 'exia_control')):
        if src_path not in sys.path:
            sys.path.insert(0, src_path)

_setup_module_path()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from exia_control.perception.pointcloud_processor import PointCloudProcessor


class PointCloudObstacleDetector(Node):

    def __init__(self):
        super().__init__('pointcloud_obstacle_detector')

        self.declare_parameter('min_obstacle_height', 0.2)
        self.declare_parameter('max_obstacle_height', 1.5)
        self.declare_parameter('ground_height', 0.0)
        self.declare_parameter('ground_tolerance', 0.15)
        self.declare_parameter('negative_obstacle_threshold', 0.3)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('grid_size', 20.0)
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('publish_rate', 10.0)

        self.processor = PointCloudProcessor(
            ground_height=self.get_parameter('ground_height').value,
            ground_tolerance=self.get_parameter('ground_tolerance').value,
            min_obstacle_height=self.get_parameter('min_obstacle_height').value,
            max_obstacle_height=self.get_parameter('max_obstacle_height').value,
            negative_obstacle_threshold=self.get_parameter('negative_obstacle_threshold').value,
            grid_resolution=self.get_parameter('grid_resolution').value,
            grid_size=self.get_parameter('grid_size').value,
            min_range=self.get_parameter('min_range').value,
            max_range=self.get_parameter('max_range').value,
        )

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_size = self.get_parameter('grid_size').value

        self.latest_result = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/points', self._pointcloud_callback, sensor_qos)

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/obstacle_costmap', costmap_qos)
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/detected_obstacles', 10)

        publish_period = 1.0 / self.get_parameter('publish_rate').value
        self.publish_timer = self.create_timer(publish_period, self._publish_outputs)

        self.get_logger().info('PointCloud Obstacle Detector initialized')
        self.get_logger().info(f'  Grid: {self.grid_size}m x {self.grid_size}m @ {self.grid_resolution}m/cell')
        self.get_logger().info(f'  Obstacle height: {self.get_parameter("min_obstacle_height").value}m - {self.get_parameter("max_obstacle_height").value}m')

    def _pointcloud_callback(self, msg: PointCloud2):
        points = self._pointcloud2_to_numpy(msg)
        if points is None or points.shape[0] == 0:
            return

        self.latest_result = self.processor.process(points)

    def _pointcloud2_to_numpy(self, msg: PointCloud2) -> np.ndarray:
        field_names = [f.name for f in msg.fields]

        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return None

        x_offset = None
        y_offset = None
        z_offset = None

        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset

        if None in (x_offset, y_offset, z_offset):
            return None

        point_step = msg.point_step
        num_points = msg.width * msg.height

        if len(msg.data) < num_points * point_step:
            return None

        points = np.zeros((num_points, 3), dtype=np.float32)

        for i in range(num_points):
            base = i * point_step
            points[i, 0] = struct.unpack_from('f', msg.data, base + x_offset)[0]
            points[i, 1] = struct.unpack_from('f', msg.data, base + y_offset)[0]
            points[i, 2] = struct.unpack_from('f', msg.data, base + z_offset)[0]

        valid_mask = np.isfinite(points).all(axis=1)
        return points[valid_mask]

    def _publish_outputs(self):
        if self.latest_result is None:
            return

        self._publish_costmap()
        self._publish_obstacle_cloud()

    def _publish_costmap(self):
        result = self.latest_result
        if result is None:
            return

        grid = result['obstacle_grid']

        if self.inflation_radius > 0:
            grid = self.processor.inflate_obstacles(grid, self.inflation_radius)

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.info.resolution = self.grid_resolution
        msg.info.width = grid.shape[1]
        msg.info.height = grid.shape[0]
        msg.info.origin.position.x = -self.grid_size / 2.0
        msg.info.origin.position.y = -self.grid_size / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()

        self.costmap_pub.publish(msg)

    def _publish_obstacle_cloud(self):
        result = self.latest_result
        if result is None:
            return

        obstacle_points = result['obstacle_points']
        if obstacle_points.shape[0] == 0:
            return

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.height = 1
        msg.width = obstacle_points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width

        data = obstacle_points.astype(np.float32).tobytes()
        msg.data = list(data)

        self.obstacles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
