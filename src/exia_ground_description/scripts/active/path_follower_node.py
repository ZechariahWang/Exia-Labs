#!/usr/bin/env python3
# Path Follower Node - Pure Pursuit demo for predefined paths

import math
import os
import sys

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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger

from exia_control.planning import (
    PurePursuitController,
    PurePursuitConfig,
    Path,
    create_line_path,
    create_circle_path,
    create_figure_eight_path,
    create_square_path,
    create_slalom_path,
)


class PathFollowerNode(Node):

    def __init__(self):
        super().__init__('path_follower_node')

        # Parameters
        self.declare_parameter('path_type', 'figure_eight')
        self.declare_parameter('path_scale', 2.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('lookahead_distance', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('auto_start', True)

        path_type = self.get_parameter('path_type').value
        scale = self.get_parameter('path_scale').value
        speed = self.get_parameter('speed').value
        lookahead = self.get_parameter('lookahead_distance').value
        tolerance = self.get_parameter('goal_tolerance').value
        auto_start = self.get_parameter('auto_start').value

        # Pure Pursuit controller
        pp_config = PurePursuitConfig(
            lookahead_distance=lookahead,
            goal_tolerance=tolerance,
            max_linear_speed=speed,
            wheelbase=1.3,
        )
        self._controller = PurePursuitController(pp_config)

        # Create path
        self._path = self._create_path(path_type, scale, speed)
        self._controller.set_path(self._path)

        # State
        self._current_pose = None
        self._current_speed = 0.0
        self._is_running = False

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self._path_pub = self.create_publisher(NavPath, '/planned_path', qos)
        self._marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_callback, qos)

        # Services
        self.create_service(Trigger, '/path_follower/start', self._start_callback)
        self.create_service(Trigger, '/path_follower/stop', self._stop_callback)
        self.create_service(Trigger, '/path_follower/reset', self._reset_callback)

        # Timers
        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.5, self._publish_visualization)

        self.get_logger().info(f'Path Follower started - Type: {path_type}, Waypoints: {len(self._path)}')

        if auto_start:
            self.get_logger().info('Auto-starting path following...')
            self._is_running = True
            self._controller.start()

    def _create_path(self, path_type: str, scale: float, speed: float) -> Path:
        if path_type == 'line':
            return create_line_path(length=scale * 3, speed=speed)
        elif path_type == 'circle':
            return create_circle_path(radius=scale, speed=speed)
        elif path_type == 'figure_eight':
            return create_figure_eight_path(size=scale * 2, speed=speed)
        elif path_type == 'square':
            return create_square_path(size=scale * 2, speed=speed)
        elif path_type == 'slalom':
            return create_slalom_path(length=scale * 4, speed=speed)
        else:
            self.get_logger().warn(f'Unknown path type: {path_type}, using figure_eight')
            return create_figure_eight_path(size=scale * 2, speed=speed)

    def _odom_callback(self, msg: Odometry):
        self._current_pose = msg.pose.pose
        self._current_speed = msg.twist.twist.linear.x

    def _control_loop(self):
        if not self._is_running or self._current_pose is None:
            return

        x = self._current_pose.position.x
        y = self._current_pose.position.y
        yaw = self._quaternion_to_yaw(self._current_pose.orientation)

        cmd, goal_reached = self._controller.compute_velocity(x, y, yaw, self._current_speed)

        if goal_reached:
            self.get_logger().info('Path complete!')
            self._is_running = False
            self._cmd_pub.publish(Twist())
            return

        self._cmd_pub.publish(cmd)

    def _publish_visualization(self):
        if self._path is None or self._path.is_empty():
            return

        # Publish nav path
        nav_path = NavPath()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = 'odom'

        for point in self._path.points:
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)

        self._path_pub.publish(nav_path)

        # Publish waypoint markers
        marker_array = MarkerArray()
        current_idx = self._controller.get_current_waypoint_index()

        for i, point in enumerate(self._path.points):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            if i < current_idx:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.8, 0.0
            elif i == current_idx:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.2, 0.2, 0.8
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def _quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _start_callback(self, request, response):
        if self._path.is_empty():
            response.success = False
            response.message = 'No path defined'
            return response

        self._is_running = True
        self._controller.start()
        response.success = True
        response.message = f'Started following {len(self._path)} waypoints'
        return response

    def _stop_callback(self, request, response):
        self._is_running = False
        self._controller.stop()
        self._cmd_pub.publish(Twist())
        response.success = True
        response.message = 'Stopped'
        return response

    def _reset_callback(self, request, response):
        self._is_running = False
        self._controller.stop()
        self._cmd_pub.publish(Twist())
        self._path.reset()
        self._controller.set_path(self._path)
        response.success = True
        response.message = 'Reset to start of path'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
