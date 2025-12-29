#!/usr/bin/env python3
"""
Simple Waypoint Follower for Ackermann Vehicle

A basic pure pursuit controller that follows a list of waypoints.
This is a good starting point for understanding path following algorithms.

Usage:
    ros2 run exia_ground_description waypoint_follower.py

Author: Zechariah Wang
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger


class WaypointFollower(Node):
    """Pure pursuit waypoint follower for Ackermann vehicles."""

    def __init__(self):
        super().__init__('waypoint_follower')

        # Parameters
        self.declare_parameter('lookahead_distance', 0.8)  # meters
        self.declare_parameter('goal_tolerance', 0.3)  # meters
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        self.declare_parameter('wheelbase', 0.4)  # meters

        self._lookahead = self.get_parameter('lookahead_distance').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._wheelbase = self.get_parameter('wheelbase').value

        # State
        self._current_pose = None
        self._waypoints = []
        self._current_waypoint_idx = 0
        self._is_active = False

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/waypoints_viz', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)

        # Services
        self.create_service(Trigger, '/waypoint/start', self._start_callback)
        self.create_service(Trigger, '/waypoint/stop', self._stop_callback)
        self.create_service(Trigger, '/waypoint/clear', self._clear_callback)

        # Control loop at 20Hz
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Waypoint Follower started')
        self.get_logger().info('  Add waypoints: ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...')
        self.get_logger().info('  Start following: ros2 service call /waypoint/start std_srvs/srv/Trigger')

    def _odom_callback(self, msg: Odometry):
        """Update current pose from odometry."""
        self._current_pose = msg.pose.pose

    def _goal_callback(self, msg: PoseStamped):
        """Add a new waypoint."""
        waypoint = (msg.pose.position.x, msg.pose.position.y)
        self._waypoints.append(waypoint)
        self.get_logger().info(f'Added waypoint {len(self._waypoints)}: ({waypoint[0]:.2f}, {waypoint[1]:.2f})')
        self._publish_waypoint_markers()

    def _start_callback(self, request, response):
        """Start waypoint following."""
        if not self._waypoints:
            response.success = False
            response.message = 'No waypoints defined'
            return response

        self._is_active = True
        self._current_waypoint_idx = 0
        response.success = True
        response.message = f'Started following {len(self._waypoints)} waypoints'
        self.get_logger().info(response.message)
        return response

    def _stop_callback(self, request, response):
        """Stop waypoint following."""
        self._is_active = False
        self._send_stop()
        response.success = True
        response.message = 'Stopped'
        return response

    def _clear_callback(self, request, response):
        """Clear all waypoints."""
        self._waypoints = []
        self._current_waypoint_idx = 0
        self._is_active = False
        self._send_stop()
        self._publish_waypoint_markers()
        response.success = True
        response.message = 'Waypoints cleared'
        return response

    def _control_loop(self):
        """Main control loop - Pure Pursuit algorithm."""
        if not self._is_active or self._current_pose is None:
            return

        if self._current_waypoint_idx >= len(self._waypoints):
            self.get_logger().info('All waypoints reached!')
            self._is_active = False
            self._send_stop()
            return

        # Get current position and heading
        x = self._current_pose.position.x
        y = self._current_pose.position.y
        yaw = self._quaternion_to_yaw(self._current_pose.orientation)

        # Get target waypoint
        target = self._waypoints[self._current_waypoint_idx]
        dx = target[0] - x
        dy = target[1] - y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if waypoint reached
        if distance < self._goal_tolerance:
            self.get_logger().info(f'Reached waypoint {self._current_waypoint_idx + 1}')
            self._current_waypoint_idx += 1
            self._publish_waypoint_markers()
            return

        # Pure Pursuit: compute steering angle
        # Transform target to vehicle frame
        target_angle = math.atan2(dy, dx)
        alpha = target_angle - yaw  # Angle to target in vehicle frame

        # Normalize angle to [-pi, pi]
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        # Pure pursuit steering: delta = atan(2 * L * sin(alpha) / lookahead)
        lookahead = min(self._lookahead, distance)
        if lookahead > 0.1:
            curvature = 2.0 * math.sin(alpha) / lookahead
            steering_angle = math.atan(self._wheelbase * curvature)
        else:
            steering_angle = 0.0

        # Convert to cmd_vel
        cmd = Twist()

        # Linear velocity - slow down when turning or near goal
        speed_factor = 1.0 - min(abs(steering_angle) / 0.6, 0.7)  # Slow for sharp turns
        approach_factor = min(distance / self._lookahead, 1.0)  # Slow near goal
        cmd.linear.x = self._max_linear * speed_factor * approach_factor

        # Angular velocity from steering angle (Ackermann kinematics)
        # omega = v * tan(delta) / L
        if abs(cmd.linear.x) > 0.01:
            cmd.angular.z = cmd.linear.x * math.tan(steering_angle) / self._wheelbase
            cmd.angular.z = max(-self._max_angular, min(self._max_angular, cmd.angular.z))

        self._cmd_pub.publish(cmd)

    def _send_stop(self):
        """Send zero velocity command."""
        cmd = Twist()
        self._cmd_pub.publish(cmd)

    def _quaternion_to_yaw(self, q):
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _publish_waypoint_markers(self):
        """Publish waypoint visualization markers."""
        marker_array = MarkerArray()

        for i, wp in enumerate(self._waypoints):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Color: green for reached, yellow for current, red for future
            if i < self._current_waypoint_idx:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            elif i == self._current_waypoint_idx:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
