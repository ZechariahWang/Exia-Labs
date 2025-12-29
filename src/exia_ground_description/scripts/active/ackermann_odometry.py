#!/usr/bin/env python3
"""
Ackermann odometry node for Exia Ground robot.
Computes odometry from wheel velocities and steering angles.

Subscribes to:
- /joint_states (sensor_msgs/JointState)

Publishes to:
- /odom (nav_msgs/Odometry)
- /tf (odom -> base_footprint transform)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a quaternion from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class AckermannOdometry(Node):
    """Computes odometry from Ackermann steering joint states."""

    # Robot geometry (must match URDF)
    WHEEL_RADIUS = 0.3      # meters
    WHEEL_BASE = 1.3        # meters (front to rear axle)
    TRACK_WIDTH = 1.1       # meters (left to right wheel)

    def __init__(self):
        super().__init__('ackermann_odometry')

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous time for integration
        self.last_time = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.get_logger().info('Ackermann odometry node started')

    def joint_callback(self, msg: JointState):
        """Process joint states and compute odometry."""
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            self.last_time = current_time
            return

        # Extract joint data
        try:
            joint_dict = dict(zip(msg.name, msg.velocity))
            pos_dict = dict(zip(msg.name, msg.position))
        except (TypeError, ValueError):
            return

        # Get rear wheel velocities (rad/s)
        left_wheel_vel = joint_dict.get('rear_left_wheel_joint', 0.0)
        right_wheel_vel = joint_dict.get('rear_right_wheel_joint', 0.0)

        # Get front steering angles (rad)
        left_steer = pos_dict.get('front_left_steer_joint', 0.0)
        right_steer = pos_dict.get('front_right_steer_joint', 0.0)

        # Average wheel velocity -> linear velocity
        avg_wheel_vel = (left_wheel_vel + right_wheel_vel) / 2.0
        linear_vel = avg_wheel_vel * self.WHEEL_RADIUS

        # Average steering angle
        avg_steer = (left_steer + right_steer) / 2.0

        # Angular velocity from Ackermann geometry
        # omega = v * tan(steer_angle) / wheelbase
        if abs(avg_steer) > 0.001:
            angular_vel = linear_vel * math.tan(avg_steer) / self.WHEEL_BASE
        else:
            angular_vel = 0.0

        # Integrate odometry
        delta_x = linear_vel * math.cos(self.theta) * dt
        delta_y = linear_vel * math.sin(self.theta) * dt
        delta_theta = angular_vel * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self.theta)

        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
