#!/usr/bin/env python3
"""
Steering controller for Ackermann visual feedback.

This node subscribes to cmd_vel and publishes joint positions
to animate the front steering joints based on angular velocity commands.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg


class SteeringController(Node):
    def __init__(self):
        super().__init__('steering_controller')

        # Parameters
        self.wheel_base = 0.4  # Distance between front and rear axles (m)
        self.wheel_separation = 0.45  # Distance between left and right wheels (m)
        self.max_steer_angle = 0.6  # Maximum steering angle (rad, ~34 degrees)

        # Current steering angle
        self.current_steer_angle = 0.0
        self.target_steer_angle = 0.0
        self.steer_rate = 2.0  # rad/s - how fast steering responds

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher for joint trajectory (Gazebo joint_pose_trajectory plugin)
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/set_joint_trajectory',
            10
        )

        # Timer for smooth steering animation
        self.timer = self.create_timer(0.02, self.update_steering)  # 50Hz

        self.get_logger().info('Steering controller started')

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to steering angle using Ackermann geometry."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        if abs(linear_vel) < 0.01:
            # If not moving, center steering
            self.target_steer_angle = 0.0
        elif abs(angular_vel) < 0.001:
            # Going straight
            self.target_steer_angle = 0.0
        else:
            # Calculate steering angle from angular velocity
            # For Ackermann: angular_vel = linear_vel * tan(steer_angle) / wheel_base
            # So: steer_angle = atan(angular_vel * wheel_base / linear_vel)
            turn_radius = linear_vel / angular_vel if angular_vel != 0 else float('inf')
            self.target_steer_angle = math.atan(self.wheel_base / turn_radius) if abs(turn_radius) > 0.01 else 0.0

        # Clamp to max steering angle
        self.target_steer_angle = max(-self.max_steer_angle,
                                       min(self.max_steer_angle, self.target_steer_angle))

    def update_steering(self):
        """Smoothly interpolate steering angle and publish to Gazebo."""
        # Smoothly move current angle toward target
        diff = self.target_steer_angle - self.current_steer_angle
        max_change = self.steer_rate * 0.02  # Change per timestep

        if abs(diff) < max_change:
            self.current_steer_angle = self.target_steer_angle
        else:
            self.current_steer_angle += max_change if diff > 0 else -max_change

        # Calculate individual steering angles using Ackermann geometry
        # For small angles, both wheels get approximately the same angle
        left_steer = self.current_steer_angle
        right_steer = self.current_steer_angle

        # Publish joint trajectory to Gazebo
        msg = JointTrajectory()
        msg.joint_names = ['front_left_steer_joint', 'front_right_steer_joint']

        point = JointTrajectoryPoint()
        point.positions = [left_steer, right_steer]
        point.time_from_start = DurationMsg(sec=0, nanosec=50000000)  # 50ms

        msg.points = [point]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()

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
