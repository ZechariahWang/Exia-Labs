#!/usr/bin/env python3
"""
Ackermann Controller - Odometry and TF publisher.

Publishes odometry and odom->base_footprint transform based on cmd_vel.
Uses Gazebo services for physics control when available.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(yaw):
    """Convert yaw to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class AckermannController(Node):
    def __init__(self):
        super().__init__('ackermann_controller')

        # State
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry timer at 50Hz
        self.timer = self.create_timer(0.02, self.update_odometry)

        self.get_logger().info('Ackermann controller started - publishing odom and TF')

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z

    def update_odometry(self):
        """Update and publish odometry."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt < 0.001:
            return
        self.last_time = now

        # Integrate velocities (dead reckoning)
        if abs(self.target_angular) < 0.001:
            self.x += self.target_linear * math.cos(self.theta) * dt
            self.y += self.target_linear * math.sin(self.theta) * dt
        else:
            r = self.target_linear / self.target_angular
            dtheta = self.target_angular * dt
            self.x += r * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += r * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            self.theta += dtheta

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        q = euler_to_quaternion(self.theta)

        # Publish TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.target_linear
        odom.twist.twist.angular.z = self.target_angular

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannController()
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
