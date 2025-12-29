#!/usr/bin/env python3
"""
ATV Hardware Controller - 3 Motor Setup.

Converts cmd_vel to throttle, brake, and steering commands for a real ATV.

Motors:
  1. Throttle - controls forward speed
  2. Brake - controls braking force
  3. Steering - controls steering angle

Subscribes:
  - /cmd_vel (geometry_msgs/Twist)

Publishes:
  - /atv/throttle (std_msgs/Float64) - 0.0 to 1.0
  - /atv/brake (std_msgs/Float64) - 0.0 to 1.0
  - /atv/steering (std_msgs/Float64) - -1.0 to 1.0 (left to right)
  - /odom (nav_msgs/Odometry)

For hardware integration, replace the publishers with your motor driver interface.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class ATVController(Node):
    def __init__(self):
        super().__init__('atv_controller')

        # ==================== PARAMETERS ====================
        # Declare parameters (can be set via launch file or yaml)
        self.declare_parameter('wheel_base', 1.0)  # meters - adjust to your ATV
        self.declare_parameter('max_speed', 5.0)   # m/s - max forward speed
        self.declare_parameter('max_steering_angle', 0.5)  # radians (~28 degrees)
        self.declare_parameter('steering_rate', 1.0)  # rad/s - how fast steering responds

        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.steering_rate = self.get_parameter('steering_rate').value

        # ==================== STATE ====================
        self.current_steering = 0.0  # Current steering angle (rad)
        self.target_steering = 0.0   # Target steering angle (rad)
        self.current_speed = 0.0     # Estimated current speed (m/s)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ==================== SUBSCRIBERS ====================
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # ==================== PUBLISHERS ====================
        # Motor command publishers
        self.throttle_pub = self.create_publisher(Float64, '/atv/throttle', 10)
        self.brake_pub = self.create_publisher(Float64, '/atv/brake', 10)
        self.steering_pub = self.create_publisher(Float64, '/atv/steering', 10)

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ==================== CONTROL LOOP ====================
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50Hz

        self.get_logger().info('ATV Controller started')
        self.get_logger().info(f'  Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'  Max speed: {self.max_speed}m/s')
        self.get_logger().info(f'  Max steering: {math.degrees(self.max_steering_angle):.1f} deg')

    def cmd_vel_callback(self, msg: Twist):
        """Process incoming velocity commands."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate target steering angle from cmd_vel
        # Ackermann: angular_vel = linear_vel * tan(steering) / wheel_base
        # Therefore: steering = atan(angular_vel * wheel_base / linear_vel)
        if abs(linear_vel) > 0.01:
            self.target_steering = math.atan2(
                angular_vel * self.wheel_base, abs(linear_vel))
        elif abs(angular_vel) > 0.01:
            # Turning in place - max steering
            self.target_steering = self.max_steering_angle if angular_vel > 0 else -self.max_steering_angle
        else:
            self.target_steering = 0.0

        # Clamp steering
        self.target_steering = max(-self.max_steering_angle,
                                   min(self.max_steering_angle, self.target_steering))

        # Store speed command
        self.target_speed = linear_vel

    def control_loop(self):
        """Main control loop - runs at 50Hz."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt < 0.001:
            return
        self.last_time = now

        # ==================== STEERING CONTROL ====================
        # Smoothly move steering toward target
        steer_diff = self.target_steering - self.current_steering
        max_steer_change = self.steering_rate * dt

        if abs(steer_diff) < max_steer_change:
            self.current_steering = self.target_steering
        else:
            self.current_steering += max_steer_change if steer_diff > 0 else -max_steer_change

        # Publish steering command (-1.0 to 1.0)
        steering_cmd = Float64()
        steering_cmd.data = self.current_steering / self.max_steering_angle
        self.steering_pub.publish(steering_cmd)

        # ==================== THROTTLE/BRAKE CONTROL ====================
        target_speed = getattr(self, 'target_speed', 0.0)

        throttle_cmd = Float64()
        brake_cmd = Float64()

        if target_speed > 0.05:
            # Forward - apply throttle
            throttle_cmd.data = min(target_speed / self.max_speed, 1.0)
            brake_cmd.data = 0.0
            self.current_speed = target_speed  # Simplified - assume instant response
        elif target_speed < -0.05:
            # Reverse or braking
            # Option 1: Use brake for stopping
            # Option 2: Use throttle in reverse
            throttle_cmd.data = 0.0
            brake_cmd.data = min(abs(target_speed) / self.max_speed, 1.0)
            self.current_speed = target_speed
        else:
            # Stopped
            throttle_cmd.data = 0.0
            brake_cmd.data = 0.0
            self.current_speed = 0.0

        self.throttle_pub.publish(throttle_cmd)
        self.brake_pub.publish(brake_cmd)

        # ==================== ODOMETRY ====================
        self.update_odometry(dt)

    def update_odometry(self, dt):
        """Update odometry based on bicycle model."""
        if abs(self.current_speed) < 0.001:
            # Not moving
            pass
        elif abs(self.current_steering) < 0.001:
            # Going straight
            self.x += self.current_speed * math.cos(self.theta) * dt
            self.y += self.current_speed * math.sin(self.theta) * dt
        else:
            # Turning - bicycle model
            turn_radius = self.wheel_base / math.tan(self.current_steering)
            angular_vel = self.current_speed / turn_radius
            dtheta = angular_vel * dt

            self.x += turn_radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += turn_radius * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            self.theta += dtheta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish TF
        self.publish_tf()

        # Publish odometry message
        self.publish_odom()

    def publish_tf(self):
        """Publish odom -> base_footprint transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Quaternion from yaw
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)

        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self):
        """Publish odometry message."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        # Velocity in robot frame
        odom.twist.twist.linear.x = self.current_speed
        if abs(self.current_steering) > 0.001:
            turn_radius = self.wheel_base / math.tan(self.current_steering)
            odom.twist.twist.angular.z = self.current_speed / turn_radius
        else:
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ATVController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop commands on shutdown
        node.get_logger().info('Shutting down - sending stop commands')
        stop = Float64()
        stop.data = 0.0
        node.throttle_pub.publish(stop)
        node.brake_pub.publish(Float64(data=1.0))  # Apply brake
        node.steering_pub.publish(stop)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
