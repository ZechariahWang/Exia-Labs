#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


class AckermannDriveNode(Node):
    WHEELBASE = 1.3
    TRACK_WIDTH = 1.1
    WHEEL_RADIUS = 0.3
    MAX_STEERING_ANGLE = 0.6
    MAX_SPEED = 5.0

    def __init__(self):
        super().__init__('ackermann_drive_node')

        self.declare_parameter('publish_tf', True)

        self.publish_tf = self.get_parameter('publish_tf').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_cmd_time = None
        self.cmd_timeout = 0.2

        self.last_time = self.get_clock().now()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            qos
        )

        self.throttle_pub = self.create_publisher(
            Float64MultiArray,
            '/throttle_controller/commands',
            qos
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos
        )

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('Ackermann drive node started')

    def cmd_vel_callback(self, msg: Twist):
        self.linear_vel = max(-self.MAX_SPEED, min(self.MAX_SPEED, msg.linear.x))
        self.angular_vel = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        if self.last_cmd_time is not None:
            cmd_age = (current_time - self.last_cmd_time).nanoseconds / 1e9
            if cmd_age > self.cmd_timeout:
                self.linear_vel = 0.0
                self.angular_vel = 0.0

        if abs(self.linear_vel) < 0.01:
            steering_angle = 0.0
            wheel_speed = 0.0
        else:
            if abs(self.angular_vel) > 0.001:
                turn_radius = self.linear_vel / self.angular_vel
                steering_angle = math.atan(self.WHEELBASE / turn_radius)
            else:
                steering_angle = 0.0

            steering_angle = max(-self.MAX_STEERING_ANGLE,
                               min(self.MAX_STEERING_ANGLE, steering_angle))

            wheel_speed = self.linear_vel / self.WHEEL_RADIUS

        left_steer, right_steer = self.compute_ackermann_angles(steering_angle)

        steering_msg = Float64MultiArray()
        steering_msg.data = [left_steer, right_steer]
        self.steering_pub.publish(steering_msg)

        throttle_msg = Float64MultiArray()
        throttle_msg.data = [wheel_speed, wheel_speed]
        self.throttle_pub.publish(throttle_msg)

        self.update_odometry(dt, steering_angle)

    def compute_ackermann_angles(self, center_angle):
        if abs(center_angle) < 0.001:
            return 0.0, 0.0

        turn_radius = self.WHEELBASE / math.tan(abs(center_angle))

        inner_radius = turn_radius - self.TRACK_WIDTH / 2
        outer_radius = turn_radius + self.TRACK_WIDTH / 2

        inner_angle = math.atan(self.WHEELBASE / inner_radius)
        outer_angle = math.atan(self.WHEELBASE / outer_radius)

        if center_angle > 0:
            return inner_angle, outer_angle
        else:
            return -outer_angle, -inner_angle

    def update_odometry(self, dt, steering_angle):
        if abs(steering_angle) < 0.001:
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
        else:
            turn_radius = self.WHEELBASE / math.tan(steering_angle)
            angular_vel = self.linear_vel / turn_radius

            self.theta += angular_vel * dt
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)

        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel if abs(steering_angle) > 0.001 else 0.0

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()
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
