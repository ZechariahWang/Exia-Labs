#!/usr/bin/env python3
# Ackermann Drive Node - Bridges cmd_vel to three-motor system, publishes odom

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
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from exia_control.hal import SimulationHAL, AckermannConfig
from exia_control.control import AckermannDriveController
from exia_control.control.ackermann_drive import DriveControllerConfig


class AckermannDriveNode(Node):

    def __init__(self):
        super().__init__('ackermann_drive_node')

        # Parameters
        self.declare_parameter('wheelbase', 1.3)
        self.declare_parameter('track_width', 1.1)
        self.declare_parameter('wheel_radius', 0.3)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('max_deceleration', 5.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('auto_brake_on_stop', True)
        self.declare_parameter('hal_type', 'simulation')

        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_steering = self.get_parameter('max_steering_angle').value
        max_speed = self.get_parameter('max_speed').value
        hal_type = self.get_parameter('hal_type').value

        # HAL and controller config
        hal_config = AckermannConfig(
            wheelbase=wheelbase,
            track_width=track_width,
            wheel_radius=wheel_radius,
            max_steering_angle=max_steering,
            max_speed=max_speed,
        )

        ctrl_config = DriveControllerConfig(
            wheelbase=wheelbase,
            track_width=track_width,
            wheel_radius=wheel_radius,
            max_steering_angle=max_steering,
            max_speed=max_speed,
            cmd_timeout=self.get_parameter('cmd_timeout').value,
            auto_brake_on_stop=self.get_parameter('auto_brake_on_stop').value,
        )

        # Initialize HAL
        self.get_logger().info(f'Using {hal_type.capitalize()} HAL')
        if hal_type == 'simulation':
            self._hal = SimulationHAL(self, hal_config)
        else:
            from exia_control.hal.hardware import HardwareHAL
            self._hal = HardwareHAL(self, hal_config)

        if not self._hal.initialize():
            self.get_logger().error('Failed to initialize HAL!')
            return

        self._controller = AckermannDriveController(ctrl_config)

        # Publishers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self._steering_debug_pub = self.create_publisher(Float64, '/ackermann/steering', 10)
        self._throttle_debug_pub = self.create_publisher(Float64, '/ackermann/throttle', 10)
        self._brake_debug_pub = self.create_publisher(Float64, '/ackermann/brake', 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, qos)

        # Services
        self.create_service(Trigger, '/ackermann/emergency_stop', self._estop_callback)
        self.create_service(Trigger, '/ackermann/clear_estop', self._clear_estop_callback)

        # Odometry state
        self._last_time = self.get_clock().now()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        # Control loop at 50Hz
        self.create_timer(0.02, self._control_loop)

        self.get_logger().info('Ackermann Drive Node started')

    def _cmd_vel_callback(self, msg: Twist):
        current_time = self.get_clock().now().nanoseconds / 1e9
        command = self._controller.cmd_vel_to_ackermann(msg, current_time)
        self._hal.set_command(command)

        # Debug publishers
        steering_msg = Float64()
        steering_msg.data = command.steering_angle
        self._steering_debug_pub.publish(steering_msg)

        throttle_msg = Float64()
        throttle_msg.data = command.throttle
        self._throttle_debug_pub.publish(throttle_msg)

        brake_msg = Float64()
        brake_msg.data = command.brake
        self._brake_debug_pub.publish(brake_msg)

    def _control_loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        state = self._hal.get_state()

        # Odometry integration
        v = state.linear_velocity
        omega = v * math.tan(state.steering_angle) / self._controller.config.wheelbase

        self._odom_yaw += omega * dt
        self._odom_x += v * math.cos(self._odom_yaw) * dt
        self._odom_y += v * math.sin(self._odom_yaw) * dt

        self._publish_odometry(state, now)

    def _publish_odometry(self, state, timestamp):
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self._odom_yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self._odom_yaw / 2)

        odom.twist.twist.linear.x = state.linear_velocity
        odom.twist.twist.angular.z = state.linear_velocity * math.tan(state.steering_angle) / self._controller.config.wheelbase

        self._odom_pub.publish(odom)

        # TF transform
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self._odom_x
        t.transform.translation.y = self._odom_y
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _estop_callback(self, request, response):
        self._hal.emergency_stop()
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def _clear_estop_callback(self, request, response):
        if self._hal.clear_emergency_stop():
            response.success = True
            response.message = 'Emergency stop cleared'
        else:
            response.success = False
            response.message = 'Cannot clear: vehicle still moving'
        return response

    def shutdown(self):
        self._hal.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
