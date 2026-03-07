#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

try:
    from Phidget22.Devices.DCMotor import DCMotor
    from Phidget22.PhidgetException import PhidgetException
    PHIDGETS_AVAILABLE = True
except ImportError:
    PHIDGETS_AVAILABLE = False

MAX_DUTY = 0.3
HUB_PORT = 3


class MotorReceiver(Node):

    def __init__(self):
        super().__init__('test_motor_receiver')
        self.declare_parameter('hub_port', HUB_PORT)
        self.declare_parameter('max_duty', MAX_DUTY)

        self._max_duty = self.get_parameter('max_duty').value
        self._motor = None
        self._motor_ok = False

        if PHIDGETS_AVAILABLE:
            self._init_motor()
        else:
            self.get_logger().error('Phidget22 not installed')

        self.create_subscription(
            TwistStamped, '/test/motor_cmd', self._cmd_cb, 10)

        self._last_rx_time = 0.0
        self.create_timer(0.5, self._timeout_check)

        self.get_logger().info('Motor receiver ready — waiting for commands')

    def _init_motor(self):
        try:
            m = DCMotor()
            m.setHubPort(self.get_parameter('hub_port').value)
            m.setIsHubPortDevice(False)
            m.openWaitForAttachment(5000)
            self._motor = m
            self._motor_ok = True
            self.get_logger().info(
                f'DCMotor attached on hub port {self.get_parameter("hub_port").value}')
        except PhidgetException as e:
            self.get_logger().error(f'DCMotor init failed: {e}')

    def _cmd_cb(self, msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        sent_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        latency_ms = (now_sec - sent_sec) * 1000.0
        self._last_rx_time = time.monotonic()

        joy_val = msg.twist.angular.z
        duty = joy_val * self._max_duty

        self.get_logger().info(
            f'Joy: {joy_val:+.3f}  Duty: {duty:+.3f}  Latency: {latency_ms:.1f} ms')

        if self._motor_ok and self._motor is not None:
            try:
                self._motor.setTargetVelocity(duty)
            except Exception as e:
                self.get_logger().warn(f'Motor command failed: {e}')

    def _timeout_check(self):
        if self._last_rx_time > 0 and time.monotonic() - self._last_rx_time > 1.0:
            if self._motor_ok and self._motor is not None:
                try:
                    self._motor.setTargetVelocity(0.0)
                except Exception:
                    pass

    def destroy_node(self):
        if self._motor is not None:
            try:
                self._motor.setTargetVelocity(0.0)
                self._motor.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
