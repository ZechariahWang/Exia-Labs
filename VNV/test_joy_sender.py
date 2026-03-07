#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped


class JoySender(Node):

    def __init__(self):
        super().__init__('test_joy_sender')
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._pub = self.create_publisher(TwistStamped, '/test/motor_cmd', 10)
        self._last_value = 0.0
        self.get_logger().info('Joy sender ready — push right stick left/right')

    def _joy_cb(self, msg):
        if len(msg.axes) < 4:
            return
        raw = msg.axes[3]
        if abs(raw) < 0.05:
            raw = 0.0
        if raw == self._last_value:
            return
        self._last_value = raw
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist.angular.z = raw
        self._pub.publish(out)
        self.get_logger().info(f'Sent: {raw:+.3f}')


def main():
    rclpy.init()
    node = JoySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
