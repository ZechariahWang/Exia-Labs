#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanQosBridge(Node):
    def __init__(self):
        super().__init__('scan_qos_bridge')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.callback, best_effort_qos)
        self.pub = self.create_publisher(
            LaserScan, '/scan_reliable', reliable_qos)

        self.get_logger().info('Scan QoS bridge: /scan (best_effort) -> /scan_reliable (reliable)')

    def callback(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanQosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
