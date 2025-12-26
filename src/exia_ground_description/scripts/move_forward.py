#!/usr/bin/env python3

# move thr bot forward to test if the config works properly

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveForward(Node):
    def __init__(self):
        super().__init__('move_forward')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('Moving forward at 0.5 m/s. Press Ctrl+C to stop.')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.5  
        msg.angular.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            stop_msg = Twist()
            node.publisher.publish(stop_msg)
            node.get_logger().info('Stopping robot.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
