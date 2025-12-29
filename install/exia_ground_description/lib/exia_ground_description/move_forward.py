#!/usr/bin/env python3
"""
Ackermann drive test script for Exia Ground robot.

Controls:
- Drives forward by default
- Use command line args to change behavior:
  --turn-left   : Turn left while driving
  --turn-right  : Turn right while driving
  --circle      : Drive in a circle
  --speed X     : Set speed (default 0.5 m/s)
"""

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AckermannDriver(Node):
    def __init__(self, linear_vel: float = 0.5, angular_vel: float = 0.0):
        super().__init__('ackermann_driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        self.linear_vel = linear_vel
        self.angular_vel = angular_vel

        if angular_vel == 0.0:
            self.get_logger().info(f'Driving forward at {linear_vel} m/s. Press Ctrl+C to stop.')
        else:
            direction = 'left' if angular_vel > 0 else 'right'
            self.get_logger().info(
                f'Driving {direction} at {linear_vel} m/s, turning at {abs(angular_vel)} rad/s. '
                'Press Ctrl+C to stop.'
            )

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Ackermann drive test for Exia Ground robot')
    parser.add_argument('--turn-left', action='store_true', help='Turn left while driving')
    parser.add_argument('--turn-right', action='store_true', help='Turn right while driving')
    parser.add_argument('--circle', action='store_true', help='Drive in a circle')
    parser.add_argument('--speed', type=float, default=0.5, help='Forward speed in m/s (default: 0.5)')
    parser.add_argument('--reverse', action='store_true', help='Drive in reverse')

    parsed_args = parser.parse_args()

    linear_vel = parsed_args.speed
    if parsed_args.reverse:
        linear_vel = -abs(linear_vel)

    angular_vel = 0.0
    if parsed_args.turn_left:
        angular_vel = 0.3
    elif parsed_args.turn_right:
        angular_vel = -0.3
    elif parsed_args.circle:
        angular_vel = 0.5

    rclpy.init(args=None)
    node = AckermannDriver(linear_vel=linear_vel, angular_vel=angular_vel)

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
