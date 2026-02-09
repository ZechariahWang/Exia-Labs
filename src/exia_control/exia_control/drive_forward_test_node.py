import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class DriveForwardTestNode(Node):
    def __init__(self):
        super().__init__('drive_forward_test_node')

        self.declare_parameter('drive_speed', 0.5)
        self.declare_parameter('drive_duration', 5.0)
        self.declare_parameter('obstacle_threshold', 1.5)
        self.declare_parameter('front_sector_angle', 1.05)

        self.drive_speed = self.get_parameter('drive_speed').value
        self.drive_duration = self.get_parameter('drive_duration').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.front_sector_angle = self.get_parameter('front_sector_angle').value

        self.drive_speed = min(self.drive_speed, 1.0)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)

        self.latest_scan = None
        self.last_scan_time = None
        self.start_time = None
        self.test_complete = False
        self.stopped_by_obstacle = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Drive Forward Test Node started')
        self.get_logger().info(f'  Speed: {self.drive_speed} m/s')
        self.get_logger().info(f'  Duration: {self.drive_duration} s')
        self.get_logger().info(f'  Obstacle threshold: {self.obstacle_threshold} m')
        self.get_logger().info('Waiting for scan data...')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.time()

    def get_front_obstacle_distance(self, scan):
        half_sector = self.front_sector_angle / 2.0
        min_distance = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if -half_sector <= angle <= half_sector:
                if scan.range_min < r < scan.range_max:
                    min_distance = min(min_distance, r)

        return min_distance

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.test_complete:
            return

        if self.latest_scan is None:
            return

        if self.last_scan_time is not None:
            if time.time() - self.last_scan_time > 0.5:
                self.get_logger().warn('No scan data for 0.5s, stopping')
                self.stop_robot()
                return

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('Starting drive forward test')

        elapsed = time.time() - self.start_time

        if elapsed >= self.drive_duration:
            self.stop_robot()
            self.test_complete = True
            self.get_logger().info(f'Test complete: drove for {self.drive_duration:.1f}s')
            return

        front_distance = self.get_front_obstacle_distance(self.latest_scan)

        if front_distance < self.obstacle_threshold:
            self.stop_robot()
            self.test_complete = True
            self.stopped_by_obstacle = True
            self.get_logger().warn(f'Obstacle detected at {front_distance:.2f}m, stopping')
            self.get_logger().info(f'Test complete: stopped after {elapsed:.1f}s due to obstacle')
            return

        cmd = Twist()
        cmd.linear.x = self.drive_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveForwardTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
