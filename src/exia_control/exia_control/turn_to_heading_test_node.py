import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time


class TurnToHeadingTestNode(Node):
    def __init__(self):
        super().__init__('turn_to_heading_test_node')

        self.declare_parameter('target_heading_deg', 90.0)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('kp_heading', 1.5)
        self.declare_parameter('heading_tolerance_deg', 5.0)
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('obstacle_threshold', 1.5)
        self.declare_parameter('front_sector_angle', 1.05)

        target_heading_deg = self.get_parameter('target_heading_deg').value
        self.target_heading = math.radians(target_heading_deg)
        self.turn_speed = min(self.get_parameter('turn_speed').value, 1.0)
        self.kp_heading = self.get_parameter('kp_heading').value
        self.heading_tolerance = math.radians(self.get_parameter('heading_tolerance_deg').value)
        self.timeout = self.get_parameter('timeout').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.front_sector_angle = self.get_parameter('front_sector_angle').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.latest_scan = None
        self.latest_odom = None
        self.last_scan_time = None
        self.last_odom_time = None
        self.start_time = None
        self.initial_heading = None
        self.test_complete = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f'Turn to Heading Test Node started')
        self.get_logger().info(f'  Target heading: {target_heading_deg:.1f} degrees')
        self.get_logger().info(f'  Turn speed: {self.turn_speed} m/s')
        self.get_logger().info(f'  Kp: {self.kp_heading}')
        self.get_logger().info(f'  Tolerance: {self.get_parameter("heading_tolerance_deg").value:.1f} degrees')
        self.get_logger().info(f'  Timeout: {self.timeout} s')
        self.get_logger().info('Waiting for scan and odom data...')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.time()

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.last_odom_time = time.time()

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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

        if self.latest_scan is None or self.latest_odom is None:
            return

        current_time = time.time()

        if self.last_scan_time is not None and current_time - self.last_scan_time > 0.5:
            self.get_logger().warn('No scan data for 0.5s, stopping')
            self.stop_robot()
            return

        if self.last_odom_time is not None and current_time - self.last_odom_time > 0.5:
            self.get_logger().warn('No odom data for 0.5s, stopping')
            self.stop_robot()
            return

        current_yaw = self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

        if self.start_time is None:
            self.start_time = current_time
            self.initial_heading = current_yaw
            self.get_logger().info(f'Starting turn test from heading {math.degrees(current_yaw):.1f} degrees')

        elapsed = current_time - self.start_time

        if elapsed >= self.timeout:
            self.stop_robot()
            self.test_complete = True
            self.get_logger().warn(f'Timeout reached after {self.timeout:.1f}s')
            self.get_logger().info(f'Final heading: {math.degrees(current_yaw):.1f} degrees')
            return

        front_distance = self.get_front_obstacle_distance(self.latest_scan)

        if front_distance < self.obstacle_threshold:
            self.stop_robot()
            self.test_complete = True
            self.get_logger().warn(f'Obstacle detected at {front_distance:.2f}m, stopping')
            self.get_logger().info(f'Final heading: {math.degrees(current_yaw):.1f} degrees')
            return

        heading_error = self.normalize_angle(self.target_heading - current_yaw)

        if abs(heading_error) < self.heading_tolerance:
            self.stop_robot()
            self.test_complete = True
            self.get_logger().info(f'Target heading reached!')
            self.get_logger().info(f'Final heading: {math.degrees(current_yaw):.1f} degrees')
            self.get_logger().info(f'Completed in {elapsed:.1f}s')
            return

        angular_vel = self.kp_heading * heading_error
        angular_vel = max(-1.0, min(1.0, angular_vel))

        cmd = Twist()
        cmd.linear.x = self.turn_speed
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurnToHeadingTestNode()

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
