#!/usr/bin/env python3
import sys
import os
import time
import threading
import rclpy
from rclpy.node import Node
from exia_msgs.msg import NavigationGoal
from std_msgs.msg import String
from std_srvs.srv import Trigger


TERMINAL_STATUSES = {
    'GOAL_REACHED', 'OBSTACLE_STOPPED', 'PATH_ENDED', 'PATH_BLOCKED',
    'MOVE_COMPLETE', 'TURN_COMPLETE',
}


class NavToCmd(Node):
    def __init__(self):
        super().__init__('nav_to_cmd')
        self.pub = self.create_publisher(NavigationGoal, '/navigation/goal', 10)
        self.cancel_client = self.create_client(Trigger, '/navigation/cancel')
        self._status_event = threading.Event()
        self._final_status = None

    def _wait_for_goal_subscriber(self, timeout_sec: float = 12.0) -> bool:
        steps = int(timeout_sec / 0.1)
        for _ in range(steps):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                return True
        return self.pub.get_subscription_count() > 0

    def send_goal(self, msg):
        if not self._wait_for_goal_subscriber(timeout_sec=12.0):
            domain = os.environ.get('ROS_DOMAIN_ID', '<unset>')
            self.get_logger().warn(
                f'No subscriber found on /navigation/goal after 12s '
                f'(ROS_DOMAIN_ID={domain})'
            )
        for _ in range(3):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)

    def send_goal_and_wait(self, msg):
        self.create_subscription(String, '/navigation/status', self._status_callback, 10)

        if not self._wait_for_goal_subscriber(timeout_sec=12.0):
            domain = os.environ.get('ROS_DOMAIN_ID', '<unset>')
            self.get_logger().warn(
                f'No subscriber found on /navigation/goal after 12s '
                f'(ROS_DOMAIN_ID={domain})'
            )
        for _ in range(3):
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('Waiting for navigation to complete (Ctrl+C to cancel)...')
        nav_timeout = 300.0
        nav_start = time.time()

        try:
            while rclpy.ok() and not self._status_event.is_set():
                if time.time() - nav_start > nav_timeout:
                    self.get_logger().error(f'Navigation timeout ({nav_timeout:.0f}s), cancelling...')
                    self.send_cancel()
                    return
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted, cancelling navigation...')
            self.send_cancel()
            return

        if self._final_status:
            parts = self._final_status.split()
            status = parts[0]
            if len(parts) >= 4:
                x, y, heading = parts[1], parts[2], parts[3]
                if status == 'GOAL_REACHED':
                    self.get_logger().info(f'Goal reached! Position: ({x}, {y}), heading: {heading} deg')
                elif status == 'OBSTACLE_STOPPED':
                    self.get_logger().warn(f'Stopped: obstacle detected. Position: ({x}, {y}), heading: {heading} deg')
                elif status == 'PATH_ENDED':
                    self.get_logger().warn(f'Path ended, goal not reached. Position: ({x}, {y}), heading: {heading} deg')
                elif status == 'PATH_BLOCKED':
                    self.get_logger().warn(f'Path blocked by obstacle. Position: ({x}, {y}), heading: {heading} deg')
                elif status == 'MOVE_COMPLETE':
                    self.get_logger().info(f'Move complete. Position: ({x}, {y}), heading: {heading} deg')
                elif status == 'TURN_COMPLETE':
                    self.get_logger().info(f'Turn complete. Position: ({x}, {y}), heading: {heading} deg')
                else:
                    self.get_logger().info(f'Status: {self._final_status}')

    def _status_callback(self, msg):
        status_type = msg.data.split()[0] if msg.data else ''
        if status_type in TERMINAL_STATUSES:
            self._final_status = msg.data
            self._status_event.set()

    def send_cancel(self):
        if not self.cancel_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Cancel service not available')
            return False
        future = self.cancel_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f'Navigation cancelled: {future.result().message}')
            return True
        self.get_logger().error('Cancel request failed')
        return False


def print_usage():
    print('Usage:')
    print('  ros2 run exia_control nav_to xy <x> <y>')
    print('  ros2 run exia_control nav_to latlon <lat> <lon>')
    print('  ros2 run exia_control nav_to latlon <lat> <lon> --origin <origin_lat> <origin_lon>')
    print('  ros2 run exia_control nav_to dms "<lat_dms>" "<lon_dms>"')
    print('  ros2 run exia_control nav_to forward <duration_or_distance> [speed]')
    print('  ros2 run exia_control nav_to turn <duration_or_heading> [angular_vel]')
    print('  ros2 run exia_control nav_to cancel')
    print()
    print('Options:')
    print('  --direct    Navigate straight to target, stop on obstacle (no replanning)')
    print('              CLI stays running until goal reached or obstacle detected')
    print()
    print('Movement commands (CLI blocks until complete):')
    print('  forward <Ns>  [speed]   Move forward for N seconds at speed m/s (default 1.0)')
    print('  forward <Nm>  [speed]   Move forward for N meters at speed m/s (default 1.0)')
    print('  turn    <Ns>  [ang_vel] Turn for N seconds at angular velocity rad/s (default 0.5)')
    print('  turn    <N>             Turn by N degrees (positive=left, negative=right)')
    print()
    print('Examples:')
    print('  ros2 run exia_control nav_to xy 30.0 15.0')
    print('  ros2 run exia_control nav_to xy 30.0 15.0 --direct')
    print('  ros2 run exia_control nav_to latlon 49.666667 11.841389')
    print("  ros2 run exia_control nav_to dms \"49D40'00\\\"N\" \"11D50'29\\\"E\"")
    print('  ros2 run exia_control nav_to forward 3s 2.0')
    print('  ros2 run exia_control nav_to forward 50m 1.5')
    print('  ros2 run exia_control nav_to turn 3s 0.5')
    print('  ros2 run exia_control nav_to turn 90')
    print('  ros2 run exia_control nav_to cancel')


def parse_move_arg(arg):
    arg = arg.strip().lower()
    if arg.endswith('s'):
        return 'time', float(arg[:-1])
    elif arg.endswith('m'):
        return 'distance', float(arg[:-1])
    else:
        return 'heading', float(arg)


def main(args=None):
    argv = sys.argv[1:]

    ros_args = []
    user_args = []
    skip_next = False
    for i, a in enumerate(argv):
        if skip_next:
            skip_next = False
            continue
        if a.startswith('--ros-args') or a.startswith('-r') or a.startswith('__'):
            ros_args.append(a)
            if a == '--ros-args':
                for j in range(i + 1, len(argv)):
                    ros_args.append(argv[j])
                break
        else:
            user_args.append(a)

    direct_mode = '--direct' in user_args
    if direct_mode:
        user_args = [a for a in user_args if a != '--direct']

    if not user_args:
        print_usage()
        return

    cmd = user_args[0].lower()

    rclpy.init(args=['--ros-args'] + ros_args if ros_args else None)
    node = NavToCmd()

    try:
        if cmd == 'cancel':
            node.send_cancel()
            return

        msg = NavigationGoal()

        if cmd == 'xy':
            if len(user_args) < 3:
                print('Error: xy requires <x> <y>')
                print_usage()
                return
            msg.coord_type = 'xy'
            msg.x = float(user_args[1])
            msg.y = float(user_args[2])
            node.get_logger().info(f'Sending goal: xy ({msg.x:.2f}, {msg.y:.2f})')

        elif cmd == 'latlon':
            if len(user_args) < 3:
                print('Error: latlon requires <lat> <lon>')
                print_usage()
                return
            msg.coord_type = 'latlon'
            msg.lat = float(user_args[1])
            msg.lon = float(user_args[2])
            origin_idx = None
            for i, a in enumerate(user_args):
                if a == '--origin':
                    origin_idx = i
                    break
            if origin_idx is not None and len(user_args) > origin_idx + 2:
                msg.origin_lat = float(user_args[origin_idx + 1])
                msg.origin_lon = float(user_args[origin_idx + 2])
                node.get_logger().info(
                    f'Sending goal: latlon ({msg.lat:.6f}, {msg.lon:.6f}) '
                    f'origin ({msg.origin_lat:.6f}, {msg.origin_lon:.6f})')
            else:
                node.get_logger().info(f'Sending goal: latlon ({msg.lat:.6f}, {msg.lon:.6f})')

        elif cmd == 'dms':
            if len(user_args) < 3:
                print('Error: dms requires "<lat_dms>" "<lon_dms>"')
                print_usage()
                return
            msg.coord_type = 'dms'
            msg.lat_dms = user_args[1]
            msg.lon_dms = user_args[2]
            origin_idx = None
            for i, a in enumerate(user_args):
                if a == '--origin':
                    origin_idx = i
                    break
            if origin_idx is not None and len(user_args) > origin_idx + 2:
                msg.origin_lat = float(user_args[origin_idx + 1])
                msg.origin_lon = float(user_args[origin_idx + 2])
            node.get_logger().info(f'Sending goal: dms ({msg.lat_dms}, {msg.lon_dms})')

        elif cmd == 'forward':
            if len(user_args) < 2:
                print('Error: forward requires <duration_or_distance> [speed]')
                print_usage()
                return
            mode, value = parse_move_arg(user_args[1])
            if value <= 0.0:
                print('Error: forward value must be positive')
                return
            speed = float(user_args[2]) if len(user_args) >= 3 else 1.0
            if speed <= 0.0:
                print('Error: speed must be positive')
                return
            if speed > 5.0:
                print('Error: speed exceeds max (5.0 m/s)')
                return
            msg.coord_type = 'forward'
            if mode == 'time':
                msg.move_type = 'time'
                msg.move_value = value
                node.get_logger().info(f'Sending forward: {value:.1f}s at {speed:.1f} m/s')
            elif mode == 'distance':
                msg.move_type = 'distance'
                msg.move_value = value
                node.get_logger().info(f'Sending forward: {value:.1f}m at {speed:.1f} m/s')
            else:
                print('Error: forward requires duration (e.g. 3s) or distance (e.g. 50m)')
                print_usage()
                return
            msg.move_speed = speed
            node.send_goal_and_wait(msg)
            return

        elif cmd == 'turn':
            if len(user_args) < 2:
                print('Error: turn requires <duration_or_heading> [angular_vel]')
                print_usage()
                return
            mode, value = parse_move_arg(user_args[1])
            if mode == 'time':
                if value <= 0.0:
                    print('Error: turn duration must be positive')
                    return
                ang_vel = float(user_args[2]) if len(user_args) >= 3 else 0.5
                msg.coord_type = 'turn'
                msg.move_type = 'time'
                msg.move_value = value
                msg.move_speed = ang_vel
                node.get_logger().info(f'Sending turn: {value:.1f}s at {ang_vel:.2f} rad/s')
            elif mode == 'heading':
                msg.coord_type = 'turn'
                msg.move_type = 'heading'
                msg.move_value = value
                msg.move_speed = float(user_args[2]) if len(user_args) >= 3 else 1.0
                node.get_logger().info(f'Sending turn: {value:.1f} degrees')
            else:
                print('Error: turn requires duration (e.g. 3s) or heading (e.g. 90)')
                print_usage()
                return
            node.send_goal_and_wait(msg)
            return

        else:
            print(f'Unknown command: {cmd}')
            print_usage()
            return

        msg.direct = direct_mode
        if direct_mode:
            node.get_logger().info('Direct navigation mode (no replanning, stop on obstacle)')
            node.send_goal_and_wait(msg)
        else:
            node.send_goal(msg)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
