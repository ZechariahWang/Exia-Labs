#!/usr/bin/env python3
# Dynamic Navigator - Goal-based navigation with continuous A* replanning

import math
import os
import sys
from enum import Enum
from typing import Optional

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from exia_control.planning.pure_pursuit import (
    PurePursuitController, PurePursuitConfig, Path as PurePursuitPath
)
from exia_control.navigation.path_validator import PathValidator
from exia_control.navigation.planner_interface import (
    PlannerInterface, PlanningStatus
)


TARGET_POINT = [22, 24]  # Goal coordinates in meters (map frame)


class NavigatorState(Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    RECOVERING = 3


class DynamicNavigator(Node):

    def __init__(self):
        super().__init__('dynamic_navigator')

        # Parameters
        self.declare_parameter('replan_period', 0.5)
        self.declare_parameter('min_replan_interval', 0.2)
        self.declare_parameter('goal_tolerance', 1.0)
        self.declare_parameter('lookahead_distance', 2.5)
        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('path_switch_threshold', 2.0)
        self.declare_parameter('obstacle_lookahead', 8.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('auto_start', True)

        self.replan_period = self.get_parameter('replan_period').value
        self.min_replan_interval = self.get_parameter('min_replan_interval').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.path_switch_threshold = self.get_parameter('path_switch_threshold').value
        self.obstacle_lookahead = self.get_parameter('obstacle_lookahead').value
        control_rate = self.get_parameter('control_rate').value

        # State
        self.state = NavigatorState.IDLE
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0

        self.current_goal: Optional[PoseStamped] = None
        self.current_path: Optional[PurePursuitPath] = None
        self.current_nav_path: Optional[Path] = None
        self.path_index = 0
        self.last_replan_time = 0.0
        self.replan_pending = False

        # Pure Pursuit controller
        pp_config = PurePursuitConfig(
            lookahead_distance=lookahead,
            goal_tolerance=self.goal_tolerance,
            max_linear_speed=self.target_speed * 1.5,
            min_linear_speed=0.2,
            wheelbase=1.3,
        )
        self.pure_pursuit = PurePursuitController(pp_config)

        # Path validator and planner
        self.path_validator = PathValidator()
        self.costmap: Optional[OccupancyGrid] = None
        self.callback_group = ReentrantCallbackGroup()
        self.planner = PlannerInterface(self)

        # Recovery state
        self._recovery_start_x = 0.0
        self._recovery_start_y = 0.0
        self._recovery_target_dist = 2.0
        self._consecutive_replan_failures = 0

        # QoS for costmap
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._costmap_callback, costmap_qos)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)

        # Services
        self.cancel_srv = self.create_service(
            Trigger, '/navigation/cancel', self._cancel_callback)

        # Timers
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)
        self.replan_timer = self.create_timer(self.replan_period, self._periodic_replan)
        self.viz_timer = self.create_timer(0.5, self._publish_visualization)

        # Auto-start
        auto_start = self.get_parameter('auto_start').value
        self._auto_start_done = False
        if auto_start:
            self.create_timer(3.0, self._delayed_auto_start, callback_group=self.callback_group)

        self.get_logger().info(f'Dynamic Navigator initialized - Target: ({TARGET_POINT[0]}, {TARGET_POINT[1]})')

    def _delayed_auto_start(self):
        if self._auto_start_done or self.state != NavigatorState.IDLE:
            return
        self._auto_start_done = True

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(TARGET_POINT[0])
        goal.pose.position.y = float(TARGET_POINT[1])
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'Auto-starting to TARGET_POINT: ({TARGET_POINT[0]}, {TARGET_POINT[1]})')
        self._goal_callback(goal)

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_speed = msg.twist.twist.linear.x

    def _goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        self.get_logger().info(f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        self.current_path = None
        self.current_nav_path = None
        self._consecutive_replan_failures = 0
        self.state = NavigatorState.PLANNING
        self._request_initial_plan()

    def _costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg
        self.path_validator.update_costmap(msg)

        if not hasattr(self, '_costmap_received'):
            self._costmap_received = True
            self.get_logger().info(f'Costmap received: {msg.info.width}x{msg.info.height}')

        if self.state == NavigatorState.EXECUTING:
            self._check_path_for_obstacles()

    def _cancel_callback(self, request, response):
        self.state = NavigatorState.IDLE
        self.current_goal = None
        self.current_path = None
        self._publish_stop()
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    def _control_loop(self):
        if self.state == NavigatorState.IDLE:
            self._publish_stop()
        elif self.state == NavigatorState.PLANNING:
            self._publish_stop()
        elif self.state == NavigatorState.EXECUTING:
            self._execute_path()
        elif self.state == NavigatorState.RECOVERING:
            self._execute_recovery()

    def _execute_path(self):
        if self.current_path is None or self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        # Check goal reached
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        dist_to_goal = math.sqrt((goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! Distance: {dist_to_goal:.2f}m')
            self.state = NavigatorState.IDLE
            self.current_goal = None
            self.current_path = None
            self._publish_stop()
            return

        # Execute Pure Pursuit
        cmd, goal_reached = self.pure_pursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw, self.robot_speed)

        if goal_reached:
            if dist_to_goal < self.goal_tolerance * 2.0:
                self.get_logger().info('Path complete, goal reached!')
                self.state = NavigatorState.IDLE
                self.current_goal = None
                self._publish_stop()
                return
            else:
                self.get_logger().info('Path ended but goal not reached, replanning...')
                self._request_replan()

        self.cmd_vel_pub.publish(cmd)

    def _execute_recovery(self):
        if not hasattr(self, '_recovery_in_progress') or not self._recovery_in_progress:
            self._recovery_in_progress = True
            self._recovery_start_x = self.robot_x
            self._recovery_start_y = self.robot_y
            self.get_logger().info('Starting recovery maneuver (backing up 2m)')

        dx = self.robot_x - self._recovery_start_x
        dy = self.robot_y - self._recovery_start_y
        dist_backed = math.sqrt(dx*dx + dy*dy)

        if dist_backed >= self._recovery_target_dist:
            self.get_logger().info(f'Recovery complete ({dist_backed:.2f}m), replanning...')
            self._recovery_in_progress = False
            self._consecutive_replan_failures = 0
            self.state = NavigatorState.PLANNING
            self._request_initial_plan()
            return

        # Check for obstacles behind
        behind_x = self.robot_x - 1.0 * math.cos(self.robot_yaw)
        behind_y = self.robot_y - 1.0 * math.sin(self.robot_yaw)
        behind_cost = self.path_validator._get_cost_at_world(behind_x, behind_y)

        if behind_cost >= 100:
            self.get_logger().warn('Obstacle behind, cannot back up further')
            self._recovery_in_progress = False
            self.state = NavigatorState.PLANNING
            self._request_initial_plan()
            return

        cmd = Twist()
        cmd.linear.x = -0.5
        self.cmd_vel_pub.publish(cmd)

    def _periodic_replan(self):
        if self.state != NavigatorState.EXECUTING or self.current_goal is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.min_replan_interval:
            return

        if not self.replan_pending:
            self._request_replan()

    def _request_initial_plan(self):
        if self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        start = PoseStamped()
        start.header.frame_id = 'odom'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        self.get_logger().info(
            f'Planning: ({self.robot_x:.2f}, {self.robot_y:.2f}) -> '
            f'({self.current_goal.pose.position.x:.2f}, {self.current_goal.pose.position.y:.2f})')

        self.replan_pending = True
        self.planner.plan_path_async(start, self.current_goal, self._on_initial_path)

    def _request_replan(self):
        if self.current_goal is None:
            return

        start = PoseStamped()
        start.header.frame_id = 'odom'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        self.replan_pending = True
        self.last_replan_time = self.get_clock().now().nanoseconds / 1e9
        self.planner.plan_path_async(start, self.current_goal, self._on_replan_received)

    def _on_initial_path(self, result):
        self.replan_pending = False

        if result.status == PlanningStatus.SUCCEEDED and result.path:
            nav_path = result.path
            self.get_logger().info(f'Initial path: {len(nav_path.poses)} poses')

            validation = self.path_validator.validate_path(nav_path, check_footprint=True)
            if not validation.is_valid:
                self.get_logger().warn(f'Initial path collision at index {validation.blocked_index}')
                self._consecutive_replan_failures += 1
                if self._consecutive_replan_failures >= 3:
                    self.get_logger().error('Multiple failures, entering recovery')
                    self.state = NavigatorState.RECOVERING
                else:
                    self.create_timer(0.5, self._request_initial_plan, callback_group=self.callback_group)
                return

            self.current_nav_path = nav_path
            self.current_path = self._nav_path_to_pp_path(nav_path)
            self.pure_pursuit.set_path(self.current_path)
            self.pure_pursuit.start()
            self._consecutive_replan_failures = 0
            self.state = NavigatorState.EXECUTING
            self.path_pub.publish(nav_path)
        else:
            self.get_logger().error(f'Initial planning failed: {result.error_message}')
            self._consecutive_replan_failures += 1
            if self._consecutive_replan_failures >= 3:
                self.get_logger().error('Multiple failures, cancelling navigation')
                self.state = NavigatorState.IDLE
                self.current_goal = None
            else:
                self.create_timer(1.0, self._request_initial_plan, callback_group=self.callback_group)

    def _on_replan_received(self, result):
        self.replan_pending = False

        if result.status != PlanningStatus.SUCCEEDED or not result.path:
            return

        new_nav_path = result.path
        validation = self.path_validator.validate_path(new_nav_path, check_footprint=True)
        if not validation.is_valid:
            return

        new_length = self._compute_path_length(new_nav_path)
        current_remaining = self._compute_remaining_path_length()

        should_switch = False
        if new_length < current_remaining - self.path_switch_threshold:
            self.get_logger().info(f'Switching to shorter path: {new_length:.1f}m vs {current_remaining:.1f}m')
            should_switch = True
        elif self.current_nav_path is not None:
            current_validation = self.path_validator.validate_path(self.current_nav_path, check_footprint=True)
            if not current_validation.is_valid:
                self.get_logger().warn('Current path blocked, switching to new path')
                should_switch = True

        if should_switch:
            self.current_nav_path = new_nav_path
            self.current_path = self._nav_path_to_pp_path(new_nav_path)
            self.pure_pursuit.set_path(self.current_path)
            self.pure_pursuit.start()
            self.path_pub.publish(new_nav_path)

    def _check_path_for_obstacles(self):
        if self.current_nav_path is None:
            return

        lookahead_path = self._create_lookahead_segment()
        if lookahead_path is None or len(lookahead_path.poses) == 0:
            return

        result = self.path_validator.validate_path(lookahead_path, check_footprint=True)
        if not result.is_valid and not self.replan_pending:
            self.get_logger().warn(
                f'Obstacle at ({result.blocked_position[0]:.1f}, {result.blocked_position[1]:.1f}), replanning')
            self._request_replan()

    def _create_lookahead_segment(self) -> Optional[Path]:
        if self.current_nav_path is None:
            return None

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(self.current_nav_path.poses):
            dx = pose.pose.position.x - self.robot_x
            dy = pose.pose.position.y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Add poses up to lookahead distance
        accumulated_dist = 0.0
        prev_x, prev_y = self.robot_x, self.robot_y

        for i in range(closest_idx, len(self.current_nav_path.poses)):
            pose = self.current_nav_path.poses[i]
            dx = pose.pose.position.x - prev_x
            dy = pose.pose.position.y - prev_y
            accumulated_dist += math.sqrt(dx*dx + dy*dy)
            if accumulated_dist > self.obstacle_lookahead:
                break
            path.poses.append(pose)
            prev_x = pose.pose.position.x
            prev_y = pose.pose.position.y

        return path

    def _nav_path_to_pp_path(self, nav_path: Path) -> PurePursuitPath:
        pp_path = PurePursuitPath()
        for pose in nav_path.poses:
            pp_path.add_point(pose.pose.position.x, pose.pose.position.y, speed=self.target_speed)
        return pp_path

    def _compute_path_length(self, nav_path: Path) -> float:
        if len(nav_path.poses) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(nav_path.poses)):
            p1 = nav_path.poses[i-1].pose.position
            p2 = nav_path.poses[i].pose.position
            total += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return total

    def _compute_remaining_path_length(self) -> float:
        if self.current_path is None or len(self.current_path) < 2:
            return 0.0

        min_dist = float('inf')
        closest_idx = 0
        for i, pt in enumerate(self.current_path.points):
            dx = pt.x - self.robot_x
            dy = pt.y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        total = min_dist
        for i in range(closest_idx, len(self.current_path) - 1):
            p1 = self.current_path[i]
            p2 = self.current_path[i + 1]
            total += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return total

    def _publish_visualization(self):
        markers = MarkerArray()

        if self.current_goal is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'odom'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'goal'
            goal_marker.id = 0
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            goal_marker.pose = self.current_goal.pose
            goal_marker.scale.x = 1.0
            goal_marker.scale.y = 1.0
            goal_marker.scale.z = 0.1
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            markers.markers.append(goal_marker)

        state_marker = Marker()
        state_marker.header.frame_id = 'odom'
        state_marker.header.stamp = self.get_clock().now().to_msg()
        state_marker.ns = 'state'
        state_marker.id = 0
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        state_marker.pose.position.x = self.robot_x
        state_marker.pose.position.y = self.robot_y
        state_marker.pose.position.z = 2.0
        state_marker.scale.z = 0.5
        state_marker.color.r = 1.0
        state_marker.color.g = 1.0
        state_marker.color.b = 1.0
        state_marker.color.a = 1.0
        state_marker.text = self.state.name
        markers.markers.append(state_marker)

        self.markers_pub.publish(markers)

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = DynamicNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
