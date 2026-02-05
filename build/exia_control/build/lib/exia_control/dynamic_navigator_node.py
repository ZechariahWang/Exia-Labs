#!/usr/bin/env python3
import math
import re
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

from exia_control.planning.pure_pursuit import (
    PurePursuitController, PurePursuitConfig, Path as PurePursuitPath
)
from exia_control.navigation.path_validator import PathValidator
from exia_control.navigation.planner_interface import (
    PlannerInterface, PlanningStatus
)

# rviz2 -d ~/exia_ws/install/exia_bringup/share/exia_bringup/rviz/exia_3d.rviz

TARGET_POINT = [22, 24]

# for hardware, just need to change these vals
# for sims update exia_world.sdf as well

USE_GPS_MODE = True
TARGET_GPS = [49.666667, 11.841389]  # 49°40'00"N 11°50'29"E
ORIGIN_GPS = [49.666400, 11.841100]


def gps_to_local(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple:
    lat0_rad = math.radians(origin_lat)
    x = (lon - origin_lon) * math.cos(lat0_rad) * 111320.0
    y = (lat - origin_lat) * 111320.0
    return x, y


def dms_to_decimal(dms_str: str) -> float:
    dms_str = dms_str.strip().upper()
    pattern = r"(\d+)[°D]\s*(\d+)['\u2032M]\s*(\d+(?:\.\d+)?)[\"″\u2033S]?\s*([NSEW])"
    match = re.match(pattern, dms_str)
    if match:
        degrees = float(match.group(1))
        minutes = float(match.group(2))
        seconds = float(match.group(3))
        direction = match.group(4)
        decimal = degrees + (minutes / 60.0) + (seconds / 3600.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal
    pattern_simple = r"(\d+)[°D]\s*(\d+(?:\.\d+)?)['\u2032M]\s*([NSEW])"
    match = re.match(pattern_simple, dms_str)
    if match:
        degrees = float(match.group(1))
        minutes = float(match.group(2))
        direction = match.group(3)
        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal
    try:
        return float(dms_str)
    except ValueError:
        raise ValueError(f"Cannot parse DMS string: {dms_str}")


def parse_dms_coordinates(coord_str: str) -> Tuple[float, float]:
    coord_str = coord_str.strip()
    parts = re.split(r'[,\s]+(?=\d)', coord_str, maxsplit=1)
    if len(parts) == 2:
        lat_str, lon_str = parts
    else:
        match = re.match(r"(.+[NS])\s*(.+[EW])", coord_str, re.IGNORECASE)
        if match:
            lat_str = match.group(1)
            lon_str = match.group(2)
        else:
            raise ValueError(f"Cannot parse coordinate pair: {coord_str}")
    lat = dms_to_decimal(lat_str)
    lon = dms_to_decimal(lon_str)
    return lat, lon


class NavigatorState(Enum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    RECOVERING = 3


class DynamicNavigator(Node):

    def __init__(self):
        super().__init__('dynamic_navigator')

        self.declare_parameter('replan_period', 0.5)
        self.declare_parameter('min_replan_interval', 0.2)
        self.declare_parameter('goal_tolerance', 1.0)
        self.declare_parameter('lookahead_distance', 1.2)
        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('path_switch_threshold', 2.0)
        self.declare_parameter('obstacle_lookahead', 8.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('auto_start', True)

        self.declare_parameter('use_gps_waypoint', USE_GPS_MODE)
        self.declare_parameter('target_lat', TARGET_GPS[0])
        self.declare_parameter('target_lon', TARGET_GPS[1])
        self.declare_parameter('origin_lat', ORIGIN_GPS[0])
        self.declare_parameter('origin_lon', ORIGIN_GPS[1])
        self.declare_parameter('target_x', float(TARGET_POINT[0]))
        self.declare_parameter('target_y', float(TARGET_POINT[1]))

        self.replan_period = self.get_parameter('replan_period').value
        self.min_replan_interval = self.get_parameter('min_replan_interval').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.path_switch_threshold = self.get_parameter('path_switch_threshold').value
        self.obstacle_lookahead = self.get_parameter('obstacle_lookahead').value
        control_rate = self.get_parameter('control_rate').value

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

        pp_config = PurePursuitConfig(
            lookahead_distance=lookahead,
            goal_tolerance=self.goal_tolerance,
            max_linear_speed=self.target_speed * 1.5,
            min_linear_speed=0.2,
            wheelbase=1.3,
        )
        self.pure_pursuit = PurePursuitController(pp_config)

        self.path_validator = PathValidator()
        self.costmap: Optional[OccupancyGrid] = None
        self.callback_group = ReentrantCallbackGroup()
        self.planner = PlannerInterface(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._recovery_start_x = 0.0
        self._recovery_start_y = 0.0
        self._recovery_target_dist = 2.0
        self._consecutive_replan_failures = 0

        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._costmap_callback, costmap_qos)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)

        self.cancel_srv = self.create_service(
            Trigger, '/navigation/cancel', self._cancel_callback)

        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)
        self.replan_timer = self.create_timer(self.replan_period, self._periodic_replan)
        self.viz_timer = self.create_timer(0.5, self._publish_visualization)

        auto_start = self.get_parameter('auto_start').value
        self._auto_start_done = False
        self._costmap_update_count = 0
        self._min_costmap_updates = 10

        self.use_gps_waypoint = self.get_parameter('use_gps_waypoint').value
        self.target_lat = self.get_parameter('target_lat').value
        self.target_lon = self.get_parameter('target_lon').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value

        if self.use_gps_waypoint:
            self.target_x, self.target_y = gps_to_local(
                self.target_lat, self.target_lon,
                self.origin_lat, self.origin_lon
            )
            self.get_logger().info(
                f'GPS waypoint mode: ({self.target_lat:.6f}, {self.target_lon:.6f}) -> '
                f'local ({self.target_x:.2f}, {self.target_y:.2f})'
            )

        if auto_start:
            self.create_timer(2.0, self._delayed_auto_start, callback_group=self.callback_group)

        if self.use_gps_waypoint:
            self.get_logger().info(
                f'Dynamic Navigator initialized - GPS Target: ({self.target_lat:.6f}, {self.target_lon:.6f})'
            )
        else:
            self.get_logger().info(
                f'Dynamic Navigator initialized - Target: ({self.target_x:.2f}, {self.target_y:.2f})'
            )

    def _delayed_auto_start(self):
        if self._auto_start_done or self.state != NavigatorState.IDLE:
            return

        if self._costmap_update_count < self._min_costmap_updates:
            self.get_logger().info(f'Waiting for costmap to stabilize ({self._costmap_update_count}/{self._min_costmap_updates} updates)...')
            return

        if self.costmap is not None:
            obstacle_count = sum(1 for c in self.costmap.data if c > 50)
            if obstacle_count < 100:
                self.get_logger().warn(f'Costmap has few obstacles ({obstacle_count}), waiting for more sensor data...')
                return
            self.get_logger().info(f'Costmap has {obstacle_count} obstacle cells')

        if not self.planner.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Waiting for planner server...')
            return

        self._auto_start_done = True

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.target_x
        goal.pose.position.y = self.target_y
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f'Auto-starting navigation to ({self.target_x:.2f}, {self.target_y:.2f})')
        self._goal_callback(goal)

    def _odom_callback(self, msg: Odometry):
        self.robot_speed = msg.twist.twist.linear.x

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            q = transform.transform.rotation
            self.robot_yaw = self._quaternion_to_yaw(q)
        except TransformException:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)

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
        self._costmap_update_count += 1

        if self._costmap_update_count == 1:
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
        start.header.frame_id = 'map'
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

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < 0.5:
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        self.replan_pending = True
        self.last_replan_time = now
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
            if self._consecutive_replan_failures >= 10:
                self.get_logger().error('Multiple failures, cancelling navigation')
                self.state = NavigatorState.IDLE
                self.current_goal = None
            else:
                self.get_logger().info(f'Retrying planning ({self._consecutive_replan_failures}/10)...')
                self.create_timer(2.0, self._request_initial_plan, callback_group=self.callback_group)

    def _on_replan_received(self, result):
        self.replan_pending = False

        if result.status != PlanningStatus.SUCCEEDED or not result.path:
            self._consecutive_replan_failures += 1
            self.get_logger().warn(f'Replan failed ({self._consecutive_replan_failures}): {result.error_message}')
            if self._consecutive_replan_failures >= 5:
                if self.current_nav_path is not None:
                    current_validation = self.path_validator.validate_path(
                        self.current_nav_path, check_footprint=False, max_poses=20)
                    if not current_validation.is_valid:
                        self.get_logger().error('Current path blocked and replanning failed, stopping')
                        self._publish_stop()
                        self.state = NavigatorState.RECOVERING
                        self._consecutive_replan_failures = 0
            return

        self._consecutive_replan_failures = 0
        new_nav_path = result.path
        self.get_logger().info(f'Replan succeeded: {len(new_nav_path.poses)} poses')

        validation = self.path_validator.validate_path(new_nav_path, check_footprint=True)
        if not validation.is_valid:
            self.get_logger().warn(f'New path has collision at index {validation.blocked_index}')
            return

        self.get_logger().info('Switching to updated path from replanning')
        self.current_nav_path = new_nav_path
        self.current_path = self._nav_path_to_pp_path(new_nav_path)
        self.pure_pursuit.set_path(self.current_path)
        self.pure_pursuit.start()
        self.path_pub.publish(new_nav_path)

    def _check_path_for_obstacles(self):
        if self.current_nav_path is None:
            return

        if self.replan_pending:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < 0.3:
            return

        lookahead_path = self._create_lookahead_segment()
        if lookahead_path is None or len(lookahead_path.poses) < 3:
            return

        result = self.path_validator.validate_path(lookahead_path, check_footprint=True)
        if not result.is_valid:
            obstacle_dist = math.sqrt(
                (result.blocked_position[0] - self.robot_x)**2 +
                (result.blocked_position[1] - self.robot_y)**2
            )
            if obstacle_dist < 6.0:
                self.get_logger().warn(
                    f'Obstacle at ({result.blocked_position[0]:.1f}, {result.blocked_position[1]:.1f}) '
                    f'dist={obstacle_dist:.1f}m, replanning')
                self._request_replan()
                if obstacle_dist < 3.0 and self.robot_speed > 0.5:
                    cmd = Twist()
                    cmd.linear.x = max(0.3, self.robot_speed * 0.5)
                    self.cmd_vel_pub.publish(cmd)

    def _create_lookahead_segment(self) -> Optional[Path]:
        if self.current_nav_path is None:
            return None

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        min_dist = float('inf')
        closest_idx = 0
        for i, pose in enumerate(self.current_nav_path.poses):
            dx = pose.pose.position.x - self.robot_x
            dy = pose.pose.position.y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
