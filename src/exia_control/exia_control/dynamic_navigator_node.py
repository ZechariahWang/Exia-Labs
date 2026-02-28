#!/usr/bin/env python3
import math
import re
import threading
import time
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

import tf2_ros
from tf2_ros import TransformException

from exia_msgs.msg import NavigationGoal

from exia_control.planning.pure_pursuit import (
    PurePursuitController, PurePursuitConfig, Path as PurePursuitPath
)
from exia_control.navigation.path_validator import PathValidator
from exia_control.navigation.planner_interface import (
    PlannerInterface, PlanningResult, PlanningStatus
)

# for scan vis:
# rviz2 -d ~/exia_ws/install/exia_bringup/share/exia_bringup/rviz/exia_3d.rviz 

TARGET_POINT = [22, 24] # if u dont wanna use gps coord

# for hardware, just need to change these vals
# for sims update exia_world.sdf and thre gps_params_yaml as well

USE_GPS_MODE = True
TARGET_GPS = [49.666667, 11.841389]  # 49°40'00"N 11°50'29"E germany hill loc i think
ORIGIN_GPS = [49.666400, 11.841100]

# this is for the foxglove:
# ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# need this for later on jetson: sudo apt install ros-humble-velodyne

#   Go to local coordinates                                                                                                                                                           
#   ros2 run exia_control nav_to xy 30.0 15.0
                                                                                                                                                                                      
#   Go to GPS coordinates                                                                                                                                                             
#   ros2 run exia_control nav_to latlon 49.666667 11.841389                                                                                                                             

#   GPS with custom origin
#   ros2 run exia_control nav_to latlon 49.666667 11.841389 --origin 49.6664 11.8411

#   DMS format
#   ros2 run exia_control nav_to dms "49D40'00\"N" "11D50'29\"E"

#   Cancel current navigation
#   ros2 run exia_control nav_to cancel

# convert lat long to x,y
def gps_to_local(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple:
    lat0_rad = math.radians(origin_lat)
    x = (lon - origin_lon) * math.cos(lat0_rad) * 111320.0
    y = (lat - origin_lat) * 111320.0
    return x, y

# helper func
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

# turn dms to lat long
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
    DIRECT_REVERSING = 4
    MOVE_FORWARD = 5
    MOVE_TURN = 6

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
        self.declare_parameter('auto_start', False)

        _dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('use_gps_waypoint', USE_GPS_MODE, _dyn)
        self.declare_parameter('target_lat', TARGET_GPS[0], _dyn)
        self.declare_parameter('target_lon', TARGET_GPS[1], _dyn)
        self.declare_parameter('origin_lat', ORIGIN_GPS[0], _dyn)
        self.declare_parameter('origin_lon', ORIGIN_GPS[1], _dyn)
        self.declare_parameter('target_x', float(TARGET_POINT[0]), _dyn)
        self.declare_parameter('target_y', float(TARGET_POINT[1]), _dyn)

        self.replan_period = self.get_parameter('replan_period').value
        self.min_replan_interval = self.get_parameter('min_replan_interval').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value
        self.path_switch_threshold = self.get_parameter('path_switch_threshold').value
        self.obstacle_lookahead = self.get_parameter('obstacle_lookahead').value
        control_rate = self.get_parameter('control_rate').value

        self._nav_lock = threading.Lock()
        self._pose_lock = threading.Lock()

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
        self._direct_mode = False

        self._waypoint_queue = []
        self._final_goal = None
        self._final_goal_gps_x = 0.0
        self._final_goal_gps_y = 0.0
        self._final_goal_uses_gps = False
        self._waypoint_segment_distance = 35.0
        self._waypoint_tolerance = 5.0
        self._waypoint_index = 0

        self._last_scan_time = time.time()
        self._last_odom_time = time.time()
        self._sensor_timeout = 2.0
        self._sensor_warned = False
        self._latest_scan: Optional[LaserScan] = None

        self._move_type = ''
        self._move_value = 0.0
        self._move_speed = 0.0
        self._move_start_time = 0.0
        self._move_start_x = 0.0
        self._move_start_y = 0.0
        self._move_target_heading = 0.0

        # configure pp controller
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

        # create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._costmap_callback, costmap_qos)
        self.nav_goal_sub = self.create_subscription(
            NavigationGoal, '/navigation/goal', self._nav_goal_callback, 10)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, scan_qos)

        self._gps_x = 0.0
        self._gps_y = 0.0
        self._gps_available = False
        self._gps_to_map_offset_x = 0.0
        self._gps_to_map_offset_y = 0.0
        self._goal_gps_x = 0.0
        self._goal_gps_y = 0.0
        self._goal_uses_gps = False
        self.gps_odom_sub = self.create_subscription(
            Odometry, '/gps/odom', self._gps_odom_callback, 10)

        # create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)

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

        self.use_gps_waypoint = bool(self.get_parameter('use_gps_waypoint').value)
        self.target_lat = float(self.get_parameter('target_lat').value)
        self.target_lon = float(self.get_parameter('target_lon').value)
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)

        # decide whether to use global lat long points or local xy
        if self.use_gps_waypoint:
            self.target_x, self.target_y = gps_to_local(
                self.target_lat, self.target_lon,
                self.origin_lat, self.origin_lon
            )
            self.get_logger().info(
                f'GPS waypoint mode: ({self.target_lat:.6f}, {self.target_lon:.6f}) -> '
                f'local ({self.target_x:.2f}, {self.target_y:.2f})'
            )

        # init prereqs
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

    # added this bc there is a race con at the beginning of initm, so we need a delay for costmap to stabilize
    def _one_shot_timer(self, period, callback):
        def wrapper():
            timer.cancel()
            timer.destroy()
            callback()
        timer = self.create_timer(period, wrapper, callback_group=self.callback_group)
        return timer

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

    def _scan_callback(self, msg: LaserScan):
        self._last_scan_time = time.time()
        with self._pose_lock:
            self._latest_scan = msg

    def _gps_odom_callback(self, msg: Odometry):
        with self._pose_lock:
            self._gps_x = msg.pose.pose.position.x
            self._gps_y = msg.pose.pose.position.y
            if not self._gps_available:
                self._gps_available = True
                self.get_logger().info(
                    f'GPS reference active: ({self._gps_x:.2f}, {self._gps_y:.2f})')
            self._gps_to_map_offset_x = self.robot_x - self._gps_x
            self._gps_to_map_offset_y = self.robot_y - self._gps_y

    def _odom_callback(self, msg: Odometry):
        self._last_odom_time = time.time()

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            ryaw = self._quaternion_to_yaw(transform.transform.rotation)
        except TransformException:
            rx = msg.pose.pose.position.x
            ry = msg.pose.pose.position.y
            ryaw = self._quaternion_to_yaw(msg.pose.pose.orientation)

        with self._pose_lock:
            self.robot_x = rx
            self.robot_y = ry
            self.robot_yaw = ryaw
            self.robot_speed = msg.twist.twist.linear.x

    def _goal_callback(self, msg: PoseStamped):
        with self._nav_lock:
            self.current_goal = msg
            self.get_logger().info(f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

            self.current_path = None
            self.current_nav_path = None
            self._consecutive_replan_failures = 0
            self.state = NavigatorState.PLANNING
        self._request_initial_plan()

    def _goal_pose_callback(self, msg: PoseStamped):
        self._direct_mode = False
        self._goal_uses_gps = False
        self._goal_callback(msg)

    def _nav_goal_callback(self, msg: NavigationGoal):
        coord_type = msg.coord_type.strip().lower()

        if coord_type == 'xy':
            target_x, target_y = msg.x, msg.y
            self.get_logger().info(f'Navigation goal (xy): ({target_x:.2f}, {target_y:.2f})')

        elif coord_type == 'latlon':
            o_lat = msg.origin_lat if msg.origin_lat != 0.0 else self.origin_lat
            o_lon = msg.origin_lon if msg.origin_lon != 0.0 else self.origin_lon
            target_x, target_y = gps_to_local(msg.lat, msg.lon, o_lat, o_lon)
            self.get_logger().info(
                f'Navigation goal (latlon): ({msg.lat:.6f}, {msg.lon:.6f}) -> ({target_x:.2f}, {target_y:.2f})')

        elif coord_type == 'dms':
            lat = dms_to_decimal(msg.lat_dms)
            lon = dms_to_decimal(msg.lon_dms)
            o_lat = msg.origin_lat if msg.origin_lat != 0.0 else self.origin_lat
            o_lon = msg.origin_lon if msg.origin_lon != 0.0 else self.origin_lon
            target_x, target_y = gps_to_local(lat, lon, o_lat, o_lon)
            self.get_logger().info(
                f'Navigation goal (dms): ({lat:.6f}, {lon:.6f}) -> ({target_x:.2f}, {target_y:.2f})')

        elif coord_type == 'forward':
            with self._nav_lock:
                self._move_type = msg.move_type
                self._move_value = msg.move_value
                self._move_speed = msg.move_speed
                self._move_start_time = time.time()
                self._move_start_x = self.robot_x
                self._move_start_y = self.robot_y
                self.current_goal = None
                self.current_path = None
                self.current_nav_path = None
                self._direct_mode = False
                self.state = NavigatorState.MOVE_FORWARD
                self.get_logger().info(
                    f'Move forward: {self._move_type}={self._move_value:.1f}, '
                    f'speed={self._move_speed:.1f} m/s')
            return

        elif coord_type == 'turn':
            with self._nav_lock:
                self._move_type = msg.move_type
                self._move_value = msg.move_value
                self._move_speed = msg.move_speed
                self._move_start_time = time.time()
                if self._move_type == 'heading':
                    target = self.robot_yaw + math.radians(msg.move_value)
                    while target > math.pi:
                        target -= 2.0 * math.pi
                    while target < -math.pi:
                        target += 2.0 * math.pi
                    self._move_target_heading = target
                self.current_goal = None
                self.current_path = None
                self.current_nav_path = None
                self._direct_mode = False
                self.state = NavigatorState.MOVE_TURN
                if self._move_type == 'heading':
                    self.get_logger().info(
                        f'Turn {msg.move_value:.1f} deg (current: {math.degrees(self.robot_yaw):.1f}, '
                        f'target: {math.degrees(self._move_target_heading):.1f}), '
                        f'forward speed={self._move_speed:.1f} m/s')
                else:
                    self.get_logger().info(
                        f'Timed turn: {self._move_value:.1f}s at {self._move_speed:.2f} rad/s')
            return

        else:
            self.get_logger().error(f'Unknown coord_type: "{coord_type}". Use "xy", "latlon", "dms", "forward", or "turn".')
            return

        self._goal_gps_x = target_x
        self._goal_gps_y = target_y
        self._goal_uses_gps = self._gps_available

        if self._goal_uses_gps:
            map_x = target_x + self._gps_to_map_offset_x
            map_y = target_y + self._gps_to_map_offset_y
            self.get_logger().info(
                f'GPS->map: ({target_x:.2f}, {target_y:.2f}) -> ({map_x:.2f}, {map_y:.2f})')
        else:
            map_x = target_x
            map_y = target_y

        self._direct_mode = msg.direct
        if self._direct_mode:
            self.get_logger().info('Direct navigation mode: no replanning, stop on obstacle')
            self._publish_status('NAVIGATING')

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = map_x
        goal.pose.position.y = map_y
        goal.pose.orientation.w = 1.0

        self._final_goal = goal
        self._final_goal_gps_x = target_x
        self._final_goal_gps_y = target_y
        self._final_goal_uses_gps = self._goal_uses_gps

        if not self._direct_mode:
            self._waypoint_queue = self._generate_waypoints(map_x, map_y)
            self._waypoint_index = 0
            if self._waypoint_queue:
                total_dist = math.sqrt((map_x - self.robot_x)**2 + (map_y - self.robot_y)**2)
                self.get_logger().info(
                    f'Waypoint queue: {len(self._waypoint_queue)} intermediates, '
                    f'total {total_dist:.1f}m')
                self._goal_callback(self._waypoint_queue[0])
                return
        else:
            self._waypoint_queue = []
            self._waypoint_index = 0

        self._goal_callback(goal)

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
        self._direct_mode = False
        self._move_type = ''
        self._waypoint_queue = []
        self._waypoint_index = 0
        self._final_goal = None
        self._publish_stop()
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    # decided data publish and stop depending on nav curr state
    def _check_sensor_health(self) -> bool:
        now = time.time()
        scan_age = now - self._last_scan_time
        odom_age = now - self._last_odom_time

        if scan_age > self._sensor_timeout or odom_age > self._sensor_timeout:
            if not self._sensor_warned:
                stale = []
                if scan_age > self._sensor_timeout:
                    stale.append(f'scan ({scan_age:.1f}s)')
                if odom_age > self._sensor_timeout:
                    stale.append(f'odom ({odom_age:.1f}s)')
                self.get_logger().error(f'Sensor timeout: {", ".join(stale)} - stopping')
                self._sensor_warned = True
            return False

        if self._sensor_warned:
            self.get_logger().info('Sensors recovered')
            self._sensor_warned = False
        return True

    def _control_loop(self):
        with self._pose_lock:
            robot_x = self.robot_x
            robot_y = self.robot_y
            robot_yaw = self.robot_yaw
            robot_speed = self.robot_speed
            gps_x = self._gps_x
            gps_y = self._gps_y
            gps_available = self._gps_available
            gps_offset_x = self._gps_to_map_offset_x
            gps_offset_y = self._gps_to_map_offset_y
            latest_scan = self._latest_scan

        with self._nav_lock:
            self.robot_x = robot_x
            self.robot_y = robot_y
            self.robot_yaw = robot_yaw
            self.robot_speed = robot_speed
            self._gps_x = gps_x
            self._gps_y = gps_y
            self._gps_available = gps_available
            self._gps_to_map_offset_x = gps_offset_x
            self._gps_to_map_offset_y = gps_offset_y
            self._latest_scan = latest_scan

            if not self._check_sensor_health():
                self._publish_stop()
                return

            if self.state == NavigatorState.IDLE:
                self._publish_stop()
            elif self.state == NavigatorState.PLANNING:
                self._publish_stop()
            elif self.state == NavigatorState.EXECUTING:
                self._execute_path()
            elif self.state == NavigatorState.RECOVERING:
                self._execute_recovery()
            elif self.state == NavigatorState.DIRECT_REVERSING:
                self._execute_direct_reverse()
            elif self.state == NavigatorState.MOVE_FORWARD:
                self._execute_move_forward()
            elif self.state == NavigatorState.MOVE_TURN:
                self._execute_move_turn()

    def _check_ahead_for_obstacle(self) -> bool:
        scan = self._latest_scan
        if scan is None:
            return False

        stop_distance = 3.0
        cone_half_angle = math.radians(30.0)

        min_range = float('inf')
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue
            beam_angle = scan.angle_min + i * scan.angle_increment
            if abs(beam_angle) <= cone_half_angle and r < min_range:
                min_range = r

        if min_range <= stop_distance:
            self._publish_stop()
            self.get_logger().warn(
                f'DIRECT NAV STOPPED: Obstacle detected at {min_range:.2f}m ahead')
            self.get_logger().info(
                f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                f'heading: {math.degrees(self.robot_yaw):.1f} deg')
            self.current_goal = None
            self.current_path = None
            self.current_nav_path = None
            self._direct_reverse_start_x = self.robot_x
            self._direct_reverse_start_y = self.robot_y
            self._direct_reverse_phase = 'straighten'
            self._direct_reverse_timer = time.time()
            self.state = NavigatorState.DIRECT_REVERSING
            return True
        return False

    def _execute_direct_reverse(self):
        if self._direct_reverse_phase == 'straighten':
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            if time.time() - self._direct_reverse_timer > 0.5:
                self._direct_reverse_phase = 'reverse'
                self._direct_reverse_start_x = self.robot_x
                self._direct_reverse_start_y = self.robot_y
            return

        if self._direct_reverse_phase == 'reverse':
            dx = self.robot_x - self._direct_reverse_start_x
            dy = self.robot_y - self._direct_reverse_start_y
            dist_reversed = math.sqrt(dx * dx + dy * dy)
            if dist_reversed >= 1.5:
                self._publish_stop()
                self.get_logger().info(
                    f'Reversed {dist_reversed:.2f}m. '
                    f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                    f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                self._publish_status('OBSTACLE_STOPPED')
                self.state = NavigatorState.IDLE
                self._direct_mode = False
                return
            cmd = Twist()
            cmd.linear.x = -0.5
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

    def _check_forward_obstacle(self) -> bool:
        scan = self._latest_scan
        if scan is None:
            return False

        stop_distance = 3.0
        cone_half_angle = math.radians(30.0)

        min_range = float('inf')
        for i, r in enumerate(scan.ranges):
            if r < scan.range_min or r > scan.range_max or math.isinf(r) or math.isnan(r):
                continue
            beam_angle = scan.angle_min + i * scan.angle_increment
            if abs(beam_angle) <= cone_half_angle and r < min_range:
                min_range = r

        return min_range <= stop_distance

    def _execute_move_forward(self):
        if self._check_forward_obstacle():
            self._publish_stop()
            self.get_logger().warn(
                f'Forward stopped: obstacle detected. '
                f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                f'heading: {math.degrees(self.robot_yaw):.1f} deg')
            self._publish_status('OBSTACLE_STOPPED')
            self._move_type = ''
            self.state = NavigatorState.IDLE
            return

        elapsed = time.time() - self._move_start_time
        dx = self.robot_x - self._move_start_x
        dy = self.robot_y - self._move_start_y
        dist_traveled = math.sqrt(dx * dx + dy * dy)

        done = False
        if self._move_type == 'time' and elapsed >= self._move_value:
            done = True
        elif self._move_type == 'distance' and dist_traveled >= self._move_value:
            done = True

        if done:
            self._publish_stop()
            self.get_logger().info(
                f'Forward complete: {dist_traveled:.2f}m in {elapsed:.1f}s. '
                f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                f'heading: {math.degrees(self.robot_yaw):.1f} deg')
            self._publish_status('MOVE_COMPLETE')
            self._move_type = ''
            self.state = NavigatorState.IDLE
            return

        cmd = Twist()
        cmd.linear.x = self._move_speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def _execute_move_turn(self):
        elapsed = time.time() - self._move_start_time

        if self._move_type == 'time':
            if elapsed >= self._move_value:
                self._publish_stop()
                self.get_logger().info(
                    f'Timed turn complete: {elapsed:.1f}s. '
                    f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                    f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                self._publish_status('TURN_COMPLETE')
                self._move_type = ''
                self.state = NavigatorState.IDLE
                return
            cmd = Twist()
            cmd.linear.x = 1.0
            cmd.angular.z = self._move_speed
            self.cmd_vel_pub.publish(cmd)

        elif self._move_type == 'heading':
            error = math.atan2(
                math.sin(self._move_target_heading - self.robot_yaw),
                math.cos(self._move_target_heading - self.robot_yaw))

            if abs(error) < math.radians(3.0):
                self._publish_stop()
                self.get_logger().info(
                    f'Heading reached: {math.degrees(self.robot_yaw):.1f} deg '
                    f'(target: {math.degrees(self._move_target_heading):.1f} deg). '
                    f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f})')
                self._publish_status('TURN_COMPLETE')
                self._move_type = ''
                self.state = NavigatorState.IDLE
                return

            kp = 2.0
            cmd = Twist()
            cmd.linear.x = self._move_speed
            cmd.angular.z = max(min(kp * error, 1.0), -1.0)
            self.cmd_vel_pub.publish(cmd)

    def _execute_path(self):
        if self.current_path is None or self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        if self._direct_mode and self._check_ahead_for_obstacle():
            return

        if self._goal_uses_gps and self._gps_available:
            dist_to_goal = math.sqrt(
                (self._goal_gps_x - self._gps_x)**2 +
                (self._goal_gps_y - self._gps_y)**2)
        else:
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y
            dist_to_goal = math.sqrt(
                (goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2)

        if self._final_goal_uses_gps and self._gps_available and self._final_goal is not None:
            dist_to_final = math.sqrt(
                (self._final_goal_gps_x - self._gps_x)**2 +
                (self._final_goal_gps_y - self._gps_y)**2)
        elif self._final_goal is not None:
            dist_to_final = math.sqrt(
                (self._final_goal.pose.position.x - self.robot_x)**2 +
                (self._final_goal.pose.position.y - self.robot_y)**2)
        else:
            dist_to_final = dist_to_goal

        is_intermediate = self._waypoint_index < len(self._waypoint_queue)
        current_tolerance = self._waypoint_tolerance if is_intermediate else self.goal_tolerance

        if dist_to_final < self.goal_tolerance:
            self.get_logger().info(f'Final goal reached! Distance: {dist_to_final:.2f}m')
            self._publish_status('GOAL_REACHED')
            self._waypoint_queue = []
            self._waypoint_index = 0
            self._final_goal = None
            self._direct_mode = False
            self.state = NavigatorState.IDLE
            self.current_goal = None
            self.current_path = None
            self._publish_stop()
            return

        if dist_to_goal < current_tolerance:
            if is_intermediate:
                self.get_logger().info(
                    f'Waypoint {self._waypoint_index + 1}/{len(self._waypoint_queue)} reached '
                    f'(dist {dist_to_goal:.2f}m)')
                self._waypoint_index += 1
                if self._waypoint_index < len(self._waypoint_queue):
                    self._advance_to_waypoint(self._waypoint_queue[self._waypoint_index])
                else:
                    self._advance_to_final_goal()
                return
            else:
                self.get_logger().info(f'Goal reached! Distance: {dist_to_goal:.2f}m')
                if self._direct_mode:
                    self.get_logger().info(
                        f'Final position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                        f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                    self._direct_mode = False
                self._publish_status('GOAL_REACHED')
                self._waypoint_queue = []
                self._waypoint_index = 0
                self._final_goal = None
                self.state = NavigatorState.IDLE
                self.current_goal = None
                self.current_path = None
                self._publish_stop()
                return

        cmd, goal_reached = self.pure_pursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw, self.robot_speed)

        if goal_reached:
            if dist_to_goal < current_tolerance * 2.0:
                if is_intermediate:
                    self.get_logger().info(
                        f'Waypoint {self._waypoint_index + 1}/{len(self._waypoint_queue)} reached via path end')
                    self._waypoint_index += 1
                    if self._waypoint_index < len(self._waypoint_queue):
                        self._advance_to_waypoint(self._waypoint_queue[self._waypoint_index])
                    else:
                        self._advance_to_final_goal()
                    return
                self.get_logger().info('Path complete, goal reached!')
                self._publish_status('GOAL_REACHED')
                self._waypoint_queue = []
                self._waypoint_index = 0
                self._final_goal = None
                self.state = NavigatorState.IDLE
                self.current_goal = None
                self._publish_stop()
                return
            else:
                if self._direct_mode:
                    self.get_logger().info(
                        f'Direct path ended, goal not reached. '
                        f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                        f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                    self._publish_status('PATH_ENDED')
                    self.state = NavigatorState.IDLE
                    self.current_goal = None
                    self.current_path = None
                    self._direct_mode = False
                    self._waypoint_queue = []
                    self._waypoint_index = 0
                    self._final_goal = None
                    self._publish_stop()
                    return
                self.get_logger().info('Path ended but goal not reached, replanning...')
                self._request_replan()

        self.cmd_vel_pub.publish(cmd)

    # this is a little scuffed, ideally we never should reach this point if the A* recalculates properly.
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

    # replan the a* every 500ms, with an interval of replanning of 200ms 
    def _periodic_replan(self):
        if self._direct_mode:
            return
        if self.state != NavigatorState.EXECUTING or self.current_goal is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.min_replan_interval:
            return

        if not self.replan_pending:
            self._request_replan()

    def _update_map_goal(self):
        if self._goal_uses_gps and self._gps_available and self.current_goal is not None:
            self.current_goal.pose.position.x = self._goal_gps_x + self._gps_to_map_offset_x
            self.current_goal.pose.position.y = self._goal_gps_y + self._gps_to_map_offset_y

    def _generate_waypoints(self, goal_x, goal_y):
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= self._waypoint_segment_distance:
            return []
        num_segments = int(dist / self._waypoint_segment_distance)
        waypoints = []
        for i in range(1, num_segments + 1):
            t = (i * self._waypoint_segment_distance) / dist
            if t >= 1.0:
                break
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.header.stamp = self.get_clock().now().to_msg()
            wp.pose.position.x = self.robot_x + t * dx
            wp.pose.position.y = self.robot_y + t * dy
            wp.pose.orientation.w = 1.0
            waypoints.append(wp)
        return waypoints

    def _advance_to_waypoint(self, wp):
        if self._final_goal_uses_gps and self._gps_available and self._final_goal is not None:
            final_map_x = self._final_goal_gps_x + self._gps_to_map_offset_x
            final_map_y = self._final_goal_gps_y + self._gps_to_map_offset_y
            self._final_goal.pose.position.x = final_map_x
            self._final_goal.pose.position.y = final_map_y
            remaining_wps = self._generate_waypoints(final_map_x, final_map_y)
            self._waypoint_queue = remaining_wps
            self._waypoint_index = 0
            if self._waypoint_queue:
                wp = self._waypoint_queue[0]
            else:
                self._advance_to_final_goal()
                return
        self._goal_gps_x = 0.0
        self._goal_gps_y = 0.0
        self._goal_uses_gps = False
        self.current_goal = wp
        self.current_path = None
        self.current_nav_path = None
        self._consecutive_replan_failures = 0
        self.state = NavigatorState.PLANNING
        self.get_logger().info(
            f'Advancing to waypoint ({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})')
        self._publish_status('NAVIGATING')
        self._request_initial_plan()

    def _advance_to_final_goal(self):
        if self._final_goal is None:
            self.state = NavigatorState.IDLE
            return
        if self._final_goal_uses_gps and self._gps_available:
            self._final_goal.pose.position.x = self._final_goal_gps_x + self._gps_to_map_offset_x
            self._final_goal.pose.position.y = self._final_goal_gps_y + self._gps_to_map_offset_y
        self.current_goal = self._final_goal
        self._goal_gps_x = self._final_goal_gps_x
        self._goal_gps_y = self._final_goal_gps_y
        self._goal_uses_gps = self._final_goal_uses_gps
        self.current_path = None
        self.current_nav_path = None
        self._consecutive_replan_failures = 0
        self.state = NavigatorState.PLANNING
        self.get_logger().info(
            f'Advancing to final goal ({self.current_goal.pose.position.x:.2f}, '
            f'{self.current_goal.pose.position.y:.2f})')
        self._publish_status('NAVIGATING')
        self._request_initial_plan()

    def _get_report_position(self):
        if self._gps_available:
            return self._gps_x, self._gps_y
        return self.robot_x, self.robot_y

    def _clamp_planning_goal(self, goal: PoseStamped) -> PoseStamped:
        if self.costmap is not None:
            half_w = (self.costmap.info.width * self.costmap.info.resolution) / 2.0
            half_h = (self.costmap.info.height * self.costmap.info.resolution) / 2.0
            max_dist = min(half_w, half_h) - 5.0
        else:
            max_dist = 45.0

        dx = goal.pose.position.x - self.robot_x
        dy = goal.pose.position.y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= max_dist:
            return goal

        scale = max_dist / dist
        clamped = PoseStamped()
        clamped.header = goal.header
        clamped.pose.position.x = self.robot_x + dx * scale
        clamped.pose.position.y = self.robot_y + dy * scale
        clamped.pose.orientation = goal.pose.orientation
        self.get_logger().info(
            f'Goal clamped: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) -> '
            f'({clamped.pose.position.x:.2f}, {clamped.pose.position.y:.2f}) '
            f'(dist {dist:.1f}m > costmap {max_dist:.1f}m)')
        return clamped

    def _request_initial_plan(self):
        if self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        self._update_map_goal()

        if self._direct_mode:
            self._plan_straight_line()
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        planning_goal = self._clamp_planning_goal(self.current_goal)

        self.get_logger().info(
            f'Planning: ({self.robot_x:.2f}, {self.robot_y:.2f}) -> '
            f'({planning_goal.pose.position.x:.2f}, {planning_goal.pose.position.y:.2f})')

        self.replan_pending = True
        self.planner.plan_path_async(start, planning_goal, self._on_initial_path)

    def _plan_straight_line(self):
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        yaw = math.atan2(dy, dx)

        nav_path = Path()
        nav_path.header.frame_id = 'map'
        nav_path.header.stamp = self.get_clock().now().to_msg()

        num_points = max(int(dist / 0.5), 2)
        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = self.robot_x + t * dx
            pose.pose.position.y = self.robot_y + t * dy
            pose.pose.orientation = self._yaw_to_quaternion(yaw)
            nav_path.poses.append(pose)

        self.get_logger().info(
            f'Direct path: ({self.robot_x:.2f}, {self.robot_y:.2f}) -> '
            f'({goal_x:.2f}, {goal_y:.2f}), {len(nav_path.poses)} pts, {dist:.1f}m')

        result = PlanningResult(status=PlanningStatus.SUCCEEDED, path=nav_path)
        self._on_initial_path(result)

    # equest replan every 500ms
    def _request_replan(self):
        if self.current_goal is None:
            return

        self._update_map_goal()

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < 0.5:
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        planning_goal = self._clamp_planning_goal(self.current_goal)

        self.replan_pending = True
        self.last_replan_time = now
        self.planner.plan_path_async(start, planning_goal, self._on_replan_received)


    def _on_initial_path(self, result):
        with self._nav_lock:
            self.replan_pending = False

            if result.status == PlanningStatus.SUCCEEDED and result.path:
                nav_path = result.path
                self.get_logger().info(f'Initial path: {len(nav_path.poses)} poses')

                validation = self.path_validator.validate_path(nav_path, check_footprint=True)
                if not validation.is_valid:
                    self.get_logger().warn(f'Initial path collision at index {validation.blocked_index}')
                    if self._direct_mode:
                        self._publish_stop()
                        self.get_logger().warn(
                            f'DIRECT NAV STOPPED: Path blocked at '
                            f'({validation.blocked_position[0]:.2f}, {validation.blocked_position[1]:.2f})')
                        self.get_logger().info(
                            f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                            f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                        self._publish_status('OBSTACLE_STOPPED')
                        self.state = NavigatorState.IDLE
                        self.current_goal = None
                        self._direct_mode = False
                        return
                    self._consecutive_replan_failures += 1
                    if self._consecutive_replan_failures >= 3:
                        self.get_logger().error('Multiple failures, entering recovery')
                        self.state = NavigatorState.RECOVERING
                    else:
                        self._one_shot_timer(0.5, self._request_initial_plan)
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
                    self._one_shot_timer(2.0, self._request_initial_plan)

    def _on_replan_received(self, result):
        with self._nav_lock:
            self.replan_pending = False
            if self._direct_mode:
                return

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
                if self._direct_mode:
                    self._publish_stop()
                    self.get_logger().warn(
                        f'DIRECT NAV STOPPED: Obstacle at '
                        f'({result.blocked_position[0]:.2f}, {result.blocked_position[1]:.2f}) '
                        f'dist={obstacle_dist:.1f}m')
                    self.get_logger().info(
                        f'Robot position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                        f'heading: {math.degrees(self.robot_yaw):.1f} deg')
                    self.current_goal = None
                    self.current_path = None
                    self.current_nav_path = None
                    self._direct_reverse_start_x = self.robot_x
                    self._direct_reverse_start_y = self.robot_y
                    self._direct_reverse_phase = 'straighten'
                    self._direct_reverse_timer = time.time()
                    self.state = NavigatorState.DIRECT_REVERSING
                    return
                self.get_logger().warn(
                    f'Obstacle at ({result.blocked_position[0]:.1f}, {result.blocked_position[1]:.1f}) '
                    f'dist={obstacle_dist:.1f}m, replanning')
                self._request_replan()

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

    # compute dist
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

    # this lowkey doesnt work at all
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

    def _publish_status(self, status_type: str):
        rx, ry = self._get_report_position()
        msg = String()
        msg.data = f'{status_type} {rx:.2f} {ry:.2f} {math.degrees(self.robot_yaw):.1f}'
        self.status_pub.publish(msg)

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
