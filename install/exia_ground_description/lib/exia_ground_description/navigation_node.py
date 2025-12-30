#!/usr/bin/env python3
"""
Navigation Node - SLAM + A* Path Planning with Pure Pursuit Execution

ROS2 node that integrates:
- Nav2 Smac Hybrid-A* global planner for path planning
- Costmap-based obstacle detection
- Pure Pursuit local path following
- Dynamic path replanning

The robot follows a global path computed by A* while monitoring
for obstacles. When the path is blocked, it triggers replanning.

Topics:
    Subscribed:
        /odom - Robot odometry
        /goal_pose - Navigation goal (from RViz or mission planner)
        /local_costmap/costmap - Local costmap for obstacle detection
        /global_costmap/costmap - Global costmap for planning

    Published:
        /cmd_vel - Velocity commands
        /planned_path - Current path being followed
        /path_markers - Visualization markers

Services:
    /navigation/start - Start navigation to current goal
    /navigation/stop - Stop navigation
    /navigation/replan - Force replanning

Author: Zechariah Wang
Date: December 2025
"""

import math
import os
import sys
from typing import Optional

# Add package to path for module imports
_script_dir = os.path.dirname(os.path.abspath(__file__))
_scripts_dir = os.path.dirname(_script_dir)
_package_dir = os.path.dirname(_scripts_dir)
_src_dir = os.path.join(_package_dir, 'src')
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from exia_control.planning import (
    PurePursuitController,
    PurePursuitConfig,
    Path,
    PathPoint,
)
from exia_control.navigation import (
    PathValidator,
    PlannerInterface,
    PlanningStatus,
    ReplanManager,
    ReplanManagerConfig,
    CostmapMonitor,
    CostmapMonitorConfig,
)


class NavigationNode(Node):
    """
    ROS2 node for autonomous navigation with SLAM and A* pathfinding.

    Integrates global planning (Nav2 Smac Hybrid-A*) with local path
    following (Pure Pursuit) and costmap-based obstacle avoidance.
    """

    def __init__(self):
        super().__init__('navigation_node')

        # Declare parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize callback groups
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._service_cb_group = ReentrantCallbackGroup()

        # TF2 for transforming robot pose to map frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Create Pure Pursuit controller
        pp_config = PurePursuitConfig(
            lookahead_distance=self._lookahead_distance,
            goal_tolerance=self._goal_tolerance,
            max_linear_speed=self._max_speed,
            min_linear_speed=self._min_speed,
            wheelbase=self._wheelbase,
        )
        self._controller = PurePursuitController(pp_config)

        # Create navigation components
        self._path_validator = PathValidator(self._robot_footprint)

        replan_config = ReplanManagerConfig(
            replan_period=self._replan_period,
            min_replan_interval=self._min_replan_interval,
            replan_distance=self._replan_distance,
            max_path_deviation=self._max_path_deviation,
        )
        self._replan_manager = ReplanManager(replan_config)

        # Use planner_server's internal global costmap for obstacle detection
        # The planner handles obstacle avoidance, we just monitor for replanning
        costmap_config = CostmapMonitorConfig(
            local_costmap_topic='/planner_server/global_costmap/costmap',
            global_costmap_topic='/planner_server/global_costmap/costmap',
        )
        self._costmap_monitor = CostmapMonitor(
            self, costmap_config,
            costmap_callback=self._costmap_update_callback
        )
        self._costmap_available = False  # Track if costmap is ready

        # Planner interface will be created after node is ready
        self._planner_interface: Optional[PlannerInterface] = None

        # State
        self._current_pose: Optional[PoseStamped] = None
        self._current_odom: Optional[Odometry] = None
        self._current_goal: Optional[PoseStamped] = None
        self._current_path: Optional[NavPath] = None
        self._is_navigating = False
        self._goal_reached = False
        self._planning_in_progress = False

        # Planning failure tracking
        self._planning_failures = 0
        self._max_planning_failures = 5
        self._last_plan_request_time = 0.0
        self._plan_retry_delay = 2.0  # Base delay between retries

        # QoS profiles
        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self._path_pub = self.create_publisher(NavPath, '/planned_path', reliable_qos)
        self._marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        self._nav_status_pub = self.create_publisher(Bool, '/navigation/active', 10)

        # Subscribers
        self.create_subscription(
            Odometry, '/odom',
            self._odom_callback, reliable_qos
        )
        self.create_subscription(
            PoseStamped, '/goal_pose',
            self._goal_callback, reliable_qos
        )

        # Services
        self.create_service(
            Trigger, '/navigation/start',
            self._start_callback,
            callback_group=self._service_cb_group
        )
        self.create_service(
            Trigger, '/navigation/stop',
            self._stop_callback,
            callback_group=self._service_cb_group
        )
        self.create_service(
            Trigger, '/navigation/replan',
            self._replan_callback,
            callback_group=self._service_cb_group
        )

        # Control loop timer
        self.create_timer(
            0.05, self._control_loop,  # 20Hz
            callback_group=self._timer_cb_group
        )

        # Visualization timer
        self.create_timer(0.5, self._publish_visualization)

        # Status timer
        self.create_timer(1.0, self._publish_status)

        # Delayed initialization for planner interface
        self.create_timer(2.0, self._delayed_init, callback_group=self._timer_cb_group)

        self.get_logger().info('Navigation Node initialized')
        self.get_logger().info(f'  Lookahead: {self._lookahead_distance}m')
        self.get_logger().info(f'  Max speed: {self._max_speed} m/s')
        self.get_logger().info(f'  Replan period: {self._replan_period}s')

    def _declare_parameters(self):
        """Declare node parameters."""
        # Pure Pursuit parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', 0.3)

        # Robot geometry
        self.declare_parameter('wheelbase', 1.3)

        # Replanning parameters
        self.declare_parameter('replan_period', 10.0)
        self.declare_parameter('min_replan_interval', 1.0)
        self.declare_parameter('replan_distance', 5.0)
        self.declare_parameter('max_path_deviation', 2.0)

        # Path validation
        self.declare_parameter('lookahead_poses', 30)
        self.declare_parameter('validate_footprint', True)

    def _load_parameters(self):
        """Load parameters from ROS."""
        self._lookahead_distance = self.get_parameter('lookahead_distance').value
        self._goal_tolerance = self.get_parameter('goal_tolerance').value
        self._max_speed = self.get_parameter('max_speed').value
        self._min_speed = self.get_parameter('min_speed').value

        self._wheelbase = self.get_parameter('wheelbase').value

        self._replan_period = self.get_parameter('replan_period').value
        self._min_replan_interval = self.get_parameter('min_replan_interval').value
        self._replan_distance = self.get_parameter('replan_distance').value
        self._max_path_deviation = self.get_parameter('max_path_deviation').value

        self._lookahead_poses = self.get_parameter('lookahead_poses').value
        self._validate_footprint = self.get_parameter('validate_footprint').value

        # Robot footprint (2.11m x 1.2m with margin)
        self._robot_footprint = [
            (1.15, 0.65),
            (1.15, -0.65),
            (-1.15, -0.65),
            (-1.15, 0.65),
        ]

    def _delayed_init(self):
        """Delayed initialization after node is fully running."""
        if self._planner_interface is None:
            self._planner_interface = PlannerInterface(
                self,
                action_name='/compute_path_to_pose',
                planner_id='GridBased'
            )

            if self._planner_interface.wait_for_server(timeout_sec=5.0):
                self.get_logger().info('Connected to Nav2 planner')
            else:
                self.get_logger().warn(
                    'Nav2 planner not available - will retry when needed'
                )

    def _odom_callback(self, msg: Odometry):
        """Handle odometry update."""
        self._current_odom = msg

        # Create PoseStamped for replan manager
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose

        # Update replan manager
        self._replan_manager.update_position(pose)

    def _goal_callback(self, msg: PoseStamped):
        """Handle new navigation goal."""
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )

        goal_changed = self._replan_manager.update_goal(msg)
        self._current_goal = msg
        self._goal_reached = False
        self._planning_failures = 0  # Reset failures on new goal

        # Request new path
        if goal_changed or self._current_path is None:
            self._request_path()

        # Auto-start navigation
        if not self._is_navigating:
            self._is_navigating = True
            self._controller.start()
            self.get_logger().info('Navigation started')

    def _costmap_update_callback(self, msg: OccupancyGrid):
        """Handle costmap update."""
        self._path_validator.update_costmap(msg)
        self._costmap_available = True

    def _control_loop(self):
        """Main control loop - runs at 20Hz."""
        # Publish stop if not navigating
        if not self._is_navigating:
            return

        if self._current_pose is None:
            return

        if self._goal_reached:
            self._cmd_pub.publish(Twist())
            return

        # Check if we have a valid path
        if self._current_path is None or len(self._current_path.poses) == 0:
            if self._current_goal is not None and not self._planning_in_progress:
                self._request_path()
            return

        # Validate current path (only if costmap is available)
        path_blocked = False
        if self._costmap_available:
            path_blocked = not self._path_validator.validate_path(
                self._current_path,
                check_footprint=self._validate_footprint,
                max_poses=self._lookahead_poses
            ).is_valid

        # Check if replanning needed
        current_time = self.get_clock().now().nanoseconds / 1e9
        replan_decision = self._replan_manager.check_replan_needed(
            current_time=current_time,
            current_path=self._current_path,
            path_blocked=path_blocked,
            path_deviation=self._compute_path_deviation()
        )

        if replan_decision.should_replan and not self._planning_in_progress:
            self.get_logger().info(
                f'Replanning: {replan_decision.reason.name} - '
                f'{replan_decision.message}'
            )
            self._request_path()

        # Get robot pose in map frame via TF (path is in map frame)
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            yaw = self._quaternion_to_yaw(transform.transform.rotation)
        except TransformException as e:
            # Fall back to odom if TF not available yet
            if self._current_pose is None:
                return
            x = self._current_pose.pose.position.x
            y = self._current_pose.pose.position.y
            yaw = self._quaternion_to_yaw(self._current_pose.pose.orientation)

        current_speed = self._current_odom.twist.twist.linear.x if self._current_odom else 0.0

        cmd, goal_reached = self._controller.compute_velocity(x, y, yaw, current_speed)

        if goal_reached:
            self.get_logger().info('Goal reached!')
            self._goal_reached = True
            self._is_navigating = False
            self._cmd_pub.publish(Twist())
            return

        self._cmd_pub.publish(cmd)

    def _request_path(self):
        """Request a new path from the global planner."""
        if self._planner_interface is None:
            self.get_logger().warn('Planner interface not initialized')
            return

        if self._current_goal is None:
            return

        if self._planning_in_progress:
            return

        # Check if we've exceeded max failures
        if self._planning_failures >= self._max_planning_failures:
            return  # Stop retrying until new goal

        # Check retry delay with exponential backoff
        current_time = self.get_clock().now().nanoseconds / 1e9
        delay = self._plan_retry_delay * (2 ** min(self._planning_failures, 3))
        if current_time - self._last_plan_request_time < delay:
            return

        # Get robot pose in map frame via TF
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().warn(f'Cannot get robot pose in map frame: {e}')
            return

        self._last_plan_request_time = current_time
        self._planning_in_progress = True

        # Create start pose in map frame from TF
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = transform.transform.translation.x
        start.pose.position.y = transform.transform.translation.y
        start.pose.position.z = transform.transform.translation.z
        start.pose.orientation = transform.transform.rotation

        # Goal should already be in map frame
        goal = self._current_goal

        self.get_logger().info('Requesting path from global planner...')

        # Request path asynchronously
        self._planner_interface.plan_path_async(start, goal, self._path_callback)

    def _path_callback(self, result):
        """Handle path planning result."""
        self._planning_in_progress = False

        if result.status == PlanningStatus.SUCCEEDED and result.path is not None:
            self._current_path = result.path
            self._planning_failures = 0  # Reset failure counter on success

            # Convert Nav2 path to Pure Pursuit path
            pp_path = self._nav_path_to_pp_path(result.path)
            self._controller.set_path(pp_path)
            self._controller.start()

            # Mark replan completed
            current_time = self.get_clock().now().nanoseconds / 1e9
            self._replan_manager.mark_replan_completed(current_time)

            self.get_logger().info(
                f'Path received with {len(result.path.poses)} waypoints'
            )
        else:
            self._planning_failures += 1
            delay = self._plan_retry_delay * (2 ** min(self._planning_failures, 3))

            if self._planning_failures >= self._max_planning_failures:
                self.get_logger().error(
                    f'Path planning failed {self._planning_failures} times - '
                    f'giving up. Set a new goal to retry.'
                )
            else:
                self.get_logger().warn(
                    f'Path planning failed ({self._planning_failures}/'
                    f'{self._max_planning_failures}): {result.error_message}. '
                    f'Retrying in {delay:.1f}s...'
                )

    def _nav_path_to_pp_path(self, nav_path: NavPath) -> Path:
        """Convert nav_msgs/Path to Pure Pursuit Path."""
        pp_path = Path()

        for pose_stamped in nav_path.poses:
            # Use add_point method which creates PathPoint internally
            pp_path.add_point(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                speed=self._max_speed
            )

        return pp_path

    def _compute_path_deviation(self) -> float:
        """Compute lateral deviation from current path."""
        if self._current_path is None:
            return 0.0

        if len(self._current_path.poses) == 0:
            return 0.0

        # Get robot position in map frame (path is in map frame)
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except TransformException:
            if self._current_pose is None:
                return 0.0
            robot_x = self._current_pose.pose.position.x
            robot_y = self._current_pose.pose.position.y

        # Find nearest point on path
        min_dist = float('inf')

        for pose in self._current_path.poses:
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            min_dist = min(min_dist, dist)

        return min_dist

    def _publish_visualization(self):
        """Publish path and marker visualization."""
        if self._current_path is None:
            return

        # Republish path
        self._current_path.header.stamp = self.get_clock().now().to_msg()
        self._current_path.header.frame_id = 'map'
        self._path_pub.publish(self._current_path)

        # Publish waypoint markers
        marker_array = MarkerArray()
        current_idx = self._controller.get_current_waypoint_index()

        for i, pose in enumerate(self._current_path.poses):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.pose.position.z = 0.1
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15

            # Color based on status
            if i < current_idx:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.8, 0.0
            elif i == current_idx:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.2, 0.2, 0.8
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        # Add goal marker
        if self._current_goal is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'goal'
            goal_marker.id = 0
            goal_marker.type = Marker.ARROW
            goal_marker.action = Marker.ADD
            goal_marker.pose = self._current_goal.pose
            goal_marker.scale.x = 0.5
            goal_marker.scale.y = 0.1
            goal_marker.scale.z = 0.1
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.5
            goal_marker.color.a = 1.0
            marker_array.markers.append(goal_marker)

        self._marker_pub.publish(marker_array)

    def _publish_status(self):
        """Publish navigation status."""
        status_msg = Bool()
        status_msg.data = self._is_navigating
        self._nav_status_pub.publish(status_msg)

    def _start_callback(self, request, response):
        """Start navigation service callback."""
        if self._current_goal is None:
            response.success = False
            response.message = 'No goal set - use /goal_pose to set a goal'
            return response

        self._is_navigating = True
        self._goal_reached = False
        self._controller.start()

        if self._current_path is None:
            self._request_path()

        response.success = True
        response.message = 'Navigation started'
        self.get_logger().info(response.message)
        return response

    def _stop_callback(self, request, response):
        """Stop navigation service callback."""
        self._is_navigating = False
        self._controller.stop()
        self._cmd_pub.publish(Twist())

        response.success = True
        response.message = 'Navigation stopped'
        self.get_logger().info(response.message)
        return response

    def _replan_callback(self, request, response):
        """Force replanning service callback."""
        if self._current_goal is None:
            response.success = False
            response.message = 'No goal set'
            return response

        self._replan_manager.request_replan()
        response.success = True
        response.message = 'Replan requested'
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
