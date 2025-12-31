#!/usr/bin/env python3
"""
Dynamic Navigator - Goal-based navigation with continuous A* replanning

Navigates to a single target coordinate using dynamic A* replanning every 500ms.
When obstacles are detected, immediately replans around them.

Architecture:
    - SLAM provides map->odom TF (via slam_toolbox)
    - Nav2 SmacPlannerHybrid for A* planning (Ackermann-aware)
    - Pure Pursuit for path execution
    - Continuous replanning for dynamic obstacle avoidance

State Machine:
    IDLE -> PLANNING -> EXECUTING -> (goal reached)
                ^          |
                |          v
                +-- RECOVERING (backup when stuck)

Author: Zechariah Wang
Date: December 2025
"""

import math
import os
import sys
from enum import Enum
from typing import Optional, Tuple, List

# Add package to path for module imports
# Handles both source (development) and install (deployment) layouts
def _setup_module_path():
    # Method 1: Use ament_index to find installed package (most reliable)
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('exia_ground_description')
        lib_path = os.path.join(pkg_prefix, 'lib', 'exia_ground_description')
        if os.path.isdir(lib_path) and lib_path not in sys.path:
            sys.path.insert(0, lib_path)
        return
    except Exception:
        pass

    # Method 2: Source layout (scripts/active/ -> src/exia_control)
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


TARGET_POINT = [24, 24]  # [x, y] coordinates in meters (map frame)


class NavigatorState(Enum):
    """Dynamic navigator state machine states."""
    IDLE = 0         # Waiting for goal
    PLANNING = 1     # Computing initial path to goal
    EXECUTING = 2    # Following path with continuous replanning
    RECOVERING = 3   # Backing up when stuck


class DynamicNavigator(Node):
    """
    Dynamic goal-based navigator with continuous A* replanning.

    Features:
        - Single goal input via /goal_pose topic
        - Continuous replanning every 500ms
        - Obstacle-triggered immediate replanning
        - Pure Pursuit path execution
        - SLAM-based localization (map frame)
    """

    def __init__(self):
        super().__init__('dynamic_navigator')

        # Declare parameters
        self.declare_parameter('replan_period', 0.5)        # 500ms replanning
        self.declare_parameter('min_replan_interval', 0.2)  # Rate limit
        self.declare_parameter('goal_tolerance', 1.0)       # meters
        self.declare_parameter('lookahead_distance', 2.5)   # Pure Pursuit
        self.declare_parameter('target_speed', 2.0)         # m/s
        self.declare_parameter('path_switch_threshold', 2.0)  # Switch path if 2m shorter
        self.declare_parameter('obstacle_lookahead', 8.0)   # Obstacle detection range
        self.declare_parameter('control_rate', 50.0)        # 50Hz control loop
        self.declare_parameter('auto_start', True)          # Auto-start to TARGET_POINT

        # Get parameters
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

        # Goal
        self.current_goal: Optional[PoseStamped] = None

        # Path tracking
        self.current_path: Optional[PurePursuitPath] = None
        self.current_nav_path: Optional[Path] = None  # For validation
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

        # Path validator for obstacle detection
        self.path_validator = PathValidator()
        self.costmap: Optional[OccupancyGrid] = None

        # Planner interface for A* planning
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
            Odometry, '/odom', self._odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self._costmap_callback, costmap_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)

        # Services
        self.cancel_srv = self.create_service(
            Trigger, '/navigation/cancel', self._cancel_callback
        )

        # Timers
        # Control loop (50Hz)
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)

        # Replan timer (500ms)
        self.replan_timer = self.create_timer(self.replan_period, self._periodic_replan)

        # Visualization timer
        self.viz_timer = self.create_timer(0.5, self._publish_visualization)

        # Auto-start to TARGET_POINT
        auto_start = self.get_parameter('auto_start').value
        self._auto_start_done = False
        if auto_start:
            self.create_timer(3.0, self._delayed_auto_start, callback_group=self.callback_group)

        self.get_logger().info('Dynamic Navigator initialized')
        self.get_logger().info(f'  Replan period: {self.replan_period}s')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance}m')
        self.get_logger().info(f'  Target speed: {self.target_speed}m/s')
        self.get_logger().info(f'  Target point: ({TARGET_POINT[0]}, {TARGET_POINT[1]})')

    # ==================== Auto-Start ====================

    def _delayed_auto_start(self):
        """Auto-start navigation to TARGET_POINT after delay."""
        if self._auto_start_done:
            return
        if self.state != NavigatorState.IDLE:
            return

        self._auto_start_done = True

        # Create goal from TARGET_POINT
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(TARGET_POINT[0])
        goal.pose.position.y = float(TARGET_POINT[1])
        goal.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Auto-starting navigation to TARGET_POINT: ({TARGET_POINT[0]}, {TARGET_POINT[1]})'
        )

        # Trigger goal callback
        self._goal_callback(goal)

    # ==================== Callbacks ====================

    def _odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_speed = msg.twist.twist.linear.x

    def _goal_callback(self, msg: PoseStamped):
        """Handle new goal input."""
        self.current_goal = msg
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        # Clear current path and start planning
        self.current_path = None
        self.current_nav_path = None
        self._consecutive_replan_failures = 0
        self.state = NavigatorState.PLANNING
        self._request_initial_plan()

    def _costmap_callback(self, msg: OccupancyGrid):
        """Handle costmap updates with obstacle detection."""
        self.costmap = msg
        self.path_validator.update_costmap(msg)

        # Log first reception
        if not hasattr(self, '_costmap_received'):
            self._costmap_received = True
            self.get_logger().info(
                f'Costmap received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m'
            )

        # Obstacle-triggered replanning during execution
        if self.state == NavigatorState.EXECUTING:
            self._check_path_for_obstacles()

    def _cancel_callback(self, request, response):
        """Handle navigation cancellation."""
        self.state = NavigatorState.IDLE
        self.current_goal = None
        self.current_path = None
        self._publish_stop()
        response.success = True
        response.message = 'Navigation cancelled'
        return response

    # ==================== Control Loop ====================

    def _control_loop(self):
        """Main control loop - state machine execution."""
        if self.state == NavigatorState.IDLE:
            self._publish_stop()
            return

        elif self.state == NavigatorState.PLANNING:
            # Waiting for planner - stop robot
            self._publish_stop()
            return

        elif self.state == NavigatorState.EXECUTING:
            self._execute_path()

        elif self.state == NavigatorState.RECOVERING:
            self._execute_recovery()

    def _execute_path(self):
        """Execute current path using Pure Pursuit."""
        if self.current_path is None or self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        # Check if goal reached
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        dist_to_goal = math.sqrt(
            (goal_x - self.robot_x)**2 + (goal_y - self.robot_y)**2
        )

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(
                f'Goal reached! Distance: {dist_to_goal:.2f}m'
            )
            self.state = NavigatorState.IDLE
            self.current_goal = None
            self.current_path = None
            self._publish_stop()
            return

        # Execute Pure Pursuit
        cmd, goal_reached = self.pure_pursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw, self.robot_speed
        )

        if goal_reached:
            # Pure Pursuit thinks we reached end of path, but check actual goal
            if dist_to_goal < self.goal_tolerance * 2.0:
                self.get_logger().info('Path complete, goal reached!')
                self.state = NavigatorState.IDLE
                self.current_goal = None
                self._publish_stop()
                return
            else:
                # Request new path to goal
                self.get_logger().info('Path ended but goal not reached, replanning...')
                self._request_replan()

        self.cmd_vel_pub.publish(cmd)

    def _execute_recovery(self):
        """Execute recovery (backup) maneuver."""
        # Initialize if needed
        if not hasattr(self, '_recovery_in_progress') or not self._recovery_in_progress:
            self._recovery_in_progress = True
            self._recovery_start_x = self.robot_x
            self._recovery_start_y = self.robot_y
            self.get_logger().info('Starting recovery maneuver (backing up 2m)')

        # Calculate distance backed up
        dx = self.robot_x - self._recovery_start_x
        dy = self.robot_y - self._recovery_start_y
        dist_backed = math.sqrt(dx*dx + dy*dy)

        # Check if backed up enough
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

        if behind_cost >= 100:  # Lethal obstacle behind
            self.get_logger().warn('Obstacle behind, cannot back up further')
            self._recovery_in_progress = False
            self.state = NavigatorState.PLANNING
            self._request_initial_plan()
            return

        # Command reverse motion
        cmd = Twist()
        cmd.linear.x = -0.5
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    # ==================== Replanning ====================

    def _periodic_replan(self):
        """Periodic replanning timer (500ms)."""
        if self.state != NavigatorState.EXECUTING:
            return

        if self.current_goal is None:
            return

        # Rate limiting
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_replan_time < self.min_replan_interval:
            return

        # Skip if a replan is already pending
        if self.replan_pending:
            return

        self._request_replan()

    def _request_initial_plan(self):
        """Request initial path to goal."""
        if self.current_goal is None:
            self.state = NavigatorState.IDLE
            return

        # Create start pose
        start = PoseStamped()
        start.header.frame_id = 'odom'  # Will be transformed by planner
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = self.robot_x
        start.pose.position.y = self.robot_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        self.get_logger().info(
            f'Planning path from ({self.robot_x:.2f}, {self.robot_y:.2f}) to '
            f'({self.current_goal.pose.position.x:.2f}, {self.current_goal.pose.position.y:.2f})'
        )

        self.replan_pending = True
        self.planner.plan_path_async(start, self.current_goal, self._on_initial_path)

    def _request_replan(self):
        """Request replan during execution."""
        if self.current_goal is None:
            return

        # Create start pose from current position
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
        """Handle initial path result."""
        self.replan_pending = False

        if result.status == PlanningStatus.SUCCEEDED and result.path:
            nav_path = result.path
            self.get_logger().info(
                f'Initial path computed: {len(nav_path.poses)} poses'
            )

            # Validate path
            validation = self.path_validator.validate_path(nav_path, check_footprint=True)
            if not validation.is_valid:
                self.get_logger().warn(
                    f'Initial path has collision at index {validation.blocked_index}'
                )
                self._consecutive_replan_failures += 1
                if self._consecutive_replan_failures >= 3:
                    self.get_logger().error('Multiple planning failures, entering recovery')
                    self.state = NavigatorState.RECOVERING
                else:
                    # Try again after short delay
                    self.create_timer(0.5, self._request_initial_plan, callback_group=self.callback_group)
                return

            # Accept path
            self.current_nav_path = nav_path
            self.current_path = self._nav_path_to_pp_path(nav_path)
            self.pure_pursuit.set_path(self.current_path)
            self.pure_pursuit.start()
            self._consecutive_replan_failures = 0
            self.state = NavigatorState.EXECUTING

            # Publish for visualization
            self.path_pub.publish(nav_path)
        else:
            self.get_logger().error(f'Initial planning failed: {result.error_message}')
            self._consecutive_replan_failures += 1
            if self._consecutive_replan_failures >= 3:
                self.get_logger().error('Multiple planning failures, cancelling navigation')
                self.state = NavigatorState.IDLE
                self.current_goal = None
            else:
                # Retry after short delay
                self.create_timer(1.0, self._request_initial_plan, callback_group=self.callback_group)

    def _on_replan_received(self, result):
        """Handle replan result."""
        self.replan_pending = False

        if result.status != PlanningStatus.SUCCEEDED or not result.path:
            self.get_logger().warn(
                f'Replan failed: {result.error_message}',
                throttle_duration_sec=2.0
            )
            return

        new_nav_path = result.path

        # Validate new path
        validation = self.path_validator.validate_path(new_nav_path, check_footprint=True)
        if not validation.is_valid:
            self.get_logger().warn(
                f'Replan path has collision, keeping current path',
                throttle_duration_sec=2.0
            )
            return

        # Compare path lengths
        new_length = self._compute_path_length(new_nav_path)
        current_remaining = self._compute_remaining_path_length()

        should_switch = False

        # Switch if new path is significantly shorter
        if new_length < current_remaining - self.path_switch_threshold:
            self.get_logger().info(
                f'Switching to shorter path: {new_length:.1f}m vs {current_remaining:.1f}m'
            )
            should_switch = True

        # Switch if current path is blocked
        elif self.current_nav_path is not None:
            current_validation = self.path_validator.validate_path(
                self.current_nav_path, check_footprint=True
            )
            if not current_validation.is_valid:
                self.get_logger().warn('Current path blocked, switching to new path')
                should_switch = True

        if should_switch:
            self.current_nav_path = new_nav_path
            self.current_path = self._nav_path_to_pp_path(new_nav_path)
            self.pure_pursuit.set_path(self.current_path)
            self.pure_pursuit.start()
            self.path_pub.publish(new_nav_path)

    # ==================== Obstacle Detection ====================

    def _check_path_for_obstacles(self):
        """Check current path for obstacles and trigger immediate replan if needed."""
        if self.current_nav_path is None:
            return

        # Create lookahead segment
        lookahead_path = self._create_lookahead_segment()
        if lookahead_path is None or len(lookahead_path.poses) == 0:
            return

        # Validate lookahead
        result = self.path_validator.validate_path(lookahead_path, check_footprint=True)

        if not result.is_valid:
            self.get_logger().warn(
                f'Obstacle detected at ({result.blocked_position[0]:.1f}, '
                f'{result.blocked_position[1]:.1f}), triggering immediate replan'
            )

            # Trigger immediate replan if not already pending
            if not self.replan_pending:
                self._request_replan()

    def _create_lookahead_segment(self) -> Optional[Path]:
        """Create lookahead path segment for obstacle checking."""
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

        # Add poses from closest point up to lookahead distance
        accumulated_dist = 0.0
        prev_x, prev_y = self.robot_x, self.robot_y

        for i in range(closest_idx, len(self.current_nav_path.poses)):
            pose = self.current_nav_path.poses[i]
            dx = pose.pose.position.x - prev_x
            dy = pose.pose.position.y - prev_y
            dist = math.sqrt(dx*dx + dy*dy)

            accumulated_dist += dist
            if accumulated_dist > self.obstacle_lookahead:
                break

            path.poses.append(pose)
            prev_x = pose.pose.position.x
            prev_y = pose.pose.position.y

        return path

    # ==================== Path Utilities ====================

    def _nav_path_to_pp_path(self, nav_path: Path) -> PurePursuitPath:
        """Convert nav_msgs/Path to Pure Pursuit Path."""
        pp_path = PurePursuitPath()
        for pose in nav_path.poses:
            pp_path.add_point(
                pose.pose.position.x,
                pose.pose.position.y,
                speed=self.target_speed
            )
        return pp_path

    def _compute_path_length(self, nav_path: Path) -> float:
        """Compute total length of path."""
        if len(nav_path.poses) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(nav_path.poses)):
            p1 = nav_path.poses[i-1].pose.position
            p2 = nav_path.poses[i].pose.position
            total += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return total

    def _compute_remaining_path_length(self) -> float:
        """Compute remaining length from current position."""
        if self.current_path is None or len(self.current_path) < 2:
            return 0.0

        # Find closest point
        min_dist = float('inf')
        closest_idx = 0
        for i, pt in enumerate(self.current_path.points):
            dx = pt.x - self.robot_x
            dy = pt.y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Sum remaining distance
        total = min_dist  # Distance to closest point
        for i in range(closest_idx, len(self.current_path) - 1):
            p1 = self.current_path[i]
            p2 = self.current_path[i + 1]
            total += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

        return total

    # ==================== Visualization ====================

    def _publish_visualization(self):
        """Publish visualization markers."""
        markers = MarkerArray()

        # Goal marker
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

        # State text
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
        """Publish zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    # ==================== Utilities ====================

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw to quaternion."""
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
