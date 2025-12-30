#!/usr/bin/env python3
"""
Mission Navigator - Simplified path following with obstacle avoidance

Follows predefined paths using Pure Pursuit. When obstacles are detected,
plans a detour using A* and rejoins the original path.

Architecture:
    - No SLAM, uses odom frame only
    - Local costmap for obstacle detection
    - Nav2 A* planner for detour planning
    - Pure Pursuit for path execution

State Machine:
    IDLE -> FOLLOWING -> BLOCKED -> DETOURING -> FOLLOWING

Author: Zechariah Wang
Date: December 2025
"""

import math
import subprocess
import json
from enum import Enum
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from exia_control.planning.pure_pursuit import (
    PurePursuitController, PurePursuitConfig, Path as PurePursuitPath
)
from exia_control.planning.paths import (
    create_figure_eight_path, create_square_path, create_circle_path,
    create_line_path, create_slalom_path, create_scaled_path,
    FIGURE_EIGHT_PATH, SQUARE_PATH, CIRCLE_PATH, LINE_PATH, SLALOM_PATH
)
from exia_control.navigation.path_validator import PathValidator
from exia_control.navigation.planner_interface import (
    PlannerInterface, PlanningStatus
)


class NavigatorState(Enum):
    """Navigator state machine states."""
    IDLE = 0
    FOLLOWING = 1
    BLOCKED = 2
    PLANNING_DETOUR = 3
    DETOURING = 4
    BACKING_UP = 5  # Reversing to gain clearance


class MissionNavigator(Node):
    """
    Simplified mission navigator for predefined path following.

    Features:
        - Follows predefined waypoint paths using Pure Pursuit
        - Detects obstacles using local costmap
        - Plans detours around obstacles using A*
        - Automatically rejoins original path after detour
    """

    def __init__(self):
        super().__init__('mission_navigator')

        # Declare parameters
        self.declare_parameter('path_type', 'figure_eight')
        self.declare_parameter('path_scale', 3.0)
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('goal_tolerance', 1.0)
        self.declare_parameter('obstacle_lookahead', 5.0)
        self.declare_parameter('detour_clearance', 5.0)  # Increased to go farther around obstacles
        self.declare_parameter('auto_start', True)
        self.declare_parameter('control_rate', 20.0)

        # Get parameters
        path_type = self.get_parameter('path_type').value
        path_scale = self.get_parameter('path_scale').value
        target_speed = self.get_parameter('target_speed').value
        lookahead = self.get_parameter('lookahead_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_lookahead = self.get_parameter('obstacle_lookahead').value
        self.detour_clearance = self.get_parameter('detour_clearance').value
        auto_start = self.get_parameter('auto_start').value
        control_rate = self.get_parameter('control_rate').value

        # State
        self.state = NavigatorState.IDLE
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_speed = 0.0
        self.current_waypoint_idx = 0
        self.detour_rejoin_idx = 0
        self.mission_started = False
        self.waypoints_visited = 0
        self.bootstrap_phase = True  # Manual control until we escape start zone
        self.start_x = 0.0
        self.start_y = 0.0

        # Load mission path
        self.mission_path = self._create_path(path_type, path_scale, target_speed)
        self.get_logger().info(
            f'Loaded {path_type} path with {len(self.mission_path)} waypoints, '
            f'scale={path_scale}, speed={target_speed}'
        )

        # Pure Pursuit controller
        pp_config = PurePursuitConfig(
            lookahead_distance=lookahead,
            goal_tolerance=self.goal_tolerance,
            max_linear_speed=target_speed * 1.5,
            min_linear_speed=0.2,
            wheelbase=1.3,
        )
        self.pure_pursuit = PurePursuitController(pp_config)
        self.pure_pursuit.set_path(self.mission_path)

        # Path validator for obstacle detection
        self.path_validator = PathValidator()
        self.costmap: Optional[OccupancyGrid] = None

        # Planner interface for detours
        self.callback_group = ReentrantCallbackGroup()
        self.planner = PlannerInterface(self)

        # Detour path (when avoiding obstacle)
        self.detour_path: Optional[PurePursuitPath] = None
        self.detour_pure_pursuit: Optional[PurePursuitController] = None

        # QoS for costmap
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        # Subscribe to global costmap for obstacle detection
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.costmap_callback, costmap_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Services
        self.start_srv = self.create_service(
            Trigger, '/mission/start', self.start_callback
        )
        self.stop_srv = self.create_service(
            Trigger, '/mission/stop', self.stop_callback
        )
        self.reset_srv = self.create_service(
            Trigger, '/mission/reset', self.reset_callback
        )

        # Control timer
        period = 1.0 / control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        # Visualization timer (RViz)
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        # Gazebo markers (one-time setup)
        self._gz_markers_published = False
        self.create_timer(3.0, self._publish_gazebo_markers_once)

        # Auto-start (one-shot timer)
        self._auto_start_done = False
        if auto_start:
            self.create_timer(2.0, self._delayed_start, callback_group=self.callback_group)

        self.get_logger().info('Mission Navigator initialized')

    def _delayed_start(self):
        """Start mission after delay for initialization."""
        if self._auto_start_done:
            return
        if self.state == NavigatorState.IDLE:
            # Record starting position for bootstrap phase
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.bootstrap_phase = True
            self.state = NavigatorState.FOLLOWING
            self.waypoints_visited = 0
            self._auto_start_done = True
            self.get_logger().info(
                f'Mission auto-started at ({self.start_x:.2f}, {self.start_y:.2f})'
            )

    def _create_path(self, path_type: str, scale: float, speed: float) -> PurePursuitPath:
        """Create mission path from type name."""
        paths = {
            'line': LINE_PATH,
            'square': SQUARE_PATH,
            'circle': CIRCLE_PATH,
            'figure_eight': FIGURE_EIGHT_PATH,
            'slalom': SLALOM_PATH,
        }
        points = paths.get(path_type.lower(), FIGURE_EIGHT_PATH)
        return create_scaled_path(points, scale=scale, speed=speed)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_speed = msg.twist.twist.linear.x

    def costmap_callback(self, msg: OccupancyGrid):
        """Update costmap for obstacle detection."""
        self.costmap = msg
        self.path_validator.update_costmap(msg)
        # Log once when first costmap received
        if not hasattr(self, '_costmap_received'):
            self._costmap_received = True
            self.get_logger().info(
                f'Costmap received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m'
            )

    def control_loop(self):
        """Main control loop - state machine execution."""
        if self.state == NavigatorState.IDLE:
            self._publish_stop()
            return

        elif self.state == NavigatorState.FOLLOWING:
            self._execute_following()

        elif self.state == NavigatorState.BLOCKED:
            self._handle_blocked()

        elif self.state == NavigatorState.PLANNING_DETOUR:
            # Waiting for planner - stop robot
            self._publish_stop()

        elif self.state == NavigatorState.DETOURING:
            self._execute_detouring()

        elif self.state == NavigatorState.BACKING_UP:
            self._execute_backing_up()

    def _execute_following(self):
        """Execute mission path following."""
        cmd = Twist()

        # Bootstrap phase: manual control until we escape the start zone
        # This is needed because looping paths (like figure-8) start and end
        # at the same point, causing Pure Pursuit to think goal is reached
        if self.bootstrap_phase:
            cmd = self._execute_bootstrap()
            self.cmd_vel_pub.publish(cmd)
            return

        # Normal Pure Pursuit following
        # Track current waypoint progress
        current_idx = self.pure_pursuit.get_current_waypoint_index()
        if current_idx > self.waypoints_visited:
            self.waypoints_visited = current_idx
            self.get_logger().info(
                f'Waypoint {current_idx}/{len(self.mission_path)} reached'
            )

        # Check for obstacles ahead
        if self._check_path_blocked():
            self.state = NavigatorState.BLOCKED
            self.get_logger().warn('Obstacle detected in path!')
            return

        # Execute Pure Pursuit
        cmd, goal_reached = self.pure_pursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw, self.robot_speed
        )

        # Only consider mission complete if we've visited most waypoints
        min_progress = len(self.mission_path) - 2
        if goal_reached and self.waypoints_visited >= min_progress:
            self.get_logger().info('Mission complete!')
            self.state = NavigatorState.IDLE
            self._publish_stop()
            return

        self.cmd_vel_pub.publish(cmd)

    def _execute_bootstrap(self) -> Twist:
        """Bootstrap phase: manually drive robot away from start zone."""
        cmd = Twist()

        # Target the second waypoint (first real target after origin)
        if len(self.mission_path) < 2:
            self.bootstrap_phase = False
            return cmd

        target_wp = self.mission_path[1]

        # Check if we've moved far enough from start
        dist_from_start = math.sqrt(
            (self.robot_x - self.start_x)**2 +
            (self.robot_y - self.start_y)**2
        )

        # Exit bootstrap once we've traveled past goal_tolerance
        # This ensures Pure Pursuit won't immediately think we're at the goal
        if dist_from_start > self.goal_tolerance * 1.5:
            self.get_logger().info(
                f'Bootstrap complete! Traveled {dist_from_start:.2f}m from start. '
                f'Switching to Pure Pursuit control.'
            )
            self.bootstrap_phase = False
            self.pure_pursuit.start()
            # Skip to waypoint 1 (we've passed waypoint 0)
            self.pure_pursuit._closest_point_idx = 1
            return cmd

        # Compute heading to target waypoint
        dx = target_wp.x - self.robot_x
        dy = target_wp.y - self.robot_y
        dist_to_target = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self.robot_yaw)

        # Simple proportional control
        # First align heading, then drive forward
        if abs(yaw_error) > 0.3:  # ~17 degrees
            # Turn in place (with minimal forward motion for Ackermann)
            cmd.linear.x = 0.3
            cmd.angular.z = yaw_error * 1.0
        else:
            # Drive toward target
            cmd.linear.x = min(0.8, dist_to_target * 0.5)
            cmd.angular.z = yaw_error * 0.8

        # Clamp values
        cmd.linear.x = max(0.2, min(1.0, cmd.linear.x))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        return cmd

    def _execute_detouring(self):
        """Execute detour path following."""
        if self.detour_pure_pursuit is None:
            self.state = NavigatorState.BLOCKED
            return

        # During detouring, only check for VERY close obstacles (emergency stop)
        # Don't use the regular forward check - it would see the obstacle we're detouring around
        emergency_dist = 1.5  # meters - emergency stop distance
        ahead_x = self.robot_x + emergency_dist * math.cos(self.robot_yaw)
        ahead_y = self.robot_y + emergency_dist * math.sin(self.robot_yaw)
        cost = self.path_validator._get_cost_at_world(ahead_x, ahead_y)
        if cost >= self.path_validator.LETHAL_THRESHOLD:
            self.get_logger().warn(f'Emergency stop during detour - obstacle at {emergency_dist}m!')
            self._publish_stop()
            return

        # Check if we can rejoin mission path
        if self._can_rejoin_mission():
            self._rejoin_mission()
            return

        # Execute detour Pure Pursuit
        cmd, goal_reached = self.detour_pure_pursuit.compute_velocity(
            self.robot_x, self.robot_y, self.robot_yaw, self.robot_speed
        )

        if goal_reached:
            # Detour complete, rejoin mission
            self._rejoin_mission()
            return

        self.cmd_vel_pub.publish(cmd)

    def _execute_backing_up(self):
        """Execute reverse maneuver to gain clearance for replanning."""
        # Initialize backup tracking if needed
        if not hasattr(self, '_backup_start_x'):
            self._backup_start_x = self.robot_x
            self._backup_start_y = self.robot_y
            self._backup_target_dist = 3.0  # Back up 3 meters
            self.get_logger().info('Starting backup maneuver (3m)')

        # Calculate distance backed up
        dx = self.robot_x - self._backup_start_x
        dy = self.robot_y - self._backup_start_y
        dist_backed = math.sqrt(dx*dx + dy*dy)

        # Check if we've backed up enough
        if dist_backed >= self._backup_target_dist:
            self.get_logger().info(f'Backup complete ({dist_backed:.2f}m), retrying detour planning')
            # Clean up and return to BLOCKED to retry planning
            del self._backup_start_x
            del self._backup_start_y
            del self._backup_target_dist
            self._plan_fail_count = 0  # Reset fail count
            self.state = NavigatorState.BLOCKED
            return

        # Check for obstacles behind (don't back into something)
        behind_x = self.robot_x - 2.0 * math.cos(self.robot_yaw)
        behind_y = self.robot_y - 2.0 * math.sin(self.robot_yaw)
        behind_cost = self.path_validator._get_cost_at_world(behind_x, behind_y)

        if behind_cost >= self.path_validator.LETHAL_THRESHOLD:
            self.get_logger().warn('Obstacle behind - cannot back up further')
            del self._backup_start_x
            del self._backup_start_y
            del self._backup_target_dist
            self.state = NavigatorState.BLOCKED
            return

        # Command reverse motion
        cmd = Twist()
        cmd.linear.x = -0.5  # Slow reverse
        cmd.angular.z = 0.0  # Straight back
        self.cmd_vel_pub.publish(cmd)

    def _handle_blocked(self):
        """Handle blocked state - plan detour."""
        self._publish_stop()

        # Cooldown to prevent rapid re-planning
        if not hasattr(self, '_last_plan_attempt'):
            self._last_plan_attempt = 0.0
            self._plan_fail_count = 0

        now = self.get_clock().now().nanoseconds / 1e9
        cooldown = min(2.0 * (self._plan_fail_count + 1), 10.0)  # Exponential backoff up to 10s

        if now - self._last_plan_attempt < cooldown:
            return  # Still in cooldown

        self._last_plan_attempt = now

        # Find rejoin point past obstacle
        self.detour_rejoin_idx = self._find_rejoin_waypoint()
        if self.detour_rejoin_idx < 0:
            self.get_logger().error('Cannot find rejoin point!')
            self.state = NavigatorState.IDLE
            return

        # Request detour path
        self.state = NavigatorState.PLANNING_DETOUR
        self._request_detour()

    def _check_path_blocked(self) -> bool:
        """Check if path ahead is blocked by obstacles."""
        if self.costmap is None:
            self.get_logger().warn('No costmap available for obstacle check', throttle_duration_sec=5.0)
            return False

        # First check: immediate forward path based on robot heading
        # This catches obstacles right in front regardless of waypoint path
        immediate_blocked = self._check_immediate_forward()
        if immediate_blocked:
            return True

        # Second check: path along waypoints
        lookahead_path = self._create_lookahead_segment()
        if lookahead_path is None or len(lookahead_path.poses) == 0:
            return False

        result = self.path_validator.validate_path(lookahead_path, check_footprint=True)

        # Debug: periodic logging of path validation (every 2 seconds)
        if not hasattr(self, '_last_path_check_log'):
            self._last_path_check_log = 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_path_check_log > 2.0:
            self._last_path_check_log = now
            # Sample costmap at robot position
            robot_cost = self.path_validator._get_cost_at_world(self.robot_x, self.robot_y)
            # Sample costmap directly ahead (3m in heading direction)
            ahead_x = self.robot_x + 3.0 * math.cos(self.robot_yaw)
            ahead_y = self.robot_y + 3.0 * math.sin(self.robot_yaw)
            ahead_cost = self.path_validator._get_cost_at_world(ahead_x, ahead_y)
            self.get_logger().info(
                f'Path check: {len(lookahead_path.poses)} poses, robot_cost={robot_cost}, '
                f'ahead_cost={ahead_cost}, valid={result.is_valid}'
            )

        # Log when path is blocked
        if not result.is_valid:
            self.get_logger().warn(
                f'Path blocked! Collision at ({result.blocked_position[0]:.1f}, '
                f'{result.blocked_position[1]:.1f}) at waypoint index {result.blocked_index}'
            )

        return not result.is_valid

    def _check_immediate_forward(self) -> bool:
        """Check for obstacles directly ahead of the robot.

        Checks points along the robot's current heading direction,
        independent of the waypoint path.
        """
        # Check points from 3m to 7m ahead - stop BEFORE entering inflation zone
        # This gives enough clearance for the planner to work
        for dist in [3.0, 4.0, 5.0, 6.0, 7.0]:
            check_x = self.robot_x + dist * math.cos(self.robot_yaw)
            check_y = self.robot_y + dist * math.sin(self.robot_yaw)
            cost = self.path_validator._get_cost_at_world(check_x, check_y)

            if cost >= self.path_validator.LETHAL_THRESHOLD:
                self.get_logger().warn(
                    f'Obstacle detected {dist:.1f}m ahead at ({check_x:.1f}, {check_y:.1f}), cost={cost}'
                )
                return True

        return False

    def _create_lookahead_segment(self) -> Optional[Path]:
        """Create nav_msgs/Path from lookahead waypoints for validation.

        Interpolates between waypoints to ensure obstacles between waypoints
        are detected.
        """
        if self.mission_path is None:
            return None

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        # Get current index
        current_idx = self.pure_pursuit.get_current_waypoint_index()

        # Start from robot's current position
        prev_x, prev_y = self.robot_x, self.robot_y
        accumulated_dist = 0.0

        # Interpolation step size (check every 0.5m along path)
        INTERP_STEP = 0.5

        for i in range(current_idx, len(self.mission_path)):
            wp = self.mission_path[i]

            # Distance to next waypoint
            dx = wp.x - prev_x
            dy = wp.y - prev_y
            segment_dist = math.sqrt(dx*dx + dy*dy)

            if segment_dist < 0.01:
                continue

            # Number of interpolation points for this segment
            num_steps = max(1, int(segment_dist / INTERP_STEP))

            for step in range(1, num_steps + 1):
                t = step / num_steps
                interp_x = prev_x + t * dx
                interp_y = prev_y + t * dy

                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = interp_x
                pose.pose.position.y = interp_y
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)

                accumulated_dist += segment_dist / num_steps

                if accumulated_dist >= self.obstacle_lookahead:
                    break

            if accumulated_dist >= self.obstacle_lookahead:
                break

            prev_x, prev_y = wp.x, wp.y

        return path

    def _find_rejoin_waypoint(self) -> int:
        """Find waypoint to rejoin after detour.

        Finds a clear waypoint that is:
        1. Ahead of the robot (in its heading direction)
        2. Far enough away to clear the obstacle
        3. Not blocked by obstacles
        """
        current_idx = self.pure_pursuit.get_current_waypoint_index()

        # Robot's forward direction
        heading_x = math.cos(self.robot_yaw)
        heading_y = math.sin(self.robot_yaw)

        # Skip at least 2 waypoints to avoid immediately re-detecting same obstacle
        min_skip = 2
        start_idx = min(current_idx + min_skip, len(self.mission_path) - 1)

        # Find first clear waypoint that's ahead of the robot
        for i in range(start_idx, len(self.mission_path)):
            wp = self.mission_path[i]

            # Vector from robot to waypoint
            dx = wp.x - self.robot_x
            dy = wp.y - self.robot_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < 0.1:
                continue

            # Check if waypoint is generally ahead (dot product > 0)
            # Allow some tolerance for waypoints to the side
            dot_product = (dx * heading_x + dy * heading_y) / dist
            is_ahead = dot_product > -0.3  # Allow up to ~107 degrees from heading

            # Check if waypoint is clear and far enough
            if is_ahead and dist >= self.detour_clearance:
                if self._is_position_clear(wp.x, wp.y):
                    self.get_logger().info(
                        f'Rejoin waypoint found at index {i} '
                        f'(distance: {dist:.2f}m, dot: {dot_product:.2f})'
                    )
                    return i

        # Fallback: find ANY clear waypoint (even if behind)
        self.get_logger().warn('No forward rejoin point found, searching all waypoints')
        for i in range(current_idx + 1, len(self.mission_path)):
            wp = self.mission_path[i]
            if self._is_position_clear(wp.x, wp.y):
                return i

        # Last resort: final waypoint
        return len(self.mission_path) - 1

    def _is_position_clear(self, x: float, y: float) -> bool:
        """Check if position is clear using costmap."""
        if self.costmap is None:
            return True

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        return self.path_validator.is_pose_valid(pose, check_footprint=False)

    def _request_detour(self):
        """Request A* path around obstacle."""
        rejoin_wp = self.mission_path[self.detour_rejoin_idx]

        # Offset start position backward slightly to avoid starting in inflated space
        # This helps when robot stopped close to obstacle
        BACKWARD_OFFSET = 1.0  # meters
        start_x = self.robot_x - BACKWARD_OFFSET * math.cos(self.robot_yaw)
        start_y = self.robot_y - BACKWARD_OFFSET * math.sin(self.robot_yaw)

        # Check if offset position is clear, if not use current position
        offset_cost = self.path_validator._get_cost_at_world(start_x, start_y)
        current_cost = self.path_validator._get_cost_at_world(self.robot_x, self.robot_y)

        if offset_cost < self.path_validator.LETHAL_THRESHOLD:
            # Use offset position
            pass
        elif current_cost < self.path_validator.LETHAL_THRESHOLD:
            # Use current position
            start_x, start_y = self.robot_x, self.robot_y
        else:
            # Both in lethal space - try to find a clear spot nearby
            for angle_offset in [0.5, -0.5, 1.0, -1.0]:  # Try sideways
                test_x = self.robot_x + 1.0 * math.cos(self.robot_yaw + angle_offset)
                test_y = self.robot_y + 1.0 * math.sin(self.robot_yaw + angle_offset)
                if self.path_validator._get_cost_at_world(test_x, test_y) < self.path_validator.LETHAL_THRESHOLD:
                    start_x, start_y = test_x, test_y
                    break
            else:
                start_x, start_y = self.robot_x, self.robot_y  # Fallback

        # Create start pose
        start = PoseStamped()
        start.header.frame_id = 'odom'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = start_x
        start.pose.position.y = start_y
        start.pose.orientation = self._yaw_to_quaternion(self.robot_yaw)

        # Compute goal orientation (heading toward next waypoint on path)
        goal_yaw = 0.0
        next_idx = self.detour_rejoin_idx + 1
        if next_idx < len(self.mission_path):
            next_wp = self.mission_path[next_idx]
            goal_yaw = math.atan2(
                next_wp.y - rejoin_wp.y,
                next_wp.x - rejoin_wp.x
            )
        else:
            # Last waypoint - use direction from current position
            goal_yaw = math.atan2(
                rejoin_wp.y - self.robot_y,
                rejoin_wp.x - self.robot_x
            )

        # Create goal pose (rejoin waypoint with proper heading)
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = rejoin_wp.x
        goal.pose.position.y = rejoin_wp.y
        goal.pose.orientation = self._yaw_to_quaternion(goal_yaw)

        self.get_logger().info(
            f'Requesting detour from ({self.robot_x:.2f}, {self.robot_y:.2f}) '
            f'to ({rejoin_wp.x:.2f}, {rejoin_wp.y:.2f}) heading={math.degrees(goal_yaw):.0f}°'
        )

        # Request path asynchronously
        self.planner.plan_path_async(start, goal, self._detour_received)

    def _detour_received(self, result):
        """Callback when detour path is received."""
        if result.status == PlanningStatus.SUCCEEDED and result.path:
            self.get_logger().info(
                f'Detour path received with {len(result.path.poses)} waypoints'
            )

            # Convert nav_msgs/Path to PurePursuitPath
            self.detour_path = self._nav_path_to_pp_path(result.path)

            # Create Pure Pursuit for detour
            config = PurePursuitConfig(
                lookahead_distance=1.5,
                goal_tolerance=1.5,  # Looser tolerance for detour endpoint
                max_linear_speed=0.8,  # Slower during detour
                wheelbase=1.3,
            )
            self.detour_pure_pursuit = PurePursuitController(config)
            self.detour_pure_pursuit.set_path(self.detour_path)
            self.detour_pure_pursuit.start()  # IMPORTANT: Start the controller!

            self.state = NavigatorState.DETOURING
            self._plan_fail_count = 0  # Reset fail count on success
            self.get_logger().info('Starting detour execution')
        else:
            self._plan_fail_count = getattr(self, '_plan_fail_count', 0) + 1
            self.get_logger().error(
                f'Detour planning failed: {result.error_message} (attempt {self._plan_fail_count})'
            )

            # After 2 failed attempts, try backing up to gain clearance
            if self._plan_fail_count >= 2:
                self.get_logger().info('Multiple planning failures - backing up to gain clearance')
                self.state = NavigatorState.BACKING_UP
            else:
                # Stay in BLOCKED state to retry after cooldown
                self.state = NavigatorState.BLOCKED

    def _nav_path_to_pp_path(self, nav_path: Path) -> PurePursuitPath:
        """Convert nav_msgs/Path to Pure Pursuit Path."""
        pp_path = PurePursuitPath()
        for pose in nav_path.poses:
            pp_path.add_point(
                pose.pose.position.x,
                pose.pose.position.y,
                speed=0.8  # Slower speed during detour
            )
        return pp_path

    def _can_rejoin_mission(self) -> bool:
        """Check if robot can rejoin mission path."""
        if self.detour_rejoin_idx >= len(self.mission_path):
            return True

        rejoin_wp = self.mission_path[self.detour_rejoin_idx]

        # Check distance to rejoin point
        dx = rejoin_wp.x - self.robot_x
        dy = rejoin_wp.y - self.robot_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Check if close enough and path ahead is clear
        if dist < 2.0:
            # Verify mission path is now clear
            lookahead_path = self._create_lookahead_from_index(self.detour_rejoin_idx)
            if lookahead_path and len(lookahead_path.poses) > 0:
                result = self.path_validator.validate_path(lookahead_path)
                return result.is_valid

        return False

    def _create_lookahead_from_index(self, start_idx: int) -> Optional[Path]:
        """Create lookahead segment from specific index."""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        accumulated_dist = 0.0
        prev_x = self.mission_path[start_idx].x
        prev_y = self.mission_path[start_idx].y

        for i in range(start_idx, min(start_idx + 10, len(self.mission_path))):
            wp = self.mission_path[i]

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

            dx = wp.x - prev_x
            dy = wp.y - prev_y
            accumulated_dist += math.sqrt(dx*dx + dy*dy)
            prev_x, prev_y = wp.x, wp.y

            if accumulated_dist >= 5.0:
                break

        return path

    def _rejoin_mission(self):
        """Rejoin mission path after detour."""
        self.get_logger().info(
            f'Rejoining mission at waypoint {self.detour_rejoin_idx}'
        )

        # Reset mission Pure Pursuit to rejoin point
        self.pure_pursuit._closest_point_idx = self.detour_rejoin_idx
        self.pure_pursuit.start()

        # Clear detour state
        self.detour_path = None
        self.detour_pure_pursuit = None

        self.state = NavigatorState.FOLLOWING

    def _publish_stop(self):
        """Publish zero velocity."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def publish_visualization(self):
        """Publish path visualization markers."""
        # Publish mission path
        self._publish_path()
        self._publish_markers()

    def _publish_path(self):
        """Publish current path as nav_msgs/Path."""
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in self.mission_path.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def _publish_markers(self):
        """Publish waypoint markers."""
        markers = MarkerArray()

        # Waypoint markers
        for i, wp in enumerate(self.mission_path.points):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wp.x
            marker.pose.position.y = wp.y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            # Color based on progress
            current_idx = self.pure_pursuit.get_current_waypoint_index()
            if i < current_idx:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 0.5
            elif i == current_idx:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 0.8

            markers.markers.append(marker)

        # State indicator
        state_marker = Marker()
        state_marker.header.frame_id = 'odom'
        state_marker.header.stamp = self.get_clock().now().to_msg()
        state_marker.ns = 'state'
        state_marker.id = 0
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        state_marker.pose.position.x = self.robot_x
        state_marker.pose.position.y = self.robot_y
        state_marker.pose.position.z = 1.5
        state_marker.scale.z = 0.5
        state_marker.color.r = 1.0
        state_marker.color.g = 1.0
        state_marker.color.b = 1.0
        state_marker.color.a = 1.0
        state_marker.text = self.state.name
        markers.markers.append(state_marker)

        self.markers_pub.publish(markers)

    # ==================== Gazebo Fortress Markers ====================

    def _publish_gazebo_markers_once(self):
        """Publish path markers to Gazebo Fortress (one-time)."""
        if self._gz_markers_published:
            return
        self._gz_markers_published = True

        self.get_logger().info('Publishing path markers to Gazebo...')

        # Publish path as line strip
        self._publish_gz_path_line()

        # Publish waypoint spheres
        self._publish_gz_waypoint_spheres()

        self.get_logger().info('Gazebo path markers published')

    def _publish_gz_path_line(self):
        """Publish path as a line strip in Gazebo."""
        if len(self.mission_path.points) < 2:
            return

        # Build points string for line strip
        points_str = ""
        for wp in self.mission_path.points:
            points_str += f"point {{ x: {wp.x} y: {wp.y} z: 0.5 }} "

        # Create marker message
        msg = (
            f'action: ADD_MODIFY '
            f'ns: "mission_path" '
            f'id: 1 '
            f'type: LINE_STRIP '
            f'material {{ '
            f'diffuse {{ r: 0.0 g: 1.0 b: 0.0 a: 1.0 }} '
            f'}} '
            f'{points_str}'
        )

        cmd = ['ign', 'topic', '-t', '/marker', '-m', 'ignition.msgs.Marker', '-p', msg]

        try:
            subprocess.run(cmd, capture_output=True, timeout=5.0)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish Gazebo path line: {e}')

    def _publish_gz_waypoint_spheres(self):
        """Publish waypoint spheres in Gazebo."""
        for i, wp in enumerate(self.mission_path.points):
            # Color: first waypoint green, last red, others blue
            if i == 0:
                r, g, b = 0.0, 1.0, 0.0  # Green for start
            elif i == len(self.mission_path.points) - 1:
                r, g, b = 1.0, 0.0, 0.0  # Red for end
            else:
                r, g, b = 0.0, 0.5, 1.0  # Blue for intermediate

            msg = (
                f'action: ADD_MODIFY '
                f'ns: "waypoints" '
                f'id: {i + 100} '
                f'type: SPHERE '
                f'pose {{ position {{ x: {wp.x} y: {wp.y} z: 0.5 }} }} '
                f'scale {{ x: 0.6 y: 0.6 z: 0.6 }} '
                f'material {{ '
                f'diffuse {{ r: {r} g: {g} b: {b} a: 0.9 }} '
                f'}}'
            )

            cmd = ['ign', 'topic', '-t', '/marker', '-m', 'ignition.msgs.Marker', '-p', msg]

            try:
                subprocess.run(cmd, capture_output=True, timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f'Failed to publish waypoint {i}: {e}')

    def _clear_gz_markers(self):
        """Clear all Gazebo markers."""
        # Delete path line
        msg = 'action: DELETE_ALL ns: "mission_path"'
        cmd = ['ign', 'topic', '-t', '/marker', '-m', 'ignition.msgs.Marker', '-p', msg]
        try:
            subprocess.run(cmd, capture_output=True, timeout=2.0)
        except Exception:
            pass

        # Delete waypoint spheres
        msg = 'action: DELETE_ALL ns: "waypoints"'
        cmd = ['ign', 'topic', '-t', '/marker', '-m', 'ignition.msgs.Marker', '-p', msg]
        try:
            subprocess.run(cmd, capture_output=True, timeout=2.0)
        except Exception:
            pass

    # Service callbacks
    def start_callback(self, request, response):
        """Handle start service."""
        if self.state == NavigatorState.IDLE:
            # Record starting position for bootstrap phase
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.bootstrap_phase = True
            self.waypoints_visited = 0
            self.state = NavigatorState.FOLLOWING
            response.success = True
            response.message = f'Mission started at ({self.start_x:.2f}, {self.start_y:.2f})'
        else:
            response.success = False
            response.message = f'Already in state: {self.state.name}'
        return response

    def stop_callback(self, request, response):
        """Handle stop service."""
        self.state = NavigatorState.IDLE
        self._publish_stop()
        response.success = True
        response.message = 'Mission stopped'
        return response

    def reset_callback(self, request, response):
        """Handle reset service."""
        self.state = NavigatorState.IDLE
        self.pure_pursuit.clear_path()
        self.pure_pursuit.set_path(self.mission_path)
        self.detour_path = None
        self.detour_pure_pursuit = None
        self.waypoints_visited = 0
        self._publish_stop()

        # Re-publish Gazebo markers
        self._gz_markers_published = False
        self._publish_gazebo_markers_once()

        response.success = True
        response.message = 'Mission reset'
        return response

    # Utility methods
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

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MissionNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
