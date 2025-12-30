"""
Planner Interface - Interface to Nav2 global planner (A*)

Provides synchronous and asynchronous methods to request paths
from the Nav2 planning system using Smac Hybrid-A*.

Author: Zechariah Wang
Date: December 2025
"""

from dataclasses import dataclass
from typing import Optional, Callable
from enum import Enum
import threading

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus


class PlanningStatus(Enum):
    """Status of planning request."""
    PENDING = 0
    PLANNING = 1
    SUCCEEDED = 2
    FAILED = 3
    CANCELED = 4


@dataclass
class PlanningResult:
    """Result of a planning request."""
    status: PlanningStatus
    path: Optional[Path] = None
    planning_time: float = 0.0
    error_message: str = ""


class PlannerInterface:
    """
    Interface to Nav2 global planner using ComputePathToPose action.

    Supports both synchronous (blocking) and asynchronous planning requests.
    Uses Smac Hybrid-A* planner configured for Ackermann vehicles.
    """

    def __init__(self, node: Node,
                 action_name: str = '/compute_path_to_pose',
                 planner_id: str = 'GridBased'):
        """
        Initialize planner interface.

        Args:
            node: ROS2 node for creating clients
            action_name: Name of ComputePathToPose action server
            planner_id: ID of planner plugin to use
        """
        self._node = node
        self._logger = node.get_logger()
        self._planner_id = planner_id

        # Create action client with reentrant callback group
        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            node,
            ComputePathToPose,
            action_name,
            callback_group=self._callback_group
        )

        # Current goal handle for cancellation
        self._current_goal_handle = None
        self._planning_lock = threading.Lock()

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """
        Wait for planner action server to become available.

        Args:
            timeout_sec: Maximum wait time

        Returns:
            True if server is available
        """
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def plan_path(self, start: PoseStamped, goal: PoseStamped,
                  timeout_sec: float = 10.0) -> PlanningResult:
        """
        Request a path synchronously (blocking).

        Args:
            start: Starting pose (in map frame)
            goal: Goal pose (in map frame)
            timeout_sec: Maximum planning time

        Returns:
            PlanningResult with path or error
        """
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            return PlanningResult(
                status=PlanningStatus.FAILED,
                error_message="Planner action server not available"
            )

        # Create goal message
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = self._planner_id
        goal_msg.use_start = True

        self._logger.info(
            f'Requesting path from ({start.pose.position.x:.2f}, '
            f'{start.pose.position.y:.2f}) to ({goal.pose.position.x:.2f}, '
            f'{goal.pose.position.y:.2f})'
        )

        # Send goal and wait for result
        import time
        start_time = time.time()

        send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        while not send_goal_future.done():
            if time.time() - start_time > timeout_sec:
                return PlanningResult(
                    status=PlanningStatus.FAILED,
                    error_message="Timeout waiting for goal acceptance"
                )
            time.sleep(0.01)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return PlanningResult(
                status=PlanningStatus.FAILED,
                error_message="Planning goal was rejected"
            )

        with self._planning_lock:
            self._current_goal_handle = goal_handle

        # Wait for result
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if time.time() - start_time > timeout_sec:
                goal_handle.cancel_goal_async()
                return PlanningResult(
                    status=PlanningStatus.FAILED,
                    error_message="Timeout waiting for planning result",
                    planning_time=time.time() - start_time
                )
            time.sleep(0.01)

        result = result_future.result()
        planning_time = time.time() - start_time

        with self._planning_lock:
            self._current_goal_handle = None

        # Check result status
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            path = result.result.path
            if len(path.poses) > 0:
                self._logger.info(
                    f'Path computed with {len(path.poses)} waypoints '
                    f'in {planning_time:.2f}s'
                )
                return PlanningResult(
                    status=PlanningStatus.SUCCEEDED,
                    path=path,
                    planning_time=planning_time
                )
            else:
                return PlanningResult(
                    status=PlanningStatus.FAILED,
                    error_message="Planner returned empty path",
                    planning_time=planning_time
                )
        elif result.status == GoalStatus.STATUS_CANCELED:
            return PlanningResult(
                status=PlanningStatus.CANCELED,
                planning_time=planning_time
            )
        else:
            return PlanningResult(
                status=PlanningStatus.FAILED,
                error_message=f"Planning failed with status {result.status}",
                planning_time=planning_time
            )

    def plan_path_async(self, start: PoseStamped, goal: PoseStamped,
                        callback: Callable[[PlanningResult], None]) -> None:
        """
        Request a path asynchronously (non-blocking).

        Args:
            start: Starting pose
            goal: Goal pose
            callback: Function called with PlanningResult when complete
        """
        if not self._action_client.server_is_ready():
            callback(PlanningResult(
                status=PlanningStatus.FAILED,
                error_message="Planner action server not available"
            ))
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = self._planner_id
        goal_msg.use_start = True

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                callback(PlanningResult(
                    status=PlanningStatus.FAILED,
                    error_message="Planning goal was rejected"
                ))
                return

            with self._planning_lock:
                self._current_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(result_callback)

        def result_callback(future):
            result = future.result()

            with self._planning_lock:
                self._current_goal_handle = None

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                path = result.result.path
                if len(path.poses) > 0:
                    callback(PlanningResult(
                        status=PlanningStatus.SUCCEEDED,
                        path=path
                    ))
                else:
                    callback(PlanningResult(
                        status=PlanningStatus.FAILED,
                        error_message="Planner returned empty path"
                    ))
            else:
                callback(PlanningResult(
                    status=PlanningStatus.FAILED,
                    error_message=f"Planning failed with status {result.status}"
                ))

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)

    def cancel_planning(self) -> bool:
        """
        Cancel current planning request.

        Returns:
            True if cancellation was requested
        """
        with self._planning_lock:
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()
                self._logger.info("Planning cancellation requested")
                return True
        return False

    def is_planning(self) -> bool:
        """Check if a planning request is in progress."""
        with self._planning_lock:
            return self._current_goal_handle is not None
