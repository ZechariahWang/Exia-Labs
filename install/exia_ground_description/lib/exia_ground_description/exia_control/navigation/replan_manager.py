"""
Replan Manager - Decides when to trigger path replanning

Monitors path validity and robot progress to determine when
the global path should be replanned.

Replanning Triggers:
1. Path blocked - Obstacle detected in current path
2. Periodic - Time since last replan exceeds threshold
3. Distance - Robot has traveled significant distance
4. Goal changed - New goal received
5. Deviation - Robot has deviated too far from path

Author: Zechariah Wang
Date: December 2025
"""

from dataclasses import dataclass
from typing import Optional
from enum import Enum
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class ReplanReason(Enum):
    """Reason for triggering replan."""
    NONE = 0
    PATH_BLOCKED = 1
    PERIODIC = 2
    DISTANCE = 3
    GOAL_CHANGED = 4
    PATH_DEVIATION = 5
    NO_PATH = 6
    MANUAL = 7


@dataclass
class ReplanDecision:
    """Decision on whether to replan."""
    should_replan: bool
    reason: ReplanReason = ReplanReason.NONE
    message: str = ""


@dataclass
class ReplanManagerConfig:
    """Configuration for replan manager."""
    # Time-based replanning
    replan_period: float = 10.0          # Seconds between periodic replans
    min_replan_interval: float = 1.0     # Minimum time between any replans

    # Distance-based replanning
    replan_distance: float = 5.0         # Distance traveled to trigger replan

    # Path deviation tolerance
    max_path_deviation: float = 2.0      # Max allowed deviation from path (m)

    # Path validity
    lookahead_poses: int = 20            # Number of poses ahead to validate


class ReplanManager:
    """
    Manages path replanning decisions.

    Tracks robot progress and path validity to determine when
    the global planner should compute a new path.
    """

    def __init__(self, config: Optional[ReplanManagerConfig] = None):
        """
        Initialize replan manager.

        Args:
            config: Configuration parameters, uses defaults if None
        """
        self._config = config or ReplanManagerConfig()

        # Tracking state
        self._last_replan_time: float = 0.0
        self._last_replan_position: Optional[PoseStamped] = None
        self._distance_since_replan: float = 0.0
        self._last_position: Optional[PoseStamped] = None
        self._current_goal: Optional[PoseStamped] = None
        self._manual_replan_requested: bool = False

    def update_position(self, pose: PoseStamped) -> None:
        """
        Update current robot position.

        Args:
            pose: Current robot pose
        """
        if self._last_position is not None:
            # Compute distance traveled
            dx = pose.pose.position.x - self._last_position.pose.position.x
            dy = pose.pose.position.y - self._last_position.pose.position.y
            self._distance_since_replan += math.sqrt(dx*dx + dy*dy)

        self._last_position = pose

    def update_goal(self, goal: PoseStamped) -> bool:
        """
        Update current goal.

        Args:
            goal: New goal pose

        Returns:
            True if goal changed significantly
        """
        if self._current_goal is None:
            self._current_goal = goal
            return True

        # Check if goal changed
        dx = goal.pose.position.x - self._current_goal.pose.position.x
        dy = goal.pose.position.y - self._current_goal.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance > 0.5:  # Goal changed by more than 0.5m
            self._current_goal = goal
            return True

        return False

    def request_replan(self) -> None:
        """Manually request a replan on next check."""
        self._manual_replan_requested = True

    def check_replan_needed(self, current_time: float,
                           current_path: Optional[Path],
                           path_blocked: bool,
                           path_deviation: float = 0.0) -> ReplanDecision:
        """
        Check if replanning is needed.

        Args:
            current_time: Current timestamp (seconds)
            current_path: Current path being followed (None if no path)
            path_blocked: Whether path validator detected collision
            path_deviation: Current deviation from path (meters)

        Returns:
            ReplanDecision indicating if replan is needed
        """
        # Check minimum interval between replans
        time_since_replan = current_time - self._last_replan_time
        if time_since_replan < self._config.min_replan_interval:
            return ReplanDecision(False)

        # Priority 1: Manual request
        if self._manual_replan_requested:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.MANUAL,
                message="Manual replan requested"
            )

        # Priority 2: No path exists
        if current_path is None or len(current_path.poses) == 0:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.NO_PATH,
                message="No valid path available"
            )

        # Priority 3: Path blocked by obstacle
        if path_blocked:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.PATH_BLOCKED,
                message="Path blocked by obstacle"
            )

        # Priority 4: Path deviation exceeded
        if path_deviation > self._config.max_path_deviation:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.PATH_DEVIATION,
                message=f"Path deviation {path_deviation:.2f}m exceeds threshold"
            )

        # Priority 5: Distance threshold
        if self._distance_since_replan >= self._config.replan_distance:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.DISTANCE,
                message=f"Traveled {self._distance_since_replan:.1f}m since last replan"
            )

        # Priority 6: Time threshold
        if time_since_replan >= self._config.replan_period:
            return ReplanDecision(
                should_replan=True,
                reason=ReplanReason.PERIODIC,
                message=f"Periodic replan after {time_since_replan:.1f}s"
            )

        return ReplanDecision(False)

    def mark_replan_completed(self, current_time: float) -> None:
        """
        Mark that a replan has been completed.

        Args:
            current_time: Current timestamp
        """
        self._last_replan_time = current_time
        self._distance_since_replan = 0.0
        self._manual_replan_requested = False

        if self._last_position is not None:
            self._last_replan_position = self._last_position

    def get_time_since_replan(self, current_time: float) -> float:
        """Get seconds since last replan."""
        return current_time - self._last_replan_time

    def get_distance_since_replan(self) -> float:
        """Get meters traveled since last replan."""
        return self._distance_since_replan

    def reset(self) -> None:
        """Reset manager state."""
        self._last_replan_time = 0.0
        self._last_replan_position = None
        self._distance_since_replan = 0.0
        self._last_position = None
        self._current_goal = None
        self._manual_replan_requested = False
