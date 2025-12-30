"""
Exia Control - Navigation Module

Provides costmap-aware path planning and obstacle avoidance.

Components:
    path_validator   - Validates paths against costmap for collisions
    planner_interface - Interface to Nav2 global planner (A*)
    replan_manager   - Decides when to trigger path replanning
    costmap_monitor  - Monitors costmap and detects obstacles
"""

from .path_validator import PathValidator, ValidationResult
from .planner_interface import PlannerInterface, PlanningStatus, PlanningResult
from .replan_manager import ReplanManager, ReplanManagerConfig, ReplanReason, ReplanDecision
from .costmap_monitor import CostmapMonitor, CostmapMonitorConfig, ObstacleInfo

__all__ = [
    'PathValidator',
    'ValidationResult',
    'PlannerInterface',
    'PlanningStatus',
    'PlanningResult',
    'ReplanManager',
    'ReplanManagerConfig',
    'ReplanReason',
    'ReplanDecision',
    'CostmapMonitor',
    'CostmapMonitorConfig',
    'ObstacleInfo',
]
