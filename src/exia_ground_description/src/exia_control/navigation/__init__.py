"""
Exia Control - Navigation Module

Provides costmap-aware path planning and obstacle avoidance.

Components:
    path_validator    - Validates paths against costmap for collisions
    planner_interface - Interface to Nav2 global planner (A*)
"""

from .path_validator import PathValidator, ValidationResult
from .planner_interface import PlannerInterface, PlanningStatus, PlanningResult

__all__ = [
    'PathValidator',
    'ValidationResult',
    'PlannerInterface',
    'PlanningStatus',
    'PlanningResult',
]
