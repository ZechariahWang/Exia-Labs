"""
Path planning and following algorithms for Ackermann vehicles.

Modules:
    pure_pursuit   - Pure Pursuit path following controller
    paths          - Predefined paths (manual coordinate lists)
"""

from .pure_pursuit import PurePursuitController, PurePursuitConfig, Path, PathPoint
from .paths import (
    create_path,
    create_scaled_path,
    create_line_path,
    create_square_path,
    create_circle_path,
    create_figure_eight_path,
    create_slalom_path,
    get_path,
    LINE_PATH,
    SQUARE_PATH,
    FIGURE_EIGHT_PATH,
    CIRCLE_PATH,
    SLALOM_PATH,
)

__all__ = [
    'PurePursuitController',
    'PurePursuitConfig',
    'Path',
    'PathPoint',
    'create_path',
    'create_scaled_path',
    'create_line_path',
    'create_square_path',
    'create_circle_path',
    'create_figure_eight_path',
    'create_slalom_path',
    'get_path',
    'LINE_PATH',
    'SQUARE_PATH',
    'FIGURE_EIGHT_PATH',
    'CIRCLE_PATH',
    'SLALOM_PATH',
]
