"""
Predefined Paths - Simple coordinate lists

Author: Zechariah Wang
Date: December 2025
"""

from typing import List, Tuple, Optional
from .pure_pursuit import Path


def create_path(points: List[Tuple[float, float]], speed: Optional[float] = None) -> Path:
    """
    Create a Path from a list of (x, y) coordinates.

    Args:
        points: List of (x, y) tuples in meters
        speed: Optional speed for all waypoints (m/s)
    """
    path = Path()
    for x, y in points:
        path.add_point(x, y, speed)
    return path


def create_scaled_path(points: List[Tuple[float, float]], scale: float = 1.0,
                       offset_x: float = 0.0, offset_y: float = 0.0,
                       speed: Optional[float] = None) -> Path:
    """
    Create a scaled and offset Path from coordinates.

    Args:
        points: List of (x, y) tuples
        scale: Scale factor to apply
        offset_x: X offset to add after scaling
        offset_y: Y offset to add after scaling
        speed: Optional speed for all waypoints (m/s)
    """
    scaled = [(x * scale + offset_x, y * scale + offset_y) for x, y in points]
    return create_path(scaled, speed)


def create_line_path(length: float = 4.0, speed: Optional[float] = None) -> Path:
    """Create a straight line path along X axis."""
    points = [(i * length / 4, 0.0) for i in range(5)]
    return create_path(points, speed)


def create_square_path(size: float = 2.0, speed: Optional[float] = None) -> Path:
    """Create a square path."""
    return create_scaled_path(SQUARE_PATH, scale=size / 2.0, speed=speed)


def create_circle_path(radius: float = 1.5, speed: Optional[float] = None) -> Path:
    """Create a circle path."""
    return create_scaled_path(CIRCLE_PATH, scale=radius / 1.5, speed=speed)


def create_figure_eight_path(size: float = 2.0, speed: Optional[float] = None) -> Path:
    """Create a figure-eight path."""
    return create_scaled_path(FIGURE_EIGHT_PATH, scale=size / 2.5, speed=speed)


def create_slalom_path(length: float = 6.0, speed: Optional[float] = None) -> Path:
    """Create a slalom path."""
    return create_scaled_path(SLALOM_PATH, scale=length / 6.0, speed=speed)

LINE_PATH = [
    (0.0, 0.0),
    (1.0, 0.0),
    (2.0, 0.0),
    (3.0, 0.0),
    (4.0, 0.0),
]

SQUARE_PATH = [
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, 2.0),
    (0.0, 2.0),
    (0.0, 0.0),
]

FIGURE_EIGHT_PATH = [
    (0.0, 0.0),
    (1.0, 0.5),
    (2.0, 1.0),
    (2.5, 0.5),
    (2.0, 0.0),
    (1.0, -0.5),
    (0.0, -1.0),
    (-0.5, -0.5),
    (0.0, 0.0),
]

CIRCLE_PATH = [
    (1.5, 0.0),
    (1.0, 1.0),
    (0.0, 1.5),
    (-1.0, 1.0),
    (-1.5, 0.0),
    (-1.0, -1.0),
    (0.0, -1.5),
    (1.0, -1.0),
    (1.5, 0.0),
]

SLALOM_PATH = [
    (0.0, 0.0),
    (1.0, 0.5),
    (2.0, 0.0),
    (3.0, -0.5),
    (4.0, 0.0),
    (5.0, 0.5),
    (6.0, 0.0),
]


def get_path(name: str, speed: Optional[float] = None) -> Path:
    """Get a predefined path by name."""
    paths = {
        'line': LINE_PATH,
        'square': SQUARE_PATH,
        'figure_eight': FIGURE_EIGHT_PATH,
        'circle': CIRCLE_PATH,
        'slalom': SLALOM_PATH,
    }
    points = paths.get(name.lower(), SQUARE_PATH)
    return create_path(points, speed)
