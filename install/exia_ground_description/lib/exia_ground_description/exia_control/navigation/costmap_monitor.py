"""
Costmap Monitor - Monitors costmap and detects obstacles

Subscribes to costmap topics and provides utilities for
obstacle detection and clearance checking.

Author: Zechariah Wang
Date: December 2025
"""

from dataclasses import dataclass
from typing import Optional, List, Tuple, Callable
import math
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


@dataclass
class ObstacleInfo:
    """Information about detected obstacle."""
    distance: float        # Distance to nearest obstacle
    angle: float          # Angle to nearest obstacle (relative to robot)
    position: Tuple[float, float]  # World position of obstacle
    cost: int             # Costmap value at obstacle


@dataclass
class CostmapMonitorConfig:
    """Configuration for costmap monitor."""
    local_costmap_topic: str = '/local_costmap/costmap'
    global_costmap_topic: str = '/global_costmap/costmap'
    lethal_threshold: int = 253
    high_cost_threshold: int = 200
    check_radius: float = 5.0  # Radius to check for obstacles (m)


class CostmapMonitor:
    """
    Monitors costmaps and provides obstacle detection utilities.

    Subscribes to both local and global costmaps and provides
    methods to check for obstacles, compute clearance, and
    detect potential collisions.
    """

    def __init__(self, node: Node,
                 config: Optional[CostmapMonitorConfig] = None,
                 costmap_callback: Optional[Callable[[OccupancyGrid], None]] = None):
        """
        Initialize costmap monitor.

        Args:
            node: ROS2 node for subscriptions
            config: Configuration parameters
            costmap_callback: Optional callback when costmap updates
        """
        self._node = node
        self._config = config or CostmapMonitorConfig()
        self._logger = node.get_logger()
        self._user_callback = costmap_callback

        # Costmap data
        self._local_costmap: Optional[OccupancyGrid] = None
        self._global_costmap: Optional[OccupancyGrid] = None
        self._local_data: Optional[np.ndarray] = None
        self._global_data: Optional[np.ndarray] = None
        self._lock = threading.Lock()

        # QoS for costmap subscriptions
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribe to costmaps
        self._local_sub = node.create_subscription(
            OccupancyGrid,
            self._config.local_costmap_topic,
            self._local_costmap_callback,
            costmap_qos
        )

        self._global_sub = node.create_subscription(
            OccupancyGrid,
            self._config.global_costmap_topic,
            self._global_costmap_callback,
            costmap_qos
        )

    def _local_costmap_callback(self, msg: OccupancyGrid) -> None:
        """Handle local costmap update."""
        with self._lock:
            self._local_costmap = msg
            self._local_data = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )

        if self._user_callback:
            self._user_callback(msg)

    def _global_costmap_callback(self, msg: OccupancyGrid) -> None:
        """Handle global costmap update."""
        with self._lock:
            self._global_costmap = msg
            self._global_data = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )

    def get_local_costmap(self) -> Optional[OccupancyGrid]:
        """Get current local costmap."""
        with self._lock:
            return self._local_costmap

    def get_global_costmap(self) -> Optional[OccupancyGrid]:
        """Get current global costmap."""
        with self._lock:
            return self._global_costmap

    def has_costmaps(self) -> bool:
        """Check if costmaps are available."""
        with self._lock:
            return self._local_costmap is not None

    def check_clearance(self, x: float, y: float, radius: float,
                       use_global: bool = False) -> float:
        """
        Check clearance around a position.

        Args:
            x, y: Position to check (world coordinates)
            radius: Radius to check around position
            use_global: Use global costmap instead of local

        Returns:
            Minimum clearance distance (0 if in collision)
        """
        with self._lock:
            if use_global:
                costmap = self._global_costmap
                data = self._global_data
            else:
                costmap = self._local_costmap
                data = self._local_data

            if costmap is None or data is None:
                return float('inf')

        info = costmap.info

        # Convert to grid coordinates
        center_gx = int((x - info.origin.position.x) / info.resolution)
        center_gy = int((y - info.origin.position.y) / info.resolution)
        radius_cells = int(radius / info.resolution)

        min_distance = float('inf')

        # Check cells within radius
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                gx = center_gx + dx
                gy = center_gy + dy

                if 0 <= gx < info.width and 0 <= gy < info.height:
                    cost = int(data[gy, gx])
                    if cost >= self._config.lethal_threshold:
                        # Compute distance to this obstacle cell
                        dist = math.sqrt(dx*dx + dy*dy) * info.resolution
                        min_distance = min(min_distance, dist)

        return min_distance

    def find_nearest_obstacle(self, x: float, y: float,
                             max_range: float = 10.0,
                             use_global: bool = False) -> Optional[ObstacleInfo]:
        """
        Find nearest obstacle to position.

        Args:
            x, y: Position to search from
            max_range: Maximum search range
            use_global: Use global costmap

        Returns:
            ObstacleInfo if obstacle found, None otherwise
        """
        with self._lock:
            if use_global:
                costmap = self._global_costmap
                data = self._global_data
            else:
                costmap = self._local_costmap
                data = self._local_data

            if costmap is None or data is None:
                return None

        info = costmap.info

        # Convert to grid coordinates
        center_gx = int((x - info.origin.position.x) / info.resolution)
        center_gy = int((y - info.origin.position.y) / info.resolution)
        range_cells = int(max_range / info.resolution)

        nearest_dist = float('inf')
        nearest_pos = None
        nearest_cost = 0

        # Search for obstacles
        for dy in range(-range_cells, range_cells + 1):
            for dx in range(-range_cells, range_cells + 1):
                gx = center_gx + dx
                gy = center_gy + dy

                if 0 <= gx < info.width and 0 <= gy < info.height:
                    cost = int(data[gy, gx])
                    if cost >= self._config.lethal_threshold:
                        dist = math.sqrt(dx*dx + dy*dy) * info.resolution
                        if dist < nearest_dist:
                            nearest_dist = dist
                            # Convert back to world coordinates
                            wx = gx * info.resolution + info.origin.position.x
                            wy = gy * info.resolution + info.origin.position.y
                            nearest_pos = (wx, wy)
                            nearest_cost = cost

        if nearest_pos is None:
            return None

        # Compute angle to obstacle
        angle = math.atan2(nearest_pos[1] - y, nearest_pos[0] - x)

        return ObstacleInfo(
            distance=nearest_dist,
            angle=angle,
            position=nearest_pos,
            cost=nearest_cost
        )

    def is_position_free(self, x: float, y: float,
                        use_global: bool = False) -> bool:
        """
        Check if position is free of obstacles.

        Args:
            x, y: Position to check
            use_global: Use global costmap

        Returns:
            True if position is free
        """
        cost = self.get_cost_at(x, y, use_global)
        return cost < self._config.lethal_threshold

    def get_cost_at(self, x: float, y: float,
                   use_global: bool = False) -> int:
        """
        Get costmap value at position.

        Args:
            x, y: World coordinates
            use_global: Use global costmap

        Returns:
            Costmap value (0-255)
        """
        with self._lock:
            if use_global:
                costmap = self._global_costmap
                data = self._global_data
            else:
                costmap = self._local_costmap
                data = self._local_data

            if costmap is None or data is None:
                return 0

        info = costmap.info

        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)

        if 0 <= gx < info.width and 0 <= gy < info.height:
            return int(data[gy, gx])

        return 255  # Unknown

    def get_obstacle_positions(self, x: float, y: float,
                              radius: float = 5.0,
                              use_global: bool = False) -> List[Tuple[float, float]]:
        """
        Get list of obstacle positions within radius.

        Args:
            x, y: Center position
            radius: Search radius
            use_global: Use global costmap

        Returns:
            List of (x, y) obstacle positions
        """
        obstacles = []

        with self._lock:
            if use_global:
                costmap = self._global_costmap
                data = self._global_data
            else:
                costmap = self._local_costmap
                data = self._local_data

            if costmap is None or data is None:
                return obstacles

        info = costmap.info

        center_gx = int((x - info.origin.position.x) / info.resolution)
        center_gy = int((y - info.origin.position.y) / info.resolution)
        radius_cells = int(radius / info.resolution)

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx*dx + dy*dy > radius_cells*radius_cells:
                    continue

                gx = center_gx + dx
                gy = center_gy + dy

                if 0 <= gx < info.width and 0 <= gy < info.height:
                    cost = int(data[gy, gx])
                    if cost >= self._config.lethal_threshold:
                        wx = gx * info.resolution + info.origin.position.x
                        wy = gy * info.resolution + info.origin.position.y
                        obstacles.append((wx, wy))

        return obstacles
