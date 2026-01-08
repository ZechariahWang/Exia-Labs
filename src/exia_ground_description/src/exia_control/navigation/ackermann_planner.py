import math
import threading
import time
from typing import Callable, Optional

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

from .planner_interface import PlannerInterface, PlanningResult, PlanningStatus
from ..perception.costmap_generator import PointCloudCostmapGenerator
from ..planning.ackermann_astar import AckermannAStar


class AckermannPlannerInterface:
    def __init__(
        self,
        node: Node,
        map_size: float = 80.0,
        resolution: float = 0.1,
        use_nav2_fallback: bool = True,
    ):
        self._node = node
        self._logger = node.get_logger()

        self._costmap_generator = PointCloudCostmapGenerator(
            node,
            map_size=map_size,
            resolution=resolution,
            min_obstacle_height=0.2,
            max_obstacle_height=1.5,
            inflation_radius=1.3,
        )

        self._planner = AckermannAStar(
            resolution=resolution,
            angle_bins=36,
            wheelbase=1.3,
            max_steering=0.6,
            robot_footprint=[(1.1, 0.63), (1.1, -0.63), (-1.1, -0.63), (-1.1, 0.63)],
            obstacle_threshold=50,
            max_iterations=50000,
            timeout_ms=500.0,
            goal_tolerance=1.5,
        )

        self._nav2_planner: Optional[PlannerInterface] = None
        if use_nav2_fallback:
            self._nav2_planner = PlannerInterface(node)

        self._planning_lock = threading.Lock()
        self._is_planning = False

        self._logger.info('AckermannPlannerInterface initialized')

    def plan_path_async(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        callback: Callable[[PlanningResult], None]
    ) -> None:
        thread = threading.Thread(
            target=self._plan_worker,
            args=(start, goal, callback),
            daemon=True
        )
        thread.start()

    def _plan_worker(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        callback: Callable[[PlanningResult], None]
    ):
        with self._planning_lock:
            self._is_planning = True

        try:
            result = self._plan_sync(start, goal)
        finally:
            with self._planning_lock:
                self._is_planning = False

        callback(result)

    def _plan_sync(self, start: PoseStamped, goal: PoseStamped) -> PlanningResult:
        start_time = time.time()

        if not self._costmap_generator.is_ready():
            self._logger.warn('Costmap not ready, waiting...')
            wait_start = time.time()
            while not self._costmap_generator.is_ready():
                if time.time() - wait_start > 2.0:
                    if self._nav2_planner:
                        self._logger.info('Falling back to Nav2 planner')
                        return self._nav2_planner.plan_path(start, goal)
                    return PlanningResult(
                        status=PlanningStatus.FAILED,
                        error_message="Costmap not available"
                    )
                time.sleep(0.05)

        costmap = self._costmap_generator.get_costmap()
        if costmap is None:
            if self._nav2_planner:
                return self._nav2_planner.plan_path(start, goal)
            return PlanningResult(
                status=PlanningStatus.FAILED,
                error_message="Failed to get costmap"
            )

        self._planner.set_costmap(costmap)

        start_yaw = self._quaternion_to_yaw(start.pose.orientation)
        goal_yaw = self._quaternion_to_yaw(goal.pose.orientation)

        self._logger.info(
            f'Planning: ({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) -> '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

        path_states = self._planner.plan(
            start.pose.position.x,
            start.pose.position.y,
            start_yaw,
            goal.pose.position.x,
            goal.pose.position.y,
            None
        )

        planning_time = time.time() - start_time

        if path_states is None or len(path_states) == 0:
            self._logger.warn(f'Custom planner failed in {planning_time:.3f}s')
            if self._nav2_planner:
                self._logger.info('Falling back to Nav2 planner')
                return self._nav2_planner.plan_path(start, goal)
            return PlanningResult(
                status=PlanningStatus.FAILED,
                error_message="No path found",
                planning_time=planning_time
            )

        nav_path = self._states_to_path(path_states)

        self._logger.info(
            f'Path found: {len(nav_path.poses)} poses in {planning_time:.3f}s'
        )

        return PlanningResult(
            status=PlanningStatus.SUCCEEDED,
            path=nav_path,
            planning_time=planning_time
        )

    def plan_path(self, start: PoseStamped, goal: PoseStamped,
                  timeout_sec: float = 10.0) -> PlanningResult:
        return self._plan_sync(start, goal)

    def _states_to_path(self, states) -> Path:
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self._node.get_clock().now().to_msg()

        for state in states:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quaternion(state.theta)
            path.poses.append(pose)

        return path

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        start = time.time()
        while not self._costmap_generator.is_ready():
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.1)
        return True

    def cancel_planning(self) -> bool:
        return False

    def is_planning(self) -> bool:
        with self._planning_lock:
            return self._is_planning

    @staticmethod
    def _quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q
