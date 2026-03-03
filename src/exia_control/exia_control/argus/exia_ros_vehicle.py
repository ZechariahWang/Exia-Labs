from __future__ import annotations

import threading
from typing import Optional

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from exia_control.argus.models import AssignedTask, VehicleState
from exia_msgs.msg import NavigationGoal


class ExiaRosVehicle:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._goal_pub = node.create_publisher(NavigationGoal, '/navigation/goal', 10)
        self._cancel_client = node.create_client(Trigger, '/navigation/cancel')

        node.create_subscription(NavSatFix, '/gnss/fix', self._gps_cb, 10)
        node.create_subscription(String, '/navigation/status', self._status_cb, 10)

        self._lat = 0.0
        self._lon = 0.0
        self._gps_valid = False
        self._motion_status: str = "idle"
        self._active_task_id: str | None = None
        self._active_task_source: str = "none"
        self._lock = threading.Lock()
        self._waypoints: list[tuple[float, float]] = []
        self._waypoint_index = 0
        self._last_status = ""

    async def start(self) -> None:
        pass

    async def stop(self) -> None:
        self.cancel_task()

    async def poll(self) -> None:
        with self._lock:
            if self._motion_status != "executing":
                return
            if not self._waypoints:
                return
            if self._last_status == "GOAL_REACHED":
                self._waypoint_index += 1
                self._last_status = ""
                if self._waypoint_index >= len(self._waypoints):
                    self._motion_status = "completed"
                    return
                self._publish_waypoint(self._waypoint_index)
            elif self._last_status in ("PATH_BLOCKED", "PATH_ENDED", "OBSTACLE_STOPPED"):
                self._motion_status = "completed"

    def assign_task(self, task: AssignedTask) -> None:
        with self._lock:
            self._waypoints = list(task.route.coordinates)
            self._waypoint_index = 0
            self._active_task_id = task.id
            self._active_task_source = task.source
            self._motion_status = "executing"
            self._last_status = ""
            if self._waypoints:
                self._publish_waypoint(0)

    def cancel_task(self) -> None:
        with self._lock:
            self._waypoints = []
            self._waypoint_index = 0
            self._active_task_id = None
            self._active_task_source = "none"
            self._motion_status = "idle"
            self._last_status = ""
        if self._cancel_client.service_is_ready():
            req = Trigger.Request()
            self._cancel_client.call_async(req)

    def get_state(self) -> Optional[VehicleState]:
        with self._lock:
            position = {"type": "Point", "coordinates": [self._lon, self._lat]}
            return VehicleState(
                position_geojson=position,
                motion_status=self._motion_status,
                active_task_id=self._active_task_id,
                active_task_source=self._active_task_source,
            )

    def set_position(self, *, lat: float, lon: float) -> None:
        raise NotImplementedError("Cannot override position on real robot")

    def _publish_waypoint(self, index: int) -> None:
        if index >= len(self._waypoints):
            return
        lon, lat = self._waypoints[index]
        msg = NavigationGoal()
        msg.coord_type = "latlon"
        msg.lat = lat
        msg.lon = lon
        self._goal_pub.publish(msg)

    def _gps_cb(self, msg: NavSatFix) -> None:
        with self._lock:
            self._lat = msg.latitude
            self._lon = msg.longitude
            self._gps_valid = True

    def _status_cb(self, msg: String) -> None:
        parts = msg.data.strip().split()
        if not parts:
            return
        status = parts[0]
        with self._lock:
            self._last_status = status
