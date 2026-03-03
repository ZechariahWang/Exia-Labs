from __future__ import annotations

import math
import threading
import time
from typing import Optional
from dataclasses import dataclass

from exia_control.argus.models import AssignedTask, VehicleState
from exia_control.blue_core.config import ConfigStore

EARTH_RADIUS_METERS = 6_371_000.0


def _to_rad(deg: float) -> float:
    return math.radians(deg)


def _haversine_meters(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    dlon = _to_rad(lon2 - lon1)
    dlat = _to_rad(lat2 - lat1)
    a = math.sin(dlat / 2) ** 2 + math.cos(_to_rad(lat1)) * math.cos(_to_rad(lat2)) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_METERS * math.asin(math.sqrt(a))


def _move_towards(lon: float, lat: float, target_lon: float, target_lat: float, step_m: float) -> tuple[float, float]:
    distance = _haversine_meters(lon, lat, target_lon, target_lat)
    if distance <= step_m or distance == 0:
        return target_lon, target_lat
    frac = step_m / distance
    return lon + (target_lon - lon) * frac, lat + (target_lat - lat) * frac


@dataclass(slots=True)
class _RouteProgress:
    task: AssignedTask
    waypoint_index: int = 0


class SimulatedArgusVehicle:
    def __init__(self, *, config_store: ConfigStore) -> None:
        cfg = config_store.get()

        self.config_store = config_store
        self._lon = float(cfg.argus_initial_lon)
        self._lat = float(cfg.argus_initial_lat)
        self._speed_mps = float(cfg.argus_sim_speed_mps)
        self._route: _RouteProgress | None = None
        self._motion_status: str = "idle"
        self.last_tick_time = time.perf_counter()
        self._lock = threading.Lock()

    async def start(self) -> None:
        pass

    async def stop(self) -> None:
        pass

    async def poll(self) -> None:
        curr_time = time.perf_counter()
        delta = curr_time - self.last_tick_time
        self._tick(delta)
        self.last_tick_time = time.perf_counter()

    def assign_task(self, task: AssignedTask) -> None:
        with self._lock:
            self._route = _RouteProgress(task=task, waypoint_index=0)
            self._motion_status = "executing"

    def cancel_task(self) -> None:
        with self._lock:
            self._route = None
            self._motion_status = "idle"

    def set_position(self, *, lat: float, lon: float) -> None:
        with self._lock:
            self._lat = float(lat)
            self._lon = float(lon)

    def get_state(self) -> Optional[VehicleState]:
        with self._lock:
            active_task_id = self._route.task.id if self._route is not None else None
            active_source = self._route.task.source if self._route is not None else "none"
            return VehicleState(
                position_geojson={"type": "Point", "coordinates": [self._lon, self._lat]},
                motion_status=self._motion_status,
                active_task_id=active_task_id,
                active_task_source=active_source,
            )

    def _tick(self, dt_seconds: float) -> None:
        cfg = self.config_store.get()
        with self._lock:
            if not cfg.argus_simulated:
                return
            if self._route is None:
                self._motion_status = "idle"
                return
            coords = self._route.task.route.coordinates
            if self._route.waypoint_index >= len(coords):
                self._motion_status = "completed"
                return
            target_lon, target_lat = coords[self._route.waypoint_index]
            step = self._speed_mps * dt_seconds
            self._lon, self._lat = _move_towards(self._lon, self._lat, target_lon, target_lat, step)
            if self._lon == target_lon and self._lat == target_lat:
                self._route.waypoint_index += 1
                if self._route.waypoint_index >= len(coords):
                    self._motion_status = "completed"
                else:
                    self._motion_status = "executing"
