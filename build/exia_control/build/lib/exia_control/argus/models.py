from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

TaskSource = Literal["blue", "override", "none"]
MotionStatus = Literal["idle", "executing", "completed"]


@dataclass(slots=True)
class GeoLineString:
    coordinates: list[tuple[float, float]]

    def to_geojson(self) -> dict[str, Any]:
        return {"type": "LineString", "coordinates": [[lon, lat] for lon, lat in self.coordinates]}


@dataclass(slots=True)
class AssignedTask:
    id: str
    source: TaskSource
    route: GeoLineString


@dataclass(slots=True)
class VehicleState:
    position_geojson: dict[str, Any]
    motion_status: MotionStatus
    active_task_id: str | None
    active_task_source: TaskSource
