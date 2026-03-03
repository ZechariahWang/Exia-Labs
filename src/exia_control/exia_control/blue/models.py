from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

TaskStatus = Literal["acknowledged", "executing", "complete", "failed", "none"]


@dataclass(slots=True)
class ArgusTask:
    id: str
    status: TaskStatus
    updated_at: str

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "ArgusTask":
        return ArgusTask(
            id=str(data["id"]),
            status=data["status"],
            updated_at=str(data["updated_at"]),
        )

    def to_dict(self) -> dict[str, Any]:
        return {"id": self.id, "status": self.status, "updated_at": self.updated_at}


@dataclass(slots=True)
class UpdateArgusRequest:
    position_geojson: dict[str, Any]
    argus_task: ArgusTask

    def to_dict(self) -> dict[str, Any]:
        return {
            "position_geojson": self.position_geojson,
            "argus_task": self.argus_task.to_dict(),
        }


@dataclass(slots=True)
class ArgusDTO:
    id: str
    device_id: str
    task_id: str
    task_status: TaskStatus
    task_updated_at: str
    location: dict[str, Any]
    location_history: dict[str, Any]
    additional_data: dict[str, Any]
    updated_at: str
    created_at: str

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "ArgusDTO":
        return ArgusDTO(
            id=str(data["id"]),
            device_id=str(data["device_id"]),
            task_id=str(data["task_id"]),
            task_status=data["task_status"],
            task_updated_at=str(data["task_updated_at"]),
            location=dict(data["location"]),
            location_history=dict(data["location_history"]),
            additional_data=dict(data["additional_data"]),
            updated_at=str(data["updated_at"]),
            created_at=str(data["created_at"]),
        )


@dataclass(slots=True)
class EntityTask:
    type: str
    target: dict[str, Any] | None = None
    description: str | None = None
    purpose: str | None = None
    raw: dict[str, Any] | None = None

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "EntityTask":
        return EntityTask(
            type=str(data.get("type", "None")),
            target=data.get("target"),
            description=data.get("description"),
            purpose=data.get("purpose"),
            raw=data,
        )
