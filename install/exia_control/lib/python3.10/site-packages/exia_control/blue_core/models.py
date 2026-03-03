from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any
from typing import Literal

TaskStatus = Literal["acknowledged", "executing", "complete", "failed", "none"]
ConnectionStatus = Literal["connected", "degraded", "disconnected"]
TaskSource = Literal["blue", "override", "none"]


@dataclass(slots=True)
class AppConfig:
    blue_base_url: str = ""
    blue_api_key: str = ""
    blue_sync_enabled: bool = True
    poll_interval_seconds: float = 5.0
    push_interval_seconds: float = 2.0
    request_timeout_seconds: float = 10.0
    retry_initial_seconds: float = 0.5
    retry_max_seconds: float = 10.0

    argus_id: str = ""
    gps_start_lat: float = 37.7749
    gps_start_lon: float = -122.4194
    gps_step_degrees: float = 0.0001
    argus_poll_interval_seconds: float = 1.0

    override_task_enabled: bool = False
    override_task_name: str = ""
    override_task_geojson: dict[str, Any] = field(default_factory=dict)

    saved_blue_task: dict[str, Any] = field(default_factory=dict)
    saved_argus_task_state: dict[str, Any] = field(default_factory=dict)

    argus_simulated: bool = True
    argus_sim_speed_mps: float = 1.0
    argus_initial_lat: float = 37.7749
    argus_initial_lon: float = -122.4194

    @staticmethod
    def defaults() -> "AppConfig":
        return AppConfig(
            blue_base_url="https://blue.example.com",
            blue_api_key="replace-with-blue-api-key",
            argus_id="argus-device-id",
        )


@dataclass(slots=True)
class LocalTask:
    id: str
    status: TaskStatus
    updated_at: str
    source: Literal["local", "remote_resume"] = "local"

    @staticmethod
    def now_iso() -> str:
        return datetime.now(timezone.utc).isoformat()


@dataclass(slots=True)
class OverrideTaskDefinition:
    name: str
    route_geojson: dict[str, Any]
