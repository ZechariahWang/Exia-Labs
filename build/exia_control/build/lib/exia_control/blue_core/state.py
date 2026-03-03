from __future__ import annotations

import threading
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Literal, Optional

from exia_control.argus.models import AssignedTask, TaskSource, VehicleState
from exia_control.blue.models import ArgusDTO, EntityTask
from exia_control.blue_core.models import ConnectionStatus, LocalTask


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(slots=True)
class RuntimeSnapshot:
    connection_status: ConnectionStatus
    last_successful_poll_at: str | None
    last_successful_push_at: str | None
    current_remote_task: EntityTask | None
    current_remote_argus: ArgusDTO | None
    local_task: LocalTask | None
    latest_position_geojson: dict[str, Any] | None
    last_error: str | None
    active_task_id: str | None
    active_task_source: TaskSource
    blue_task_candidate: AssignedTask | None
    override_task_candidate: AssignedTask | None
    last_task_transition_reason: str | None
    task_execution_state: Literal["idle", "executing", "cancelled", "completed"]
    vehicle_state: Optional[VehicleState]


@dataclass(slots=True)
class RuntimeState:
    connection_status: ConnectionStatus = "disconnected"
    last_successful_poll_at: str | None = None
    last_successful_push_at: str | None = None
    current_remote_task: EntityTask | None = None
    current_remote_argus: ArgusDTO | None = None

    local_task: LocalTask | None = None
    latest_position_geojson: dict[str, Any] | None = None
    last_error: str | None = None
    active_task_id: str | None = None
    active_task_source: TaskSource = "none"
    blue_task_candidate: AssignedTask | None = None
    override_task_candidate: AssignedTask | None = None
    last_task_transition_reason: str | None = None
    task_execution_state: Literal["idle", "executing", "cancelled", "completed"] = "idle"
    vehicle_state: Optional[VehicleState] = None

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return RuntimeSnapshot(
                connection_status=self.connection_status,
                last_successful_poll_at=self.last_successful_poll_at,
                last_successful_push_at=self.last_successful_push_at,
                current_remote_task=self.current_remote_task,
                current_remote_argus=self.current_remote_argus,
                local_task=self.local_task,
                latest_position_geojson=self.latest_position_geojson,
                last_error=self.last_error,
                active_task_id=self.active_task_id,
                active_task_source=self.active_task_source,
                blue_task_candidate=self.blue_task_candidate,
                override_task_candidate=self.override_task_candidate,
                last_task_transition_reason=self.last_task_transition_reason,
                task_execution_state=self.task_execution_state,
                vehicle_state=self.vehicle_state,
            )

    def set_remote_task(self, task: EntityTask | None) -> None:
        with self._lock:
            self.current_remote_task = task
            self.last_successful_poll_at = _utc_now_iso()
            self.connection_status = "connected"
            self.last_error = None

    def set_remote_argus(self, argus: ArgusDTO | None) -> None:
        with self._lock:
            self.current_remote_argus = argus
            self.connection_status = "connected"
            self.last_error = None

    def set_push_success(self, position_geojson: dict[str, Any]) -> None:
        with self._lock:
            self.last_successful_push_at = _utc_now_iso()
            self.latest_position_geojson = position_geojson
            self.connection_status = "connected"
            self.last_error = None

    def set_error(self, message: str) -> None:
        with self._lock:
            self.last_error = message
            self.connection_status = "degraded"

    def set_task_validation_error(self, message: str) -> None:
        with self._lock:
            self.last_error = message

    def set_disconnected(self) -> None:
        with self._lock:
            self.connection_status = "disconnected"

    def set_local_task(self, task: LocalTask | None) -> None:
        with self._lock:
            self.local_task = task

    def set_blue_task_candidate(self, task: AssignedTask | None) -> None:
        with self._lock:
            self.blue_task_candidate = task

    def set_override_task_candidate(self, task: AssignedTask | None) -> None:
        with self._lock:
            self.override_task_candidate = task

    def set_active_task(self, task_id: str | None, source: TaskSource, reason: str) -> None:
        with self._lock:
            self.active_task_id = task_id
            self.active_task_source = source
            self.last_task_transition_reason = reason
            self.task_execution_state = "executing" if task_id is not None else "idle"

    def set_vehicle_state(self, vehicle_state: Optional[VehicleState]) -> None:
        with self._lock:
            self.vehicle_state = vehicle_state

    def set_task_execution_state(self, state: Literal["idle", "executing", "cancelled", "completed"]) -> None:
        with self._lock:
            self.task_execution_state = state

    def set_task_transition_reason(self, reason: str) -> None:
        with self._lock:
            self.last_task_transition_reason = reason
