from __future__ import annotations

from typing import Any

from exia_control.argus.models import AssignedTask
from exia_control.argus.vehicle import ArgusVehicle
from exia_control.blue_core.state import RuntimeState


class TaskCoordinator:
    def __init__(self, *, state: RuntimeState, vehicle: ArgusVehicle) -> None:
        self._state = state
        self._vehicle = vehicle

    def on_blue_task_polled(self, new_blue_task: AssignedTask | None) -> None:
        self._state.set_blue_task_candidate(new_blue_task)
        snapshot = self._state.snapshot()
        if snapshot.override_task_candidate is not None:
            return
        if new_blue_task is None:
            if snapshot.active_task_source == "blue" and snapshot.active_task_id is not None:
                self._vehicle.cancel_task()
                self._state.set_active_task(None, "none", "Blue task cleared, Argus idle")
            return
        if snapshot.active_task_source == "blue" and snapshot.active_task_id == new_blue_task.id:
            return
        if snapshot.active_task_id is not None:
            self._vehicle.cancel_task()
        self._vehicle.assign_task(new_blue_task)
        self._state.set_active_task(new_blue_task.id, "blue", "Blue task changed, started new task")

    def on_override_set(self, override_task: AssignedTask) -> None:
        self._state.set_override_task_candidate(override_task)
        snapshot = self._state.snapshot()
        if snapshot.active_task_id is not None:
            self._vehicle.cancel_task()
        self._vehicle.assign_task(override_task)
        self._state.set_active_task(override_task.id, "override", "Override task set, preempted current task")

    def on_override_cleared(self) -> None:
        self._state.set_override_task_candidate(None)
        snapshot = self._state.snapshot()
        if snapshot.active_task_source == "override" and snapshot.active_task_id is not None:
            self._vehicle.cancel_task()
        blue_task = snapshot.blue_task_candidate
        if blue_task is not None:
            self._vehicle.assign_task(blue_task)
            self._state.set_active_task(blue_task.id, "blue", "Override cleared, resumed Blue task")
            return
        self._state.set_active_task(None, "none", "Override cleared, no Blue task, Argus idle")

    def on_operator_cancel(self) -> None:
        snapshot = self._state.snapshot()
        if snapshot.active_task_id is None:
            return
        self._vehicle.cancel_task()
        self._state.set_task_execution_state("cancelled")
        self._state.set_task_transition_reason("Task cancelled by operator")

    def on_operator_start_resume(self) -> None:
        snapshot = self._state.snapshot()
        desired = snapshot.override_task_candidate or snapshot.blue_task_candidate
        if desired is None:
            return
        self._vehicle.assign_task(desired)
        self._state.set_active_task(desired.id, desired.source, "Task started/resumed by operator")

    def hydrate_from_persisted(self, *, blue_task: AssignedTask | None, saved_argus_task_state: dict[str, Any]) -> None:
        if blue_task is not None:
            self._state.set_blue_task_candidate(blue_task)

        task_id = saved_argus_task_state.get("active_task_id")
        exec_state = saved_argus_task_state.get("task_execution_state")
        source = saved_argus_task_state.get("active_task_source")
        if not isinstance(task_id, str) or not task_id:
            if blue_task is None:
                return
            self._vehicle.assign_task(blue_task)
            self._state.set_active_task(blue_task.id, "blue", "Resumed persisted Blue task on startup")
            return
        if exec_state != "executing":
            return

        if blue_task is not None and blue_task.id == task_id:
            self._vehicle.assign_task(blue_task)
            self._state.set_active_task(blue_task.id, "blue", "Resumed persisted task on startup")
            return

        if source not in {"blue", "override", "none"}:
            source = "blue"
        self._state.set_active_task(task_id, source, "Resumed task metadata from persisted Argus state")
        self._state.set_task_execution_state("executing")
