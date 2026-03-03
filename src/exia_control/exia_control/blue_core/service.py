from __future__ import annotations

import asyncio
import json
import logging
import uuid
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from typing import Any

from exia_control.argus.models import AssignedTask, GeoLineString
from exia_control.argus.task_parser import parse_blue_task_route
from exia_control.argus.vehicle import ArgusVehicle
from exia_control.blue.client import BlueClient, BlueClientError
from exia_control.blue.models import ArgusTask, EntityTask, UpdateArgusRequest
from exia_control.blue_core.config import ConfigStore
from exia_control.blue_core.models import LocalTask
from exia_control.blue_core.scheduler import Backoff, recurring_job
from exia_control.blue_core.state import RuntimeState
from exia_control.blue_core.task_coordinator import TaskCoordinator


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class AppService:
    def __init__(
        self, *, config_store: ConfigStore, state: RuntimeState, client: BlueClient, vehicle: ArgusVehicle, logger: logging.Logger
    ) -> None:
        self._config_store = config_store
        self._state = state
        self._client = client
        self._vehicle = vehicle
        self._logger = logger
        self._stop_event = asyncio.Event()
        self._tasks: list[asyncio.Task[None]] = []
        self._task_coordinator = TaskCoordinator(state=self._state, vehicle=self._vehicle)
        self._last_resume_persist_fingerprint: str | None = None

    async def start(self) -> None:
        cfg = self._config_store.get()
        self._state.set_disconnected()

        await self._vehicle.start()
        self._state.set_vehicle_state(self._vehicle.get_state())
        self._restore_persisted_task_state(cfg)

        poll_backoff = Backoff(cfg.retry_initial_seconds, cfg.retry_max_seconds)
        push_backoff = Backoff(cfg.retry_initial_seconds, cfg.retry_max_seconds)
        poll_argus_backoff = Backoff(cfg.retry_initial_seconds, cfg.retry_max_seconds)

        self._tasks = [
            asyncio.create_task(
                recurring_job(
                    name="poll_argus_tasks",
                    interval_seconds_supplier=lambda: self._config_store.get().poll_interval_seconds,
                    stop_event=self._stop_event,
                    run_once=self._poll_once,
                    backoff=poll_backoff,
                    logger=self._logger,
                )
            ),
            asyncio.create_task(
                recurring_job(
                    name="push_argus_state",
                    interval_seconds_supplier=lambda: self._config_store.get().push_interval_seconds,
                    stop_event=self._stop_event,
                    run_once=self._push_once,
                    backoff=push_backoff,
                    logger=self._logger,
                )
            ),
            asyncio.create_task(
                recurring_job(
                    name="poll_argus_state",
                    interval_seconds_supplier=lambda: self._config_store.get().argus_poll_interval_seconds,
                    stop_event=self._stop_event,
                    run_once=self._poll_argus,
                    backoff=poll_argus_backoff,
                    logger=self._logger,
                )
            ),
        ]

    async def stop(self) -> None:
        self._persist_resume_state(force=True)
        self._stop_event.set()
        for task in self._tasks:
            task.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()
        await self._vehicle.stop()

    async def _poll_argus(self) -> None:
        try:
            await self._vehicle.poll()
            vehicle_state = self._vehicle.get_state()
            self._state.set_vehicle_state(vehicle_state)
            self._persist_resume_state()
        except Exception as e:
            self._logger.warning(str(e))

    async def _poll_once(self) -> None:
        cfg = self._config_store.get()
        if not cfg.blue_sync_enabled:
            self._state.set_disconnected()
            return

        await self._sync_blue_client_config(cfg)
        try:
            task = await self._client.get_argus_task(cfg.argus_id)
            self._state.set_remote_task(task)
            parsed = parse_blue_task_route(task)
            if parsed.error:
                self._logger.warning(parsed.error)
                self._state.set_task_validation_error(parsed.error)
                return
            if parsed.apply_update:
                self._task_coordinator.on_blue_task_polled(parsed.task)
                self._persist_resume_state()

        except BlueClientError as exc:
            self._state.set_error(str(exc))
            self._state.set_disconnected()
            raise

    async def _push_once(self) -> None:
        cfg = self._config_store.get()
        if not cfg.blue_sync_enabled:
            self._state.set_disconnected()
            return

        snapshot = self._state.snapshot()
        vehicle_state = snapshot.vehicle_state or self._vehicle.get_state()
        self._state.set_vehicle_state(vehicle_state)
        position = vehicle_state.position_geojson

        await self._sync_blue_client_config(cfg)

        if vehicle_state.active_task_id is None:
            argus_task = ArgusTask(id="none", status="none", updated_at=_utc_now_iso())
        else:
            execution_state = snapshot.task_execution_state
            status = "executing"
            if execution_state == "cancelled":
                status = "failed"
            elif vehicle_state.motion_status == "completed":
                status = "complete"
            elif vehicle_state.motion_status == "idle":
                status = "none"
            argus_task = ArgusTask(id=vehicle_state.active_task_id, status=status, updated_at=_utc_now_iso())

        try:
            payload = UpdateArgusRequest(position_geojson=position, argus_task=argus_task)
            updated = await self._client.update_argus(cfg.argus_id, payload)
            self._state.set_remote_argus(updated)
            self._state.set_push_success(position)
            self._persist_resume_state()
        except BlueClientError as exc:
            self._state.set_error(str(exc))
            self._state.set_disconnected()
            raise

    async def _sync_blue_client_config(self, cfg) -> None:
        ensure_config = getattr(self._client, "ensure_config", None)
        if callable(ensure_config):
            await ensure_config(
                base_url=cfg.blue_base_url,
                api_key=cfg.blue_api_key,
                timeout_seconds=cfg.request_timeout_seconds,
            )

    def set_position(self, *, lat: float, lon: float) -> None:
        self._vehicle.set_position(lat=lat, lon=lon)
        self._state.set_vehicle_state(self._vehicle.get_state())
        self._persist_resume_state()

    def create_local_task(self) -> None:
        self._state.set_local_task(LocalTask(id=str(uuid.uuid4()), status="acknowledged", updated_at=_utc_now_iso()))

    def stop_local_task(self) -> None:
        snapshot = self._state.snapshot()
        task_id = snapshot.local_task.id if snapshot.local_task is not None else str(uuid.uuid4())
        self._state.set_local_task(LocalTask(id=task_id, status="none", updated_at=_utc_now_iso()))

    def resume_remote_task(self) -> None:
        snapshot = self._state.snapshot()
        remote = snapshot.current_remote_argus
        if remote is None or not remote.task_id:
            return
        self._state.set_local_task(
            LocalTask(
                id=remote.task_id,
                status=remote.task_status,
                updated_at=remote.task_updated_at,
                source="remote_resume",
            )
        )

    def set_override_task(self, name: str, route_geojson: dict) -> None:
        coords_raw = route_geojson.get("coordinates")
        if route_geojson.get("type") != "LineString" or not isinstance(coords_raw, list):
            raise ValueError("Override task must be a GeoJSON LineString")
        coords: list[tuple[float, float]] = []
        for item in coords_raw:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                raise ValueError("Invalid coordinate in override route")
            coords.append((float(item[0]), float(item[1])))
        task = AssignedTask(id=f"override-{uuid.uuid4()}", source="override", route=GeoLineString(coordinates=coords))
        self._task_coordinator.on_override_set(task)

        cfg = self._config_store.get()
        cfg.override_task_enabled = True
        cfg.override_task_name = name
        cfg.override_task_geojson = route_geojson
        self._config_store.update(cfg)
        self._config_store.save()
        self._persist_resume_state()

    def clear_override_task(self) -> None:
        self._task_coordinator.on_override_cleared()
        cfg = self._config_store.get()
        cfg.override_task_enabled = False
        cfg.override_task_name = ""
        cfg.override_task_geojson = {}
        self._config_store.update(cfg)
        self._config_store.save()
        self._persist_resume_state()

    def start_resume_task(self) -> None:
        self._task_coordinator.on_operator_start_resume()
        self._persist_resume_state()

    def stop_cancel_task(self) -> None:
        self._task_coordinator.on_operator_cancel()
        self._persist_resume_state()

    def _restore_persisted_task_state(self, cfg) -> None:
        blue_task = self._parse_saved_blue_task(cfg.saved_blue_task)
        saved_argus = cfg.saved_argus_task_state if isinstance(cfg.saved_argus_task_state, dict) else {}
        self._task_coordinator.hydrate_from_persisted(blue_task=blue_task, saved_argus_task_state=saved_argus)
        executing = saved_argus.get("task_execution_state") == "executing" and bool(saved_argus.get("active_task_id"))
        if not executing:
            self._clear_persisted_resume_state()
            return
        self._persist_resume_state(force=True)

    def _parse_saved_blue_task(self, payload: dict[str, Any]) -> AssignedTask | None:
        if not isinstance(payload, dict) or not payload:
            return None
        try:
            if isinstance(payload.get("raw"), dict):
                entity_task = EntityTask(
                    type=str(payload.get("type", "None")),
                    target=payload.get("target"),
                    description=payload.get("description"),
                    purpose=payload.get("purpose"),
                    raw=payload.get("raw"),
                )
            else:
                entity_task = EntityTask.from_dict(payload)
        except Exception:
            self._logger.warning("Ignoring invalid persisted blue task payload")
            return None
        parsed = parse_blue_task_route(entity_task)
        if parsed.error:
            self._logger.warning("Ignoring invalid persisted blue task payload: %s", parsed.error)
            return None
        return parsed.task

    def _entity_task_to_dict(self, task: EntityTask) -> dict[str, Any]:
        raw: dict[str, Any] = {}
        if isinstance(task.raw, dict):
            raw = task.raw
        elif is_dataclass(task):
            payload = asdict(task)
            raw = payload.get("raw") if isinstance(payload.get("raw"), dict) else {}
        result: dict[str, Any] = {"type": task.type}
        if raw:
            result["raw"] = raw
        if task.description is not None:
            result["description"] = task.description
        if task.purpose is not None:
            result["purpose"] = task.purpose
        if task.target is not None:
            result["target"] = task.target
        return result

    def _build_resume_payloads(self) -> tuple[dict[str, Any], dict[str, Any]]:
        snapshot = self._state.snapshot()
        vehicle_state = snapshot.vehicle_state or self._vehicle.get_state()
        active_task_id = snapshot.active_task_id or vehicle_state.active_task_id
        is_executing = snapshot.task_execution_state == "executing" and active_task_id is not None
        if not is_executing:
            return {}, {}

        blue_payload: dict[str, Any] = {}
        if snapshot.current_remote_task is not None:
            blue_payload = self._entity_task_to_dict(snapshot.current_remote_task)
        elif snapshot.blue_task_candidate is not None:
            blue_payload = {
                "type": "Custom",
                "raw": {
                    "id": snapshot.blue_task_candidate.id,
                    "path": snapshot.blue_task_candidate.route.to_geojson(),
                },
            }

        argus_payload = {
            "active_task_id": active_task_id,
            "active_task_source": snapshot.active_task_source,
            "motion_status": vehicle_state.motion_status,
            "position_geojson": vehicle_state.position_geojson,
            "task_execution_state": snapshot.task_execution_state,
        }
        return blue_payload, argus_payload

    def _clear_persisted_resume_state(self) -> None:
        cfg = self._config_store.get()
        if not cfg.saved_blue_task and not cfg.saved_argus_task_state:
            return
        cfg.saved_blue_task = {}
        cfg.saved_argus_task_state = {}
        self._config_store.update(cfg)
        self._config_store.save()
        self._last_resume_persist_fingerprint = json.dumps({"saved_blue_task": {}, "saved_argus_task_state": {}}, sort_keys=True)

    def _persist_resume_state(self, *, force: bool = False) -> None:
        saved_blue_task, saved_argus_task_state = self._build_resume_payloads()
        payload = {"saved_blue_task": saved_blue_task, "saved_argus_task_state": saved_argus_task_state}
        fingerprint = json.dumps(payload, sort_keys=True, default=str)
        if not force and self._last_resume_persist_fingerprint == fingerprint:
            return
        cfg = self._config_store.get()
        cfg.saved_blue_task = saved_blue_task
        cfg.saved_argus_task_state = saved_argus_task_state
        self._config_store.update(cfg)
        self._config_store.save()
        self._last_resume_persist_fingerprint = fingerprint
