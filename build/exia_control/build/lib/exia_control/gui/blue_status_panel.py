from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from typing import Any

import dearpygui.dearpygui as dpg

from exia_control.blue_core.models import AppConfig
from exia_control.blue_core.config import ConfigStore
from exia_control.blue_core.state import RuntimeState


class BlueStatusPanel:
    def __init__(
        self,
        state: RuntimeState,
        config: ConfigStore,
        on_set_override_task,
        on_clear_override_task,
        on_override_inputs_changed,
        on_start_resume_task,
        on_stop_cancel_task,
        on_set_blue_sync_enabled,
    ) -> None:
        self._state = state
        self._config = config
        self._on_set_override_task = on_set_override_task
        self._on_clear_override_task = on_clear_override_task
        self._on_override_inputs_changed = on_override_inputs_changed
        self._on_start_resume_task = on_start_resume_task
        self._on_stop_cancel_task = on_stop_cancel_task
        self._on_set_blue_sync_enabled = on_set_blue_sync_enabled
        self._suppress_toggle_callbacks = False

    def build(self) -> None:
        with dpg.table(
            header_row=False,
            policy=dpg.mvTable_SizingStretchProp,
            resizable=True,
            borders_innerV=True,
            borders_outerV=True,
            borders_innerH=False,
            borders_outerH=False,
        ):
            dpg.add_table_column(init_width_or_weight=2.2)
            dpg.add_table_column(init_width_or_weight=1.0)
            with dpg.table_row():
                with dpg.table_cell():
                    with dpg.child_window(border=True, width=-1, height=200):
                        dpg.add_text("Blue Connection")
                        dpg.add_separator()
                        dpg.add_slider_int(
                            tag="blue_sync_toggle",
                            label="Blue Sync (0=Off, 1=On)",
                            min_value=0,
                            max_value=1,
                            default_value=1,
                            callback=self._on_blue_sync_toggle_changed,
                            width=55,
                        )
                        dpg.add_text("", tag="blue_status_sync")
                        dpg.add_text("", tag="blue_status_connection")
                        dpg.add_text("", tag="blue_status_last_poll")
                        dpg.add_text("", tag="blue_status_last_push")
                        dpg.add_text("", tag="blue_status_error")

                    with dpg.child_window(border=True, width=-1, height=-1):
                        with dpg.table(
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchProp,
                            borders_innerV=False,
                            borders_outerV=False,
                            borders_innerH=False,
                            borders_outerH=False,
                        ):
                            dpg.add_table_column(init_width_or_weight=2.0)
                            dpg.add_table_column(init_width_or_weight=1.0)
                            with dpg.table_row():
                                with dpg.table_cell():
                                    dpg.add_text("Current Task")
                                with dpg.table_cell():
                                    with dpg.group(horizontal=True):
                                        dpg.add_button(
                                            tag="blue_btn_start_resume",
                                            label="Start/Resume",
                                            callback=lambda s, a, u: self._on_start_resume_task(),
                                            show=False,
                                        )
                                        dpg.add_button(
                                            tag="blue_btn_stop_cancel",
                                            label="Stop/Cancel",
                                            callback=lambda s, a, u: self._on_stop_cancel_task(),
                                            show=False,
                                        )
                        dpg.add_separator()
                        dpg.add_text("", tag="blue_task_source")
                        dpg.add_text("", tag="blue_task_id")
                        dpg.add_text("", tag="blue_task_name")
                        dpg.add_text("", tag="blue_task_type")
                        dpg.add_text("", tag="blue_task_status")
                        dpg.add_text("", tag="blue_task_description")
                        dpg.add_text("", tag="blue_task_reason")
                        dpg.add_separator()
                        dpg.add_button(
                            tag="blue_btn_set_override",
                            label="Set Override Task",
                            callback=lambda s, a, u: self._on_set_override_task(),
                            show=True,
                        )
                        dpg.add_button(
                            tag="blue_btn_clear_override",
                            label="Clear Override Task",
                            callback=lambda s, a, u: self._on_clear_override_task(),
                            show=False,
                        )
                        with dpg.group(tag="blue_override_form", show=False):
                            dpg.add_text("Override Task Details")
                            dpg.add_input_text(
                                tag="blue_override_name",
                                label="Task Title",
                                default_value="Manual Override",
                                width=320,
                                callback=self._on_override_input_changed,
                            )
                            dpg.add_input_text(
                                tag="blue_override_coords",
                                label="Path (lat,lon per line)",
                                multiline=True,
                                width=-1,
                                height=120,
                                default_value="37.7749,-122.4194\n37.7755,-122.4180",
                                callback=self._on_override_input_changed,
                            )
                        dpg.add_text("", tag="blue_override_status")

                with dpg.table_cell():
                    with dpg.child_window(border=True, width=-1, height=-1):
                        dpg.add_text("Current Argus")
                        dpg.add_separator()
                        dpg.add_text("", tag="blue_argus_id")
                        dpg.add_text("", tag="blue_argus_device")
                        dpg.add_text("", tag="blue_argus_task_source")
                        dpg.add_text("", tag="blue_argus_task_name")
                        dpg.add_text("", tag="blue_argus_task_id")
                        dpg.add_text("", tag="blue_argus_task_type")
                        dpg.add_text("", tag="blue_argus_task_status")
                        dpg.add_text("", tag="blue_argus_task_desc")
                        dpg.add_text("", tag="blue_argus_position")
                        dpg.add_text("", tag="blue_argus_additional")
                        with dpg.collapsing_header(label="Raw JSON", default_open=False):
                            dpg.add_input_text(tag="blue_argus_raw_json", multiline=True, readonly=True, width=-1, height=260)

    def refresh(self) -> None:
        cfg = self._config.get()
        snapshot = self._state.snapshot()

        if dpg.does_item_exist("blue_sync_toggle"):
            desired = 1 if cfg.blue_sync_enabled else 0
            current = dpg.get_value("blue_sync_toggle")
            if current != desired:
                self._suppress_toggle_callbacks = True
                dpg.set_value("blue_sync_toggle", desired)
                self._suppress_toggle_callbacks = False

        dpg.set_value("blue_status_sync", f"Blue Sync Enabled: {cfg.blue_sync_enabled}")
        dpg.set_value("blue_status_connection", f"Connection: {snapshot.connection_status}")
        dpg.set_value("blue_status_last_poll", f"Last poll: {snapshot.last_successful_poll_at}")
        dpg.set_value("blue_status_last_push", f"Last push: {snapshot.last_successful_push_at}")
        dpg.set_value("blue_status_error", f"Last error: {snapshot.last_error}")
        dpg.set_value("blue_task_source", f"Task Source: {snapshot.active_task_source}")
        dpg.set_value("blue_task_id", f"Task ID: {snapshot.active_task_id}")
        dpg.set_value("blue_task_reason", f"Task transition: {snapshot.last_task_transition_reason}")

        argus_payload = {}
        if snapshot.current_remote_argus is not None:
            if is_dataclass(snapshot.current_remote_argus):
                argus_payload = asdict(snapshot.current_remote_argus)
            else:
                argus_payload = getattr(snapshot.current_remote_argus, "__dict__", {})
        argus = snapshot.current_remote_argus
        remote_task = snapshot.current_remote_task
        remote_raw = remote_task.raw if (remote_task is not None and isinstance(remote_task.raw, dict)) else {}
        task_name = remote_raw.get("name") or remote_raw.get("title") or (remote_task.description if remote_task else "-")
        task_type = remote_task.type if remote_task else "-"
        task_description = remote_task.description if remote_task else "-"
        task_id = snapshot.active_task_id or (argus.task_id if argus else None) or remote_raw.get("id") or "-"
        if argus is not None and argus.task_status != "none":
            task_status = argus.task_status
        elif snapshot.active_task_id is not None:
            task_status = "executing"
        else:
            task_status = "none"

        dpg.set_value("blue_task_name", f"Task Name: {task_name if task_name else '-'}")
        dpg.set_value("blue_task_type", f"Task Type: {task_type}")
        dpg.set_value("blue_task_status", f"Task Status: {task_status}")
        dpg.set_value("blue_task_description", f"Task Description: {task_description if task_description else '-'}")
        task_available = (
            snapshot.active_task_id is not None
            or snapshot.override_task_candidate is not None
            or snapshot.blue_task_candidate is not None
        )
        if snapshot.task_execution_state == "executing":
            dpg.configure_item("blue_btn_start_resume", show=False)
            dpg.configure_item("blue_btn_stop_cancel", show=task_available)
        else:
            dpg.configure_item("blue_btn_start_resume", show=task_available)
            dpg.configure_item("blue_btn_stop_cancel", show=False)

        override_enabled = snapshot.override_task_candidate is not None
        dpg.configure_item("blue_btn_set_override", show=not override_enabled)
        dpg.configure_item("blue_btn_clear_override", show=override_enabled)
        dpg.configure_item("blue_override_form", show=override_enabled)

        dpg.set_value("blue_argus_id", f"ID: {argus.id if argus else '-'}")
        dpg.set_value("blue_argus_device", f"Device ID: {argus.device_id if argus else '-'}")
        dpg.set_value("blue_argus_task_source", f"Task Source: {snapshot.active_task_source}")
        dpg.set_value("blue_argus_task_name", f"Task Name: {task_name if task_name else '-'}")
        dpg.set_value("blue_argus_task_id", f"Task ID: {task_id}")
        dpg.set_value("blue_argus_task_type", f"Task Type: {task_type}")
        dpg.set_value("blue_argus_task_status", f"Task Status: {task_status}")
        dpg.set_value("blue_argus_task_desc", f"Task Description: {task_description if task_description else '-'}")

        vehicle_position = snapshot.vehicle_state.position_geojson if snapshot.vehicle_state is not None else {}
        reported_position = self._format_coordinate_value(argus.location if argus else vehicle_position)
        dpg.set_value("blue_argus_position", f"Current Position: {reported_position}")
        dpg.set_value("blue_argus_additional", f"Additional Data: {json.dumps(argus.additional_data) if argus else '{}'}")
        dpg.set_value("blue_argus_raw_json", json.dumps(argus_payload, indent=2))

    def _on_blue_sync_toggle_changed(self, sender, app_data, user_data) -> None:
        if self._suppress_toggle_callbacks:
            return
        self._on_set_blue_sync_enabled(bool(app_data))

    def _on_override_input_changed(self, sender, app_data, user_data) -> None:
        name = dpg.get_value("blue_override_name")
        coords = dpg.get_value("blue_override_coords")
        self._on_override_inputs_changed(name, coords)

    @staticmethod
    def _format_coordinate_value(value: dict) -> str:
        if isinstance(value, dict) and value.get("type") == "Point":
            coords = value.get("coordinates")
            if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                try:
                    lon = float(coords[0])
                    lat = float(coords[1])
                    return f"{lat:.6f},{lon:.6f}"
                except (TypeError, ValueError):
                    pass
        return json.dumps(value)

    @staticmethod
    def override_geojson_to_text(route_geojson: dict[str, Any]) -> str:
        coords = route_geojson.get("coordinates") if isinstance(route_geojson, dict) else None
        if not isinstance(coords, list):
            return ""
        lines: list[str] = []
        for item in coords:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                continue
            lon = item[0]
            lat = item[1]
            lines.append(f"{lat},{lon}")
        return "\n".join(lines)

    def apply_override_config_values(self, config: AppConfig) -> None:
        if dpg.does_item_exist("blue_override_name"):
            dpg.set_value("blue_override_name", config.override_task_name or "Manual Override")
        if dpg.does_item_exist("blue_override_coords"):
            dpg.set_value("blue_override_coords", self.override_geojson_to_text(config.override_task_geojson))
