from __future__ import annotations

import json
import time
from dataclasses import asdict
from typing import Any

import dearpygui.dearpygui as dpg

from exia_control.blue_core.config import ConfigError, ConfigStore, validate_config_dict
from exia_control.blue_core.models import AppConfig


class BlueConfigPanel:
    def __init__(self, config_store: ConfigStore, debounce_seconds: float = 0.8, on_config_applied=None) -> None:
        self._config_store = config_store
        self._debounce_seconds = debounce_seconds
        self._on_config_applied = on_config_applied
        self._last_edit_ts: float | None = None
        self._dirty = False
        self._status_tag = "blue_cfg_status"
        self._field_tags: dict[str, str] = {}
        self._field_types: dict[str, type] = {}

    def build(self) -> None:
        cfg = self._config_store.get()
        with dpg.group(horizontal=False):
            dpg.add_text("Blue Configuration")
            with dpg.table(
                header_row=False,
                policy=dpg.mvTable_SizingStretchProp,
                row_background=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
            ):
                dpg.add_table_column(width_fixed=True, init_width_or_weight=280)
                dpg.add_table_column()
                for key, value in asdict(cfg).items():
                    tag = f"blue_cfg_{key}"
                    self._field_tags[key] = tag
                    self._field_types[key] = type(value)
                    with dpg.table_row():
                        with dpg.table_cell():
                            dpg.add_text(self._format_label(key))
                        with dpg.table_cell():
                            if isinstance(value, bool):
                                dpg.add_checkbox(tag=tag, label=f"##{key}", default_value=value, callback=self._on_change)
                            elif isinstance(value, float):
                                dpg.add_input_float(
                                    tag=tag,
                                    label=f"##{key}",
                                    default_value=value,
                                    callback=self._on_change,
                                    format="%.6f",
                                    step=0.1,
                                    width=-1,
                                )
                            elif isinstance(value, dict):
                                dpg.add_input_text(
                                    tag=tag,
                                    label=f"##{key}",
                                    default_value=json.dumps(value),
                                    callback=self._on_change,
                                    width=-1,
                                    multiline=True,
                                    height=80,
                                )
                            else:
                                dpg.add_input_text(
                                    tag=tag,
                                    label=f"##{key}",
                                    default_value=str(value),
                                    callback=self._on_change,
                                    width=-1,
                                )
            with dpg.group(horizontal=True):
                dpg.add_text("Config saved", tag=self._status_tag)
                dpg.add_button(tag="blue_btn_save_config", label="Save now", callback=self._on_save_clicked)

    def _on_change(self, sender=None, app_data=None, user_data=None) -> None:
        self._dirty = True
        self._last_edit_ts = time.monotonic()
        dpg.set_value(self._status_tag, "Editing...")

    def maybe_autosave(self) -> None:
        if not self._dirty or self._last_edit_ts is None:
            return
        if (time.monotonic() - self._last_edit_ts) < self._debounce_seconds:
            return
        self._save_current()

    def _save_current(self) -> None:
        data: dict[str, Any] = {}
        for key, tag in self._field_tags.items():
            value = dpg.get_value(tag)
            expected = self._field_types.get(key, str)
            if expected is dict and isinstance(value, str):
                stripped = value.strip()
                data[key] = json.loads(stripped) if stripped else {}
            else:
                data[key] = value
        try:
            config = validate_config_dict(data)
            self._config_store.update(config)
            self._config_store.save()
            if self._on_config_applied is not None:
                self._on_config_applied(config)
            dpg.set_value(self._status_tag, "Config saved")
            self._dirty = False
        except ConfigError as exc:
            dpg.set_value(self._status_tag, f"Config error: {exc}")
        except Exception as exc:
            dpg.set_value(self._status_tag, f"Config error: {exc}")

    def _on_save_clicked(self, sender=None, app_data=None, user_data=None) -> None:
        self._save_current()

    @staticmethod
    def _format_label(key: str) -> str:
        return key.replace("_", " ").capitalize()

    @staticmethod
    def refresh_from_config(config: AppConfig) -> None:
        for key, value in asdict(config).items():
            tag = f"blue_cfg_{key}"
            if dpg.does_item_exist(tag):
                dpg.set_value(tag, value)
