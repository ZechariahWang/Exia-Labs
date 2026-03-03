from __future__ import annotations

import threading
import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

import tomli_w

from exia_control.blue_core.models import AppConfig


class ConfigError(ValueError):
    pass


def _require_non_empty_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"Invalid config '{key}': expected non-empty string")
    return value.strip()


def _require_positive_float(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"Invalid config '{key}': expected numeric value") from exc
    if result <= 0:
        raise ConfigError(f"Invalid config '{key}': expected > 0")
    return result


def _require_float(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"Invalid config '{key}': expected numeric value") from exc


def _require_bool(data: dict[str, Any], key: str) -> bool:
    value = data.get(key)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lower = value.strip().lower()
        if lower in {"true", "1", "yes", "on"}:
            return True
        if lower in {"false", "0", "no", "off"}:
            return False
    raise ConfigError(f"Invalid config '{key}': expected boolean value")


def _require_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if value is None:
        return ""
    if isinstance(value, str):
        return value
    raise ConfigError(f"Invalid config '{key}': expected string")


def _require_dict(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key)
    if value is None:
        return {}
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        raw = value.strip()
        if not raw:
            return {}
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid config '{key}': expected JSON object") from exc
        if not isinstance(parsed, dict):
            raise ConfigError(f"Invalid config '{key}': expected JSON object")
        return parsed
    raise ConfigError(f"Invalid config '{key}': expected object")


def validate_config_dict(data: dict[str, Any]) -> AppConfig:
    merged: dict[str, Any] = asdict(AppConfig.defaults())
    merged.update(data)
    cfg = AppConfig(
        blue_base_url=_require_non_empty_str(merged, "blue_base_url"),
        blue_api_key=_require_non_empty_str(merged, "blue_api_key"),
        blue_sync_enabled=_require_bool(merged, "blue_sync_enabled"),
        argus_id=_require_non_empty_str(merged, "argus_id"),
        poll_interval_seconds=_require_positive_float(merged, "poll_interval_seconds"),
        push_interval_seconds=_require_positive_float(merged, "push_interval_seconds"),
        request_timeout_seconds=_require_positive_float(merged, "request_timeout_seconds"),
        retry_initial_seconds=_require_positive_float(merged, "retry_initial_seconds"),
        retry_max_seconds=_require_positive_float(merged, "retry_max_seconds"),
        gps_start_lat=_require_float(merged, "gps_start_lat"),
        gps_start_lon=_require_float(merged, "gps_start_lon"),
        gps_step_degrees=_require_positive_float(merged, "gps_step_degrees"),
        argus_poll_interval_seconds=_require_positive_float(merged, "argus_poll_interval_seconds"),
        argus_simulated=_require_bool(merged, "argus_simulated"),
        argus_sim_speed_mps=_require_positive_float(merged, "argus_sim_speed_mps"),
        argus_initial_lat=_require_float(merged, "argus_initial_lat"),
        argus_initial_lon=_require_float(merged, "argus_initial_lon"),
        override_task_enabled=_require_bool(merged, "override_task_enabled"),
        override_task_name=_require_str(merged, "override_task_name"),
        override_task_geojson=_require_dict(merged, "override_task_geojson"),
        saved_blue_task=_require_dict(merged, "saved_blue_task"),
        saved_argus_task_state=_require_dict(merged, "saved_argus_task_state"),
    )
    if cfg.retry_initial_seconds > cfg.retry_max_seconds:
        raise ConfigError("Invalid config: retry_initial_seconds must be <= retry_max_seconds")
    return cfg


def load_config(path: Path) -> AppConfig:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    return validate_config_dict(data)


def save_config(path: Path, config: AppConfig) -> None:
    payload = asdict(config)
    path.write_text(tomli_w.dumps(payload), encoding="utf-8")


class ConfigStore:
    def __init__(self, path: Path, config: AppConfig) -> None:
        self.path = path
        self._config = config
        self._lock = threading.Lock()

    @classmethod
    def load_or_create(cls, path: Path) -> "ConfigStore":
        if not path.exists():
            defaults = AppConfig.defaults()
            save_config(path, defaults)
            return cls(path, defaults)
        return cls(path, load_config(path))

    def get(self) -> AppConfig:
        with self._lock:
            return AppConfig(**asdict(self._config))

    def update(self, config: AppConfig) -> None:
        with self._lock:
            self._config = config

    def save(self) -> None:
        with self._lock:
            save_config(self.path, self._config)
