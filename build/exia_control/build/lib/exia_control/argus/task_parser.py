from __future__ import annotations

from dataclasses import dataclass

from exia_control.blue.models import EntityTask
from exia_control.argus.models import AssignedTask, GeoLineString


@dataclass(slots=True)
class BlueTaskParseResult:
    task: AssignedTask | None
    error: str | None
    apply_update: bool


def parse_blue_task_route(entity_task: EntityTask | None) -> BlueTaskParseResult:
    if entity_task is None:
        return BlueTaskParseResult(task=None, error=None, apply_update=True)
    if entity_task.type.lower() == "none":
        return BlueTaskParseResult(task=None, error=None, apply_update=True)
    if entity_task.type != "Custom":
        return BlueTaskParseResult(task=None, error=None, apply_update=False)
    if entity_task.raw is None:
        return BlueTaskParseResult(task=None, error="Invalid Custom task: missing payload object", apply_update=False)

    raw = entity_task.raw
    task_id = raw.get("id")
    if not isinstance(task_id, str) or not task_id.strip():
        return BlueTaskParseResult(task=None, error="Invalid Custom task: missing id", apply_update=False)

    path_obj = raw.get("path")
    if not isinstance(path_obj, dict):
        return BlueTaskParseResult(task=None, error="Invalid Custom task: missing path", apply_update=False)

    geometry = path_obj.get("geometry") if path_obj.get("type") == "Feature" else path_obj
    if not isinstance(geometry, dict) or geometry.get("type") != "LineString":
        return BlueTaskParseResult(task=None, error="Invalid Custom task: path must be LineString", apply_update=False)

    coords = geometry.get("coordinates")
    if not isinstance(coords, list) or len(coords) < 2:
        return BlueTaskParseResult(
            task=None,
            error="Invalid Custom task: path coordinates must include at least 2 points",
            apply_update=False,
        )

    parsed: list[tuple[float, float]] = []
    for item in coords:
        if not isinstance(item, (list, tuple)) or len(item) != 2:
            return BlueTaskParseResult(task=None, error="Invalid Custom task: bad coordinate pair", apply_update=False)
        try:
            lat = float(item[0])
            lon = float(item[1])
        except (TypeError, ValueError):
            return BlueTaskParseResult(task=None, error="Invalid Custom task: coordinate values must be numeric", apply_update=False)
        parsed.append((lon, lat))

    task = AssignedTask(id=task_id.strip(), source="blue", route=GeoLineString(coordinates=parsed))
    return BlueTaskParseResult(task=task, error=None, apply_update=True)
