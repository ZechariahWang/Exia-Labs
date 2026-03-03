from __future__ import annotations

from typing import Protocol, Optional

from exia_control.argus.models import AssignedTask, VehicleState


class ArgusVehicle(Protocol):
    async def start(self) -> None:
        ...

    async def stop(self) -> None:
        ...

    async def poll(self) -> None:
        ...

    def assign_task(self, task: AssignedTask) -> None:
        ...

    def cancel_task(self) -> None:
        ...

    def get_state(self) -> Optional[VehicleState]:
        ...

    def set_position(self, *, lat: float, lon: float) -> None:
        ...
