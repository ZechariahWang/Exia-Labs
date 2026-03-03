from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import httpx

from exia_control.blue.models import ArgusDTO, EntityTask, UpdateArgusRequest


class BlueClientError(RuntimeError):
    pass


@dataclass(slots=True)
class BlueClient:
    base_url: str
    api_key: str
    timeout_seconds: float
    _client: httpx.AsyncClient | None = None
    _configured_base_url: str | None = None
    _configured_api_key: str | None = None
    _configured_timeout: float | None = None

    def _settings_changed(self, base_url: str, api_key: str, timeout_seconds: float) -> bool:
        return (
            self._configured_base_url != base_url.rstrip("/")
            or self._configured_api_key != api_key
            or self._configured_timeout != float(timeout_seconds)
        )

    async def _rebuild_client(self, *, base_url: str, api_key: str, timeout_seconds: float) -> None:
        if self._client is not None:
            await self._client.aclose()
        self._client = httpx.AsyncClient(
            base_url=base_url.rstrip("/"),
            timeout=timeout_seconds,
            headers={"Authorization": f"Bearer {api_key}"},
        )
        self._configured_base_url = base_url.rstrip("/")
        self._configured_api_key = api_key
        self._configured_timeout = float(timeout_seconds)

    async def __aenter__(self) -> "BlueClient":
        await self._rebuild_client(base_url=self.base_url, api_key=self.api_key, timeout_seconds=self.timeout_seconds)
        return self

    async def __aexit__(self, exc_type, exc, tb) -> None:
        if self._client is not None:
            await self._client.aclose()
            self._client = None
        self._configured_base_url = None
        self._configured_api_key = None
        self._configured_timeout = None

    async def ensure_config(self, *, base_url: str, api_key: str, timeout_seconds: float) -> None:
        if self._client is None:
            raise BlueClientError("BlueClient must be used as an async context manager")
        if self._settings_changed(base_url, api_key, timeout_seconds):
            await self._rebuild_client(base_url=base_url, api_key=api_key, timeout_seconds=timeout_seconds)

    @property
    def client(self) -> httpx.AsyncClient:
        if self._client is None:
            raise BlueClientError("BlueClient must be used as an async context manager")
        return self._client

    async def _request_json(self, method: str, path: str, *, json_body: dict[str, Any] | None = None) -> Any:
        try:
            response = await self.client.request(method, path, json=json_body)
            response.raise_for_status()
            if response.content:
                return response.json()
            return None
        except httpx.HTTPStatusError as exc:
            raise BlueClientError(f"Blue request failed ({exc.response.status_code}): {exc.response.text}") from exc
        except httpx.HTTPError as exc:
            raise BlueClientError(f"Blue request failed: {exc}") from exc

    async def get_argus(self, argus_id: str) -> ArgusDTO | None:
        data = await self._request_json("GET", f"/api/argus/{argus_id}/")
        if data is None:
            return None
        return ArgusDTO.from_dict(data)

    async def get_argus_task(self, argus_id: str) -> EntityTask | None:
        data = await self._request_json("GET", f"/api/argus/{argus_id}/tasks/")
        if data is None:
            return None
        return EntityTask.from_dict(data)

    async def update_argus(self, argus_id: str, request: UpdateArgusRequest) -> ArgusDTO:
        data = await self._request_json("POST", f"/api/argus/{argus_id}/", json_body=request.to_dict())
        if not isinstance(data, dict):
            raise BlueClientError("Unexpected update response from Blue")
        return ArgusDTO.from_dict(data)
