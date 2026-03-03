from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable


class Backoff:
    def __init__(self, initial_seconds: float, max_seconds: float) -> None:
        self._initial = initial_seconds
        self._max = max_seconds
        self._current = initial_seconds

    def reset(self) -> None:
        self._current = self._initial

    def next_delay(self) -> float:
        delay = self._current
        self._current = min(self._max, self._current * 2)
        return delay


async def recurring_job(
    *,
    name: str,
    interval_seconds_supplier: Callable[[], float],
    stop_event: asyncio.Event,
    run_once: Callable[[], Awaitable[None]],
    backoff: Backoff,
    logger: logging.Logger,
) -> None:
    while not stop_event.is_set():
        try:
            await run_once()
            backoff.reset()
            await asyncio.wait_for(stop_event.wait(), timeout=max(0.1, interval_seconds_supplier()))
        except asyncio.TimeoutError:
            continue
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            delay = backoff.next_delay()
            logger.warning("%s failed: %s; retrying in %.2fs", name, exc, delay)
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=delay)
            except asyncio.TimeoutError:
                continue
