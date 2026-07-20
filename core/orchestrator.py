"""Deterministic lifecycle orchestration for Titan Brain modules."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import NamedTuple

from core.module import BaseModule
from core.types.infrastructure import ModuleState


class ShutdownFailure(NamedTuple):
    """Serializable information about one failed module shutdown."""

    module_id: str
    error_type: str
    message: str


class ShutdownReport(NamedTuple):
    """Immutable aggregate result of an orchestrated shutdown."""

    errors: tuple[ShutdownFailure, ...]


@dataclass(frozen=True)
class RetryPolicy:
    """Retry parameters applied to failed module startup attempts."""

    max_retries: int = 3
    initial_delay: float = 1.0
    backoff_factor: float = 2.0

    def __post_init__(self) -> None:
        if self.max_retries < 0:
            raise ValueError("max_retries must be greater than or equal to zero")
        if self.initial_delay < 0:
            raise ValueError("initial_delay must be greater than or equal to zero")
        if self.backoff_factor < 1:
            raise ValueError("backoff_factor must be greater than or equal to one")


class LifecycleOrchestrator:
    """Start, retry, rollback, and stop modules in deterministic order."""

    def __init__(
        self, modules: list[BaseModule], policy: RetryPolicy | None = None
    ) -> None:
        self._modules = tuple(modules)
        self._policy = policy if policy is not None else RetryPolicy()
        self._started_modules: list[BaseModule] = []

    async def start_all(self) -> None:
        """Start all modules or roll back every successful prior start."""
        if self._started_modules:
            raise RuntimeError("Orchestrator already has started modules")

        try:
            for module in self._modules:
                await self._start_with_retry(module)
                self._started_modules.append(module)
        except Exception:
            await self.stop_all()
            raise

    async def stop_all(self) -> ShutdownReport:
        """Stop started modules in reverse order and collect all failures."""
        failures: list[ShutdownFailure] = []
        for module in reversed(self._started_modules):
            try:
                await module.stop()
            except Exception as error:
                failures.append(
                    ShutdownFailure(
                        module_id=str(module.module_id),
                        error_type=type(error).__name__,
                        message=str(error),
                    )
                )

        self._started_modules.clear()
        return ShutdownReport(errors=tuple(failures))

    async def _start_with_retry(self, module: BaseModule) -> None:
        delay = self._policy.initial_delay
        last_error: Exception | None = None

        for attempt in range(self._policy.max_retries + 1):
            try:
                await module.start()
                if module.state is ModuleState.READY:
                    return
                raise RuntimeError(f"Module did not reach READY: {module.module_id}")
            except Exception as error:
                last_error = error
                await self._cleanup_failed_start(module)

            if attempt < self._policy.max_retries:
                await asyncio.sleep(delay)
                delay *= self._policy.backoff_factor

        if last_error is None:
            raise RuntimeError(f"Failed to start module {module.module_id}")
        raise last_error

    @staticmethod
    async def _cleanup_failed_start(module: BaseModule) -> None:
        """Run best-effort module cleanup while preserving the startup error."""
        try:
            await module.stop()
        except Exception:
            return None
