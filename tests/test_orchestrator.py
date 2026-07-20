"""Tests for LifecycleOrchestrator startup, rollback, and shutdown reports."""

from __future__ import annotations

import asyncio

import pytest

from core.bus import CognitiveBus
from core.module import BaseModule
from core.orchestrator import LifecycleOrchestrator, RetryPolicy
from core.types.identifiers import ModuleId, create_capability_id, create_module_id
from core.types.infrastructure import Capability, ModuleState
from core.types.message import CognitiveMessage
from core.types.payload import BasePayload


class ControlledModule(BaseModule):
    """Configurable module used to test lifecycle orchestration."""

    def __init__(
        self,
        module_id: ModuleId,
        bus: CognitiveBus,
        *,
        start_failures: int = 0,
        stop_fails: bool = False,
    ) -> None:
        super().__init__(module_id, bus)
        self.start_failures = start_failures
        self.stop_fails = stop_fails
        self.start_attempts = 0
        self.stop_attempts = 0

    @property
    def capability(self) -> Capability:
        return Capability(
            capability_id=create_capability_id(f"capability_{self.module_id}"),
            consumes=(),
            produces=(),
            version="1",
        )

    async def on_start(self) -> None:
        self.start_attempts += 1
        if self.start_attempts <= self.start_failures:
            raise RuntimeError("start failed")

    async def on_stop(self) -> None:
        self.stop_attempts += 1
        if self.stop_fails:
            raise RuntimeError("stop failed")

    async def handle(self, message: CognitiveMessage[BasePayload]) -> None:
        return None


def test_start_failure_retries_cleans_up_and_rolls_back_prior_modules() -> None:
    bus = CognitiveBus()
    first = ControlledModule(create_module_id("First"), bus)
    failing = ControlledModule(
        create_module_id("Failing"), bus, start_failures=2
    )
    orchestrator = LifecycleOrchestrator(
        [first, failing],
        RetryPolicy(max_retries=1, initial_delay=0.0),
    )

    with pytest.raises(RuntimeError, match="start failed"):
        asyncio.run(orchestrator.start_all())

    assert first.state.value == "STOPPED"
    assert first.stop_attempts == 1
    assert failing.start_attempts == 2
    assert failing.stop_attempts == 2


def test_stop_all_collects_failures_and_continues_shutdown() -> None:
    bus = CognitiveBus()
    successful = ControlledModule(create_module_id("Successful"), bus)
    failing = ControlledModule(create_module_id("Failing"), bus, stop_fails=True)
    orchestrator = LifecycleOrchestrator(
        [successful, failing],
        RetryPolicy(initial_delay=0.0),
    )

    asyncio.run(orchestrator.start_all())
    report = asyncio.run(orchestrator.stop_all())

    assert report.errors[0].module_id == "Failing"
    assert report.errors[0].error_type == "RuntimeError"
    assert successful.state is ModuleState.STOPPED
    assert failing.state is ModuleState.FAILED
