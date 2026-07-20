"""Base lifecycle implementation for Titan Brain modules."""

from __future__ import annotations

from abc import ABC, abstractmethod

from core.bus import CognitiveBus
from core.types.identifiers import ModuleId
from core.types.infrastructure import Capability, ModuleState
from core.types.message import CognitiveMessage
from core.types.payload import BasePayload


class BaseModule(ABC):
    """Abstract base class for modules managed by a CognitiveBus."""

    def __init__(self, module_id: ModuleId, bus: CognitiveBus) -> None:
        self.module_id = module_id
        self.bus = bus
        self._state = ModuleState.INITIALIZING

    @property
    def state(self) -> ModuleState:
        """Return the current lifecycle state."""
        return self._state

    def _set_state(self, new_state: ModuleState) -> None:
        self._state = new_state

    @property
    @abstractmethod
    def capability(self) -> Capability:
        """Return the module's declared input and output contracts."""

    async def on_start(self) -> None:
        """Perform optional asynchronous initialization before registration."""
        return None

    async def on_stop(self) -> None:
        """Perform optional asynchronous cleanup after unregistration."""
        return None

    async def start(self) -> None:
        """Initialize and register the module when its state permits it."""
        if self._state not in {
            ModuleState.INITIALIZING,
            ModuleState.FAILED,
            ModuleState.STOPPED,
        }:
            return

        try:
            await self.on_start()
            self.bus.register_module(self.module_id, self.capability, self)
        except Exception:
            self._set_state(ModuleState.FAILED)
            raise
        else:
            self._set_state(ModuleState.READY)

    async def stop(self) -> None:
        """Unregister the module and execute its optional cleanup hook."""
        if self._state is ModuleState.STOPPED:
            return

        self._set_state(ModuleState.STOPPING)
        self.bus.unregister_module(self.module_id)
        try:
            await self.on_stop()
        except Exception:
            self._set_state(ModuleState.FAILED)
            raise
        else:
            self._set_state(ModuleState.STOPPED)

    async def __call__(self, message: CognitiveMessage[BasePayload]) -> None:
        """Deliver a message only while the module is ready."""
        if self._state is not ModuleState.READY:
            return
        await self.handle(message)

    @abstractmethod
    async def handle(self, message: CognitiveMessage[BasePayload]) -> None:
        """Process a validated message."""
