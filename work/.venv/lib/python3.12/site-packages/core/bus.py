"""Asynchronous in-memory message bus for Titan Brain modules."""

from __future__ import annotations

import asyncio
import time
from typing import Coroutine, Protocol

from pydantic import ValidationError

from core.observer import BusObserver
from core.types.delivery import DeliveryFailure, DeliveryReport
from core.types.identifiers import ModuleId
from core.types.infrastructure import Capability
from core.types.message import Broadcast, CognitiveMessage, IncomingMessageEnvelope
from core.types.payload import BasePayload


class MessageHandler(Protocol):
    """Asynchronous consumer of validated CognitiveBus messages."""

    async def __call__(self, message: CognitiveMessage[BasePayload]) -> None:
        """Handle one validated message."""


class BusError(Exception):
    """Base exception for CognitiveBus operations."""


class UnknownPayloadError(BusError):
    """Raised when a payload type has no registered model."""


class InvalidDestinationError(BusError):
    """Raised when a message cannot be routed to its destination."""


class RegistrationError(BusError):
    """Raised when a module or payload registration is invalid."""


class CognitiveBus:
    """Register module handlers and route validated messages to consumers."""

    def __init__(self) -> None:
        self._handlers: dict[ModuleId, MessageHandler] = {}
        self._payload_models: dict[str, type[BasePayload]] = {}
        self._routing_index: dict[str, set[ModuleId]] = {}
        self._capability_map: dict[ModuleId, Capability] = {}
        self._observers: list[BusObserver] = []

    def add_observer(self, observer: BusObserver) -> None:
        """Attach an observer without changing the dispatch critical path."""
        if observer not in self._observers:
            self._observers.append(observer)

    def remove_observer(self, observer: BusObserver) -> None:
        """Detach an observer when it no longer needs bus events."""
        if observer in self._observers:
            self._observers.remove(observer)

    def register_module(
        self,
        module_id: ModuleId,
        capability: Capability,
        handler: MessageHandler,
    ) -> None:
        """Register a module and index every payload type it consumes."""
        if module_id in self._handlers:
            raise RegistrationError(f"Module already registered: {module_id}")

        payload_types = tuple(
            (self._payload_type_for_model(payload_model), payload_model)
            for payload_model in capability.consumes
        )
        for payload_type, payload_model in payload_types:
            registered_model = self._payload_models.get(payload_type)
            if registered_model is not None and registered_model is not payload_model:
                raise RegistrationError(
                    f"Payload type {payload_type!r} is already bound to "
                    f"{registered_model.__name__}"
                )

        self._handlers[module_id] = handler
        self._capability_map[module_id] = capability
        for payload_type, payload_model in payload_types:
            self._payload_models[payload_type] = payload_model
            self._routing_index.setdefault(payload_type, set()).add(module_id)

    def unregister_module(self, module_id: ModuleId) -> None:
        """Remove a module and all of its payload subscriptions."""
        if module_id not in self._handlers:
            return

        del self._handlers[module_id]
        capability = self._capability_map.pop(module_id)
        for payload_model in capability.consumes:
            payload_type = self._payload_type_for_model(payload_model)
            subscribers = self._routing_index.get(payload_type)
            if subscribers is None:
                continue

            subscribers.discard(module_id)
            if not subscribers:
                del self._routing_index[payload_type]
                del self._payload_models[payload_type]

    async def send(self, raw: IncomingMessageEnvelope) -> DeliveryReport:
        """Validate a raw payload, construct a typed message, and dispatch it."""
        payload_type = raw.payload.get("payload_type")
        if not isinstance(payload_type, str):
            raise UnknownPayloadError("Payload is missing a string payload_type")

        payload_model = self._payload_models.get(payload_type)
        if payload_model is None:
            raise UnknownPayloadError(f"No model is registered for {payload_type!r}")

        try:
            payload = payload_model.model_validate(raw.payload)
        except ValidationError as error:
            raise BusError(f"Payload validation failed: {error}") from error

        message = CognitiveMessage[BasePayload](
            message_id=raw.message_id,
            trace_id=raw.trace_id,
            correlation_id=raw.correlation_id,
            source=raw.source,
            destination=raw.destination,
            timestamp_ns=raw.timestamp_ns,
            priority=raw.priority,
            telemetry=raw.telemetry,
            payload=payload,
        )
        start_ns = time.monotonic_ns()
        for observer in self._observers:
            self._notify(observer.on_message_received(message))

        report = await self._dispatch(message)
        latency_ns = time.monotonic_ns() - start_ns
        for observer in self._observers:
            self._notify(
                observer.on_dispatch_complete(message, report, latency_ns)
            )
        return report

    async def _dispatch(
        self, message: CognitiveMessage[BasePayload]
    ) -> DeliveryReport:
        """Route a validated message and collect delivery outcomes."""
        payload_type = message.payload.payload_type
        subscribers = self._routing_index.get(payload_type, set())

        if isinstance(message.destination, Broadcast):
            targets = subscribers - {message.source}
        else:
            if message.destination not in self._handlers:
                raise InvalidDestinationError(
                    f"Destination module is not registered: {message.destination}"
                )
            if message.destination not in subscribers:
                raise InvalidDestinationError(
                    f"Destination does not consume {payload_type!r}: "
                    f"{message.destination}"
                )
            targets = {message.destination}

        delivered_to: list[ModuleId] = []
        failures: list[DeliveryFailure] = []
        for target in sorted(targets):
            try:
                await self._handlers[target](message)
            except Exception as error:
                failures.append(
                    DeliveryFailure(
                        module_id=target,
                        error=f"{type(error).__name__}: {error}",
                    )
                )
            else:
                delivered_to.append(target)

        return DeliveryReport(
            message_id=message.message_id,
            delivered_to=tuple(delivered_to),
            failures=tuple(failures),
        )

    @staticmethod
    def _payload_type_for_model(payload_model: type[BasePayload]) -> str:
        field = payload_model.model_fields.get("payload_type")
        if field is None or not isinstance(field.default, str) or not field.default:
            raise RegistrationError(
                f"Payload model {payload_model.__name__} must define a "
                "non-empty default payload_type"
            )
        return field.default

    @staticmethod
    def _notify(callback: Coroutine[object, object, None]) -> None:
        """Schedule one observer callback and consume its outcome safely."""
        task = asyncio.create_task(callback)
        task.add_done_callback(CognitiveBus._consume_observer_result)

    @staticmethod
    def _consume_observer_result(task: asyncio.Task[None]) -> None:
        """Prevent observer errors or cancellation from leaking into the bus."""
        try:
            task.result()
        except asyncio.CancelledError:
            return None
        except Exception:
            return None
