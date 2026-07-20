"""Infrastructure contracts for module capabilities and lifecycle states."""

from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

from core.types.identifiers import CapabilityId
from core.types.payload import BasePayload


class ModuleState(StrEnum):
    """Explicit lifecycle states for all Titan Brain modules."""

    INITIALIZING = "INITIALIZING"
    READY = "READY"
    BUSY = "BUSY"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"


class Capability(BaseModel):
    """Internal declaration of payload contracts a module consumes and produces."""

    model_config = ConfigDict(
        frozen=True,
        extra="forbid",
        strict=True,
        arbitrary_types_allowed=True,
    )

    capability_id: CapabilityId
    consumes: tuple[type[BasePayload], ...]
    produces: tuple[type[BasePayload], ...]
    version: str = Field(min_length=1)
