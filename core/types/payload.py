"""Base contract for all Titan Brain data transfer objects."""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, Field


class BasePayload(BaseModel):
    """Base class for versioned, self-describing payloads."""

    model_config = ConfigDict(frozen=True, extra="forbid", strict=True)

    schema_version: int = Field(ge=1)
    payload_type: str = Field(min_length=1)
