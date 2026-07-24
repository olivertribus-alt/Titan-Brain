"""
Titan Brain - TB-EVAL-009C: Multi-Sensor Fusion Envelope Evaluator
Guarantees O(1) worst-case safety envelope union across heterogenous sensors with fail-closed timeout guards.
"""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorReading:
    """Represents a bounded telemetry frame from an individual sensor."""

    sensor_id: str
    distance_m: float
    timestamp_s: float
    is_critical: bool = False
    confidence: float = 1.0


@dataclass(frozen=True)
class MultiSensorFusionResult:
    """Result of deterministic worst-case sensor envelope fusion."""

    fused_distance_m: float
    strictest_sensor_id: str | None
    active_sensors_count: int
    stale_sensors: list[str]
    is_emergency: bool
    emergency_reason: str | None
    timestamp_s: float


class MultiSensorEnvelopeEvaluator:
    """
    Deterministic O(1) evaluator fusing K heterogeneous sensors.

    Invariants:
    1. Conservative Worst-Case Union: d_fused = min_(k in S_valid)(d_k).
    2. Stale Guard: Any sensor exceeding stale_timeout_s is flagged stale.
    3. Fail-Closed: If any sensor marked is_critical goes STALE or INVALID,
       the evaluator enters EMERGENCY_STOP (fused_distance_m = 0.0).
    """

    def __init__(
        self,
        stale_timeout_s: float = 0.200,
        min_confidence: float = 0.50,
        max_sensors: int = 16,
    ) -> None:
        self.stale_timeout_s = max(0.01, float(stale_timeout_s))
        self.min_confidence = max(0.0, min(1.0, float(min_confidence)))
        self.max_sensors = max_sensors

        self._readings: dict[str, SensorReading] = {}
        self._critical_sensors: set[str] = set()

    def register_critical_sensor(self, sensor_id: str) -> None:
        """Explicitly register a sensor ID as critical for safety system."""
        self._critical_sensors.add(sensor_id)

    def update_sensor(self, reading: SensorReading) -> None:
        """Update last known reading for a sensor in O(1)."""
        if (
            len(self._readings) >= self.max_sensors
            and reading.sensor_id not in self._readings
        ):
            return  # Reject overflow sensors to guarantee O(1) bound

        if reading.is_critical:
            self._critical_sensors.add(reading.sensor_id)

        self._readings[reading.sensor_id] = reading

    def evaluate_fusion(
        self, current_time_s: float | None = None
    ) -> MultiSensorFusionResult:
        """
        Executes O(1) worst-case fusion across all active sensors.

        Returns:
            MultiSensorFusionResult with fused min-distance and emergency status.
        """
        now = current_time_s if current_time_s is not None else time.time()

        stale_sensors: list[str] = []
        valid_readings: list[SensorReading] = []
        critical_stale_or_missing: list[str] = []

        # Check all registered critical sensors for missing state
        for crit_id in sorted(self._critical_sensors):
            if crit_id not in self._readings:
                critical_stale_or_missing.append(crit_id)

        # Process registered readings (bounded loop over K elements -> O(1))
        for sensor_id in sorted(self._readings.keys()):
            reading = self._readings[sensor_id]
            is_stale = (now - reading.timestamp_s) > self.stale_timeout_s

            if is_stale or reading.confidence < self.min_confidence:
                stale_sensors.append(sensor_id)
                if (
                    sensor_id in self._critical_sensors
                    and sensor_id not in critical_stale_or_missing
                ):
                    critical_stale_or_missing.append(sensor_id)
            else:
                valid_readings.append(reading)

        # Fail-closed trigger if any critical sensor is missing or stale
        if critical_stale_or_missing:
            return MultiSensorFusionResult(
                fused_distance_m=0.0,
                strictest_sensor_id=critical_stale_or_missing[0],
                active_sensors_count=len(valid_readings),
                stale_sensors=stale_sensors,
                is_emergency=True,
                emergency_reason=f"CRITICAL_SENSOR_STALE: {', '.join(critical_stale_or_missing)}",
                timestamp_s=now,
            )

        # Fail-closed trigger if no valid sensors remain
        if not valid_readings:
            return MultiSensorFusionResult(
                fused_distance_m=0.0,
                strictest_sensor_id=None,
                active_sensors_count=0,
                stale_sensors=stale_sensors,
                is_emergency=True,
                emergency_reason="NO_VALID_SENSORS_AVAILABLE",
                timestamp_s=now,
            )

        # Conservative Worst-Case Union (Min Distance Wins)
        strictest_reading = min(valid_readings, key=lambda r: r.distance_m)

        return MultiSensorFusionResult(
            fused_distance_m=max(0.0, float(strictest_reading.distance_m)),
            strictest_sensor_id=strictest_reading.sensor_id,
            active_sensors_count=len(valid_readings),
            stale_sensors=stale_sensors,
            is_emergency=False,
            emergency_reason=None,
            timestamp_s=now,
        )
