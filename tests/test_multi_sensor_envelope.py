"""Unit test suite for TB-EVAL-009C Multi-Sensor Envelope Evaluator."""

from core.multi_sensor_envelope import (
    MultiSensorEnvelopeEvaluator,
    SensorReading,
)


def test_conservative_worst_case_min_distance() -> None:
    evaluator = MultiSensorEnvelopeEvaluator(stale_timeout_s=0.200)
    now = 1000.0

    evaluator.update_sensor(SensorReading("lidar_front", 2.5, now))
    evaluator.update_sensor(SensorReading("depth_cam", 1.1, now))
    evaluator.update_sensor(SensorReading("sonar_rear", 3.8, now))

    res = evaluator.evaluate_fusion(now)
    assert not res.is_emergency
    assert res.fused_distance_m == 1.1
    assert res.strictest_sensor_id == "depth_cam"
    assert res.active_sensors_count == 3


def test_critical_sensor_stale_triggers_emergency() -> None:
    evaluator = MultiSensorEnvelopeEvaluator(stale_timeout_s=0.200)
    now = 1000.0

    evaluator.update_sensor(SensorReading("lidar_primary", 3.0, now, is_critical=True))
    evaluator.update_sensor(SensorReading("sonar_aux", 1.5, now, is_critical=False))

    future = now + 0.300
    res = evaluator.evaluate_fusion(future)

    assert res.is_emergency
    assert res.fused_distance_m == 0.0
    assert "CRITICAL_SENSOR_STALE" in (res.emergency_reason or "")
    assert "lidar_primary" in res.stale_sensors


def test_low_confidence_filtering() -> None:
    evaluator = MultiSensorEnvelopeEvaluator(min_confidence=0.6)
    now = 1000.0

    evaluator.update_sensor(SensorReading("sonar", 0.5, now, confidence=0.3))
    evaluator.update_sensor(SensorReading("lidar", 2.0, now, confidence=0.9))

    res = evaluator.evaluate_fusion(now)
    assert not res.is_emergency
    assert res.fused_distance_m == 2.0
    assert res.strictest_sensor_id == "lidar"
