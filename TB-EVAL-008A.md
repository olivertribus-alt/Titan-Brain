# TB-EVAL-008A — Dynamic Envelope Evaluation Core

## Scope

`DynamicEnvelopeEvaluator` converts one constant-size `SensorFrame` into
immutable linear and angular motion authority. The core has no ROS dependency
and performs bounded O(1) work over forward clearance, lateral clearance,
confidence, and sensor age.

## Safety model

The evaluator reuses the verified inverse braking model from TB-EVAL-005. The
linear limit is derived from forward clearance. Angular velocity is converted
to tangential swept-footprint speed with a configured conservative radius and
is bounded by the smaller of forward and lateral clearance.

Missing, stale, low-confidence, malformed, or explicitly rejected evidence
produces a zero-authority `FAIL_CLOSED` result. Results include the limiting
zone, stopping-distance evidence, policy version, and machine-readable reason.

## Verification

`tests/test_dynamic_envelope_evaluator.py` covers monotonic limits, stop
boundaries, forward and lateral independence, malformed inputs, timeout
boundaries, immutable evidence, and bit-deterministic repeated evaluation.
