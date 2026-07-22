# TB-ACT-001A — Actuator Feedback Core and Stop Detection

## Status

Implemented as a dependency-free, immutable core contract. This slice
classifies measured actuator feedback; stop-execution timing and hardware fault
latching remain explicit responsibilities of TB-ACT-001B.

## Contract

`ActuatorFeedback` carries the measured body-frame response:

- `measured_linear_x`, `measured_linear_y`, and `measured_angular_z`;
- the originating `correlation_id` and `sequence_id`;
- a non-negative `timestamp_ns`.

`ActuatorStatus` is an immutable audit result with `state`, `is_stopped`,
`is_fresh`, `is_valid`, correlation, sequence, and evaluation timestamps.

The state enum is deliberately closed:

- `STOPPED` — every measured axis is within its configured stop threshold;
- `MOVING` — the sample is valid and fresh, but at least one axis exceeds its
  threshold;
- `INVALID_DATA` — malformed, non-finite, negative-time, future, or
  desynchronised input;
- `STALE_DATA` — structurally valid feedback at or beyond the freshness budget.

## Safety rules

The stop predicate is inclusive and all-axis:

```text
abs(vx) <= epsilon_stop_linear
and abs(vy) <= epsilon_stop_linear
and abs(wz) <= epsilon_stop_angular
```

Correlation IDs must match the expected control-plane correlation. An optional
expected sequence ID can enforce ingress ordering. Any rejection is fail-closed
with `is_stopped = false`; stale data is not treated as a successful stop.

The freshness boundary is conservative: a sample whose age is exactly the
configured `stale_threshold_ns` is classified as `STALE_DATA`.

## Deliberate exclusions

TB-ACT-001A does not:

- measure or model the stop-execution window;
- latch a persistent hardware fault;
- command a relay, electronic brake, or emergency-stop circuit;
- define ROS 2 messages or publish actuator feedback;
- infer physical acknowledgement from a missing sample.

Those responsibilities belong to TB-ACT-001B through 001D.

## Verification

- 28 focused tests cover the core contract and its invalid-data branches;
- `core/actuator_feedback.py` has 100% statement and branch coverage;
- the full suite passes with 371 tests;
- Ruff and strict Mypy pass for the repository's CI paths.
