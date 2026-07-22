# TB-ACT-001B — Stop Acknowledgement Monitor and Hardware Latching

## Status

Implemented as a pure-Python stateful monitor layered on the TB-ACT-001A
feedback evaluator. The module performs no ROS or hardware I/O; an adapter may
publish its immutable results and drive the physical escalation path later.

## Stop execution contract

`StopRequest` starts a budget at `requested_timestamp_ns`. A fresh actuator
sample that is classified as `STOPPED` before the inclusive
`stop_budget_ns` boundary emits `StopAcknowledgement` containing:

- the original `correlation_id`;
- the stop-request sequence and actuator-feedback sequence;
- request and acknowledgement timestamps;
- the measured three-axis velocities and derived stop latency.

Valid but moving feedback keeps the monitor in `STOP_PENDING`. Missing feedback
is also pending until the budget expires. At the budget boundary the monitor
enters `HARDWARE_FAULT_LATCH` with reason `STOP_TIMEOUT`.

## Fail-closed and latch rules

While a stop is pending, any stale, malformed, non-finite, future-timestamped,
correlation-mismatched, or replayed/regressing feedback immediately enters the
persistent `HARDWARE_FAULT_LATCH` state. The transition result retains the
specific fault reason for audit; subsequent calls remain latched and cannot
produce an acknowledgement or clear the fault through time or feedback.

Clock regression during the stop window is treated as a hardware-assurance
fault. A new stop request resets only a successful acknowledgement; it never
overrides an active latch.

## Explicit recovery

Only `reset_fault` (or its `reset` convenience wrapper) with a valid,
timestamped `HardwareResetRequest` can clear the latch. Reset requests that are
malformed, future-dated, or earlier than the latch timestamp are rejected and
leave the monitor latched.

## Deliberate exclusions

TB-ACT-001B does not:

- publish ROS 2 messages or call a motor-driver API;
- actuate a relay or emergency-stop circuit itself;
- infer a physical stop from absent feedback;
- provide encoder fault recovery or hardware reset implementation.

Those integration and fault-injection responsibilities are reserved for
TB-ACT-001C and TB-ACT-001D.

## Verification

- 30 focused tests cover acknowledgement, timeout, stale/invalid data,
  correlation and sequence faults, clock faults, latch persistence, and reset;
- `core/stop_ack_monitor.py` has 100% statement and branch coverage;
- the full suite passes with 401 tests;
- Ruff and strict Mypy pass for the repository's CI paths.
