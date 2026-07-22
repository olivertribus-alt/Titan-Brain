# TB-SAFE-001A — Core Supervisor & Watchdog Matrix

## Scope

TB-SAFE-001A adds a dependency-free external safety-loop supervisor. It
monitors monotonic heartbeats from the control arbiter, actuator feedback
monitor, and odometry source and emits only a relay request. It has no ROS 2,
DDS, GPIO, or relay-driver dependency.

## Contract

The default timeout for each channel is **200 ms** (`200_000_000 ns`). The
timeout is configurable independently for:

- `control_arbiter_timeout_ns`
- `actuator_monitor_timeout_ns`
- `odometry_timeout_ns`
- `initialization_timeout_ns`

The supervisor accepts only non-negative integer monotonic timestamps. A
missing, non-finite, negative, or regressing timestamp is fail-closed.

## State and relay matrix

| State | Condition | Relay request |
| --- | --- | --- |
| `INITIALIZING` | One or more required channels has not registered | `REQUEST_SAFETY_OPEN` |
| `OK` | All channels have fresh, healthy heartbeats | `REQUEST_SAFETY_CLOSED` |
| `TRIPPED` | Timeout, invalid heartbeat, reported error, or clock regression | `REQUEST_SAFETY_OPEN` |
| `HARDWARE_FAULT_LATCH` | Explicit hardware fault latch | `REQUEST_SAFETY_OPEN` |

`TRIPPED` never returns to `OK` automatically. `HARDWARE_FAULT_LATCH` is
sticky for the lifetime of the supervisor instance; the reset protocol is
intentionally outside this pure-core slice and will be specified in 001B.

## Deterministic API

- `SafetySupervisor.receive_heartbeat(channel, timestamp_ns=...)` records one
  local monotonic receipt and evaluates the matrix immediately.
- `SafetySupervisor.evaluate(now_ns=...)` checks initialization and freshness
  without accepting new heartbeats.
- `SafetySupervisor.latch_hardware_fault(...)` enters the sticky hardware
  latch.

The `timestamp_ns=None` convenience path uses `time.monotonic_ns()`. Tests and
adapters should pass explicit timestamps when deterministic replay is needed.

## Verification

`tests/test_safety_supervisor.py` covers initialization, per-channel timeout,
invalid values, clock regression, unhealthy reports, relay invariants, sticky
latching, and immutable heartbeat snapshots. ROS 2 integration and physical
relay feedback are explicitly deferred to TB-SAFE-001B/001C.
