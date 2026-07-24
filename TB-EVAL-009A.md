# TB-EVAL-009A — Recovery & Degradation Manager

## Status

Implemented on `feat/tb-eval-009`.

## Objective

TB-EVAL-009A adds an explicit, deterministic lifecycle above the dynamic
motion envelope. Unsafe evidence removes authority immediately. Authority is
restored only after a continuous healthy dwell window, and distance hysteresis
prevents state chatter around the warning boundary.

## Deterministic core

`core/safety_recovery_manager.py` exposes a pure O(1) transition function and
a small stateful façade. Each call consumes constant-size evidence:

- validity and freshness of the envelope diagnostics;
- validity and active state of `SystemFaultStatus`;
- minimum observed clearance;
- time validity;
- the dynamic envelope's current linear and angular limits.

The lifecycle states are:

| State | Entry | Motion authority |
| --- | --- | --- |
| `NORMAL` | healthy evidence and clearance above the release threshold | dynamic envelope limits |
| `DEGRADED` | warning zone or non-critical warning | minimum of the dynamic envelope and the degraded caps |
| `RECOVERY` | first healthy update after startup or an emergency stop | minimum of the dynamic envelope and 0.2 m/s linear recovery cap |
| `EMERGENCY_STOP` | fault, invalid/stale sensor, invalid fault status, stop-margin breach, or invalid/regressed time | exact zero |

The default policy uses:

- stop margin: 0.30 m;
- warning distance: 1.00 m;
- normal release threshold: 1.10 m;
- recovery dwell: 1.00 s;
- degraded caps: 0.50 m/s and 0.50 rad/s;
- recovery caps: 0.20 m/s and 0.50 rad/s.

Entry into `EMERGENCY_STOP` takes one transition and clears any active
recovery timer. A new critical input during recovery restarts the complete
dwell window. Startup is fail-closed: even initially healthy data must complete
the dwell before full authority is granted.

## ROS 2 adapter

`safety_recovery_manager_node` runs at 50 Hz and:

- subscribes to `/safety/envelope_diagnostics`;
- subscribes to `/safety/system_fault_status`;
- publishes `/safety/lifecycle_status`;
- detects stale, future, repeated, or regressed diagnostic timestamps;
- publishes correlated state, transition reason, thresholds, dwell progress,
  and effective authority limits.

`SafetyLifecycleStatus.msg` is a bounded reliable control-plane contract. The
node is included in `safety_control_plane.launch.py`.

## Authoritative enforcement

The production safety arbiter enables `lifecycle_gate_enabled`. It consumes
`/safety/lifecycle_status` after applying the dynamic envelope and before the
kinematic governor:

- missing, stale, future, malformed, or emergency lifecycle status forces the
  existing emergency governor bypass and exact-zero `/cmd_vel`;
- degraded and recovery limits can only reduce authority already granted by
  the dynamic envelope;
- the recovery cap cannot expand a stricter sensor-derived limit;
- `/cmd_vel` remains owned by the single `SafetyVelocityArbiterNode` publisher.

The lifecycle gate defaults to disabled when no deployment configuration is
provided, preserving isolated TB-EVAL-007 tests and non-control-plane users.
The shared production YAML explicitly enables it.

## Verification

The Python suite covers:

- startup and post-fault dwell boundaries;
- immediate hard-fault, sensor, time, and stop-margin overrides;
- degraded-to-normal distance hysteresis;
- interrupted recovery and full dwell restart;
- non-expansion of dynamic envelope limits;
- invalid configuration and evidence;
- immutable, bit-deterministic transition results.

The ROS 2 Jazzy suite additionally covers:

- bounded reliable QoS and topic ownership;
- correlated lifecycle publication;
- recovery and degraded authority;
- sticky timestamp-regression handling;
- fail-closed missing lifecycle status;
- enforced recovery clamping and emergency governor bypass in the final
  `/cmd_vel` authority.
