# TB-EVAL-003 — Hysteresis and Deterministic Recovery State

Status: implementation baseline; ROS 2 Jazzy container validation pending.

## Purpose

TB-EVAL-003 prevents clearance noise near a safety boundary from repeatedly
releasing and reapplying actuator stop authority. Entry into an unsafe state is
never delayed. Only recovery is filtered.

The existing `evaluate_safety()` function remains stateless and pure. The new
`core/stability.py` layer consumes its instantaneous result together with an
explicit `now_ns`. It never reads a system clock internally.

## States

| State | Effective actuator-facing action |
| --- | --- |
| `OK` | Instantaneous `proceed` or `clamp` |
| `WARNING` | `protective_stop` |
| `E_STOP` | `emergency_stop` |
| `RECOVERY_HOLDING` | The latched `protective_stop` or `emergency_stop` |

An instantaneous `emergency_stop`, `protective_stop`, invalid safe-evidence
record, invalid time, or clock regression is applied in the same invocation.
No hysteresis or timer exists on the unsafe-entry path.

## Recovery contract

After `WARNING` or `E_STOP`, an instantaneous safe result can begin recovery
only when:

```text
observed_clearance >= required_clearance + clearance_hysteresis_m
```

The release boundary is inclusive. Once the boundary is satisfied, accepted
safe observations must continue for at least `recovery_hold_time_ns`. During
that interval the public state is `RECOVERY_HOLDING`, but the effective action
remains the latched stop action.

Any new warning or emergency result cancels the timer immediately. A nominally
safe sample below the hysteretic release threshold also cancels the timer and
returns to the latched unsafe state. The next qualifying sample starts a full
new hold interval.

The ROS adapter also compares consecutive accepted receipt times against its
existing observation-watchdog timeout. A gap beyond that limit forces an
internal `E_STOP` latch before the returning safe sample is evaluated, so two
safe samples separated by a transport outage cannot masquerade as a continuous
recovery window.

## Cold start and clock policy

State is deliberately in-memory and is not reconstructed from `IncidentStore`.
Consequently, the first safe observation after process startup is treated as a
recovery from an unknown `E_STOP`: it must satisfy hysteresis and the complete
hold interval before movement is released. A process restart therefore cannot
bypass recovery.

`monotonic_time_ns` records the highest accepted evaluator time. A regression
forces `E_STOP` and preserves that floor. Recovery cannot start until the
supplied clock catches up.

## ROS 2 integration

The launch profile enables stabilization with explicit parameters:

- `stability_policy_version`;
- `clearance_hysteresis_m`;
- `recovery_hold_time_s` (converted once to integer nanoseconds).

Missing or invalid enabled parameters prevent `SafetyObservationNode` from
initializing. No physical or temporal defaults are silently substituted.

The established `SafetyEvaluationStatus` DDS schema remains unchanged and
continues to carry the effective action consumed by `VelocityArbiterNode`.
Transition diagnostics use a separate `SafetyStabilityStatus` message on
`/safety/stability_status`, reporting state, reason, instantaneous and effective
actions, hold progress, and the active release threshold.

## Audit and persistence

Immediate raw incidents retain their original `DecisionEvidence`. A nominally
safe decision blocked by hysteresis or recovery hold becomes a new incident
evidence record containing the stability policy, transition reason, release
threshold, hold duration, elapsed hold time, and instantaneous action.

## Model boundary

This state machine filters release decisions; it is not a signal averaging
filter and never suppresses a new hazard. Its timing is deterministic in the
provided clock domain but is not a real-time scheduling guarantee. Hardware
stopping behavior, DDS latency, sensor accuracy, and the TB-EVAL-002B
current-velocity versus desired-command boundary remain separate validation
obligations.
