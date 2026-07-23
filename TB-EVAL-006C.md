# TB-EVAL-006C — E2E Kinematic Profiling and Fault Injection

## Scope

TB-EVAL-006C closes the dependency-free verification loop for the command
governor introduced in 006A and exposed by the ROS 2 adapter in 006B.  The
harness in `core/command_governor_fault_injection.py` replays integer
nanosecond command streams and records the resulting `GovernorResult` trace.
It does not require ROS 2, DDS, or a hardware simulator, so the kinematic
invariants are deterministic and fast to regress.

## Acceptance matrix

| Scenario | Injected condition | Required evidence |
| --- | --- | --- |
| Jerk limit | `0 -> 1.0 m/s` at 50 Hz | `|Δa/Δt| <= 5.0 m/s³` |
| Asymmetric ramp | `a_accel=1.0`, `a_decel=2.0` | braking magnitude may be higher than launch acceleration |
| Emergency cutoff | Safety trip during acceleration | next result is an immediate hard zero with `EMERGENCY_STOP` |
| Stale command | Raw command watchdog expires | adapter selects fail-closed emergency zero |
| Safety timeout | Safety heartbeat/status watchdog expires | adapter selects fail-closed emergency zero |
| Invalid input | Non-finite velocity reaches the boundary | zero output and `is_safe=false` |

The canonical matrix is returned by `standard_fault_cases()` and is executed
by `scripts/verify_tb_eval_006.py`.  A non-zero script exit code means at
least one contract was violated.

## Kinematic evidence

For every result the harness derives linear acceleration and jerk from the
immutable governor output.  The jerk scenario uses the production defaults
(`a_accel=1.0 m/s²`, `a_decel=2.0 m/s²`, `j_max=5.0 m/s³`) with 20 ms samples;
the measured acceleration change stays within the configured jerk envelope.
The asymmetric scenario uses a large jerk budget to isolate the acceleration
and deceleration limits and demonstrates the 2.0 m/s² controlled braking cap.

## Fail-closed and ROS boundary

The emergency, stale-command, and safety-timeout cases all use the explicit
`GovernorCommand(emergency_stop=True)` path that the 006B node selects after
its watchdog detects missing/stale data.  This bypasses normal ramping and
resets the governor baseline before any later command can move the vehicle.
The invalid-input case verifies that NaN/Infinity is rejected by the core.

The local Python gate covers the complete matrix.  ROS 2 Jazzy launch and DDS
transport execution remain covered by the container gate, where
`command_governor_node` is built and exercised with generated message types.

## Reproduction

```bash
python scripts/verify_tb_eval_006.py
python -m pytest tests/test_command_governor_fault_injection.py
```
