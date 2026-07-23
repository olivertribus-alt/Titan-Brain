# TB-EVAL-007A — Priority Arbitration Core

## Scope

TB-EVAL-007A adds `PrioritySelectorCore`, a dependency-free and deterministic
source selector for the existing command-control authority.  The selector
does not publish `/cmd_vel`, create motion, clamp a permitted-motion envelope,
or replace the TB-EVAL-004/005 arbiter.  It chooses at most one fresh command
frame for the downstream envelope arbiter and TB-EVAL-006 governor.

## Strict precedence

The selector evaluates candidates in this order:

```text
E_STOP / HARDWARE_FAULT / LATCHED_SAFETY_FAULT
    > teleoperation
    > autonomy
```

`PermittedMotionEnvelope` remains a limiter/filter.  It is applied after this
source selection by the existing envelope-aware arbiter, while the command
governor applies acceleration, deceleration, and jerk shaping after the
selection/clamping boundary.  No second ROS output authority is introduced.

## Fail-closed timing and source policy

- The default command freshness budget is `100 ms` and the timeout boundary is
  strict: age equal to the budget is still fresh; age greater than the budget
  is rejected.
- A future command timestamp is rejected as `FUTURE_TIMESTAMP`.
- A current-time regression returns `LATCHED_SAFETY_FAULT` and remains sticky
  for subsequent evaluations; no implicit recovery is possible.
- Invalid current time, invalid system fault state, malformed/non-finite
  frames, unknown source IDs, priority mismatches, and unknown priorities are
  fail-closed.
- Supported source IDs are explicit aliases for autonomy/navigation and
  teleoperation/manual control.  A missing or stale pair of inputs returns
  `NO_VALID_COMMAND_SOURCE` or the most specific validation reason observed.

The implementation has bounded O(1) work: it validates at most one
teleoperation and one autonomy frame and never scans an unbounded collection.
Absolute zero-allocation behavior is not claimed for the Python runtime.

## Verification

`tests/test_priority_selector.py` covers precedence, fault states, stale and
future timestamps, clock regression latching, source validation, malformed
inputs, and immutable result contracts.  ROS 2 integration and status
telemetry are reserved for TB-EVAL-007B; replay and Jazzy E2E fault injection
are reserved for TB-EVAL-007C.
