# TB-EVAL-006A — Core Governor & Kinematic Limits Engine

## Scope

TB-EVAL-006A adds a dependency-free command governor in
`core/command_governor.py`.  The module is independent of ROS 2 and operates
on immutable command/result contracts, making the dynamic profile directly
unit-testable and replayable.

## Default profile

| Quantity | Default |
| --- | ---: |
| Maximum linear velocity | `1.0 m/s` |
| Maximum angular velocity | `1.0 rad/s` |
| Linear acceleration | `1.0 m/s²` |
| Linear deceleration | `2.0 m/s²` |
| Angular acceleration | `1.0 rad/s²` |
| Angular deceleration | `2.0 rad/s²` |
| Linear jerk | `5.0 m/s³` |
| Angular jerk | `5.0 rad/s³` |

All limits are configurable per deployment and must be finite and strictly
positive.  Negative and positive motion are shaped symmetrically.

## Governing equations

For each body axis, the target is first saturated to its speed limit.  The
requested acceleration is then selected from the asymmetric acceleration or
deceleration limit:

```text
a_req = clamp((v_target - v_current) / dt, -a_limit, +a_limit)
```

The acceleration change is bounded by jerk:

```text
a = clamp(a_req, a_previous - j_max*dt, a_previous + j_max*dt)
v_next = v_current + a*dt
```

The final value is clipped to the target so a reversal cannot overshoot or
accelerate away from the requested command.

## Emergency and timing policy

`GovernorCommand(emergency_stop=True)` immediately emits a zero command,
resets the dynamic baseline, and bypasses ramp/jerk shaping.  It is the
explicit path for a stop asserted by the safety supervisor; an ordinary zero
command remains subject to the configured deceleration and jerk profile.

Timestamps are non-negative integer monotonic nanoseconds.  Invalid timestamps,
`dt <= 0`, clock regression, malformed commands, and non-finite velocity input
all fail closed with a zero output and `is_safe=False`.  A failed evaluation
resets the dynamic baseline so stale acceleration cannot leak into the next
valid command.

## Deferred scope

ROS 2 message and node integration, safety-intent wiring, and end-to-end
container tests are deferred to TB-EVAL-006B–006D.  This slice contains only
the deterministic core and its unit-test contract.
