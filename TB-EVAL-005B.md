# TB-EVAL-005B — Directional Limits and Fail-Closed Angular Policy

## Status

Implemented as a dependency-free control-plane core contract. ROS transport,
correlation, expiry, and live arbiter enforcement remain in TB-EVAL-005C/005D.

## Directional translation authority

TB-EVAL-005B applies the TB-EVAL-005A inverse braking model independently to
the robot-relative forward, reverse, left, and right clearances. The resulting
limits map to the ROS body-frame convention:

```text
-reverse_limit <= linear_x <= forward_limit
-right_limit   <= linear_y <= left_limit
```

The asymmetric signed interval is intentional. A single absolute limit would
discard directional clearance evidence and could authorize motion toward the
more constrained sector.

Each sector result retains the clearance, margin, reaction time, assured
deceleration, policy version, and scalar permitted speed required to audit the
derived planar limits. All sectors must use identical physical assumptions.

## Angular policy

The maximum permitted angular velocity is unconditionally `0.0 rad/s` in this
sub-slice. Four point/sector clearances cannot prove that a rotating footprint
will remain collision-free. The diagnostic reason distinguishes:

- `blocked_insufficient_clearance` when at least one direction grants only
  `STOP_ONLY` translational authority;
- `blocked_swept_footprint_unavailable` when every direction grants positive
  translational authority but rotation remains unproven.

This distinction is diagnostic only. Neither state authorizes rotation.

## Contract boundary

TB-EVAL-005B prepares immutable, signed planar limits for later clamping but
does not yet:

- consume a desired velocity command;
- clamp or publish `/cmd_vel`;
- define a ROS `PermittedMotionEnvelope` message;
- attach timestamps, sequence IDs, or correlation IDs;
- expire or replay-protect an envelope;
- implement swept-footprint geometry.

Those responsibilities remain isolated in TB-EVAL-005C and TB-EVAL-005D.

## Acceptance criteria

- Every clearance affects only its matching body-frame direction.
- Forward/reverse and left/right limits retain their independent magnitudes.
- Zero negative-axis authority is serialized as canonical positive zero.
- Angular authority remains zero with explicit fail-closed reasoning.
- Forged sector order, policy, physics, axis limits, or summaries are rejected.
- Repeated calculations are bit-stable and deterministic.
- The directional core retains 100% statement and branch coverage.
