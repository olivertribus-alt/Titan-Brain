# TB-EVAL-005A — Inverse Braking and Scalar Motion Envelope

## Status

Implemented as an isolated dependency-free core contract. Directional
composition, ROS transport, and arbiter enforcement belong to later
TB-EVAL-005 sub-slices.

## Objective

Invert the established TB-EVAL-002A stopping model so that a measured clearance
produces a maximum permitted closing speed:

```text
available_clearance = max(0, observed_clearance - clearance_margin)

max_speed =
    sqrt((assured_deceleration * reaction_time)^2
         + 2 * assured_deceleration * available_clearance)
    - assured_deceleration * reaction_time
```

The implementation uses the rationalized positive quadratic root and high
precision intermediate arithmetic. The emitted binary float is adjusted
conservatively so it never exceeds the mathematical root or the existing
forward stopping-distance boundary.

## Contract decisions

- Clearance at or below the configured margin grants `STOP_ONLY` authority.
- Positive authority is the greatest representable value proven not to exceed
  the inverse boundary.
- The result is immutable and repeats every physical assumption required to
  audit its derivation.
- Invalid, non-finite, negative, overflowing, or internally inconsistent input
  fails closed with no fabricated speed limit.
- No default reaction time, deceleration, or clearance margin is invented.

## Deliberate exclusions

TB-EVAL-005A does not yet:

- combine forward, reverse, left, and right sector limits;
- authorize angular motion or model a swept footprint;
- attach timestamps, sequence IDs, or observation correlation IDs;
- define a ROS message or topic;
- clamp `/cmd_vel_raw` or change arbiter behavior;
- validate physical stopping performance.

Those responsibilities are reserved for TB-EVAL-005B through 005D and the
subsequent actuator-assurance milestone.

## Acceptance criteria

- Forward and inverse formulas agree at exact boundaries.
- Greater clearance produces monotonically non-decreasing authority.
- The next representable value above the reported boundary is rejected.
- Exhausted clearance margin always produces zero authority.
- Invalid physical values and numerical overflow fail closed.
- Manual construction cannot forge inconsistent evidence.
- One hundred identical evaluations produce identical objects, JSON, and
  SHA-256 hashes.
