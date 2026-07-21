# TB-EVAL-002A: Directional Braking Envelope

## Status

Implemented as an isolated deterministic core contract. It is not connected to
the live evaluator or ROS transport in this milestone.

## Objective

Calculate whether the measured clearance in each active robot-relative sector
is sufficient for a configured stopping model:

```text
required_clearance =
    closing_speed * reaction_time
    + closing_speed^2 / (2 * assured_deceleration)
    + clearance_margin
```

The result exposes every formula term so a later `DecisionEvidence` record can
explain the comparison without reconstructing hidden configuration.

## Contract decisions

- `reaction_time_ns` is a compulsory deployment budget. The non-real-time
  `<250 ms` CI assertion is evidence about one test environment, not a value the
  model silently imports or a guaranteed worst-case reaction time.
- `assured_deceleration_mps2` means a conservative deceleration magnitude that
  the deployed robot can actually achieve under its validated load, surface,
  slope, brake, battery, and environmental conditions. It is intentionally not
  named `a_max`.
- Closing speeds and clearances use the robot-relative forward, reverse, left,
  and right sectors. Transport code must transform inputs into the declared
  robot frame before calling this module.
- A sector is active only when the robot has positive closing speed toward it.
  Therefore an obstacle behind a forward-moving robot does not block forward
  translation.
- Exact equality between observed and required clearance is sufficient. Any
  lower value violates the envelope.
- Missing, non-finite, negative, contradictory, or physically invalid values
  are rejected by the strict immutable contracts.

## Deliberate exclusions

TB-EVAL-002A does not yet:

- change the existing `SafetyObservation` or ROS message contracts;
- command `CLAMP`, `PROTECTIVE_STOP`, or `EMERGENCY_STOP`;
- model angular motion or a swept robot footprint;
- account for moving-obstacle velocity or calculate TTC;
- model jerk, brake buildup, slope, payload, tyre/floor friction, localization
  uncertainty, or sensor uncertainty independently;
- claim compliance or certification against ISO 3691-4 or another safety
  standard.

Those factors must either be included conservatively in deployment inputs or
added as separately versioned model terms before this result controls motion.

## Acceptance criteria

- Formula terms and boundary comparisons are directly tested.
- Forward, reverse, left, right, diagonal, and stationary cases are explicit.
- Inactive sectors cannot block motion.
- Opposing speeds on one body axis are rejected.
- NaN, infinity, invalid configuration, and arithmetic overflow are rejected.
- One hundred identical evaluations produce identical objects, JSON, and
  SHA-256 hashes.

## References

- [ISO 3691-4:2023 scope](https://www.iso.org/standard/83545.html) — applicable
  safety requirements and verification context for driverless industrial
  trucks; referenced for scope only, not as a compliance claim.
- [Nav2 Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)
  — official ROS documentation for velocity-dependent zones and the distinction
  between holonomic and non-holonomic direction handling.
- [ROS coordinate-frame guidance](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html)
  — `base_link` is affixed to the robot and is the intended frame family for
  robot-relative directional inputs.
