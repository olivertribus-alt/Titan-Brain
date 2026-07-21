# TB-EVAL-002B — Dynamic Braking Evaluator Integration

Status: implementation baseline; physical validation remains pending.

## Scope

TB-EVAL-002B connects the isolated TB-EVAL-002A braking-envelope mathematics
to the deterministic safety evaluator and the ROS 2 Jazzy observation node.
It does not add TTC, moving-obstacle prediction, angular swept-footprint
geometry, or actuator-level braking validation.

## Core contract

`SafetyObservation.directional_data` is an optional, immutable bundle of:

- forward, reverse, left, and right obstacle clearance;
- robot-relative planar velocity (`linear_x_mps`, `linear_y_mps`,
  `angular_z_radps`).

The retained scalar `clearance_m` is the legacy alias for frontal clearance.
When directional data is present it must exactly equal `forward_m`; conflicting
values are rejected before evaluation.

`SafetyRuleConfig.braking_envelope` is optional. Its absence preserves the
TB-PoC-001 scalar-clearance evaluator. Its presence enables dynamic mode and
requires complete directional data.

The ROS coordinate convention is `+x = forward`, `-x = reverse`, `+y = left`,
and `-y = right`. Only sectors toward which the robot is translating are
assessed. The limiting active sector is the sector with the smallest clearance
surplus.

## Deterministic policy

| Condition | Action | Rule |
| --- | --- | --- |
| Dynamic envelope satisfied | `proceed` | `EV-SAFE-DYN-00` |
| Envelope violated, confidence sufficient | `emergency_stop` | `EV-SAFE-DYN-01` |
| Envelope violated, confidence insufficient | `protective_stop` | `EV-SAFE-DYN-02` |
| Dynamic mode lacks directional data | `protective_stop` | `EV-SAFE-DYN-03` |
| Angular velocity is non-zero | `protective_stop` | `EV-SAFE-DYN-04` |

When translational velocity is exactly zero, the established scalar clearance
rule remains the stationary safety floor. Angular motion fails closed because
TB-EVAL-002A deliberately has no swept-footprint model; silently ignoring
`angular_z_radps` would be unsafe.

Decision evidence reports the limiting sector, observed and required
clearance, closing speed, reaction distance, braking distance, configured
margin, confidence, sensor, and both policy versions.

## ROS transport and compatibility

The original `titan_brain_msgs/SafetyObservation` schema and
`/safety/observation` topic remain unchanged. A separate
`titan_brain_msgs/DirectionalSafetyObservation` schema is published on
`/safety/directional_observation`. This avoids changing the DDS type hash of
the established message.

Legacy mode remains source- and wire-compatible when dynamic braking is
disabled. When dynamic braking is enabled, receiving a legacy observation is
accepted at the transport boundary but produces `EV-SAFE-DYN-03` and therefore
a fail-closed `protective_stop`.

The launch profile enables dynamic braking and explicitly supplies:

- `safety_policy_version`;
- `clearance_threshold_m` and `confidence_threshold`;
- `braking_policy_version`;
- `reaction_time_ns`;
- `assured_deceleration_mps2`;
- `clearance_margin_m`.

If dynamic mode is enabled and any required parameter is absent or invalid,
the ROS node refuses to initialize. It does not silently substitute a physical
assumption.

## Model boundary

The configuration does not contain sector angles or dimensions. The current
math consumes already-extracted nearest clearances, so unused geometry
parameters would be misleading dead configuration. Sector extraction geometry
belongs in a future perception contract and must be independently tested.

`reaction_time_ns`, `assured_deceleration_mps2`, and `clearance_margin_m` are
deployment assumptions, not measured guarantees. The existing non-real-time
CI transport deadline is not proof of real hardware stopping performance.

The dynamic evaluator assesses the velocity in the observation; it does not
yet authorize a different future navigation command. The E2E regression uses
matching observed and commanded translational velocity. A later actuator
contract must bind the evaluated envelope to the desired command (or carry an
explicit permitted-velocity envelope) before dynamic `proceed` can be treated
as authorization to accelerate. Until then, the existing static floor and
fail-closed transport behavior remain necessary but do not constitute physical
proof for arbitrary command transitions.
