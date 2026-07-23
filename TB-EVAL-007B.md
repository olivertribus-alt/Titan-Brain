# TB-EVAL-007B — ROS 2 Control-Plane Adapter

## Scope

`SafetyVelocityArbiterNode` is the ROS 2 adapter for the dependency-free
`PrioritySelectorCore` from 007A.  It is intended to replace the legacy
velocity arbiter when deployed; the default launch is deliberately unchanged
so two nodes cannot accidentally publish competing `/cmd_vel` commands.

## Deterministic tick

At 50 Hz the node executes one fixed pipeline:

1. select fresh teleoperation over autonomy;
2. require and validate a fresh `PermittedMotionEnvelope`;
3. symmetrically clamp linear velocity and enforce the fail-closed angular
   policy from TB-EVAL-005;
4. pass the result through the TB-EVAL-006 kinematic governor;
5. publish exactly one final `TwistStamped` on `/cmd_vel` and an
   `ArbitrationStatus` audit record.

Missing or stale safety state, envelopes, commands, future timestamps, invalid
frames, and any non-OK fault state produce a zero command.  The node exposes a
single console entry point, `safety_velocity_arbiter_node`; operators must
select it in place of `velocity_arbiter_node` rather than launching both.

## Contracts

`SystemFaultStatus` is the explicit control-plane fault input.  The existing
`ArbitrationStatus` contract now carries `active_source`, `system_fault_state`,
and a machine-readable `rejection_reason` for audit and observability.
