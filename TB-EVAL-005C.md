# TB-EVAL-005C — Arbiter and Control-Plane Integration

## Status

Implemented across the dependency-free arbiter core and ROS 2 Jazzy transport.
Envelope freshness, source replay protection, and strict intent/envelope
correlation matching were completed in the follow-up TB-EVAL-005D slice.

## Control-plane flow

For each accepted directional observation, `SafetyObservationNode` publishes:

1. an authoritative `SafetyIntent`;
2. a `PermittedMotionEnvelope` derived from the same observation and carrying
   the same `correlation_id` and sequence ID.

The envelope contains a timestamp, policy version, body frame, and asymmetric
signed limits for `linear.x` and `linear.y`. Its angular interval is exactly
zero because TB-EVAL-005B has no swept-footprint model.

If directional evidence is missing, rejected, or cannot produce a finite
envelope, the evaluator publishes an all-zero envelope. It never reuses or
fabricates positive motion authority.

## Arbiter enforcement

`VelocityArbiterNode` is the sole `/cmd_vel` publisher and now subscribes to
`/safety/permitted_motion_envelope`. The live path calls the mandatory
`evaluate_with_envelope` policy:

```text
min_linear_x <= cmd.linear.x <= max_linear_x
min_linear_y <= cmd.linear.y <= max_linear_y
min_angular_z == cmd.angular.z == max_angular_z == 0
```

Values outside the interval are deterministically clamped and reported as
`motion_envelope_clamped`. A missing, malformed, non-finite, angular-enabling,
or wrong-frame envelope forces an exactly zero output with a distinct reason.
An existing `E_STOP`, timeout, invalid command, or other upstream forced-zero
reason retains priority.

The prior `DynamicSafetyCommandArbiter.evaluate` method remains unchanged for
the isolated TB-EVAL-004 contract. Production ROS transport uses only the new
mandatory-envelope entry point.

## Deliberate 005D boundary

This sub-slice validates message structure and frame but does not yet:

- expire an old envelope;
- reject envelope sequence replay or payload mutation;
- require envelope and `SafetyIntent` correlation/sequence equality;
- expose envelope metadata in arbitration telemetry;
- inject correlation, replay, and timeout faults in live DDS tests.

Those checks require stateful ingress tracking and were implemented together in
TB-EVAL-005D so their ordering and recovery behavior share one specification.

## Acceptance criteria

- The message package exports `PermittedMotionEnvelope.msg`.
- The evaluator publishes positive authority only from validated directional
  evidence and zero authority otherwise.
- The arbiter fails closed when the envelope is missing or invalid.
- Asymmetric forward/reverse and left/right clamps preserve ROS body signs.
- Angular output remains exactly zero.
- Existing safety-stop reasons retain priority over envelope diagnostics.
- Core arbitration retains 100% statement and branch coverage.
