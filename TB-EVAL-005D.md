# TB-EVAL-005D — Correlation, expiry, replay protection, and E2E fault injection

## Status

Implemented as the stateful validation layer around the TB-EVAL-005C motion
envelope. The dependency-free arbiter validates the envelope before applying
any command, while the ROS 2 tests exercise the same policy through live DDS
when the Jazzy container gate is available.

## Validation order

The arbiter keeps the fail-closed ordering explicit:

1. Existing safety, command, and intent faults retain priority.
2. The envelope must be present, structurally valid, finite, and in the
   configured output frame.
3. Its `correlation_id` must match the accepted `SafetyIntent`; its source
   sequence must match the intent source sequence.
4. The envelope timestamp must not be in the future and must be younger than
   `motion_envelope_stale_threshold_ns` (the boundary is exclusive).
5. A command must arrive after the matched envelope ingress event. A command
   already queued before a new envelope is rejected with a zero output.
6. Only then may the command be clamped to the signed envelope intervals.

`geometry_msgs/msg/Twist` has no producer sequence or correlation fields. The
ROS adapter therefore assigns a monotonic local ingress sequence to every
received command and compares it with the accepted envelope ingress sequence.
This prevents queued-command reuse at the control boundary; end-to-end source
identity remains carried by `SafetyIntent` and `PermittedMotionEnvelope`.

Every rejection produces an exactly zero `Twist`, an explicit
`ArbitrationReason`, and the latest audit correlation ID.

Envelope authority faults (missing, malformed, wrong-frame, mismatched,
future-dated, or expired) latch the existing recovery guard. A later valid
envelope with the same `NORMAL` intent cannot silently release motion; a newer
`NORMAL` intent and a command received after that release are required. The
special `motion_envelope_command_required` result is the ingress-order guard:
it rejects only the queued pre-envelope command and permits a fresh command
under the already-valid intent.

## Replay and mutation protection

The ROS adapter tracks source sequence IDs independently for intents and
envelopes. A lower sequence is a regression. Reusing a sequence with a
different payload (including a changed correlation ID) is a mutation fault;
an identical replay does not refresh freshness. Consequently, replaying an
old valid envelope cannot extend its lifetime or release a stopped command.

## Fault-injection coverage

The acceptance suite covers:

- missing, malformed, wrong-frame, and angular-enabling envelopes;
- stale and future envelope timestamps;
- correlation and sequence desynchronization;
- source sequence regression, same-sequence mutation, and identical replay;
- commands queued before the matched envelope;
- aggressive positive and negative commands clamped to asymmetric limits;
- live DDS verification that a mutated envelope and an expired envelope force
  `/cmd_vel` to zero and preserve audit metadata.

The ROS 2 launch test also checks that the envelope correlation and sequence
remain identical across `SafetyIntent`, `ArbitrationStatus`, and command-path
observability records. No observability topic has authority over `/cmd_vel`.

## Acceptance criteria

- At or beyond the configured envelope age budget, output is forced to zero.
- Future timestamps, replay, mutation, and identity mismatch fail closed.
- An aggressive `/cmd_vel_raw` can never exceed the current envelope.
- Angular output remains zero until swept-footprint authority exists.
- Every forced-zero result remains audit-correlated.
- Core tests pass with full branch coverage for the new validation paths.
- ROS 2 Jazzy launch tests provide the final DDS/E2E verification.
