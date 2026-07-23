# TB-EVAL-008B — ROS 2 Dynamic Envelope Node

## Scope

`DynamicEnvelopeNode` adapts `/scan` and `/safety/system_fault_status` into the
authoritative `/safety/permitted_motion_envelope` consumed by
`SafetyVelocityArbiterNode`. It also publishes auditable
`/safety/envelope_diagnostics`.

## Bounded scan reduction

The node rejects empty scans, malformed metadata, NaN or negative-infinite
ranges, out-of-contract distances, missing sector coverage, and scans larger
than `max_scan_samples`. This hard bound makes scan processing constant with
respect to untrusted input size without subsampling away obstacles. Positive
infinity is treated as the standard LaserScan “no return” value and bounded by
`range_max`. A scan containing no finite return in any sector is rejected as
missing usable evidence rather than interpreted as an unconditionally clear
path.

## Fail-closed invariants

- Missing, stale, or future-dated scans publish a zero envelope.
- Missing, stale, future-dated, unknown, or non-OK fault state publishes zero.
- ROS clock regression latches a zero envelope with no implicit recovery.
- Every envelope and diagnostic record shares a monotonic sequence ID,
  correlation ID, timestamp, and policy version.
- The 008B launch disables the legacy envelope publisher in
  `SafetyObservationNode`, leaving exactly one envelope authority.

The node publishes at 50 Hz, faster than the 50 ms arbiter envelope timeout.
The dynamic swept-radius model provides symmetric angular authority; malformed
or asymmetric angular envelopes remain fail-closed in the arbiter.

## Verification

ROS tests cover QoS, sector reduction, sample bounds, invalid scan data,
staleness and future time, hard faults, sticky clock regression, correlated
diagnostics, and end-to-end clamping through `SafetyVelocityArbiterNode`.
