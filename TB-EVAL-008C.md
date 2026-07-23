# TB-EVAL-008C — Fault Injection and Safety Validation Gate

## Scope

TB-EVAL-008C validates the complete live ROS 2 path from
`DynamicEnvelopeNode` through `SafetyVelocityArbiterNode`. The gate injects
sensor blackouts, transient obstacles, timestamp faults, corrupt scan data,
invalid frames, and system faults over real DDS topics while the production
safety-control-plane launch is running.

## Hardened invariants

- A scan blackout longer than 200 ms produces `SCAN_TIMEOUT`, a fail-closed
  zero envelope, and a forced-zero `/cmd_vel`.
- One close beam is sufficient to produce a protective stop. No subsampling or
  temporal debounce may hide a transient obstacle.
- Regressed or repeated scan timestamps latch `CLOCK_REGRESSION_LATCHED`.
  Later fresh scans cannot restore authority without restarting the node.
- NaN, negative infinity, out-of-range values, and scans with no finite returns
  are invalid evidence. An individual positive-infinity beam remains a valid
  LaserScan “no return” only when finite evidence is present elsewhere.
- Missing frame IDs fail closed.
- A hardware or safety fault overrides nominal scan evidence.
- A valid stop-only motion envelope bypasses ordinary acceleration, jerk, and
  deceleration shaping in the arbiter and invokes the emergency-stop path.

## Live DDS scenarios

`test_integration_rosbag_008c.py` launches the installed
`safety_control_plane.launch.py` and exercises:

1. LIDAR publication loss while fault and command inputs remain fresh.
2. A single-beam ghost-obstacle spike below the clearance margin.
3. Full NaN and positive-infinity floods, followed by recovery evidence that
   proves the nodes remain alive.
4. A blank scan frame ID.
5. A hardware fault while a clear scan and non-zero command are present.
6. A regressed scan timestamp followed by fresh evidence to prove the timing
   fault remains latched.

Each injected stop is correlated across `EnvelopeDiagnostics`,
`PermittedMotionEnvelope`, and `ArbitrationStatus`. The gate requires an
exact-zero command and bounds envelope-to-command propagation to 250 ms.
Transient obstacle evaluation is independently bounded to 100 ms.

## Verification boundary

Dependency-free tests and ROS node tests run alongside the live launch test.
The authoritative runtime result is the ROS 2 Jazzy Container Gate because the
local macOS environment does not provide the Jazzy middleware, generated
messages, or DDS runtime.
