# TB-EVAL-009B — High-Frequency Telemetry & Diagnostic Blackbox

## Status

Implemented on `feat/tb-eval-009b`.

## Objective

TB-EVAL-009B adds a bounded diagnostic recorder to the safety control plane.
It preserves the correlated state immediately before and after a safety
incident without adding an unbounded queue, a blocking write, or another
publisher to the authoritative command path.

## Deterministic core

`core/telemetry_blackbox.py` provides an in-memory ring with fixed deployment
bounds:

- default sample rate: 50 Hz;
- rolling pre-trigger capacity: 500 frames (10 seconds);
- post-trigger window: 50 frames (1 second);
- maximum frozen snapshot: 550 frames;
- O(1) append and oldest-frame eviction for every ordinary control tick.

Each immutable frame can correlate:

- the latest teleoperation request from `/teleop/cmd_vel`;
- the latest autonomy request from `/autonomy/cmd_vel`;
- the authoritative output from `/cmd_vel`;
- `ArbitrationStatus`;
- `EnvelopeDiagnostics`;
- `SafetyLifecycleStatus`.

Missing or malformed transport evidence is represented as an absent field,
not synthesized into a healthy value. Sequence identifiers must increase
strictly and the recorder timestamp must be monotonic.

Copying the pre-trigger window is intentionally performed only on an incident
boundary. While that snapshot collects its bounded post-trigger window,
duplicate triggers are rejected so the original causal evidence cannot be
overwritten. The recorder then rearms automatically.

## Trigger and snapshot semantics

One of three auditable causes starts a snapshot:

| Trigger | Detection |
| --- | --- |
| `EMERGENCY_STOP` | transition from a known non-emergency lifecycle state into `EMERGENCY_STOP` |
| `HARD_FAULT` | new hardware or latched safety fault |
| `MANUAL` | `/safety/telemetry_blackbox/trigger` service request |

A hard fault has priority if it arrives in the same sampling interval as a
lifecycle emergency transition. Startup in an already stopped state is not
treated as a transition and therefore does not create a false incident.
Recorder-clock regression is clamped to the last monotonic tick and captured
as an emergency incident.

The frozen JSON document contains:

- schema and policy versions;
- a monotonically increasing snapshot identifier;
- trigger type, reason, timestamp, and frame index;
- freeze timestamp;
- the immutable ordered frame window.

## ROS 2 adapter

`telemetry_blackbox_node` samples the cached control-plane state from one
50 Hz timer. Topic callbacks perform only constant-size cache replacement.
The node uses reliable, volatile QoS with depth 1 for command topics and depth
10 for safety diagnostics.

The node is included in `safety_control_plane.launch.py`. Its default export
directory is `/tmp/titan_brain_blackbox`. Completed snapshots are first
written to a temporary file and then atomically renamed to
`blackbox-<snapshot-id>-<trigger-time>.json`. The final path is also emitted
to the ROS log.

Deployment parameters are:

- `policy_version`;
- `timer_period_sec`;
- `capacity_frames`;
- `post_trigger_frames`;
- `snapshot_output_directory` (must be absolute).

## Safety invariants

- The recorder is observational only and never publishes `/cmd_vel`.
- All steady-state memory use is bounded by configuration.
- Snapshot size is bounded by pre-trigger capacity plus the post-trigger
  window.
- A trigger never replaces an incident whose post-window is still being
  collected.
- Non-finite command or authority values are rejected from a frame.
- JSON publication is atomic; a failed write leaves the recorder snapshot
  available for a later export attempt.
- Existing single-point output, strictest-limit, and emergency bypass
  invariants remain unchanged.

## Verification

The dependency-free suite covers ring eviction, exact pre/post boundaries,
duplicate trigger rejection, rearming, monotonic sequence and time guards,
partial evidence, non-finite input rejection, immutable snapshots, and
deterministic JSON.

The ROS 2 Jazzy suite covers QoS and graph endpoints, complete correlated
sampling, lifecycle and hard-fault triggers, same-tick trigger priority,
startup suppression, clock regression, manual service dumps, atomic export,
and invalid output-directory configuration. The existing 007B and 008C graph
gates include the new passive subscribers.
