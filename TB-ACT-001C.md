# TB-ACT-001C — ROS 2 Actuator Integration

## Status

Implemented as a ROS 2 Jazzy adapter around the dependency-free TB-ACT-001A
and TB-ACT-001B contracts. The node owns no actuator command topic; it only
consumes control intent and measured feedback, then publishes acknowledgement
and safety diagnostics.

## Message contracts

`ActuatorFeedback.msg` carries a stamped, correlated three-axis measurement and
the transport-side validity flags. `StopAcknowledgement.msg` carries the
monitor state, correlation and request/feedback sequences, stop latency, and a
critical priority marker for `HARDWARE_FAULT_LATCH`.

## Topic and QoS contract

| Topic | Type | Policy |
| --- | --- | --- |
| `/actuator/feedback` | `ActuatorFeedback` | Keep-last 5, best-effort, volatile |
| `/safety/arbitration_status` | `ArbitrationStatus` | Keep-last 10, reliable, volatile |
| `/cmd_vel` | `geometry_msgs/Twist` | Keep-last 10, reliable, volatile fallback |
| `/actuator/stop_acknowledgement` | `StopAcknowledgement` | Keep-last 10, reliable, volatile |
| `/actuator/status` | `StopAcknowledgement` | Keep-last 10, reliable, volatile |

`ArbitrationStatus` is the authoritative correlation source. `/cmd_vel` is a
conservative fallback only while no stop window is active; it cannot restart an
already acknowledged or latched window.

## Runtime rules

- A zero/forced-zero arbitration result starts the stop budget.
- Feedback is passed through the TB-ACT-001A evaluator and then the
  TB-ACT-001B monitor; no ROS-side duplicate stop math is introduced.
- Acknowledgement is published only for fresh, valid feedback measured after
  the stop request.
- Invalid/stale transport flags are converted to a deliberately invalid core
  payload and therefore latch fail-closed.
- A hardware latch publishes `critical=true`, `priority=255`, and the specific
  fault reason; it cannot clear through timer ticks or later feedback.
- Startup publishes an explicit non-latched `IDLE` status, while missing input
  streams never fabricate a stop acknowledgement.

## Deliberate exclusions

TB-ACT-001C does not drive a relay, electronic brake, or motor-driver API.
Those hardware actions and fault-injection scenarios remain in TB-ACT-001D.

## Verification

- Static message/package contract tests pass locally.
- Full local Python suite passes with 403 tests.
- Ruff and strict Mypy pass for the repository CI paths.
- ROS node tests and generated-message imports require the ROS 2 Jazzy
  container gate (`/opt/ros/jazzy` is not installed in the local environment).
