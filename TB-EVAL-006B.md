# TB-EVAL-006B — ROS 2 Command Governor Node & Integration

## Scope

TB-EVAL-006B wraps the dependency-free `CommandGovernor` in
`CommandGovernorNode`.  The node subscribes to raw velocity requests and the
external safety supervisor, then publishes the only profiled command topic:

| Direction | Topic | Type |
| --- | --- | --- |
| Input | `/cmd_vel_raw` | `geometry_msgs/msg/Twist` |
| Input | `/safety/supervisor_status` | `titan_brain_msgs/msg/SafetySupervisorStatus` |
| Output | `/cmd_vel_governed` | `geometry_msgs/msg/Twist` |

The node itself has no actuator authority; the downstream safety arbiter still
owns the final `/cmd_vel` topic.

## Runtime policy

- The timer runs at **50 Hz** by default (`20 ms`).
- Startup is fail-closed and publishes zero until a fresh `STATE_OK` status
  with a closed relay request is received.
- `TRIPPED`, `HARDWARE_FAULT_LATCH`, `INITIALIZING`, active fault diagnostics,
  or an open relay request immediately invoke the governor's emergency bypass.
- A missing or stale raw command (`cmd_timeout_sec`, default `200 ms`) emits a
  hard zero by default.  `stale_command_emergency_stop: false` may select the
  normal governed deceleration profile instead.  A stale or missing safety
  status (`safety_timeout_sec`, default `250 ms`) always emits a hard zero.
- Lateral `Twist.linear.y` is rejected because 006A governs only the selected
  longitudinal axis; no unsupported motion is silently passed through.

## Configuration

The `command_governor_node` section in `config/titan_brain.yaml` contains all
speed, acceleration, deceleration, jerk, timeout, and loop-period parameters.
`launch/command_governor.launch.py` starts the node independently; the full
`titan_brain.launch.py` also includes it.

## Teardown and verification

The adapter handles `KeyboardInterrupt`, checks `rclpy.ok()`, and catches the
Jazzy `_rclpy_pybind11.RCLError` during shared-context teardown.  ROS 2 node
tests cover QoS, startup zero, normal shaping, immediate safety bypass,
stale-command fallback, and unsupported lateral input.  Container build and
runtime verification are deferred to the ROS 2 Jazzy CI gate.
