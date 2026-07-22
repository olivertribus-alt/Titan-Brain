# TB-SAFE-001B — Hardware Relay Feedback & Latching Logic

## Scope

TB-SAFE-001B extends the TB-SAFE-001A supervisor with a dependency-free
auxiliary-contact feedback contract and a physical relay transition budget.
The software request remains the sole desired state; feedback only verifies
that the physical contactor followed it.

The default `relay_budget_ns` is **50 ms** (`50_000_000 ns`). It is measured
with the same monotonic timestamp discipline as the heartbeat watchdog.

## Relay verification

| Desired request | Feedback | Within budget | After budget |
| --- | --- | --- | --- |
| `REQUEST_SAFETY_OPEN` | open | transition acknowledged | — |
| `REQUEST_SAFETY_OPEN` | closed | pending | `WELDED_CONTACTS` → latch |
| `REQUEST_SAFETY_CLOSED` | closed | transition acknowledged | — |
| `REQUEST_SAFETY_CLOSED` | open | pending | `RELAY_TIMEOUT` → latch |
| stable `OK` | unexpected open | — | `UNINTENDED_DISCONNECT` → latch |

Missing feedback after a transition budget is a `RELAY_TIMEOUT`. Invalid,
non-boolean, stale, or clock-regressing feedback immediately enters
`HARDWARE_FAULT_LATCH`. The latch always emits
`REQUEST_SAFETY_OPEN`.

## Explicit reset protocol

`reset_hardware_fault()` is the only release path and requires all of:

1. exact `reset_authorization_token` (default `TB-SAFE-RESET-001B`),
2. a strictly increasing `sequence_id`,
3. valid fresh heartbeats from all three required channels, and
4. valid feedback confirming the relay is physically open.

The accepted reset moves the software state to `OK` and starts a new
close-transition budget. The relay must then acknowledge
`REQUEST_SAFETY_CLOSED`; no reset is automatic or time-based.

## Verification

The tests cover relay acknowledgement, transition timeout, welded contacts,
unexpected disconnect, invalid feedback, monotonic regression, sticky latch,
and authorization/sequence/heartbeat/open-feedback reset guards. ROS 2 relay
drivers and physical GPIO/CAN integration remain deferred to TB-SAFE-001C.
