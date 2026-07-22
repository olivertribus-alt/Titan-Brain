# TB-SAFE-001D — Fault Injection and Relay Emulation

## Scope

TB-SAFE-001D closes the TB-SAFE-001 verification loop with a deterministic,
dependency-free fault matrix.  The harness in
`core/safety_fault_injection.py` feeds only timestamped evidence into
`SafetySupervisor`; it has no ROS, DDS or hardware authority.

## Covered scenarios

| Scenario | Injected condition | Required outcome |
| --- | --- | --- |
| `missing_heartbeat` | No channel registers before initialization budget | `TRIPPED` / `INITIALIZATION_TIMEOUT` |
| `stale_heartbeat` | A registered channel exceeds its watchdog budget | `TRIPPED` / `HEARTBEAT_TIMEOUT` |
| `welded_relay_contacts` | Contacts remain closed after an open request | `HARDWARE_FAULT_LATCH` / `WELDED_CONTACTS` |
| `clock_regression` | Supervisor time moves backwards | `TRIPPED` / `CLOCK_REGRESSION` |
| `sequence_replay` | A component reports a replayed/out-of-order pulse | `TRIPPED` / `HEARTBEAT_ERROR` |
| `unauthorized_reset` | Reset is attempted without the configured token | `HARDWARE_FAULT_LATCH` / `RESET_REJECTED` |

The `SafetyRelayEmulator` also supports nominal and unintended-open modes.  A
welded contact is sticky at the feedback boundary: issuing
`REQUEST_SAFETY_OPEN` does not fabricate an open indication.

## Safety properties verified

- Every injected fault produces a non-closed relay request.
- Hardware faults remain latched after later healthy heartbeats.
- A clock regression cannot be interpreted as fresh evidence.
- Reset authorization and sequence checks are enforced by the existing core;
  the fault suite verifies the unauthorized path without bypassing it.
- Reports are immutable and contain the final reason, state, feedback and
  number of processed events for audit replay.

## Verification command

The dependency-free matrix can be exercised directly with:

```bash
python scripts/verify_tb_safe_001.py
python -m pytest tests/test_safety_fault_injection.py tests/test_safety_supervisor.py
```

For a complete project gate, run `scripts/quality-gate.sh all` inside the ROS
2 Jazzy CI image.  That gate additionally builds the generated messages and
executes the ROS node tests from TB-SAFE-001C.

The archival PDF can be regenerated with
`python scripts/create_tb_safe_001_summary.py` using the bundled ReportLab
runtime.  The canonical artifact is written to
`output/pdf/TB-SAFE-001_Executive_Summary.pdf`.
