"""Run the dependency-free TB-EVAL-006C verification matrix."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.command_governor_fault_injection import (  # noqa: E402
    run_fault_injection,
    standard_fault_cases,
)


def main() -> int:
    """Return zero only when every governor scenario passes its contract."""
    failures: list[str] = []
    cases = standard_fault_cases()
    for case in cases:
        report = run_fault_injection(case)
        if not report.passed:
            failures.append(
                f"{case.scenario.value}: final reason="
                f"{report.final_result.reason.value}, "
                f"linear={report.final_result.linear_velocity_mps:.6f}, "
                f"jerk={report.max_observed_jerk_mps3:.6f}"
            )
        else:
            print(
                f"PASS {case.scenario.value}: "
                f"events={report.events_processed}, "
                f"linear={report.final_result.linear_velocity_mps:.6f}, "
                f"jerk={report.max_observed_jerk_mps3:.6f}"
            )
    if failures:
        print("TB-EVAL-006C verification failed:")
        print("\n".join(failures))
        return 1
    print(f"TB-EVAL-006C verification passed: {len(cases)} scenarios")
    return 0


if __name__ == "__main__":
    sys.exit(main())
