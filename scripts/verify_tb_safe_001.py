"""Run the dependency-free TB-SAFE-001D acceptance matrix."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from core.safety_fault_injection import (
    run_safety_fault_injection,
    standard_fault_cases,
)


def main() -> int:
    """Return zero only when every documented safety fault stays fail-closed."""
    failures: list[str] = []
    for case in standard_fault_cases():
        report = run_safety_fault_injection(case)
        if (
            report.final_state is not case.expected_state
            or report.final_reason is not case.expected_reason
        ):
            failures.append(
                f"{case.scenario.value}: expected "
                f"{case.expected_state.value}/{case.expected_reason.value}, "
                f"got {report.final_state.value}/{report.final_reason.value}"
            )
    if failures:
        print("TB-SAFE-001D verification failed:")
        print("\n".join(failures))
        return 1
    print(
        "TB-SAFE-001D verification passed: "
        f"{len(standard_fault_cases())} fault scenarios"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
