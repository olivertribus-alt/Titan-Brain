#!/usr/bin/env bash
set -eo pipefail

TB_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TB_REPOSITORY_ROOT="$(cd -- "${TB_SCRIPT_DIR}/.." && pwd)"
TB_PYTHON_BIN="${TB_PYTHON_BIN:-python}"
TB_GATE_MODE="${1:-all}"

cd "${TB_REPOSITORY_ROOT}"

run_python_gate() {
    "${TB_PYTHON_BIN}" -m ruff check core tests ros2_ws
    "${TB_PYTHON_BIN}" -m mypy core tests
    "${TB_PYTHON_BIN}" -m pytest \
        --cov=core \
        --cov-report=term-missing \
        --cov-fail-under=85
}

run_ros_gate() {
    local ros_setup="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
    if [[ ! -f "${ros_setup}" ]]; then
        echo "ROS setup not found: ${ros_setup}" >&2
        return 2
    fi

    source "${ros_setup}"

    "${TB_PYTHON_BIN}" -m colcon --log-base ros2_ws/log build \
        --base-paths ros2_ws/src \
        --build-base ros2_ws/build \
        --install-base ros2_ws/install \
        --event-handlers console_direct+

    source ros2_ws/install/setup.bash

    # Colcon validates package-level test integration and result metadata.
    "${TB_PYTHON_BIN}" -m colcon --log-base ros2_ws/log test \
        --base-paths ros2_ws/src \
        --build-base ros2_ws/build \
        --install-base ros2_ws/install \
        --event-handlers console_direct+ \
        --return-code-on-test-failure
    "${TB_PYTHON_BIN}" -m colcon test-result \
        --test-result-base ros2_ws/build \
        --verbose

    # This explicit invocation is the authoritative ROS runtime/launch gate.
    "${TB_PYTHON_BIN}" -m pytest \
        ros2_ws/src/titan_brain_ros/test \
        -q
}

case "${TB_GATE_MODE}" in
    all)
        run_python_gate
        run_ros_gate
        ;;
    python)
        run_python_gate
        ;;
    ros)
        run_ros_gate
        ;;
    *)
        echo "Usage: $0 [all|python|ros]" >&2
        exit 2
        ;;
esac
