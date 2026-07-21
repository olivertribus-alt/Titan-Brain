"""Static contracts for the reproducible ROS 2 development environment."""

from __future__ import annotations

import json
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).parents[1]
ROS_BASE_DIGEST = (
    "sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f"
)


def test_dockerfile_pins_jazzy_and_isolates_python_tooling() -> None:
    dockerfile = (REPOSITORY_ROOT / "Dockerfile").read_text(encoding="utf-8")

    assert f"ros:jazzy-ros-base-noble@{ROS_BASE_DIGEST}" in dockerfile
    assert "python3 -m venv --system-site-packages" in dockerfile
    assert "--constraint requirements/constraints.txt" in dockerfile
    assert "--as-root pip:false" in dockerfile
    assert "--break-system-packages" not in dockerfile
    assert 'ENTRYPOINT ["/usr/local/bin/titan-brain-entrypoint"]' in dockerfile


def test_entrypoint_sources_ros_venv_and_optional_workspace_overlay() -> None:
    entrypoint = (REPOSITORY_ROOT / "docker" / "entrypoint.sh").read_text(
        encoding="utf-8"
    )

    assert 'source "/opt/ros/${ROS_DISTRO}/setup.bash"' in entrypoint
    assert 'source "${TB_VENV}/bin/activate"' in entrypoint
    assert '[[ -f "${TB_WORKSPACE}/ros2_ws/install/setup.bash" ]]' in entrypoint
    assert 'exec "$@"' in entrypoint


def test_one_script_runs_python_colcon_and_ros_runtime_gates() -> None:
    script = (REPOSITORY_ROOT / "scripts" / "quality-gate.sh").read_text(
        encoding="utf-8"
    )

    assert '"${TB_PYTHON_BIN}" -m ruff check core tests ros2_ws' in script
    assert '"${TB_PYTHON_BIN}" -m mypy core tests' in script
    assert "--cov-fail-under=85" in script
    assert "-m colcon --log-base ros2_ws/log build" in script
    assert "-m colcon --log-base ros2_ws/log test" in script
    assert "--python-testing pytest" in script
    assert "-m colcon test-result" in script
    assert "ros2_ws/src/titan_brain_ros/test" in script
    assert 'case "${TB_GATE_MODE}" in' in script


def test_devcontainer_builds_the_same_image_and_mounts_the_same_workspace() -> None:
    config = json.loads(
        (REPOSITORY_ROOT / ".devcontainer" / "devcontainer.json").read_text(
            encoding="utf-8"
        )
    )

    assert config["build"] == {"dockerfile": "../Dockerfile", "context": ".."}
    assert config["workspaceFolder"] == "/workspace/titan-brain"
    assert "target=/workspace/titan-brain" in config["workspaceMount"]
    assert config["postCreateCommand"] == "scripts/quality-gate.sh python"


def test_constraints_pin_all_direct_project_and_quality_dependencies() -> None:
    constraints = (
        REPOSITORY_ROOT / "requirements" / "constraints.txt"
    ).read_text(encoding="utf-8")

    for package in ("pydantic", "mypy", "pytest", "pytest-cov", "ruff"):
        assert f"{package}==" in constraints

    # ROS 2 Jazzy launch_testing still uses the hook argument removed in pytest 9.
    assert "pytest==8.4.2" in constraints
