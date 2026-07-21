#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${TB_VENV}/bin/activate"

if [[ -f "${TB_WORKSPACE}/ros2_ws/install/setup.bash" ]]; then
    source "${TB_WORKSPACE}/ros2_ws/install/setup.bash"
fi

exec "$@"
