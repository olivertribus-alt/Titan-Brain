# syntax=docker/dockerfile:1.7

# Multi-architecture digest for the official ros:jazzy-ros-base-noble image.
# Update it deliberately as described in TB-DEPLOY-001B.md.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=jazzy \
    TB_VENV=/opt/titan-brain-venv \
    TB_WORKSPACE=/workspace/titan-brain \
    PATH=/opt/titan-brain-venv/bin:${PATH}

RUN apt-get update \
    && apt-get install --no-install-recommends -y \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR ${TB_WORKSPACE}

# Install Python tooling into an isolated environment while retaining access to
# Debian-managed ROS Python modules such as rclpy and rosidl.
COPY pyproject.toml ./
COPY requirements/constraints.txt requirements/constraints.txt
COPY core core
RUN python3 -m venv --system-site-packages "${TB_VENV}" \
    && "${TB_VENV}/bin/python" -m pip install --upgrade pip \
    && "${TB_VENV}/bin/python" -m pip install \
        --constraint requirements/constraints.txt \
        -e ".[dev]"

# Resolve ROS dependencies before copying the rest of the repository so that
# ordinary source changes do not invalidate this comparatively expensive layer.
COPY ros2_ws/src ros2_ws/src
RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && apt-get update \
    && rosdep install \
        --from-paths ros2_ws/src \
        --ignore-src \
        --rosdistro "${ROS_DISTRO}" \
        --as-root pip:false \
        -y \
    && rm -rf /var/lib/apt/lists/*

COPY . .
COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/titan-brain-entrypoint

ENTRYPOINT ["/usr/local/bin/titan-brain-entrypoint"]
CMD ["bash"]
