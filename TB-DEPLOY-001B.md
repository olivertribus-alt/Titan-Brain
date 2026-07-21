# TB-DEPLOY-001B: Reproducible ROS 2 Jazzy Development Environment

## Status

Implemented locally. The image must pass the container quality gate in GitHub
Actions before this milestone can be closed.

## Scope

The development image provides one shared environment for the Python quality
gate, ROS 2 Jazzy package build, package-level `colcon test`, and the explicit
ROS runtime/launch tests.

It is a development and CI image. It is not a hardened production runtime image,
does not configure DDS security or hardware access, and does not establish a
real-time latency guarantee.

## Reproducibility boundary

- The official `ros:jazzy-ros-base-noble` base is pinned by a multi-architecture
  OCI digest.
- Direct Python dependencies are pinned in `requirements/constraints.txt` and
  used by both GitHub Actions and the image build.
- The virtual environment uses `--system-site-packages` so Debian-managed ROS
  modules remain importable without modifying the system Python installation.
- Transitive Python dependencies are not hash-locked. The environment is a
  controlled baseline, not a claim of bit-for-bit reproducibility.

## Usage

Build the image:

```bash
docker build --tag titan-brain:jazzy-dev .
```

Run the complete quality gate:

```bash
docker run --rm titan-brain:jazzy-dev scripts/quality-gate.sh all
```

Run only one layer:

```bash
docker run --rm titan-brain:jazzy-dev scripts/quality-gate.sh python
docker run --rm titan-brain:jazzy-dev scripts/quality-gate.sh ros
```

Open an interactive ROS-aware shell:

```bash
docker run --rm -it titan-brain:jazzy-dev
```

The entrypoint always sources `/opt/ros/jazzy/setup.bash`, activates the isolated
Python environment, and sources `ros2_ws/install/setup.bash` when a local build
exists.

For VS Code, open the repository and select **Reopen in Container**. The mounted
workspace remains at `/workspace/titan-brain`, matching the image's editable
Python installation.

## Updating the baseline

Base-image digest and direct Python constraints are security and compatibility
inputs. Update them in an isolated pull request, rebuild the image, and require
all Python and ROS gates to pass. Do not silently replace the digest with a
floating tag.

Host networking, ROS domain IDs, device mounts, GPU access, and DDS policies are
deployment-specific and intentionally remain outside this development image.
