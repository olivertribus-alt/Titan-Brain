from setuptools import find_packages, setup

package_name = "titan_brain_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/safety_control_plane.launch.py"],
        ),
        ("share/" + package_name + "/config", ["config/titan_brain.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Titan Brain Maintainers",
    maintainer_email="maintainers@titanbrain.local",
    description="ROS 2 package for Titan Brain safety critical control plane",
    license="PROPRIETARY",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_observation_node = titan_brain_ros.safety_observation_node:main",  # noqa: E501
            "safety_loop_supervisor_node = titan_brain_ros.safety_loop_supervisor_node:main",  # noqa: E501
            "safety_velocity_arbiter_node = titan_brain_ros.safety_velocity_arbiter_node:main",  # noqa: E501
            "dynamic_motion_envelope_node = titan_brain_ros.dynamic_motion_envelope_node:main",  # noqa: E501
            "safety_recovery_manager_node = titan_brain_ros.safety_recovery_manager_node:main",  # noqa: E501
            "telemetry_blackbox_node = titan_brain_ros.telemetry_blackbox_node:main",  # noqa: E501
            "actuator_feedback_monitor_node = titan_brain_ros.actuator_feedback_monitor_node:main",  # noqa: E501
            "command_governor_node = titan_brain_ros.command_governor_node:main",  # noqa: E501
            "command_path_observability_node = titan_brain_ros.command_path_observability_node:main",  # noqa: E501
            "multi_sensor_envelope_node = titan_brain_ros.multi_sensor_envelope_node:main",  # noqa: E501
        ],
    },
)
