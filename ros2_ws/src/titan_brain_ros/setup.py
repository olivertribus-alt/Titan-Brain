from setuptools import find_packages, setup

PACKAGE_NAME = "titan_brain_ros"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", ["config/velocity_arbiter.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Oliver Tribus",
    maintainer_email="olivertribus-alt@users.noreply.github.com",
    description="ROS 2 Jazzy node wrapper for Titan Brain safety evaluation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "safety_observation_node = "
            "titan_brain_ros.safety_observation_node:main",
            "velocity_arbiter_node = "
            "titan_brain_ros.velocity_arbiter_node:main",
        ],
    },
)
