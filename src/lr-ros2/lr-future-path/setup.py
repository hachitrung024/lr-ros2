"""Setuptools entry point for the lr_future_path ROS package."""

from glob import glob

from setuptools import find_packages, setup


package_name = "lr_future_path"

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
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover",
    maintainer_email="maintainers@example.com",
    description=(
        "Replay SVO VIO or MAVLink ground-truth trajectories"
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "svo_pose_cache_node = "
            "lr_future_path.svo_pose_cache_node:main",
            "future_ground_truth_node = "
            "lr_future_path.future_ground_truth_node:main",
            "mavlink_pose_node = "
            "lr_future_path.mavlink_pose_node:main",
        ],
    },
)
