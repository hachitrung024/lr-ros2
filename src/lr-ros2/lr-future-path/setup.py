"""Setuptools entry point for the lr_future_path ROS package."""

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover",
    maintainer_email="maintainers@example.com",
    description=(
        "Provide the fixed ENU origin and reference future path from the recording DB"
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "mavlink_ground_truth_node = lr_future_path.mavlink_ground_truth_node:main",
        ],
    },
)
