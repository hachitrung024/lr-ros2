from glob import glob

from setuptools import find_packages, setup

package_name = "prediction_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover Team",
    maintainer_email="dev@example.com",
    description="ROS 2 runtime wrapper for landfill rover prediction_core",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "prediction_node = prediction_ros.prediction_node:main",
            "mock_upstream_node = prediction_ros.mock_upstream_node:main",
        ],
    },
)
