from glob import glob

from setuptools import find_packages, setup

package_name = "lr_prediction_bridge"

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
    extras_require={"test": ["pytest"]},
    maintainer="Landfill Rover Team",
    maintainer_email="dev@example.com",
    description="Upstream adapters for Prediction safety_perception_msgs topics",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "prediction_bridge_node = lr_prediction_bridge.prediction_bridge_node:main",
            "trajectory_adapter_node = lr_prediction_bridge.trajectory_adapter_node:main",
            "rover_state_adapter_node = lr_prediction_bridge.rover_state_adapter_node:main",
            "geometry_adapter_node = lr_prediction_bridge.geometry_adapter_node:main",
            "empty_objects_bridge_node = lr_prediction_bridge.empty_objects_bridge_node:main",
            "tracked_objects_adapter_node = lr_prediction_bridge.tracked_objects_adapter_node:main",
            "upstream_contract_stub = lr_prediction_bridge.upstream_contract_stub:main",
        ],
    },
)
