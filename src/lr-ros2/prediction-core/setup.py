from glob import glob

from setuptools import find_packages, setup


package_name = "prediction_core"

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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover Team",
    maintainer_email="dev@example.com",
    description="ROS-independent collision and rollover prediction engine",
    license="Proprietary",
)
