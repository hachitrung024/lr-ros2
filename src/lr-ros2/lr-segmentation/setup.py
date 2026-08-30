from glob import glob

from setuptools import find_packages, setup


package_name = "lr_segmentation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "requirements.txt"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover",
    maintainer_email="maintainers@example.com",
    description="RGB segmentation overlay for ROS 2",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "segmentation_node = lr_segmentation.node:main",
        ],
    },
)
