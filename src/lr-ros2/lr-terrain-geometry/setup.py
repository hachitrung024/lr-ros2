from glob import glob
from setuptools import find_packages, setup


package_name = "lr_terrain_geometry"

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
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Landfill Rover",
    maintainer_email="maintainers@example.com",
    description="Rolling terrain plane estimation for ZED point clouds",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "terrain_geometry_node = lr_terrain_geometry.node:main",
        ],
    },
)
