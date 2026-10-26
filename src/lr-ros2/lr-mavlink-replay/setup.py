from glob import glob
from setuptools import find_packages, setup

setup(
    name="lr_mavlink_replay",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/lr_mavlink_replay"]),
        ("share/lr_mavlink_replay", ["package.xml", "requirements.txt"]),
        ("share/lr_mavlink_replay/launch", glob("launch/*.launch.py")),
    ],
    extras_require={"test": ["pytest"]},
    install_requires=["setuptools", "pymavlink>=2.4.49,<3"],
    entry_points={"console_scripts": ["telemetry_replay = lr_mavlink_replay.node:main"]},
)
