"""Install the host-side ZED streaming source."""

from glob import glob

from setuptools import find_packages, setup


setup(
    name='lr_zed_stream',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lr_zed_stream']),
        ('share/lr_zed_stream', ['package.xml']),
        ('share/lr_zed_stream/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Landfill Rover',
    maintainer_email='maintainers@example.com',
    description='Host-side ZED NET source and SVO playback controls.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['stream_sender = lr_zed_stream.sender_node:main']},
)
