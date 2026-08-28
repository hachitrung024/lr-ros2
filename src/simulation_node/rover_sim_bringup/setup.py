from setuptools import find_packages,setup
from glob import glob
package_name='rover_sim_bringup'
setup(name=package_name,version='0.1.0',packages=find_packages(),
 data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),
             ('share/'+package_name+'/launch',glob('launch/*.launch.py')),('share/'+package_name+'/config',glob('config/*.yaml'))],
 install_requires=['setuptools'],zip_safe=True,maintainer='ACLAB',maintainer_email='maintainer@example.com',license='Apache-2.0',
 entry_points={'console_scripts':['cmd_bridge=rover_sim_bringup.cmd_bridge:main','state_bridge=rover_sim_bringup.state_bridge:main',
                                 'contact_bridge=rover_sim_bringup.contact_bridge:main','camera_bridge=rover_sim_bringup.camera_bridge:main',
                                 'rollover_monitor=rover_sim_bringup.rollover_monitor:main','teleop=rover_sim_bringup.teleop:main']})
