from setuptools import setup
import os
from glob import glob

package_name = 'flocking_swarm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),    
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shirui',
    maintainer_email='shirui@idt.local',
    description='9-UAV distributed flocking controller for PX4 SITL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arming_node = flocking_swarm.arming_node:main',
            'virtual_leader_node = flocking_swarm.virtual_leader_node:main',
            'flock_controller_node = flocking_swarm.flock_controller_node:main',
        ],
    },
)
