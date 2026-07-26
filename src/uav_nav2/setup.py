#!/usr/bin/env python3
import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'uav_nav2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'uav_nav2'),
            glob('uav_nav2/behavior_tree.xml')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Nav2-based 2D obstacle avoidance at fixed altitude for PX4 UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_nav2_bridge = uav_nav2.nav2_bridge_node:main',
            'node_nav2_odometry = uav_nav2.nav2_odometry_node:main',
        ],
    },
)