from setuptools import find_packages, setup

package_name = 'uav_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/click_navigate.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/click_navigate.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Rviz2 click-to-navigate with A* path planning and VFH+ obstacle avoidance',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node_click_navigate = uav_navigation.click_navigate_node:main',
        ],
    },
)