from setuptools import find_packages, setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pid_params.yaml']),
        ('share/' + package_name + '/config', ['config/tracking_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wsz',
    maintainer_email='your.email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node_offboard_control = offboard_control.offboard_control:main',
            'offboard_control_8 = offboard_control.offboard_control_8:main',
            'node_mission = offboard_control.mission:main',
            'node_competition_mission = offboard_control.competition_mission:main',
            'node_test_pid = offboard_control.test_pid:main',
            'node_yolo_tracking = offboard_control.yolo_tracking:main',
        ],
    },
)
