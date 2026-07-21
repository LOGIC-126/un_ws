from setuptools import find_packages, setup

package_name = 'amount'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'zhongzhuan = amount.zhongzhuan:main',
            'uv2_pub_node = amount.uv2_ros:main',
            'detection_world_node = amount.detection_world_node:main',
        ],
    },
)
