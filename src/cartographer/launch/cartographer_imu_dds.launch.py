from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    'lsn10p_launch.py'
                ])
            ]),
            launch_arguments={'output': 'screen'}.items()
        ),
        
        # 2. 启动你新写的 IMU 转换节点 (PX4 -> ROS2 Imu)
        Node(
            package='ekf2_trans',
            executable='node_px4_imu',  # 确保这是你在 setup.py 中注册的 entry_point 名字
            name='px4_imu_translator',
            output='screen'
        ),

        # 3. 静态 TF：雷达
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']
        ),

        # 4. 静态 TF：IMU (取消注释，Cartographer 必须有这个 TF 树)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            # 这里的坐标 ['x', 'y', 'z', 'yaw', 'pitch', 'roll'] 请根据实际安装位置修改
            arguments=['0', '0', '0.03', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 5. Cartographer 节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments = [
                '-configuration_directory', FindPackageShare('cartographer').find('cartographer') + '/config',
                '-configuration_basename', '_imu_2d.lua'
            ],
            remappings=[
                # 修改点：将内部话题 imu 映射为你节点发布的 /imu
                # 如果你的 Python 代码里写的是 self.create_publisher(Imu, '/imu', 10)
                ('imu', '/imu')
            ],
            output='screen'
        ),
        
        # 6. 地图服务节点
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': False},
                {'resolution': 0.05}
            ],
        ),
    ])