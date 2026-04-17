from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import  PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 雷达数据输出
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lslidar_driver'),
                    'launch',
                    'lsn10p_launch.py'
                ])
            ]),
            launch_arguments={
                'output': 'screen'
            }.items()
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']
        ),

        # Cartographer节点
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments = [
                '-configuration_directory', FindPackageShare('cartographer').find('cartographer') + '/config',
                '-configuration_basename', '_2d.lua'],
            output='screen'
        ),
        
        # 地图服务节点
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': False},
                {'resolution': 0.05}],
        ),
        
        

    ])