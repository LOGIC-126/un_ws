# Nav2 固定高度 2D 避障导航 Launch 文件
#
# 启动节点:
#   1. planner_server (NavFn + 内部 global costmap)
#   2. controller_server (DWB 全向 + 内部 local costmap)
#   3. bt_navigator (简化行为树)
#   4. lifecycle_manager (自动激活)
#   5. nav2_odometry_bridge (TF→/odom)
#   6. nav2_bridge (/cmd_vel→/uav/target_position + /goal_pose→action)
#   7. static TF map→odom (identity)
#   8. rviz2 (可选)
#
# 前置条件: Cartographer 已启动, 提供 /map + map→base_link TF

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_nav2')

    # ——— Launch 参数 ———
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch Rviz2')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock')

    # ——— 配置文件路径 ———
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    planner_path = os.path.join(pkg_dir, 'config', 'planner_server_params.yaml')
    dwb_path = os.path.join(pkg_dir, 'config', 'dwb_controller_params.yaml')
    bt_path = os.path.join(pkg_dir, 'uav_nav2', 'behavior_tree.xml')
    rviz_path = os.path.join(pkg_dir, 'rviz', 'navigate.rviz')

    # ——— lifecycle 节点列表 (planner/controller 内部管理各自的 costmap) ———
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'bt_navigator',
    ]

    return LaunchDescription([
        rviz_arg,
        use_sim_time_arg,

        # ================================================================
        # 自定义节点
        # ================================================================

        # Odometry 桥接: TF map→base_link → /odom + odom→base_link TF
        Node(
            package='uav_nav2',
            executable='node_nav2_odometry',
            name='nav2_odometry_bridge',
            parameters=[nav2_params_path],
            output='screen',
        ),

        # Nav2 桥接: /cmd_vel → /uav/target_position + /goal_pose → action
        Node(
            package='uav_nav2',
            executable='node_nav2_bridge',
            name='nav2_bridge',
            parameters=[nav2_params_path],
            output='screen',
        ),

        # 静态 TF: map → odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # ================================================================
        # Nav2 规划器 (内部含 global costmap)
        # ================================================================

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[
                planner_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # ================================================================
        # Nav2 控制器 (内部含 local costmap, DWB 全向模式)
        # ================================================================

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[
                dwb_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('cmd_vel', '/cmd_vel')],
            output='screen',
        ),

        # ================================================================
        # Nav2 BT Navigator
        # ================================================================

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'default_nav_to_pose_bt_xml': bt_path,
                'default_nav_through_poses_bt_xml': bt_path,
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                ],
            }],
            output='screen',
        ),

        # ================================================================
        # Nav2 Lifecycle Manager
        # ================================================================

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
            output='screen',
        ),

        # ================================================================
        # Rviz2
        # ================================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
