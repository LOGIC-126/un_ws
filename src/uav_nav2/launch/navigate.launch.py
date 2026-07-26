#!/usr/bin/env python3
"""
Nav2 固定高度 2D 避障导航 Launch 文件

启动节点:
  1. Nav2 lifecycle manager (自动激活)
  2. Nav2 global costmap (map 帧, Cartographer /map 输入)
  3. Nav2 local costmap (odom 帧, /scan 输入)
  4. Nav2 planner server (NavFn)
  5. Nav2 controller server (DWB 全向模式)
  6. Nav2 BT navigator (简化行为树)
  7. nav2_odometry_bridge (TF→/odom)
  8. nav2_bridge (/cmd_vel→/uav/target_position)
  9. static TF map→odom (identity)
 10. rviz2 (可选, 带 Nav2 目标工具)

前置条件: Cartographer 已启动, 提供 /map + map→base_link TF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_nav2')

    # ——— Launch 参数 ———
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch Rviz2 with Nav2 config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock')

    # ——— 配置文件路径 ———
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    global_costmap_path = os.path.join(pkg_dir, 'config', 'global_costmap_params.yaml')
    local_costmap_path = os.path.join(pkg_dir, 'config', 'local_costmap_params.yaml')
    planner_path = os.path.join(pkg_dir, 'config', 'planner_server_params.yaml')
    dwb_path = os.path.join(pkg_dir, 'config', 'dwb_controller_params.yaml')
    bt_path = os.path.join(pkg_dir, 'uav_nav2', 'behavior_tree.xml')

    # ——— Nav2 生命周期节点列表 ———
    lifecycle_nodes = [
        'global_costmap',
        'local_costmap',
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

        # 1. Odometry 桥接: TF map→base_link → /odom + odom→base_link TF
        Node(
            package='uav_nav2',
            executable='node_nav2_odometry',
            name='nav2_odometry_bridge',
            parameters=[nav2_params_path],
            output='screen',
        ),

        # 2. Nav2 桥接: /cmd_vel → /uav/target_position (NED, 固定高度)
        Node(
            package='uav_nav2',
            executable='node_nav2_bridge',
            name='nav2_bridge',
            parameters=[nav2_params_path],
            output='screen',
        ),

        # 3. 静态 TF: map → odom (identity)
        #    Cartographer 提供 map→base_link, Nav2 需要 odom→base_link.
        #    设 map→odom 为 identity 后, odometry 桥接发布的
        #    odom→base_link 恰好等于 map→base_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # ================================================================
        # Nav2 Costmap 节点
        # ================================================================

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            parameters=[
                global_costmap_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            parameters=[
                local_costmap_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # ================================================================
        # Nav2 规划器 + 控制器
        # ================================================================

        Node(
            package='nav2_planner',
            executable='nav2_planner',
            name='planner_server',
            parameters=[
                planner_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        Node(
            package='nav2_controller',
            executable='nav2_controller',
            name='controller_server',
            parameters=[
                dwb_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('cmd_vel', '/cmd_vel')],
            output='screen',
        ),

        # ================================================================
        # Nav2 BT Navigator (简化行为树)
        # ================================================================

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'default_bt_xml_filename': bt_path,
                'plugin_lib_names': [
                    'nav2_compute_path_to_pose_action_bt_node',
                    'nav2_follow_path_action_bt_node',
                ],
            }],
            output='screen',
        ),

        # ================================================================
        # Nav2 Lifecycle Manager (自动激活所有 Nav2 节点)
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
        # Rviz2 (可选)
        # ================================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'navigate.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])