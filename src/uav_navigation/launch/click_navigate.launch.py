#!/usr/bin/env python3
"""
启动点击导航节点 + Rviz2.
用法: ros2 launch uav_navigation click_navigate.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'uav_navigation'

    return LaunchDescription([

        # ── 点击导航节点 ──
        Node(
            package=pkg,
            executable='node_click_navigate',
            name='click_navigate_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare(pkg), 'config', 'nav_params.yaml'
                ])
            ],
        ),

        # ── Rviz2 可视化 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([
                    FindPackageShare(pkg), 'rviz', 'click_navigate.rviz'
                ]),
                '-f', 'map',          # Fixed Frame
            ],
        ),

    ])