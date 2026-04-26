#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import tf2_ros
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformException
import tf_transformations
import math
import numpy as np

class Ekf2LinkDDS(Node):
    def __init__(self):
        super().__init__('ekf2_link_dds')

        # 参数配置
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_frequency', 50.0) # 建议 30-50Hz
        self.declare_parameter('fixed_z', 0.0)
        
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.fixed_z = self.get_parameter('fixed_z').value

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 发布者
        self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info("DDS 坐标修正节点启动：将 ROS FLU 映射为 PX4 NED")

    def timer_callback(self):
        try:
            # 1. 获取 TF 变换 (ROS 默认：x-北/前, y-西/左, z-上)
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame,
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            # 2. 提取 Yaw 并处理角度转换
            # 在 ROS 中，Yaw 是逆时针(CCW)为正；在 NED 中，Yaw 是顺时针(CW)为正
            q_ros = [r.x, r.y, r.z, r.w]
            _, _, yaw_ros = tf_transformations.euler_from_quaternion(q_ros)

            # 3. 构建 PX4 NED 坐标系下的数据
            odom_msg = VehicleOdometry()
            odom_msg.timestamp = int(now.nanoseconds / 1000)
            odom_msg.timestamp_sample = odom_msg.timestamp
            
            # 设置坐标参考系为 NED
            odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

            # --- 关键：位置映射 ---
            # PX4_X (North) = ROS_X
            # PX4_Y (East)  = -ROS_Y  (翻转 Y，解决向东走 Y 减小的问题)
            # PX4_Z (Down)  = -Fixed_Z (翻转 Z，向上为负)
            odom_msg.position = [t.x, -t.y, -self.fixed_z]

            # --- 关键：姿态映射 (仅 Yaw 融合) ---
            # 同样，由于 Y 轴翻转，NED 下的 Yaw 应该是 -yaw_ros
            px4_yaw = -yaw_ros
            q_ned = tf_transformations.quaternion_from_euler(0.0, 0.0, px4_yaw)
            # px4_msgs 使用 [w, x, y, z] 顺序
            odom_msg.q = [q_ned[3], q_ned[0], q_ned[1], q_ned[2]]

            # 4. 填充协方差 (针对 2D SLAM 调优)
            # 位置标准差设为 0.02m (方差 0.0004)，高度标准差给大一点（如果不准）
            odom_msg.position_variance = [0.0004, 0.0004, 0.01]
            # 角度方差，Yaw 设小，Roll/Pitch 设大表示不可信
            odom_msg.orientation_variance = [0.01, 0.01, 0.0004]

            # 速度标记为无效
            odom_msg.velocity = [float('nan')] * 3
            
            self.odom_pub.publish(odom_msg)

        except TransformException:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = Ekf2LinkDDS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()