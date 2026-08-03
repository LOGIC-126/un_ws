#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF→PX4视觉里程计桥接节点.

将 SLAM/cartographer 的 TF (map→base_link) 映射为 PX4 NED 坐标系,
以 VehicleOdometry 发布到 /fmu/in/vehicle_visual_odometry.
  - XY + Yaw: TF → NED position
  - Z (高度): fixed_z 参数固定值

飞控端配合:
  EKF2_EV_CTRL bit0=1 (vision位置)
  EKF2_HGT_REF = 0          (baro作为高度源, 不使用vision高度)
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformException
import tf_transformations
import math


class Ekf2LinkDDS(Node):
    def __init__(self):
        super().__init__('ekf2_link_dds')

        # 参数配置
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('fixed_z', 0.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.fixed_z = self.get_parameter('fixed_z').value

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 发布者
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info(
            "DDS 视觉里程计节点启动: ROS FLU → PX4 NED "
            f"(fixed_z={self.fixed_z})"
        )

    def timer_callback(self):
        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            q_ros = [r.x, r.y, r.z, r.w]
            _, _, yaw_ros = tf_transformations.euler_from_quaternion(q_ros)

            odom_msg = VehicleOdometry()
            odom_msg.timestamp = int(now.nanoseconds / 1000)
            odom_msg.timestamp_sample = odom_msg.timestamp
            odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

            # ROS FLU → PX4 NED
            # PX4_X (North) = ROS_X
            # PX4_Y (East)  = -ROS_Y
            # PX4_Z (Down)  = -fixed_z
            odom_msg.position = [t.x, -t.y, -self.fixed_z]

            # Yaw: ROS CCW+ → NED CW+
            px4_yaw = -yaw_ros
            q_ned = tf_transformations.quaternion_from_euler(0.0, 0.0, px4_yaw)
            odom_msg.q = [q_ned[3], q_ned[0], q_ned[1], q_ned[2]]

            # 协方差: XY 2cm, Z 10cm, Yaw可信
            odom_msg.position_variance = [0.0004, 0.0004, 0.01]
            odom_msg.orientation_variance = [0.01, 0.01, 0.0004]

            # 速度标记为无效 (不融合vision速度)
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
