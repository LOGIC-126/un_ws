#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 从TF树中获取2D SLAM位姿 (x, y, yaw)，转换为带协方差的PoseWithCovarianceStamped消息，
       发布到 /mavros/vision_pose/pose_cov 供PX4 EKF2融合。
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf2_ros import TransformException
import tf_transformations
import math
import numpy as np

class Ekf2_link(Node):
    def __init__(self):
        super().__init__('ekf2_link')

        # 参数
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_frequency', 100.0)
        self.declare_parameter('fixed_z', 0.0)
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('tf_timeout_sec', 1.0)          
        self.declare_parameter('tf_wait_for_ready', True)
        
        # 协方差相关参数
        self.declare_parameter('pos_xy_noise', 0.01)
        self.declare_parameter('pos_z_noise', 100.0)
        self.declare_parameter('yaw_noise', 0.01)
        self.declare_parameter('roll_pitch_noise', 10000.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.fixed_z = self.get_parameter('fixed_z').value
        self.output_frame = self.get_parameter('output_frame_id').value
        self.tf_timeout = self.get_parameter('tf_timeout_sec').value
        self.wait_for_tf = self.get_parameter('tf_wait_for_ready').value
        
        self.pos_xy_noise = self.get_parameter('pos_xy_noise').value
        self.pos_z_noise = self.get_parameter('pos_z_noise').value
        self.yaw_noise = self.get_parameter('yaw_noise').value
        self.roll_pitch_noise = self.get_parameter('roll_pitch_noise').value

        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 等待 TF 就绪
        if self.wait_for_tf:
            self.get_logger().info(f"等待 TF {self.source_frame} -> {self.target_frame} 就绪...")
            self.wait_for_transform()

        # 发布带协方差的消息到 MAVROS
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 
                                              '/mavros/vision_pose/pose_cov', 10)

        # 定时器
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 上次有效位姿
        self.last_valid_pose = None

        self.get_logger().info(
            f"2D SLAM带协方差节点启动：监听 {self.source_frame}->{self.target_frame}，"
            f"固定高度={self.fixed_z}m，频率={self.frequency}Hz，TF超时={self.tf_timeout}s"
        )

    def wait_for_transform(self):
        """等待 source_frame -> target_frame 的变换可用"""
        start = self.get_clock().now()
        timeout_sec = 5.0
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    self.source_frame, self.target_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info("TF 已就绪")
                return
            except TransformException:
                if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout_sec:
                    self.get_logger().error(f"等待 TF 超时 ({timeout_sec}s)，继续尝试运行")
                    break
                rclpy.spin_once(self, timeout_sec=0.1)

    def build_covariance_matrix(self):
        """构建 6x6 协方差矩阵 (顺序: x, y, z, roll, pitch, yaw)"""
        cov = np.zeros(36)
        cov[0] = self.pos_xy_noise   # x
        cov[7] = self.pos_xy_noise   # y
        cov[14] = self.pos_z_noise   # z
        cov[21] = self.roll_pitch_noise  # roll
        cov[28] = self.roll_pitch_noise  # pitch
        cov[35] = self.yaw_noise         # yaw
        return cov.tolist()

    def timer_callback(self):
        try:
            # 获取变换（使用可配置的超时时间）
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )

            # 提取 x, y, 四元数 -> 转欧拉角获得 yaw
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q_in = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, yaw = tf_transformations.euler_from_quaternion(q_in)

            # 构造新的四元数：只保留 yaw，roll=pitch=0
            q_out = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

            # 创建 PoseWithCovarianceStamped 消息
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.output_frame
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = self.fixed_z
            pose_msg.pose.pose.orientation = Quaternion(x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3])
            
            pose_msg.pose.covariance = self.build_covariance_matrix()

            self.pose_pub.publish(pose_msg)
            self.last_valid_pose = pose_msg

            # 调试输出
            self.get_logger().debug(
                f"位姿: x={x:.2f}, y={y:.2f}, z={self.fixed_z:.2f}, yaw={math.degrees(yaw):.1f}°",
                throttle_duration_sec=0.1
            )

        except TransformException as ex:
            if self.last_valid_pose:
                self.get_logger().warn(f"TF异常: {ex}，使用上次位姿", throttle_duration_sec=2.0)
                self.pose_pub.publish(self.last_valid_pose)
            else:
                self.get_logger().error(f"无法获取TF且无历史数据: {ex}", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = Ekf2_link()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()