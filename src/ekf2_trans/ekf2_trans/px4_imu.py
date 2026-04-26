#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined

class Px4ImuTranslator(Node):
    def __init__(self):
        super().__init__('px4_imu_translator')

        # 1. 记录上一次发布的时间戳，用于解决 Cartographer 的 time_ <= time 崩溃问题
        self.last_timestamp = self.get_clock().now()

        # 2. 配置针对 PX4 的 QoS (Best Effort)
        # PX4 默认发布 sensor_combined 使用的是 Best Effort，必须匹配才能收到数据
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅 PX4 原始话题
        self.subscription = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.listener_callback,
            qos_profile)

        # 发布标准 IMU 话题
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        
        self.get_logger().info('PX4 to IMU Translator Node Started')
        self.get_logger().info('Correction: Using ROS System Time to prevent timestamp backward jumps.')

    def listener_callback(self, msg):
        imu_msg = Imu()
        
        # --- 核心修复：时间戳单调递增逻辑 ---
        now = self.get_clock().now()
        
        # 如果当前系统时间因为抖动小于或等于上一帧时间，则强行递增 1 纳秒
        if now <= self.last_timestamp:
            now = self.last_timestamp + Duration(nanoseconds=1)
        
        self.last_timestamp = now
        imu_msg.header.stamp = now.to_msg()
        # -----------------------------------

        # 坐标系必须与 static_transform_publisher 中的 child_frame_id 一致
        imu_msg.header.frame_id = 'imu_link'

        # 3. 坐标系转换 (PX4 FRD -> ROS FLU)
        # X: 前, Y: 右 -> 左 (取反), Z: 下 -> 上 (取反)
        
        # 角速度
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = -float(msg.gyro_rad[1])
        imu_msg.angular_velocity.z = -float(msg.gyro_rad[2])

        # 线加速度
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = -float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = -float(msg.accelerometer_m_s2[2])

        # 4. 协方差处理
        # Cartographer 不需要具体的协方差数值，但建议设置 orientation_covariance[0] = -1
        # 表示该消息不包含姿态信息（由 Cartographer 内部进行积分计算）
        imu_msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Px4ImuTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()