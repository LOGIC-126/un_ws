#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF→PX4视觉里程计桥接节点 + 激光高度源.

两路数据融合为 VehicleOdometry 发布到 /fmu/in/vehicle_visual_odometry:
  - XY + Yaw: TF (SLAM/cartographer) → NED position
  - Z (高度): 激光定高 + IMU速度预测异常检测 → 滤波高度 → vision height

飞控端配合:
  EKF2_RNG_CTRL = 0      # 禁用EKF激光融合 (高度由本节点以vision方式提供)
  EKF2_HGT_REF  = 3      # (或 EKF2_HGT_MODE=3) 使用vision作为高度源
  EKF2_AID_MASK bit3=1   # 启用vision position融合
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry, DistanceSensor, VehicleLocalPosition
from tf2_ros import TransformException
import tf_transformations
import math


class Ekf2LinkDDS(Node):
    def __init__(self):
        super().__init__('ekf2_link_dds')

        # ---- TF/SLAM 参数 ----
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('fixed_z', 0.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.fixed_z = self.get_parameter('fixed_z').value

        # ---- 激光高度参数 ----
        self.declare_parameter('use_laser_height', True)
        self.declare_parameter('laser.car_height_threshold', 0.2)
        self.declare_parameter('laser.confirm_frames', 3)
        self.declare_parameter('laser.recovery_frames', 5)

        self.use_laser_height = self.get_parameter('use_laser_height').value
        self.car_height_threshold = self.get_parameter('laser.car_height_threshold').value
        self.confirm_frames = self.get_parameter('laser.confirm_frames').value
        self.recovery_frames = self.get_parameter('laser.recovery_frames').value

        # ---- TF 监听器 ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- QoS (匹配PX4 BEST_EFFORT) ----
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 发布者 ----
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)

        # ---- 订阅者 (BEST_EFFORT匹配PX4) ----
        self.distance_sensor_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.distance_sensor_callback, self.qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, self.qos_profile)

        # ---- 激光高度检测器状态 (IMU预测+激光验证, 零baro) ----
        self.raw_laser_distance = -1.0
        self.laser_distance_valid = False
        self.vehicle_local_pos = None       # VehicleLocalPosition 缓存

        self.ref_ground_z = None            # NED 地面参考高度
        self.ground_z_initialized = False
        self.anomaly_active = False
        self.anomaly_frame_count = 0
        self.normal_frame_count = 0
        self.prev_laser_distance = -1.0
        self.filtered_laser_innovation = 0.0
        self.filtered_laser_height = None   # 发布到 vision position[2]

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info(
            "DDS视觉里程计节点启动 "
            f"(laser_height={'ON' if self.use_laser_height else 'OFF'}, "
            f"fixed_z={self.fixed_z})"
        )

    # ==================== 订阅回调 ====================

    def distance_sensor_callback(self, msg: DistanceSensor) -> None:
        if msg.current_distance > 0.0:
            self.raw_laser_distance = msg.current_distance
            self.laser_distance_valid = True
        else:
            self.laser_distance_valid = False

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_pos = msg

    # ==================== 激光高度处理 (IMU预测+激光验证, 零baro) ====================

    def _update_laser_height(self, laser_distance: float, dt: float) -> float:
        """
        处理原始激光距离 → 滤波高度, 含小车异常检测.

        返回: 滤波后的激光高度 (NED, m).
        飞控端 EKF2_RNG_CTRL=0 保证 vehicle_local_position 不被激光污染.
        """
        if not self.laser_distance_valid or laser_distance <= 0.0:
            return None

        if self.vehicle_local_pos is None:
            return None

        ekf_z = self.vehicle_local_pos.z
        vz = self.vehicle_local_pos.vz

        # Step 0: 初始化地面参考 (仅一次)
        if not self.ground_z_initialized and laser_distance > 0.1:
            self.ref_ground_z = ekf_z + laser_distance
            self.ground_z_initialized = True
            self.prev_laser_distance = laser_distance
            self.filtered_laser_height = ekf_z
            self.filtered_laser_innovation = 0.0
            self.get_logger().info(
                f"[LaserHgt] Ground init: ref_z={self.ref_ground_z:.3f} "
                f"(ekf_z={ekf_z:.3f}, laser={laser_distance:.3f})"
            )
            return ekf_z

        if not self.ground_z_initialized:
            return None

        # Step 1: IMU预测激光变化
        if self.prev_laser_distance > 0:
            predicted_laser = self.prev_laser_distance - vz * dt
        else:
            predicted_laser = laser_distance

        laser_innovation = laser_distance - predicted_laser

        # LPF平滑innovation (过滤激光50Hz噪声)
        alpha_innov = 0.2
        self.filtered_laser_innovation = (
            (1.0 - alpha_innov) * self.filtered_laser_innovation
            + alpha_innov * laser_innovation
        )
        self.prev_laser_distance = laser_distance

        # Step 2: 异常检测
        innovation_trigger = (
            abs(self.filtered_laser_innovation) > self.car_height_threshold
        )

        # Step 3: 激光推算高度
        laser_height_raw = self.ref_ground_z - laser_distance
        alpha = 0.15  # ~3Hz @ 50Hz
        if self.filtered_laser_height is None:
            self.filtered_laser_height = laser_height_raw
        else:
            self.filtered_laser_height = (
                (1.0 - alpha) * self.filtered_laser_height
                + alpha * laser_height_raw
            )

        # Step 4: 状态机
        if innovation_trigger:
            self.normal_frame_count = 0
            self.anomaly_frame_count += 1
            if self.anomaly_frame_count > self.confirm_frames + 5:
                self.anomaly_frame_count = self.confirm_frames + 5

            if not self.anomaly_active and \
               self.anomaly_frame_count >= self.confirm_frames:
                self.anomaly_active = True
                self.get_logger().warn(
                    f"[LaserHgt] CAR DETECTED! "
                    f"innov={self.filtered_laser_innovation:+.2f}m, "
                    f"ref_ground_z adapting..."
                )

            # 用innovation更新ref_ground_z (零baro!)
            self.ref_ground_z += self.filtered_laser_innovation

        else:
            self.anomaly_frame_count = max(0, self.anomaly_frame_count - 1)

            if self.anomaly_active:
                self.normal_frame_count += 1
                if self.normal_frame_count >= self.recovery_frames:
                    self.get_logger().info(
                        f"[LaserHgt] Recovered. "
                        f"ref_ground_z={self.ref_ground_z:.3f}"
                    )
                    self.anomaly_active = False
                    self.anomaly_frame_count = 0
                    self.normal_frame_count = 0
            else:
                # 正常模式: 缓慢修正地面参考
                alpha_ground = 0.005
                self.ref_ground_z += self.filtered_laser_innovation * alpha_ground

        return self.filtered_laser_height

    # ==================== 主循环 ====================

    def timer_callback(self):
        # 1. 更新激光高度
        laser_z = None
        if self.use_laser_height:
            dt = 1.0 / self.frequency
            laser_z = self._update_laser_height(self.raw_laser_distance, dt)

        # 2. 获取 TF (XY + Yaw from SLAM)
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

            # 3. 构建 VehicleOdometry (NED)
            odom_msg = VehicleOdometry()
            odom_msg.timestamp = int(now.nanoseconds / 1000)
            odom_msg.timestamp_sample = odom_msg.timestamp
            odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

            # XY from SLAM, Z from laser (or fixed_z fallback)
            px4_z = laser_z if (laser_z is not None) else -self.fixed_z
            odom_msg.position = [t.x, -t.y, px4_z]

            # Yaw
            px4_yaw = -yaw_ros
            q_ned = tf_transformations.quaternion_from_euler(0.0, 0.0, px4_yaw)
            odom_msg.q = [q_ned[3], q_ned[0], q_ned[1], q_ned[2]]

            # 协方差: XY 2cm, Z 激光模式5cm / fixed模式10cm
            z_var = 0.0025 if (laser_z is not None) else 0.01
            odom_msg.position_variance = [0.0004, 0.0004, z_var]
            odom_msg.orientation_variance = [0.01, 0.01, 0.0004]

            # 速度标记为无效 (不融合vision速度)
            odom_msg.velocity = [float('nan')] * 3

            self.odom_pub.publish(odom_msg)

        except TransformException:
            # TF丢失时仍发布高度 (如果可用)
            if laser_z is not None:
                odom_msg = VehicleOdometry()
                odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                odom_msg.timestamp_sample = odom_msg.timestamp
                odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
                odom_msg.position = [float('nan'), float('nan'), laser_z]
                odom_msg.position_variance = [1e6, 1e6, 0.0025]
                odom_msg.orientation_variance = [1e6, 1e6, 1e6]
                odom_msg.q = [1.0, 0.0, 0.0, 0.0]  # identity
                odom_msg.velocity = [float('nan')] * 3
                self.odom_pub.publish(odom_msg)


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