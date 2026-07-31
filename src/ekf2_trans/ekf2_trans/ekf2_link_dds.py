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

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value

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

        # ---- 激光高度检测器状态 (IMU速度区分, 零baro) ----
        self.raw_laser_distance = -1.0
        self.laser_distance_valid = False
        self.vehicle_local_pos = None

        self.ref_ground_z = None            # NED 地面参考高度
        self.ground_z_initialized = False
        self.fast_laser_height = None       # 滤波高度输出 (α=0.15)
        self.prev_laser_for_imu = -1.0      # 上一帧激光 (IMU对比用)
        self.surface_motion_accum = 0.0     # 地面运动累积器
        self.surface_motion_count = 0       # 累积帧计数

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info(
            "DDS视觉里程计节点启动 "
            f"(laser_height={'ON' if self.use_laser_height else 'OFF'})"
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

    # ==================== 激光高度处理 (IMU速度区分: 谁在动?) ====================

    def _update_laser_height(self, laser_distance: float, dt: float) -> float:
        """
        IMU速度区分异常检测: 用IMU vz区分"无人机在动"和"地面在动".

        核心:
          delta_laser_h = -(laser - prev_laser)  → 激光说高度变了多少
          delta_imu_h   = vz * dt                → IMU说高度变了多少
          surface_delta = delta_laser_h - delta_imu_h  → 地面动了多少

          无人机正常升降 → delta_laser ≈ delta_imu → surface≈0
          小车出现/消失   → delta_laser有, delta_imu≈0 → surface≠0 → 异常!

        累积5帧(100ms)后判定, 避免单帧激光噪声.
        """
        if not self.laser_distance_valid or laser_distance <= 0.0:
            return None

        if self.vehicle_local_pos is None:
            return None

        ekf_z = self.vehicle_local_pos.z
        vz = self.vehicle_local_pos.vz

        # ---- Step 0: 初始化 ----
        if not self.ground_z_initialized and laser_distance > 0.1:
            self.ref_ground_z = ekf_z + laser_distance
            self.ground_z_initialized = True
            self.fast_laser_height = ekf_z
            self.prev_laser_for_imu = laser_distance
            self.surface_motion_accum = 0.0
            self.surface_motion_count = 0
            self.get_logger().info(
                f"[LaserHgt] Ground init: ref_z={self.ref_ground_z:.3f} "
                f"(z={ekf_z:.3f}, laser={laser_distance:.3f})"
            )
            return ekf_z

        if not self.ground_z_initialized:
            return None

        # ---- Step 1: 激光推算高度 (始终输出) ----
        laser_height_raw = self.ref_ground_z - laser_distance
        if self.fast_laser_height is None:
            self.fast_laser_height = laser_height_raw
        else:
            self.fast_laser_height = (
                0.85 * self.fast_laser_height + 0.15 * laser_height_raw
            )

        # ---- Step 2: IMU速度区分 — 谁在动? ----
        if hasattr(self, 'prev_laser_for_imu') and self.prev_laser_for_imu > 0:
            # 激光推算的高度变化 (激光减小=无人机下降, NED正方向)
            delta_laser_h = -(laser_distance - self.prev_laser_for_imu)
            # IMU推算的高度变化
            delta_imu_h = vz * dt
            # 地面运动 = 激光看到的变化 - 无人机自身运动
            surface_delta = delta_laser_h - delta_imu_h

            self.surface_motion_accum += surface_delta
            self.surface_motion_count += 1
        self.prev_laser_for_imu = laser_distance

        # 每5帧(100ms)判定一次
        anomaly_this_frame = False
        if self.surface_motion_count >= 5:
            anomaly_this_frame = (
                abs(self.surface_motion_accum) > self.car_height_threshold
            )
            if anomaly_this_frame:
                # 地面突变 → 修正ref_ground_z (符号: surface>0=地面升=小车出现)
                self.ref_ground_z -= self.surface_motion_accum
                # 重置高度输出, 跳过LPF收敛延迟
                self.fast_laser_height = self.ref_ground_z - laser_distance
                self.get_logger().warn(
                    f"[LaserHgt] SURFACE JUMP! "
                    f"surface_delta={self.surface_motion_accum:+.2f}m, "
                    f"ref_ground_z→{self.ref_ground_z:.3f}"
                )
            # 重置累积器
            self.surface_motion_accum = 0.0
            self.surface_motion_count = 0

        # ---- Step 3: 正常时缓慢修正地面参考 ----
        if not anomaly_this_frame and not self.anomaly_active:
            # 用surface_motion的慢速EMA来跟踪地形渐变
            pass  # ref_ground_z 保持不变, 依赖异常检测时的大跳更新

        return self.fast_laser_height

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

            # XYZ 3D定位: X/Y来自SLAM, Z来自激光(未就绪时NaN)
            # PX4 EKF: innovation gate用 position_variance 过滤异常
            odom_msg.position = [
                t.x, -t.y,
                laser_z if (laser_z is not None) else float('nan')
            ]

            # Yaw from SLAM
            px4_yaw = -yaw_ros
            q_ned = tf_transformations.quaternion_from_euler(0.0, 0.0, px4_yaw)
            odom_msg.q = [q_ned[3], q_ned[0], q_ned[1], q_ned[2]]

            # 协方差: XY=2cm, Z=10cm (激光精度, EKF可平滑融合)
            # 飞控需设: EKF2_AID_MASK bit3=vision_pos  EKF2_HGT_REF=3
            odom_msg.position_variance = [0.0004, 0.0004, 0.01]
            odom_msg.orientation_variance = [0.01, 0.01, 0.0004]
            odom_msg.velocity = [float('nan')] * 3

            self.odom_pub.publish(odom_msg)

        except TransformException:
            # TF丢失时仍发布激光高度
            if laser_z is not None:
                odom_msg = VehicleOdometry()
                odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                odom_msg.timestamp_sample = odom_msg.timestamp
                odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
                odom_msg.position = [float('nan'), float('nan'), laser_z]
                odom_msg.position_variance = [1e6, 1e6, 0.01]
                odom_msg.orientation_variance = [1e6, 1e6, 1e6]
                odom_msg.q = [1.0, 0.0, 0.0, 0.0]
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