#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF→PX4视觉里程计桥接节点 + 激光高度源.

两路数据融合为 VehicleOdometry 发布到 /fmu/in/vehicle_visual_odometry:
  - XY + Yaw: TF (SLAM/cartographer) → NED position
  - Z (高度): 激光定高 → 低通滤波 → vision height

飞控端配合:
  EKF2_EV_CTRL bit0+bit3=9 (vision位置+高度)
  EKF2_HGT_REF = 3          (vision作为高度源)
  EKF2_RNG_CTRL = 0         (禁用激光直接融合)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from px4_msgs.msg import VehicleOdometry, DistanceSensor, VehicleLocalPosition
from nav_msgs.msg import Odometry
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
        self.declare_parameter('use_laser_height', True)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.use_laser_height = self.get_parameter('use_laser_height').value

        # ---- 小车高度补偿 (默认关闭, 需同时订阅 /car/odom) ----
        self.declare_parameter('use_car_compensation', False)
        self.declare_parameter('car.compensation_distance', 0.5)   # 水平距离阈值(m)
        self.declare_parameter('car.compensation_height', 0.3)     # 高度补偿量(m)
        self.declare_parameter('car.offset_x', 0.6)                # odom→map X偏移(前+)
        self.declare_parameter('car.offset_y', -0.36)              # odom→map Y偏移(左+右-)
        self.use_car_comp = self.get_parameter('use_car_compensation').value
        self.car_comp_dist = self.get_parameter('car.compensation_distance').value
        self.car_comp_height = self.get_parameter('car.compensation_height').value
        self.car_offset_x = self.get_parameter('car.offset_x').value
        self.car_offset_y = self.get_parameter('car.offset_y').value

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

        # ---- 订阅者 ----
        self.distance_sensor_sub = self.create_subscription(
            DistanceSensor, '/fmu/out/distance_sensor',
            self.distance_sensor_callback, self.qos_profile)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, self.qos_profile)

        # ---- 小车位置 (补偿用) ----
        self.car_odom_sub = self.create_subscription(
            Odometry, '/car/odom', self.car_odom_callback, 10)
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_position_valid = False
        self.car_comp_active = False     # 当前是否在补偿区内

        # ---- 激光高度状态 ----
        self.raw_laser_distance = -1.0
        self.laser_distance_valid = False
        self.vehicle_local_pos = None
        self.ref_ground_z = None            # 地面NED高度
        self.ground_z_initialized = False
        self.filtered_laser_height = None   # LPF高度输出

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  DDS 视觉里程计 + 激光高度源")
        self.get_logger().info(f"  laser_height: {'ON' if self.use_laser_height else 'OFF'}")
        if self.use_car_comp:
            self.get_logger().info("  ★ CAR COMPENSATION MODE ★")
            self.get_logger().info(f"    dist<{self.car_comp_dist}m → height-{self.car_comp_height}m")
            self.get_logger().info(f"    odom→map offset: x={self.car_offset_x}, y={self.car_offset_y}")
        else:
            self.get_logger().info("  car_compensation: OFF")
        self.get_logger().info("=" * 55)

    # ==================== 回调 ====================

    def distance_sensor_callback(self, msg: DistanceSensor) -> None:
        if msg.current_distance > 0.0:
            self.raw_laser_distance = msg.current_distance
            self.laser_distance_valid = True
        else:
            self.laser_distance_valid = False

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_pos = msg

    def car_odom_callback(self, msg: Odometry) -> None:
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        self.car_position_valid = True

    # ==================== 激光高度 ====================

    def _update_laser_height(self, laser_distance: float) -> float:
        """简单低通滤波激光高度."""
        if not self.laser_distance_valid or laser_distance <= 0.0:
            return None
        if self.vehicle_local_pos is None:
            return None

        ekf_z = self.vehicle_local_pos.z

        # 初始化地面参考 (仅一次, 不依赖baro)
        # 无人机在地面时 laser=安装偏移, 直接以此为 ref_ground_z
        # 飞行中: laser_height = ref_ground_z - laser_distance, 偏移自然抵消
        if not self.ground_z_initialized and laser_distance > 0.1:
            self.ref_ground_z = laser_distance
            self.ground_z_initialized = True
            self.filtered_laser_height = 0.0  # 在地面,高度为0
            self.get_logger().info(
                f"[LaserHgt] Init: ref_z={self.ref_ground_z:.3f} "
                f"(laser={laser_distance:.3f}, baro_z={ekf_z:.3f} ignored)"
            )
            return 0.0

        if not self.ground_z_initialized:
            return None

        # 激光推算高度
        laser_height_raw = self.ref_ground_z - laser_distance

        # LPF (α=0.15, ~3Hz) 平滑输出
        if self.filtered_laser_height is None:
            self.filtered_laser_height = laser_height_raw
        else:
            self.filtered_laser_height = (
                0.85 * self.filtered_laser_height + 0.15 * laser_height_raw
            )

        return self.filtered_laser_height

    # ==================== 主循环 ====================

    def timer_callback(self):
        # 1. 激光高度
        laser_z = None
        if self.use_laser_height:
            laser_z = self._update_laser_height(self.raw_laser_distance)

        # 2. TF + 发布
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

            # ---- 小车高度补偿: drone接近小车时自动下调高度 ----
            comp_z = laser_z
            if self.use_car_comp and laser_z is not None and self.car_position_valid:
                car_x_map = self.car_x + self.car_offset_x
                car_y_map = self.car_y + self.car_offset_y
                dist = math.hypot(t.x - car_x_map, t.y - car_y_map)
                in_zone = dist < self.car_comp_dist

                if in_zone and not self.car_comp_active:
                    self.car_comp_active = True
                    self.get_logger().warn(
                        f"★★★ ENTER CAR ZONE! dist={dist:.2f}m, "
                        f"height -{self.car_comp_height}m ★★★"
                    )
                elif not in_zone and self.car_comp_active:
                    self.car_comp_active = False
                    self.get_logger().info(
                        f"    exit car zone, dist={dist:.2f}m"
                    )

                if in_zone:
                    comp_z = laser_z - self.car_comp_height
                    self.get_logger().info(
                        f"[Comp] dist={dist:.2f}m → z={comp_z:.3f}",
                        throttle_duration_sec=1.0
                    )

            odom_msg.position = [t.x, -t.y, comp_z if (comp_z is not None) else float('nan')]

            px4_yaw = -yaw_ros
            q_ned = tf_transformations.quaternion_from_euler(0.0, 0.0, px4_yaw)
            odom_msg.q = [q_ned[3], q_ned[0], q_ned[1], q_ned[2]]

            odom_msg.position_variance = [0.0004, 0.0004, 0.01]
            odom_msg.orientation_variance = [0.01, 0.01, 0.0004]
            odom_msg.velocity = [float('nan')] * 3

            self.odom_pub.publish(odom_msg)

        except TransformException:
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