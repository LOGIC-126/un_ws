#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF 小车追踪节点 (仿写 yolo_tracking.py, 参考 ekf2_link_dds.py TF 监听模式)

通过 TF2 监听小车 map→car_base_link 变换，计算小车相对无人机的 NED 偏移，
驱动无人机追踪小车。

坐标系 (与 ekf2_link_dds.py 完全一致):
  ROS map (Cartographer): x=北, y=西, z=上, yaw=CCW+
  PX4 NED:               x=北, y=东, z=下, yaw=CW+
  转换: ned_x=ros_x, ned_y=-ros_y, ned_z=-ros_z, ned_yaw=-ros_yaw

相对定位法:
  1. TF 查询 map→base_link (无人机 map 位姿)
  2. TF 查询 map→car_frame  (小车 map 位姿)
  3. delta = car_map - drone_map   (ROS FLU 相对偏移)
  4. ned_delta = (delta.x, -delta.y)  → NED 偏移
  5. target_ned = drone_ned + ned_delta  → 目标 NED

可选融合 YOLO 视觉检测, 提高追踪鲁棒性。

状态机: INIT → TAKEOFF → WAIT ⇄ TRACK ⇄ LOST → DROP → RTH (全自动, 无需RC)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum

import tf2_ros
from tf2_ros import TransformException
from tf2_msgs.msg import TFMessage
import tf_transformations

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, PoseArray


# ====== 状态枚举 ======
class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    WAIT = 2
    TRACK = 3
    LOST = 4
    DROP = 5
    RTH = 6    # Return To Home 返航


class TFTrackingNode(Node):
    """TF 小车追踪节点: TF2 监听 + 状态机驱动, 追踪小车"""

    def __init__(self):
        super().__init__('tf_tracking_node')

        # ====== 参数声明 (可通过命令行 --ros-args -p 覆盖) ======
        self.declare_parameter('takeoff_height', -1.2)
        self.declare_parameter('arrival_threshold', 0.3)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('search_timeout', 10.0)
        self.declare_parameter('drop_distance_threshold', 0.5)
        self.declare_parameter('drop_dwell_time', 3.0)

        # —— TF 帧配置 ——
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('drone_frame', 'base_link')
        self.declare_parameter('car_frame', 'base_footprint')  # 小车 frame (通常 base_footprint)

        # —— 小车 odom→map 偏移 (与 ekf2_link_dds.py 一致) ——
        self.declare_parameter('car.offset_x', 0.6)   # odom→map X偏移(前+)
        self.declare_parameter('car.offset_y', -0.36)  # odom→map Y偏移(左+右-)

        # —— 追踪参数 ——
        self.declare_parameter('max_distance', 15.0)

        # —— 视觉融合 ——
        self.declare_parameter('enable_vision_fusion', False)
        self.declare_parameter('fusion_alpha', 0.8)
        self.declare_parameter('vision_match_threshold', 1.5)
        self.declare_parameter('fence_radius', 1.5)

        # 读取参数值
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.drop_distance_threshold = self.get_parameter('drop_distance_threshold').value
        self.drop_dwell_time = self.get_parameter('drop_dwell_time').value
        self.drop_height = -0.5   # 抛投高度 NED (0.5m, 明显低于追踪高度)
        self.rth_delay = 5.0     # 抛投完成后等待时间 (s), 然后返航

        self.map_frame = self.get_parameter('map_frame').value
        self.drone_frame = self.get_parameter('drone_frame').value
        self.car_frame = self.get_parameter('car_frame').value
        self.car_offset_x = self.get_parameter('car.offset_x').value
        self.car_offset_y = self.get_parameter('car.offset_y').value
        self.max_distance = self.get_parameter('max_distance').value

        self.enable_vision_fusion = self.get_parameter('enable_vision_fusion').value
        self.fusion_alpha = self.get_parameter('fusion_alpha').value
        self.vision_match_threshold = self.get_parameter('vision_match_threshold').value
        self.fence_radius = self.get_parameter('fence_radius').value

        # ====== TF2 监听器 (多话题: /tf + /car/tf, 参考 waypoint_tracker) ======
        self.tf_buffer = tf2_ros.Buffer()
        # 手动订阅所有 TF 源，统一塞进 Buffer
        for topic in ('/tf', '/tf_static', '/car/tf', '/car/tf_static'):
            self.create_subscription(
                TFMessage, topic,
                lambda msg: [self.tf_buffer.set_transform(t, 'default_authority')
                             for t in msg.transforms],
                100)

        # ====== QoS ======
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ====== 发布器 ======
        self.target_position_pub = self.create_publisher(
            Pose, '/uav/target_position', qos_profile)
        self.drop_complete_pub = self.create_publisher(
            Int32, '/car/drop_complete', 10)

        # ====== 订阅器 ======
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos_profile)

        # —— YOLO 视觉检测 (可选, 融合模式) ——
        if self.enable_vision_fusion:
            self.world_coords_sub = self.create_subscription(
                PoseArray, '/detection/world_coordinates',
                self.world_coordinates_callback, 10)
        else:
            self.world_coords_sub = None

        # —— 小车启动触发 (waypoint_tracker 行驶≥0.5m 后发布) ——
        self._car_triggered = False
        self.car_trigger_sub = self.create_subscription(
            Int32, '/car/trigger', self._car_trigger_callback, 10)

        # ====== 状态变量 ======
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        self.state = FlightState.INIT

        # 小车位姿缓存 (无人机 NED 坐标系)
        self._car_ned = None             # (ned_x, ned_y) or None

        # 检测数据缓存 (视觉融合)
        self._latest_detections = None

        # 目标确认计数器
        self._car_detection_count = 0

        # 锁定目标 (NED: x=北, y=东)
        self._locked_target = None

        # 丢失计时
        self._lost_start_time = None
        self._search_start_time = None

        # 抛投计时
        self._drop_start_time = None
        self._drop_enter_time = None   # 进入DROP状态的时刻

        # 悬停位置
        self._hover_x = 0.0
        self._hover_y = 0.0

        # 目标缓存
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # ====== 定时器 (20Hz) ======
        self.timer = self.create_timer(0.05, self.timer_callback)

        mode_str = "视觉融合" if self.enable_vision_fusion else "纯TF"
        self.get_logger().info(
            f"TF追踪节点已启动 ({mode_str}) | "
            f"map={self.map_frame} | drone={self.drone_frame} | car={self.car_frame} | "
            f"起飞高度={self.takeoff_height}m | 最大距离={self.max_distance}m"
        )

    # ==================== 订阅回调 ====================

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _car_trigger_callback(self, msg: Int32) -> None:
        """接收小车 waypoint_tracker 的启动信号 (行驶≥0.5m 后触发)"""
        if msg.data == 1 and not self._car_triggered:
            self._car_triggered = True
            self.get_logger().info(f'收到小车启动触发信号 → 无人机将自主起飞')

    def world_coordinates_callback(self, msg: PoseArray) -> None:
        self._latest_detections = msg

    # ==================== TF 查询 (参考 ekf2_link_dds.py:176-181) ====================

    def _lookup_car_ned(self):
        """
        TF 查询小车在 map 帧位置 → 相对偏移 → 无人机 NED 坐标。

        与 ekf2_link_dds.py:222 相同的 FLU→NED 转换:
          ned_x = ros_x,  ned_y = -ros_y

        小车 odom→map 偏移 (与 ekf2_link_dds.py 一致):
          car_map = car_tf + (car_offset_x, car_offset_y)

        返回 (ned_x, ned_y) 或 None
        """
        try:
            # 1. 查询无人机在 map 帧的位姿
            drone_tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.drone_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            drone_map_x = drone_tf.transform.translation.x
            drone_map_y = drone_tf.transform.translation.y

            # 2. 查询小车在 TF 中的位姿 (odom帧) + odom→map 偏移 = map帧
            car_tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.car_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            car_map_x = car_tf.transform.translation.x + self.car_offset_x
            car_map_y = car_tf.transform.translation.y + self.car_offset_y

        except TransformException as e:
            self.get_logger().warn(
                f"TF查询失败 ({self.map_frame}→{self.drone_frame} 或 "
                f"{self.map_frame}→{self.car_frame}): {e}",
                throttle_duration_sec=3.0)
            return None

        # 3. 相对偏移 (ROS FLU: x=北, y=西)
        delta_x = car_map_x - drone_map_x    # 北向差
        delta_y = car_map_y - drone_map_y    # 西向差

        # 4. FLU → NED (与 ekf2_link_dds.py:222 一致)
        ned_delta_x = delta_x      # 北→北
        ned_delta_y = -delta_y     # 西→东 (取反)

        # 5. 无人机当前 NED + 相对偏移 → 小车在无人机 NED 中的位置
        drone = self.vehicle_local_position
        if math.isnan(drone.x) or math.isnan(drone.y):
            return None

        ned_x = drone.x + ned_delta_x
        ned_y = drone.y + ned_delta_y

        return (ned_x, ned_y)

    # ==================== 坐标转换 (视觉融合) ====================

    @staticmethod
    def _world_pose_to_ned(pose: Pose):
        """
        将 detection_world_node.py 发布的坐标转为 NED。
        detection_world_node.py 编码:
          pose.x = world_north,  pose.y = -world_east
        逆变换:
          ned_north = pose.x,  ned_east = -pose.y
        """
        return (pose.position.x, -pose.position.y)

    # ==================== 目标获取 ====================

    def _get_target_ned(self):
        """
        获取追踪目标 NED 坐标。

        纯 TF 模式: 直接返回 TF 查询结果
        融合模式: TF + YOLO 互补滤波

        返回 (ned_x, ned_y, distance) 或 None
        """
        car = self._lookup_car_ned()
        if car is None:
            return None

        drone = self.vehicle_local_position
        if math.isnan(drone.x) or math.isnan(drone.y):
            return None

        # 距离检查
        dx = car[0] - drone.x
        dy = car[1] - drone.y
        dist = math.hypot(dx, dy)
        if dist > self.max_distance:
            return None

        # —— 视觉融合 ——
        if self.enable_vision_fusion and self._latest_detections is not None:
            yolo_target = self._match_vision_to_car(car)
            if yolo_target is not None:
                yolo_x, yolo_y = yolo_target
                alpha = self.fusion_alpha
                fused_x = alpha * car[0] + (1.0 - alpha) * yolo_x
                fused_y = alpha * car[1] + (1.0 - alpha) * yolo_y
                fused_dist = math.hypot(fused_x - drone.x, fused_y - drone.y)
                return (fused_x, fused_y, fused_dist)

        return (car[0], car[1], dist)

    def _match_vision_to_car(self, car_ned):
        """从 YOLO 检测中找离小车 TF 位置最近的目标。返回 (ned_x, ned_y) 或 None"""
        if self._latest_detections is None:
            return None
        poses = self._latest_detections.poses
        if not poses:
            return None

        best = None
        best_dist = self.vision_match_threshold

        for pose in poses:
            ned_x, ned_y = self._world_pose_to_ned(pose)
            d = math.hypot(ned_x - car_ned[0], ned_y - car_ned[1])
            if d < best_dist:
                best_dist = d
                best = (ned_x, ned_y)

        return best

    # ==================== 发布 ====================

    def publish_target_position(self) -> None:
        msg = Pose()
        msg.position.x = float(self.target_x)
        msg.position.y = float(self.target_y)
        msg.position.z = float(self.target_z)
        msg.orientation.w = math.cos(self.target_yaw / 2.0)
        msg.orientation.z = math.sin(self.target_yaw / 2.0)
        self.target_position_pub.publish(msg)

    def set_target_position(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        fx, fy, fz, fyaw = float(x), float(y), float(z), float(yaw)
        if (fx == self.target_x and fy == self.target_y and
                fz == self.target_z and fyaw == self.target_yaw):
            return
        self.target_x = fx
        self.target_y = fy
        self.target_z = fz
        self.target_yaw = fyaw
        self.publish_target_position()

    def check_arrived(self, x: float, y: float, z: float) -> bool:
        pos = self.vehicle_local_position
        if math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z):
            return False
        dist = math.sqrt((pos.x - x) ** 2 + (pos.y - y) ** 2 + (pos.z - z) ** 2)
        return dist < self.arrival_threshold

    # ==================== 状态机辅助 ====================

    def _reset_tracking_counters(self) -> None:
        self._car_detection_count = 0
        self._lost_start_time = None
        self._search_start_time = None

    def _record_hover_position(self) -> None:
        drone = self.vehicle_local_position
        if not math.isnan(drone.x) and not math.isnan(drone.y):
            self._hover_x = drone.x
            self._hover_y = drone.y

    # ==================== 主循环 ====================

    def timer_callback(self) -> None:
        is_armed = (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.state != FlightState.INIT:
            if not is_armed or not is_offboard:
                self.get_logger().warn("掉出 Offboard / 上锁状态, 退回 INIT.")
                self.state = FlightState.INIT
                self._reset_tracking_counters()
                self._locked_target = None
                return

        self.run_state_machine()

        # 状态显示 (每2秒)
        self._print_status()

    def _print_status(self) -> None:
        """每2秒打印追踪状态"""
        now = self.get_clock().now()
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = now
            return
        if (now - self._last_log_time).nanoseconds * 1e-9 < 2.0:
            return
        self._last_log_time = now

        drone = self.vehicle_local_position
        state_name = self.state.name
        car = self._lookup_car_ned()  # 不阻塞, 快速查询

        if car is not None:
            dx = car[0] - drone.x
            dy = car[1] - drone.y
            dist = math.hypot(dx, dy)
            # 抛投接近提示
            drop_hint = ""
            if self.state == FlightState.TRACK and self._drop_start_time is not None:
                dwell = (now - self._drop_start_time).nanoseconds * 1e-9
                drop_hint = f" | 抛投倒计时 {dwell:.1f}s/{self.drop_dwell_time}s (阈值{self.drop_distance_threshold}m)"
            elif self.state == FlightState.DROP:
                drop_hint = f" | 抛投执行中 Z目标={self.drop_height:.1f}m"
            self.get_logger().info(
                f"[{state_name}] "
                f"无人机 NED({drone.x:.2f}, {drone.y:.2f}, {drone.z:.2f}) | "
                f"小车 NED({car[0]:.2f}, {car[1]:.2f}) | "
                f"目标 NED({self.target_x:.2f}, {self.target_y:.2f}) | "
                f"距离={dist:.2f}m{drop_hint}"
            )
        else:
            self.get_logger().info(
                f"[{state_name}] "
                f"无人机 NED({drone.x:.2f}, {drone.y:.2f}, {drone.z:.2f}) | "
                f"小车: 未检测到 | "
                f"目标 NED({self.target_x:.2f}, {self.target_y:.2f})"
            )

    def run_state_machine(self) -> None:
        # ============================
        # INIT: 地面等待小车 /car/trigger 启动信号
        # ============================
        if self.state == FlightState.INIT:
            self.set_target_position(0.0, 0.0, 0.0)

            if self._car_triggered:
                self.get_logger().info("INIT → TAKEOFF (car trigger)")
                self.set_target_position(0.0, 0.0, self.takeoff_height)
                self.state = FlightState.TAKEOFF
            else:
                self.get_logger().info(
                    "等待小车 /car/trigger 启动信号...",
                    throttle_duration_sec=3.0,
                )

        # ============================
        # TAKEOFF: 爬升至固定高度
        # ============================
        elif self.state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.takeoff_height)

            if self.check_arrived(0.0, 0.0, self.takeoff_height):
                self.get_logger().info("TAKEOFF → WAIT (到达起飞高度)")
                self._record_hover_position()
                self._reset_tracking_counters()
                self._locked_target = None
                self.state = FlightState.WAIT

        # ============================
        # WAIT: 悬停, 检测到小车即自动追踪
        # ============================
        elif self.state == FlightState.WAIT:
            self.set_target_position(self._hover_x, self._hover_y, self.takeoff_height)

            target = self._get_target_ned()
            if target is not None:
                self._car_detection_count += 1
            else:
                self._car_detection_count = 0

            if self._car_detection_count >= self.confirm_frames:
                target = self._get_target_ned()
                if target is not None:
                    ned_x, ned_y, dist = target
                    self._locked_target = (ned_x, ned_y)
                    self.get_logger().info(
                        f"WAIT → TRACK (锁定小车 @ NED({ned_x:.2f}, {ned_y:.2f}), "
                        f"距离={dist:.2f}m, 确认={self._car_detection_count}帧)"
                    )
                    self._reset_tracking_counters()
                    self.state = FlightState.TRACK

        # ============================
        # TRACK: 追踪小车
        # ============================
        elif self.state == FlightState.TRACK:
            target = self._get_target_ned()

            if target is not None:
                ned_x, ned_y, dist = target
                self._locked_target = (ned_x, ned_y)

                if self._lost_start_time is not None:
                    self.get_logger().info("目标重新出现，继续追踪")
                    self._lost_start_time = None

                self.set_target_position(ned_x, ned_y, self.takeoff_height)

                # 抛投检测: 接近小车超过 dwell 时间 → DROP
                if dist < self.drop_distance_threshold:
                    if self._drop_start_time is None:
                        self._drop_start_time = self.get_clock().now()
                    else:
                        dwell = (self.get_clock().now() - self._drop_start_time).nanoseconds * 1e-9
                        if dwell >= self.drop_dwell_time:
                            self.get_logger().info(
                                f'TRACK → DROP (距小车 {dist:.2f}m, 停留 {dwell:.1f}s)')
                            self._drop_start_time = None
                            self._drop_enter_time = self.get_clock().now()
                            self.state = FlightState.DROP
                            return
                else:
                    self._drop_start_time = None

            else:
                now = self.get_clock().now()

                if self._lost_start_time is None:
                    self._lost_start_time = now

                elapsed = (now - self._lost_start_time).nanoseconds * 1e-9

                if elapsed >= self.lost_timeout:
                    self.get_logger().info(
                        f"TRACK → LOST (目标消失 {elapsed:.1f}s > {self.lost_timeout}s)"
                    )
                    self._reset_tracking_counters()
                    self._search_start_time = self.get_clock().now()
                    self.state = FlightState.LOST
                else:
                    if self._locked_target is not None:
                        lx, ly = self._locked_target
                        self.set_target_position(lx, ly, self.takeoff_height)

        # ============================
        # LOST: 最后位置搜索
        # ============================
        elif self.state == FlightState.LOST:
            if self._locked_target is not None:
                lx, ly = self._locked_target
                self.set_target_position(lx, ly, self.takeoff_height)

            target = self._get_target_ned()
            now = self.get_clock().now()

            if target is not None:
                self._car_detection_count += 1
                if self._car_detection_count >= self.confirm_frames:
                    ned_x, ned_y, dist = target
                    elapsed = (now - self._search_start_time).nanoseconds * 1e-9 if self._search_start_time else 0.0
                    self.get_logger().info(
                        f"LOST → TRACK (重新锁定 @ NED({ned_x:.2f}, {ned_y:.2f}), "
                        f"搜索耗时={elapsed:.1f}s)"
                    )
                    self._locked_target = (ned_x, ned_y)
                    self._reset_tracking_counters()
                    self.state = FlightState.TRACK
                    return
            else:
                self._car_detection_count = 0

            if self._search_start_time is not None:
                elapsed = (now - self._search_start_time).nanoseconds * 1e-9
                if elapsed >= self.search_timeout:
                    self.get_logger().info(
                        f"LOST → WAIT (搜索超时 {elapsed:.1f}s > {self.search_timeout}s)"
                    )
                    self._record_hover_position()
                    self._reset_tracking_counters()
                    self._locked_target = None
                    self.state = FlightState.WAIT

        # ============================
        # DROP: 抛投 — 降低至0.5m高度, 持续追踪小车, 5s后返航
        # ============================
        elif self.state == FlightState.DROP:
            # 计时
            if self._drop_enter_time is None:
                self._drop_enter_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self._drop_enter_time).nanoseconds * 1e-9

            # 超时 → 返航
            if elapsed >= self.rth_delay:
                self.get_logger().info(f'DROP → RTH (抛投完成 {elapsed:.1f}s)')
                # 发布抛投完成信号给小车
                msg = Int32()
                msg.data = 1
                self.drop_complete_pub.publish(msg)
                self._drop_enter_time = None
                self.state = FlightState.RTH
                return

            # TODO: 实际抛投逻辑 (舵机/电磁铁/释放装置等)
            self.get_logger().info(
                f'[DROP] 抛投 {elapsed:.1f}s/{self.rth_delay}s...',
                throttle_duration_sec=1.0)

            target = self._get_target_ned()
            if target is not None:
                ned_x, ned_y, _ = target
                self._locked_target = (ned_x, ned_y)
                self.set_target_position(ned_x, ned_y, self.drop_height)
            elif self._locked_target is not None:
                lx, ly = self._locked_target
                self.set_target_position(lx, ly, self.drop_height)

        # ============================
        # RTH: 返航 — 返回起飞点 (0,0)
        # ============================
        elif self.state == FlightState.RTH:
            self.get_logger().info(
                '[RTH] 返航中, 返回起飞点 (0,0)',
                throttle_duration_sec=2.0)

            self.set_target_position(0.0, 0.0, self.takeoff_height)


def main(args=None):
    rclpy.init(args=args)
    node = TFTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("用户终止节点.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
