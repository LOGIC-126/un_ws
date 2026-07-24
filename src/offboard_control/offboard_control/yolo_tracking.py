#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 目标追踪节点

状态机: INIT → TAKEOFF → WAIT ⇄ TRACK ⇄ LOST
  - aux1 (通道9) 上升沿: 触发起飞
  - aux1 HIGH: 允许追踪 (WAIT→TRACK 自动)
  - aux1 LOW:  禁用追踪 (TRACK/LOST→WAIT)

控制链路:
  订阅 /detection/world_coordinates (PoseArray, map帧)
  发布 /uav/target_position (Pose, NED) → offboard_control.py → PX4

摄像头朝下安装, 固定追踪高度, 不控制偏航。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, ManualControlSetpoint
from geometry_msgs.msg import Pose, PoseArray


# ====== 状态枚举 ======
class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    WAIT = 2
    TRACK = 3
    LOST = 4


class YoloTrackingNode(Node):
    """YOLO 目标追踪节点: 状态机驱动, 追踪围栏内最近目标"""

    def __init__(self):
        super().__init__('yolo_tracking_node')

        # ====== 参数声明 (可通过命令行 --ros-args -p 覆盖) ======
        self.declare_parameter('takeoff_height', -1.2)
        self.declare_parameter('arrival_threshold', 0.2)
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('fence_radius', 1.5)
        self.declare_parameter('lost_timeout', 1.5)
        self.declare_parameter('search_timeout', 10.0)
        self.declare_parameter('rc_trigger_aux', 'aux1')
        self.declare_parameter('rc_trigger_threshold', 0.5)

        # 读取参数值
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.fence_radius = self.get_parameter('fence_radius').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_timeout = self.get_parameter('search_timeout').value

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

        # ====== 订阅器 ======
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos_profile)
        self.manual_control_sub = self.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint',
            self.manual_control_callback, qos_profile)
        self.world_coords_sub = self.create_subscription(
            PoseArray, '/detection/world_coordinates',
            self.world_coordinates_callback, 10)

        # ====== 状态变量 ======
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        self.state = FlightState.INIT

        # RC 持续监控 (非一次性锁存)
        self._last_rc_raw = False
        self._rc_level = False           # 当前电平
        self._rc_rising_edge = False     # 粘性上升沿
        self._rc_falling_edge = False    # 粘性下降沿

        # 检测数据缓存
        self._latest_detections = None   # PoseArray or None

        # 目标确认计数器
        self._consecutive_detections = 0
        self._consecutive_misses = 0

        # 锁定目标 (NED: x=北, y=东)
        self._locked_target = None       # (ned_x, ned_y) or None

        # 丢失计时
        self._lost_start_time = None     # rclpy Time or None
        self._search_start_time = None   # rclpy Time or None

        # 悬停位置 (进入WAIT时记录)
        self._hover_x = 0.0
        self._hover_y = 0.0

        # 目标缓存 (用于发布)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # ====== 定时器 (20Hz) ======
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"YOLO追踪节点已启动 | 起飞高度={self.takeoff_height}m | "
            f"围栏={self.fence_radius}m | 确认帧={self.confirm_frames} | "
            f"丢失超时={self.lost_timeout}s | 搜索超时={self.search_timeout}s"
        )

    # ==================== 订阅回调 ====================

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def manual_control_callback(self, msg: ManualControlSetpoint) -> None:
        """持续监控 aux1: 上升沿/下降沿 + 当前电平"""
        if not msg.valid:
            return
        aux_name = self.get_parameter('rc_trigger_aux').value
        val = getattr(msg, aux_name, 0.0)
        current_raw = (val > self.get_parameter('rc_trigger_threshold').value)

        if current_raw and not self._last_rc_raw:
            self._rc_rising_edge = True
            self.get_logger().info(f"RC 上升沿 ({aux_name}={val:.2f})")
        if not current_raw and self._last_rc_raw:
            self._rc_falling_edge = True
            self.get_logger().info(f"RC 下降沿 ({aux_name}={val:.2f})")

        self._last_rc_raw = current_raw
        self._rc_level = current_raw

    def world_coordinates_callback(self, msg: PoseArray) -> None:
        """缓存最新检测结果"""
        self._latest_detections = msg

    # ==================== 坐标转换 ====================

    @staticmethod
    def _world_pose_to_ned(pose: Pose):
        """
        将 detection_world_node.py 发布的 map 帧坐标转为 NED。
        detection_world_node.py 计算:
          msg.x = world_north       (NED 北分量)
          msg.y = -world_east       (NED 东分量取负)
        逆变换:
          ned_north = msg.x
          ned_east  = -msg.y
        """
        return (pose.position.x, -pose.position.y)

    # ==================== 目标选择 ====================

    def _select_nearest_target(self):
        """
        从最新检测帧中选出围栏内最近目标。
        返回 (ned_x, ned_y, distance) 或 None。
        仅使用 2D 水平距离 (摄像头朝下, 目标在地面)。
        """
        if self._latest_detections is None:
            return None
        poses = self._latest_detections.poses
        if not poses:
            return None

        drone = self.vehicle_local_position
        # NaN 守卫
        if math.isnan(drone.x) or math.isnan(drone.y):
            return None

        best = None
        best_dist = float('inf')

        for pose in poses:
            ned_x, ned_y = self._world_pose_to_ned(pose)
            dx = ned_x - drone.x
            dy = ned_y - drone.y
            dist = math.hypot(dx, dy)

            if dist < self.fence_radius and dist < best_dist:
                best_dist = dist
                best = (ned_x, ned_y, dist)

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
        """仅变更时发布, 避免冗余"""
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
        self._consecutive_detections = 0
        self._consecutive_misses = 0
        self._lost_start_time = None
        self._search_start_time = None

    def _record_hover_position(self) -> None:
        drone = self.vehicle_local_position
        if not math.isnan(drone.x) and not math.isnan(drone.y):
            self._hover_x = drone.x
            self._hover_y = drone.y

    # ==================== 主循环 ====================

    def timer_callback(self) -> None:
        # 安全防护: 检测 offboard/armed 丢失
        is_armed = (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.state != FlightState.INIT:
            if not is_armed or not is_offboard:
                self.get_logger().warn("掉出 Offboard / 上锁状态, 退回 INIT.")
                self.state = FlightState.INIT
                self._reset_tracking_counters()
                self._locked_target = None
                self._rc_rising_edge = False
                self._rc_falling_edge = False
                return

        self.run_state_machine()

        # 消费粘性边沿标志 (每帧清除)
        self._rc_rising_edge = False
        self._rc_falling_edge = False

    def run_state_machine(self) -> None:
        # ============================
        # INIT: 地面等待 aux1 上升沿
        # ============================
        if self.state == FlightState.INIT:
            self.set_target_position(0.0, 0.0, 0.0)

            if self._rc_rising_edge:
                self.get_logger().info("INIT → TAKEOFF (aux1 ↑)")
                self.set_target_position(0.0, 0.0, self.takeoff_height)
                self.state = FlightState.TAKEOFF
            else:
                self.get_logger().info(
                    "等待 RC aux1 上升沿触发...",
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
        # WAIT: 悬停, 等待目标 + aux1=HIGH
        # ============================
        elif self.state == FlightState.WAIT:
            # 在悬停位置保持高度
            self.set_target_position(self._hover_x, self._hover_y, self.takeoff_height)

            # 统计连续检测帧
            target = self._select_nearest_target()
            if target is not None:
                self._consecutive_detections += 1
            else:
                self._consecutive_detections = 0

            # aux1=HIGH 且 连续确认N帧 → TRACK
            if self._rc_level and self._consecutive_detections >= self.confirm_frames:
                target = self._select_nearest_target()
                if target is not None:
                    ned_x, ned_y, dist = target
                    self._locked_target = (ned_x, ned_y)
                    self.get_logger().info(
                        f"WAIT → TRACK (锁定目标 @ NED({ned_x:.2f}, {ned_y:.2f}), "
                        f"距离={dist:.2f}m, 确认={self._consecutive_detections}帧)"
                    )
                    self._reset_tracking_counters()
                    self.state = FlightState.TRACK

            # aux1↓ 无影响 (已在 WAIT)

        # ============================
        # TRACK: 追踪锁定目标
        # ============================
        elif self.state == FlightState.TRACK:
            # aux1↓ → 立即回 WAIT
            if self._rc_falling_edge:
                self.get_logger().info("TRACK → WAIT (aux1 ↓)")
                self._record_hover_position()
                self._reset_tracking_counters()
                self._locked_target = None
                self.state = FlightState.WAIT
                return

            target = self._select_nearest_target()

            if target is not None:
                ned_x, ned_y, dist = target
                self._locked_target = (ned_x, ned_y)
                self._consecutive_misses = 0

                if self._lost_start_time is not None:
                    self.get_logger().info("目标重新出现，继续追踪")
                    self._lost_start_time = None

                self.set_target_position(ned_x, ned_y, self.takeoff_height)

            else:
                # 目标消失
                self._consecutive_misses += 1
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
                    # 短暂丢失: 保持最后已知位置
                    if self._locked_target is not None:
                        lx, ly = self._locked_target
                        self.set_target_position(lx, ly, self.takeoff_height)

        # ============================
        # LOST: 最后位置搜索
        # ============================
        elif self.state == FlightState.LOST:
            # aux1↓ → WAIT
            if self._rc_falling_edge:
                self.get_logger().info("LOST → WAIT (aux1 ↓)")
                self._record_hover_position()
                self._reset_tracking_counters()
                self._locked_target = None
                self.state = FlightState.WAIT
                return

            # 保持在最后已知位置
            if self._locked_target is not None:
                lx, ly = self._locked_target
                self.set_target_position(lx, ly, self.takeoff_height)

            # 尝试重新锁定
            target = self._select_nearest_target()
            now = self.get_clock().now()

            if target is not None:
                self._consecutive_detections += 1
                if self._consecutive_detections >= self.confirm_frames:
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
                self._consecutive_detections = 0

            # 搜索超时 → WAIT
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


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackingNode()
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
