#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR (测试任务改编)
@说明: 飞行任务控制 — 简单状态机测试 (正方形路径)
       控制链路: 发布 Pose 到 /uav/target_position
       任务: 正方形 ABCD (边长1m)，A(0,0), B(1,0), C(1,1), D(0,1)
             路径: A->B->C->D->A  →  A->C->D->B->A  → 降落
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import math

# PX4 反馈 (仅状态读取)
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

# 通用控制消息
from geometry_msgs.msg import Pose


class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    GOTOTAR = 2
    MISSION = 3
    LAND = 4
    DONE = 5


class SimpleTestMissionNode(Node):
    # ====== 硬编码安全开关 ======
    ENABLE_AUTO_TAKEOFF = True

    # ====== 任务参数 ======
    TAKE_HEIGHT = -1.2          # 起飞/任务高度 (NED, m)
    # 正方形 ABCD 边长 1m，坐标定义 (NED: x→北, y→东)
    A = (0.0, 0.0)
    B = (1.0, 0.0)
    C = (1.0, 1.0)
    D = (0.0, 1.0)

    def __init__(self):
        super().__init__('simple_test_mission_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----- 发布器 (解耦) -----
        self.target_position_pub = self.create_publisher(
            Pose, '/uav/target_position', qos_profile)

        # ----- 订阅器 (仅反馈) -----
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.vehicle_status_callback, qos_profile)

        # ----- 内部变量 -----
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        self.state = FlightState.INIT
        self.current_mission_index = 0
        self.wait_start_time = None

        # 任务点列表 (x, y, z, mission_type) — 由 generate_mission_points 生成
        self.mission_points = []

        # 目标缓存 (NED)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # 根据开关一次性发布初始目标位姿
        if self.ENABLE_AUTO_TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)

        self.timer = self.create_timer(0.05, self.timer_callback)

        mode_str = "自主起飞" if self.ENABLE_AUTO_TAKEOFF else "手动解锁 (等待外部指令)"
        self.get_logger().info(
            f"简单测试任务节点已启动 (控制解耦模式: {mode_str})."
        )

    # --- 1. 订阅回调 ---
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    # --- 2. 控制发布 ---
    def publish_target_position(self) -> None:
        msg = Pose()
        msg.position.x = float(self.target_x)
        msg.position.y = float(self.target_y)
        msg.position.z = float(self.target_z)
        msg.orientation.w = math.cos(self.target_yaw / 2.0)
        msg.orientation.z = math.sin(self.target_yaw / 2.0)
        self.target_position_pub.publish(msg)

    # --- 3. 状态机辅助 ---
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

    def check_distance(self, x: float, y: float, z: float, threshold: float = 0.15) -> bool:
        pos = self.vehicle_local_position
        dist = math.sqrt((pos.x - x)**2 + (pos.y - y)**2 + (pos.z - z)**2)
        return dist < threshold

    # --- 4. 任务点生成 (测试用) ---
    def generate_mission_points(self) -> list:
        """
        按题目要求生成航点序列：
        第一圈: A -> B -> C -> D -> A
        第二段: A -> C -> D -> B -> A (最后降落由状态机 LAND 完成)
        所有航点保持任务高度，类型设为 "pass" 直接通过。
        """
        z = self.TAKE_HEIGHT
        # 第一圈
        p1 = [self.A, self.B, self.C, self.D, self.A]
        # 第二段 (注意第一个 A 会紧接着上一个 A，飞机短暂悬停后继续)
        p2 = [self.A, self.C, self.D, self.B, self.A]

        points = []
        for (x, y) in p1:
            points.append((x, y, z, "pass"))
        for (x, y) in p2:
            points.append((x, y, z, "pass"))

        self.get_logger().info(
            f"测试路径已生成，共 {len(points)} 个航点 (高度 {z} m)."
        )
        return points

    # --- 5. 任务执行 (所有类型均为 pass) ---
    def execute_mission(self, mission_type: str) -> bool:
        if mission_type == "pass":
            return True
        # 保留其他类型兼容，但本测试不会用到
        if mission_type == "scan":
            return True
        if mission_type == "back":
            return True
        if mission_type == "wait":
            return self.Wait()
        return True

    def Wait(self, duration: float = 5.0) -> bool:
        now = self.get_clock().now()
        if self.wait_start_time is None:
            self.wait_start_time = now
            self.get_logger().info(f"悬停等待 {duration} 秒...")
            return False
        elapsed = (now - self.wait_start_time).nanoseconds / 1e9
        if elapsed < duration:
            return False
        self.wait_start_time = None
        return True

    # --- 6. 主循环 (状态机) ---
    def timer_callback(self) -> None:
        # 安全防护
        is_armed = (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.state not in [FlightState.LAND, FlightState.DONE]:
            if not is_armed or not is_offboard:
                if self.state != FlightState.INIT:
                    self.get_logger().warn("掉出 Offboard / 上锁状态，退回待命.")
                    self.state = FlightState.INIT
                    self.wait_start_time = None
                return

        self.run_state_machine()

    def run_state_machine(self) -> None:
        if self.state == FlightState.INIT:
            if not self.mission_points:
                self.mission_points = self.generate_mission_points()

            if self.ENABLE_AUTO_TAKEOFF:
                self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            else:
                self.set_target_position(0.0, 0.0, 0.0)
                self.get_logger().info(
                    "自主起飞已禁用，保持 z=0 待命。请手动解锁并切 OFFBOARD...",
                    throttle_duration_sec=3.0,
                )
                return

            self.get_logger().info(
                f"测试路径已生成，共 {len(self.mission_points)} 个航点。状态: INIT -> TAKEOFF"
            )
            self.state = FlightState.TAKEOFF

        elif self.state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            if self.check_distance(0.0, 0.0, self.TAKE_HEIGHT):
                self.get_logger().info("起飞完成，开始航点遍历.")
                self.current_mission_index = 0
                self.state = FlightState.GOTOTAR

        elif self.state == FlightState.GOTOTAR:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, _ = point
                self.set_target_position(target_x, target_y, target_z)

                if self.check_distance(target_x, target_y, target_z):
                    self.get_logger().info(
                        f"到达航点 {self.current_mission_index + 1}/{len(self.mission_points)}"
                    )
                    self.state = FlightState.MISSION

        elif self.state == FlightState.MISSION:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, mission_type = point
                self.set_target_position(target_x, target_y, target_z)

                if self.execute_mission(mission_type):
                    self.get_logger().info(
                        f"航点 {self.current_mission_index + 1} [{mission_type}] 完成"
                    )
                    if self.current_mission_index >= len(self.mission_points) - 1:
                        self.get_logger().info("全部航点遍历完毕，准备降落.")
                        self.state = FlightState.LAND
                    else:
                        self.current_mission_index += 1
                        self.state = FlightState.GOTOTAR

        elif self.state == FlightState.LAND:
            self.set_target_position(0.0, 0.0, 0.0)
            if self.vehicle_local_position.z >= -0.15:
                self.get_logger().info("已着陆，任务结束.")
                self.state = FlightState.DONE

        elif self.state == FlightState.DONE:
            self.get_logger().info("简单测试任务完成，关闭节点.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("用户强行终止节点.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()