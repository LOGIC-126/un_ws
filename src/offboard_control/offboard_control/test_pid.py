#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR (PX4 直连版)
@说明: 正方形路径测试 — 状态机通过 /uav/target_position 发布目标位姿，
       由 offboard_control 桥接 PX4，不再依赖 MAVROS。
       任务: A(0,0) B(1,0) C(1,1) D(0,1)
             路径: A->B->C->D->A -> A->C->D->B->A -> 降落
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
    ENABLE_AUTO_TAKEOFF = True   # True: 自动起飞; False: 手动
    TAKE_HEIGHT = -1.2           # 起飞高度 (NED, m)

    # 正方形顶点 (边长1m)
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

        # ----- 发布器：解耦方式，发布 Pose 到 /uav/target_position -----
        self.target_position_pub = self.create_publisher(
            Pose, '/uav/target_position', qos_profile)

        # ----- 订阅 PX4 状态与位置 (直连，不走 MAVROS) -----
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)

        # ----- 内部变量 -----
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        self.flight_state = FlightState.INIT
        self.current_mission_index = 0
        self.wait_start_time = None

        self.mission_points = []

        # 目标缓存 (NED)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # 初始发送一次目标位姿
        if self.ENABLE_AUTO_TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)

        # 主循环 (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        mode_str = "自主起飞" if self.ENABLE_AUTO_TAKEOFF else "手动解锁 (等待外部指令)"
        self.get_logger().info(f"PX4 直连测试节点已启动 ({mode_str})")

    # --- PX4 回调 ---
    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    # --- 控制发布 ---
    def publish_target_position(self):
        msg = Pose()
        msg.position.x = float(self.target_x)
        msg.position.y = float(self.target_y)
        msg.position.z = float(self.target_z)
        msg.orientation.w = math.cos(self.target_yaw / 2.0)
        msg.orientation.z = math.sin(self.target_yaw / 2.0)
        self.target_position_pub.publish(msg)

    def set_target_position(self, x, y, z, yaw=0.0):
        fx, fy, fz, fyaw = float(x), float(y), float(z), float(yaw)
        if (fx == self.target_x and fy == self.target_y and
            fz == self.target_z and fyaw == self.target_yaw):
            return
        self.target_x = fx
        self.target_y = fy
        self.target_z = fz
        self.target_yaw = fyaw
        self.publish_target_position()

    def check_distance(self, x, y, z, threshold=0.15):
        pos = self.vehicle_local_position
        dist = math.sqrt((pos.x - x)**2 + (pos.y - y)**2 + (pos.z - z)**2)
        return dist < threshold

    # --- 航点生成 ---
    def generate_mission_points(self):
        z = self.TAKE_HEIGHT
        p1 = [self.A, self.B, self.C, self.D, self.A]
        p2 = [self.A, self.C, self.D, self.B, self.A]

        def interpolate_path(path):
            """在每两个相邻航点之间插入中点"""
            result = []
            for i in range(len(path)):
                x, y = path[i]
                result.append((x, y, z, "pass"))
                # 在当前点和下一个点之间插入中点（最后一个点之后不插入）
                if i < len(path) - 1:
                    nx, ny = path[i + 1]
                    mx, my = (x + nx) / 2.0, (y + ny) / 2.0
                    result.append((mx, my, z, "pass"))
            return result

        points = []
        points.extend(interpolate_path(p1))
        points.extend(interpolate_path(p2))
        self.get_logger().info(f"生成 {len(points)} 个航点")
        return points

    def execute_mission(self, mission_type: str) -> bool:
        # 本任务只有 "pass"
        if mission_type == "pass":
            return True
        if mission_type == "wait":
            return self.Wait()
        return True

    def Wait(self, duration=5.0):
        now = self.get_clock().now()
        if self.wait_start_time is None:
            self.wait_start_time = now
            return False
        if (now - self.wait_start_time).nanoseconds * 1e-9 < duration:
            return False
        self.wait_start_time = None
        return True

    # --- 主状态机 ---
    def timer_callback(self):
        # 安全防护：必须已经解锁且处于 Offboard 模式
        is_armed = (self.vehicle_status.arming_state ==
                    VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state ==
                       VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.flight_state not in [FlightState.LAND, FlightState.DONE]:
            if not is_armed or not is_offboard:
                if self.flight_state != FlightState.INIT:
                    self.get_logger().warn("掉出 Offboard 或上锁，退回 INIT")
                    self.flight_state = FlightState.INIT
                return

        self.run_state_machine()

    def run_state_machine(self):
        is_armed = (self.vehicle_status.arming_state ==
                    VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state ==
                       VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.flight_state == FlightState.INIT:
            if not self.mission_points:
                self.mission_points = self.generate_mission_points()
            # 始终发送目标位置 (心跳)
            if self.ENABLE_AUTO_TAKEOFF:
                self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            else:
                self.set_target_position(0.0, 0.0, 0.0)
            # 等解锁+Offboard 后自动进入 TAKEOFF
            if is_armed and is_offboard:
                self.get_logger().info("已解锁且处于 Offboard，开始任务")
                self.flight_state = FlightState.TAKEOFF

        elif self.flight_state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            if self.check_distance(0.0, 0.0, self.TAKE_HEIGHT):
                self.get_logger().info("起飞完成")
                self.current_mission_index = 0
                self.flight_state = FlightState.GOTOTAR

        elif self.flight_state == FlightState.GOTOTAR:
            if self.current_mission_index < len(self.mission_points):
                tx, ty, tz, _ = self.mission_points[self.current_mission_index]
                self.set_target_position(tx, ty, tz)
                if self.check_distance(tx, ty, tz):
                    self.get_logger().info(f"到达 {self.current_mission_index+1}/{len(self.mission_points)}")
                    self.flight_state = FlightState.MISSION

        elif self.flight_state == FlightState.MISSION:
            if self.current_mission_index < len(self.mission_points):
                tx, ty, tz, mtype = self.mission_points[self.current_mission_index]
                self.set_target_position(tx, ty, tz)
                if self.execute_mission(mtype):
                    self.get_logger().info(f"航点 {self.current_mission_index+1} 完成")
                    if self.current_mission_index >= len(self.mission_points) - 1:
                        self.get_logger().info("全部完成，开始降落")
                        self.flight_state = FlightState.LAND
                    else:
                        self.current_mission_index += 1
                        self.flight_state = FlightState.GOTOTAR

        elif self.flight_state == FlightState.LAND:
            self.set_target_position(0.0, 0.0, 0.0)
            if self.vehicle_local_position.z >= -0.15:
                self.get_logger().info("已着陆，任务结束")
                self.flight_state = FlightState.DONE

        elif self.flight_state == FlightState.DONE:
            self.get_logger().info("任务完成，关闭节点")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestMissionNode()
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
