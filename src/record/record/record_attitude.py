#!/usr/bin/env python3
"""
数据记录节点（适配可视化脚本格式）：
  - 订阅 /fmu/out/vehicle_attitude（飞控姿态）
  - 订阅 /detection_world_coordinates（检测目标世界坐标，JSON）
  - 姿态 CSV 列：timestamp_sec, timestamp_nanosec, roll_deg, pitch_deg, yaw_deg
  - 检测 CSV 列：timestamp_sec, timestamp_nanosec, class_name, world_x, world_y, world_z, is_new
  - is_new 以字符串 'true'/'false' 存储
  - 文件命名规则与你的可视化脚本自动匹配
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude
from std_msgs.msg import String

import json
import math
import csv
import time
import threading


def quaternion_to_euler(q):
    """
    四元数 → 欧拉角 (度)
    参数 q: (x, y, z, w)
    返回: (roll_deg, pitch_deg, yaw_deg)
    """
    x, y, z, w = q
    roll_rad  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw_rad   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)


def timestamp_split(unix_float):
    """将 Unix 浮点秒拆分为整数秒和纳秒"""
    sec = int(unix_float)
    nsec = int((unix_float - sec) * 1e9)
    return sec, nsec


class DataLoggerNode(Node):
    def __init__(self):
        super().__init__('data_logger')

        # 使用启动时间戳命名文件
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")

        # ---- 姿态 CSV 文件 ----
        self.attitude_filename = f"record_attitude_{timestamp_str}.csv"
        self.attitude_file = open(self.attitude_filename, mode='w', newline='', encoding='utf-8')
        self.attitude_writer = csv.writer(self.attitude_file)
        self.attitude_writer.writerow([
            'timestamp_sec', 'timestamp_nanosec',
            'roll_deg', 'pitch_deg', 'yaw_deg'
        ])

        # ---- 检测目标 CSV 文件 ----
        self.detection_filename = f"record_world_coords_{timestamp_str}.csv"
        self.detection_file = open(self.detection_filename, mode='w', newline='', encoding='utf-8')
        self.detection_writer = csv.writer(self.detection_file)
        self.detection_writer.writerow([
            'timestamp_sec', 'timestamp_nanosec',
            'class_name', 'world_x', 'world_y', 'world_z', 'is_new'
        ])

        self.csv_lock = threading.Lock()

        self.get_logger().info(f"姿态记录文件: {self.attitude_filename}")
        self.get_logger().info(f"检测记录文件: {self.detection_filename}")

        # ---- QoS 配置（与 PX4 消息兼容） ----
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅 ----
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile
        )
        self.detection_sub = self.create_subscription(
            String,
            '/detection_world_coordinates',
            self._detection_callback,
            10
        )

        self.get_logger().info("数据记录节点启动，等待数据...")

    # ============ 回调函数 ============
    def _attitude_callback(self, msg):
        """接收姿态消息，写入姿态 CSV"""
        # VehicleAttitude.q 顺序为 [w, x, y, z]
        w, x, y, z = msg.q
        roll, pitch, yaw = quaternion_to_euler((x, y, z, w))

        now = time.time()
        sec, nsec = timestamp_split(now)

        with self.csv_lock:
            self.attitude_writer.writerow([
                sec, nsec,
                f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}"
            ])
            self.attitude_file.flush()

    def _detection_callback(self, msg):
        """接收检测结果，写入检测 CSV"""
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not isinstance(detections, list):
            return

        now = time.time()
        sec, nsec = timestamp_split(now)

        with self.csv_lock:
            for det in detections:
                cls = det.get('class', 'unknown')
                world = det.get('world_absolute', [0.0, 0.0, 0.0])
                is_new = det.get('is_new', False)
                is_new_str = 'true' if is_new else 'false'

                self.detection_writer.writerow([
                    sec, nsec,
                    cls,
                    f"{world[0]:.4f}", f"{world[1]:.4f}", f"{world[2]:.4f}",
                    is_new_str
                ])
            self.detection_file.flush()

    def destroy_node(self):
        """关闭文件并销毁节点"""
        with self.csv_lock:
            self.attitude_file.close()
            self.detection_file.close()
        self.get_logger().info(f"文件已关闭: {self.attitude_filename}, {self.detection_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()