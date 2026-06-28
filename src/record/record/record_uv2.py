#!/usr/bin/env python3
"""
数据记录节点：订阅 uv2_ros 的相机坐标和世界坐标，以及无人机位置和姿态，
合并写入单个 CSV 文件。

CSV 列:
  timestamp_sec, timestamp_nanosec,
  drone_x, drone_y, drone_z,
  roll_deg, pitch_deg, yaw_deg,
  class_name,
  cam_x, cam_y, cam_z,
  world_x, world_y, world_z,
  raw_x, raw_y
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from std_msgs.msg import String

import json
import math
import csv
import time
import threading


def quaternion_to_euler(q):
    x, y, z, w = q
    roll_rad  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw_rad   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll_rad), math.degrees(pitch_rad), math.degrees(yaw_rad)


def timestamp_split(unix_float):
    sec = int(unix_float)
    nsec = int((unix_float - sec) * 1e9)
    return sec, nsec


class Uv2Recorder(Node):
    def __init__(self):
        super().__init__('uv2_recorder')

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"record_uv2_{timestamp_str}.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nanosec',
            'drone_x', 'drone_y', 'drone_z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'class_name',
            'cam_x', 'cam_y', 'cam_z',
            'world_x', 'world_y', 'world_z',
            'raw_x', 'raw_y'
        ])

        self.csv_lock = threading.Lock()

        # ---- 数据缓冲区 ----
        self.buf_lock = threading.Lock()
        self.drone_position = None       # (x, y, z)
        self.drone_attitude = None       # (roll_deg, pitch_deg, yaw_deg)
        self.latest_cam = None           # [{class, camera_relative: [x,y,z]}]
        self.latest_raw = None           # [{class, world_absolute: [x,y,z]}] 原始

        self.get_logger().info(f"记录文件: {self.csv_filename}")

        # ---- QoS ----
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅无人机位置 ----
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            px4_qos
        )

        # ---- 订阅无人机姿态 ----
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            px4_qos
        )

        # ---- 订阅相机坐标 ----
        self.cam_sub = self.create_subscription(
            String,
            '/detection/camera_coordinates',
            self._cam_callback,
            10
        )

        # ---- 订阅原始世界坐标 ----
        self.raw_sub = self.create_subscription(
            String,
            '/detection/world_coordinates_raw',
            self._raw_callback,
            10
        )

        # ---- 订阅滤波后世界坐标 (触发写入) ----
        self.world_sub = self.create_subscription(
            String,
            '/detection/world_coordinates',
            self._world_callback,
            10
        )

        self.get_logger().info("uv2 记录节点启动，等待数据...")

    # ==================== 回调 ====================
    def _position_callback(self, msg):
        with self.buf_lock:
            self.drone_position = (msg.x, msg.y, msg.z)

    def _attitude_callback(self, msg):
        w, x, y, z = msg.q
        roll, pitch, yaw = quaternion_to_euler((x, y, z, w))
        with self.buf_lock:
            self.drone_attitude = (roll, pitch, yaw)

    def _raw_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            data = [data]
        with self.buf_lock:
            self.latest_raw = data

    def _cam_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            data = [data]
        with self.buf_lock:
            self.latest_cam = data

    def _world_callback(self, msg):
        """收到滤波后世界坐标时，触发一次写入（附带所有缓冲数据）"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            data = [data]

        with self.buf_lock:
            pos = self.drone_position
            att = self.drone_attitude
            cam_list = self.latest_cam
            raw_list = self.latest_raw
            world_list = data

        if pos is None or att is None:
            self.get_logger().warn(
                "收到检测数据但无人机状态未就绪",
                throttle_duration_sec=3.0
            )
            return

        # 按 class 匹配相机坐标和原始世界坐标
        cam_by_class = {}
        if cam_list:
            for c in cam_list:
                cam_by_class[c.get('class', '')] = c.get('camera_relative', [0, 0, 0])

        raw_by_class = {}
        if raw_list:
            for r in raw_list:
                raw_by_class[r.get('class', '')] = r.get('world_absolute', [0, 0])[:2]

        now = time.time()
        sec, nsec = timestamp_split(now)

        with self.csv_lock:
            for w in world_list:
                cls = w.get('class', 'unknown')
                world = w.get('world_absolute', [0, 0, 0])
                cam = cam_by_class.get(cls, [0, 0, 0])
                raw = raw_by_class.get(cls, [world[0], world[1]])  # 没拿到原始就用滤波值

                self.csv_writer.writerow([
                    sec, nsec,
                    pos[0], pos[1], pos[2],
                    att[0], att[1], att[2],
                    cls,
                    cam[0], cam[1], cam[2],
                    world[0], world[1], world[2],
                    raw[0], raw[1]
                ])
            self.csv_file.flush()

    def destroy_node(self):
        with self.csv_lock:
            self.csv_file.close()
        self.get_logger().info(f"文件已关闭: {self.csv_filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Uv2Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
