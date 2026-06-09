"""
@说明: UDP接收检测目标中心像素坐标 + ROS2接收无人机位姿 → 计算目标世界坐标

数据流:
  rknn_yolo_detect.py --UDP--> 像素中心坐标
  ROS2 /fmu/out/vehicle_local_position_v1 --> 无人机世界坐标(位置)
  ROS2 /imu                              --> 无人机四元数(姿态)

处理流程:
  1. 像素(u,v) ──[MonocularPlaneMeasurer]──→ 相机坐标系3D点(X_cam, Y_cam, Z)
  2. 相机坐标系 ──[CAM_TO_BODY]──→ 机体坐标系
  3. 机体坐标系 ──[四元数旋转]──→ 世界坐标系(以相机为原点)
  4. 相对世界坐标 + 无人机世界坐标 = 目标绝对世界坐标
  5. 去重、计数、存储

模块分工:
  MonocularPlaneMeasurer: 纯数学模块，只负责像素坐标 → 相机坐标系3D点
  zhongzhuan.py:         所有ROS2通信、UDP通信、坐标变换、去重计数
"""

import socket
import json
import time
import math
import numpy as np
import threading

from amount.MonocularPlaneMeasurer import MonocularPlaneMeasurer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude ,
)
from sensor_msgs.msg import Imu


# ============ 类别映射 (与 rknn_yolo_detect.py 保持一致) ============
CLASSES = ["elephant", "tiger", "wolf", "monkey", "peacock"]

# ============ UDP 配置 ============
UDP_HOST = '127.0.0.1'
UDP_PORT = 8888

# ============ 测量配置 ============
CAMERA_OFFSET_Z = 0.075   # 相机到无人机中心的垂直偏移 (米)

# ============ MonocularPlaneMeasurer 功能开关 ============
USE_UNDISTORT = True              # 是否对像素点进行畸变校正
USE_ROTATE_DECOUPLING = True      # 是否进行像素级旋转解耦 (补偿roll/pitch)

# ============ 目标去重配置 ============
DEDUP_THRESHOLD = 0.5     # 世界坐标距离阈值（米），小于此距离视为同一目标

# ============ 相机内参 (传给 MonocularPlaneMeasurer) ============
CAMERA_MATRIX = np.array([
    [306.94344552, 0.0, 327.01283787],
    [0.0, 305.86926008, 236.35170675],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.array([0.07910935, -0.14167218, -0.0030752, -0.00107756, 0.05718679])

# ============ 相机 → 机体 坐标变换矩阵 ============
# 相机坐标系 (OpenCV标准):  X→右,  Y→下(图像),  Z→前(镜头朝向)
# 机体坐标系 (NED):         X→前,  Y→右,      Z→下
#
# 典型下视相机安装 (镜头朝下，图像上方=机头方向):
#   机体X(前) = -相机Y (图像下方 = 机体后方, 取反即前方)
#   机体Y(右) =  相机X (图像右方 = 机体右方)
#   机体Z(下) =  相机Z (镜头朝下)
#
# P_body = CAM_TO_BODY @ P_cam
# 如果你的相机安装方向不同，请修改此矩阵
CAM_TO_BODY = np.array([
    [ 0, -1,  0],   # 机体X = -相机Y
    [ 1,  0,  0],   # 机体Y =  相机X
    [ 0,  0,  1]    # 机体Z =  相机Z
])


def quaternion_to_rotation_matrix(q):
    """
    将四元数 (x, y, z, w) 转换为旋转矩阵 (body → world)
    采用 Hamilton 约定，与 ROS geometry_msgs/Quaternion 一致
    """
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    R = np.array([
        [1 - 2*(yy + zz),  2*(xy - wz),      2*(xz + wy)],
        [2*(xy + wz),      1 - 2*(xx + zz),  2*(yz - wx)],
        [2*(xz - wy),      2*(yz + wx),      1 - 2*(xx + yy)]
    ])
    return R


def quaternion_to_euler(q):
    """
    从四元数 (x, y, z, w) 提取欧拉角
    返回: (roll_deg, pitch_deg, yaw_rad)
      roll:  横滚角 (度), 右倾为正
      pitch: 俯仰角 (度), 前倾为负
      yaw:   航向角 (弧度), (-π, π)
    """
    x, y, z, w = q
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll_rad = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch_rad = math.asin(max(-1.0, min(1.0, sinp)))

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll_rad), math.degrees(pitch_rad), yaw_rad


class DetectionWorldConverter(Node):
    """
    ROS2节点: 接收检测结果和无人机位姿，计算目标绝对世界坐标并存储

    外置模块:
      MonocularPlaneMeasurer — 仅负责像素坐标 → 相机坐标系3D点转换
    """

    def __init__(self):
        super().__init__('detection_world_converter')

        # ---- 外置模块: 像素 → 相机坐标 ----
        self.measurer = MonocularPlaneMeasurer(CAMERA_MATRIX, DIST_COEFFS)

        # ---- 无人机状态 (线程安全) ----
        self.drone_position = None      # (x, y, z)  世界坐标
        self.drone_quaternion = None    # (x, y, z, w) 四元数
        self.state_lock = threading.Lock()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- ROS2 订阅者 ----
        # 话题1: 无人机世界坐标位置
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            qos_profile
        )

        # 话题2: 无人机四元数姿态
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._quaternion_callback,
            qos_profile
        )

        # ---- UDP 接收 ----
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)
        self.udp_running = True

        self.latest_data = None
        self.data_lock = threading.Lock()

        # ---- 结果存储 ----
        self.target_world_positions = []  # 最新一批目标绝对世界坐标
        self.results_lock = threading.Lock()

        # ---- 目标数量库 ----
        self.class_counts = {c: 0 for c in CLASSES}       # 各类别已发现的目标数量
        self.class_positions = {c: [] for c in CLASSES}    # 各类别已记录的目标世界坐标列表
        self.counts_lock = threading.Lock()

        # 统计
        self.receive_count = 0

        # ROS2 定时器: 100Hz 处理UDP数据
        self.process_timer = self.create_timer(0.01, self._process_callback)

        self.get_logger().info('检测世界坐标转换节点已启动')
        self.get_logger().info(f'UDP监听: {UDP_HOST}:{UDP_PORT}')
        self.get_logger().info('订阅话题: /fmu/out/vehicle_local_position_v1 (位置), /imu (四元数)')

    # ==================== ROS2 回调 ====================

    def _position_callback(self, msg):
        """接收无人机世界坐标并储存"""
        with self.state_lock:
            self.drone_position = (
                msg.x,
                msg.y,
                msg.z
            )

    def _quaternion_callback(self, msg):
        """接收无人机四元数 (VehicleAttitude) 并储存"""
        with self.state_lock:
            # VehicleAttitude 消息的 q 字段为 [w, x, y, z]
            w, x, y, z = msg.q
            self.drone_quaternion = (x, y, z, w)  # 存储为 (x, y, z, w)

    # ==================== 坐标转换 ====================

    def _pixel_to_world_relative(self, u, v):
        """
        将像素坐标转换为以相机为原点的世界坐标 (米)

        流程:
          1. [Monocular] 畸变校正 — 修正镜头光学畸变
          2. [Monocular] 旋转解耦 — 像素层面补偿 roll/pitch 倾斜
          3. [Monocular] 像素 → 相机坐标系3D点
          4. CAM_TO_BODY 矩阵 → 机体坐标系
          5. 四元数旋转 (仅 yaw, 或完整旋转) → 世界坐标系

        返回: np.array([wx, wy, wz]) 或 None (四元数未就绪时)
        """
        # 获取当前无人机状态
        with self.state_lock:
            quat = self.drone_quaternion
            pos = self.drone_position
        if quat is None or pos is None:
            return None

        # 动态平面距离: 无人机离地高度 - 相机偏移
        plane_dist = max(-pos[2] - CAMERA_OFFSET_Z, 0.1)

        # 提取欧拉角 (旋转解耦需要)
        if USE_ROTATE_DECOUPLING:
            roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)

        # ---- Step 1: 畸变校正 (MonocularPlaneMeasurer) ----
        if USE_UNDISTORT:
            (u, v), = self.measurer.undistort_points([(u, v)])

        # ---- Step 2: 旋转解耦 (MonocularPlaneMeasurer) ----
        if USE_ROTATE_DECOUPLING:
            (u, v), = self.measurer.rotate_decoupling(
                [(u, v)], roll_deg, pitch_deg
            )

        # ---- Step 3: 像素 → 相机坐标系3D点 (MonocularPlaneMeasurer) ----
        X_cam, Y_cam = self.measurer.pixel_to_world_xy(u, v, plane_dist)
        Z_cam = plane_dist
        P_cam = np.array([X_cam, Y_cam, Z_cam])

        # ---- Step 4: 相机 → 机体 ----
        P_body = CAM_TO_BODY @ P_cam

        # ---- Step 5: 机体 → 世界 (旋转) ----
        if USE_ROTATE_DECOUPLING:
            # roll/pitch 已在像素层处理，3D 层只做 yaw 旋转
            cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
            R_yaw = np.array([
                [cos_y, -sin_y, 0],
                [sin_y,  cos_y, 0],
                [0,      0,     1]
            ])
            P_world_rel = R_yaw @ P_body
        else:
            # 完整四元数旋转
            R_body_to_world = quaternion_to_rotation_matrix(quat)
            P_world_rel = R_body_to_world @ P_body

        return P_world_rel

    # ==================== 目标去重与计数 ====================

    def _check_and_add(self, class_name, position):
        """
        检查目标是否为新目标 (与已记录位置的距离 > DEDUP_THRESHOLD)。
        如果是新目标则加入库中并增加计数。

        返回: True=新目标, False=已存在
        """
        with self.counts_lock:
            existing = self.class_positions.get(class_name, [])
            for rec in existing:
                dist = np.linalg.norm(np.array(rec) - position)
                if dist < DEDUP_THRESHOLD:
                    return False  # 已存在相近目标

            # 新目标，入库
            pos_tuple = tuple(map(float, position))
            self.class_positions[class_name].append(pos_tuple)
            self.class_counts[class_name] = self.class_counts.get(class_name, 0) + 1
            return True

    def get_class_counts(self):
        """获取各类别目标数量 (线程安全)"""
        with self.counts_lock:
            return dict(self.class_counts)

    # ==================== 主处理循环 ====================

    def _process_callback(self):
        """定时器回调: 获取最新UDP数据并计算目标世界坐标"""
        with self.data_lock:
            data = self.latest_data
            self.latest_data = None  # 取出后清空，避免重复处理

        if data is None:
            return

        # 检查无人机状态是否就绪
        with self.state_lock:
            pos = self.drone_position
            quat = self.drone_quaternion

        if pos is None:
            self.get_logger().warn(
                '尚未收到无人机位置坐标',
                throttle_duration_sec=5.0
            )
            return
        if quat is None:
            self.get_logger().warn(
                '尚未收到无人机四元数',
                throttle_duration_sec=5.0
            )
            return

        detections = data.get('detections', [])
        if not detections:
            return

        world_results = []
        for det in detections:
            class_id = det.get('class_id', -1)
            class_name = CLASSES[class_id] if 0 <= class_id < len(CLASSES) else f"unknown({class_id})"

            # 读取中心像素坐标 (rknn_yolo_detect.py 发送 center 字段)
            center = det.get('center', None)
            if center is None:
                bbox = det.get('bbox', [0, 0, 0, 0])
                cu = (bbox[0] + bbox[2]) / 2.0
                cv = (bbox[1] + bbox[3]) / 2.0
            else:
                cu, cv = center[0], center[1]

            # 像素 → 相对世界坐标 (以相机为原点)
            P_world_rel = self._pixel_to_world_relative(cu, cv)
            if P_world_rel is None:
                continue

            # 加上无人机世界坐标 = 目标绝对世界坐标
            P_target_world = np.array(pos) + P_world_rel

            # ---- 去重: 检查是否与已记录目标位置相近 ----
            is_new = self._check_and_add(class_name, P_target_world)

            world_results.append({
                'class': class_name,
                'world_absolute': tuple(map(float, P_target_world)),
                'is_new': is_new
            })

        # 存储最终结果
        with self.results_lock:
            self.target_world_positions = world_results

        # 打印结果
        self._print_results(world_results)

    def _print_results(self, results):
        """打印检测结果到控制台"""
        if not results:
            return

        with self.counts_lock:
            counts = dict(self.class_counts)

        print(f"\n[{time.strftime('%H:%M:%S')}] 检测到 {len(results)} 个目标")
        print(f"{'#':>3} {'类别':>10} {'绝对世界坐标 (m)':>32} {'新目标':>6} {'累计数量':>6}")
        print("-" * 65)

        for i, r in enumerate(results):
            w = r['world_absolute']
            cls = r['class']
            flag = '✓' if r['is_new'] else ''
            print(f"{i+1:>3} {cls:>10}  ({w[0]:>+8.3f}, {w[1]:>+8.3f}, {w[2]:>+8.3f})  {flag:>6}  {counts.get(cls, 0):>6}")

        print("-" * 65)

    # ==================== 公开接口 ====================

    def get_target_world_positions(self):
        """获取最新一批目标绝对世界坐标 (线程安全)"""
        with self.results_lock:
            return list(self.target_world_positions)

    def get_drone_state(self):
        """获取当前存储的无人机状态 (线程安全)"""
        with self.state_lock:
            return {
                'position': self.drone_position,
                'quaternion': self.drone_quaternion
            }

    # ==================== UDP 线程 ====================

    def start_udp(self):
        """启动UDP接收线程"""
        try:
            self.udp_socket.bind((UDP_HOST, UDP_PORT))
            self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.udp_thread.start()
            self.get_logger().info(f'UDP接收已启动: {UDP_HOST}:{UDP_PORT}')
        except Exception as e:
            self.get_logger().error(f'UDP绑定失败: {e}')
            raise

    def _udp_loop(self):
        """UDP接收循环 (后台线程)"""
        while self.udp_running:
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                detection_data = json.loads(data.decode('utf-8'))
                with self.data_lock:
                    self.latest_data = detection_data
                self.receive_count += 1
            except socket.timeout:
                continue
            except Exception as e:
                if self.udp_running:
                    self.get_logger().error(f'UDP接收错误: {e}')
                break

    # ==================== 生命周期 ====================

    def stop(self):
        """停止节点并清理资源"""
        self.udp_running = False
        if self.udp_socket:
            self.udp_socket.close()
        self.get_logger().info('节点已停止')


def main():
    rclpy.init()

    converter = DetectionWorldConverter()
    converter.start_udp()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        converter.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
