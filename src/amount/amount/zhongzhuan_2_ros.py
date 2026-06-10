"""
@说明: UDP + ROS2 接收 → MonocularPlaneMeasurer 全权处理坐标转换
数据流:
  rknn_yolo_detect.py --UDP--> 像素中心坐标
  ROS2 /fmu/out/vehicle_local_position_v1 --> 无人机世界坐标(位置)
  ROS2 /fmu/out/vehicle_attitude          --> 无人机姿态四元数

处理流程:
  1. 像素(u,v) ──[pixel_to_world_with_decoupling]──→ (X, Y) 机体相对坐标
                     内部: 畸变校正 → 旋转解耦(roll/pitch) → 小孔成像
  2. (X,Y) ──[get_world_position]──→ (W_X, W_Y) 绝对世界坐标
                     内部: yaw 2D旋转 → + 无人机位置
  3. 去重、计数、存储
  4. 通过 /detection_world_coordinates 话题发布世界坐标 (新增)
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
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from std_msgs.msg import String       # 新增：用于发布世界坐标


# ============ 类别映射 ============
CLASSES = ["elephant", "tiger", "wolf", "monkey", "peacock"]

# ============ UDP 配置 ============
UDP_HOST = '127.0.0.1'
UDP_PORT = 8888

# ============ 测量配置 ============
CAMERA_OFFSET_Z = 0.075   # 相机到无人机中心的垂直偏移 (米)
USE_UNDISTORT = True      # 是否畸变校正

# ============ 目标去重配置 ============
DEDUP_THRESHOLD = 0.5     # 世界坐标距离阈值（米）

# ============ 相机内参 ============
CAMERA_MATRIX = np.array([
    [306.94344552, 0.0, 327.01283787],
    [0.0, 305.86926008, 236.35170675],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.array([0.07910935, -0.14167218, -0.0030752, -0.00107756, 0.05718679])


def quaternion_to_euler(q):
    """
    从四元数提取欧拉角
    参数 q: (x, y, z, w) 顺序
    返回: (roll_deg, pitch_deg, yaw_rad)
    """
    x, y, z, w = q
    roll_rad  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw_rad   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll_rad), math.degrees(pitch_rad), yaw_rad


class DetectionWorldConverter(Node):
    """
    ROS2节点: 接收检测结果和无人机位姿，计算目标绝对世界坐标并存储，
    并通过话题发布世界坐标。
    """

    def __init__(self):
        super().__init__('detection_world_converter')

        self.measurer = MonocularPlaneMeasurer(CAMERA_MATRIX, DIST_COEFFS)

        self._last_print_time = 0.0
        self._logger = self.get_logger()

        # ---- 无人机状态 ----
        self.drone_position = None          # (x, y, z) NED
        self.drone_attitude_quat = None     # (x, y, z, w) 四元数
        self.state_lock = threading.Lock()

        # ---- QoS 配置 (与PX4发布者匹配) ----
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅位置话题 ----
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            qos_profile
        )

        # ---- 订阅姿态话题 ----
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile
        )

        # ---- 发布世界坐标话题 (新增) ----
        self.world_pub = self.create_publisher(
            String,
            '/detection_world_coordinates',
            10
        )

        # ---- UDP 接收 ----
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)
        self.udp_running = True
        self.latest_data = None
        self.data_lock = threading.Lock()

        # ---- 结果存储 ----
        self.target_world_positions = []
        self.results_lock = threading.Lock()

        # ---- 目标数量库 ----
        self.class_counts = {c: 0 for c in CLASSES}
        self.class_positions = {c: [] for c in CLASSES}
        self.counts_lock = threading.Lock()

        self.receive_count = 0

        # ROS2 定时器: 100Hz 处理UDP数据
        self.process_timer = self.create_timer(0.01, self._process_callback)
        # 1Hz 打印欧拉角
        self.euler_print_timer = self.create_timer(1.0, self._print_euler_callback)

        self.get_logger().info('检测世界坐标转换节点已启动 (使用飞控姿态话题 /fmu/out/vehicle_attitude)')
        self.get_logger().info(f'UDP监听: {UDP_HOST}:{UDP_PORT}')
        self.get_logger().info('发布世界坐标话题: /detection_world_coordinates')

    # ==================== ROS2 回调 ====================

    def _position_callback(self, msg):
        with self.state_lock:
            self.drone_position = (msg.x, msg.y, msg.z)

    def _attitude_callback(self, msg):
        """
        接收 VehicleAttitude 消息，其 q 数组为 [w, x, y, z]。
        转换为内部存储顺序 (x, y, z, w)
        """
        w, x, y, z = msg.q
        with self.state_lock:
            self.drone_attitude_quat = (x, y, z, w)

    # ==================== 打印当前欧拉角 ====================
    def _print_euler_callback(self):
        """每秒打印一次当前的欧拉角（roll, pitch, yaw）"""
        with self.state_lock:
            quat = self.drone_attitude_quat
        if quat is None:
            return

        roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(
            f"[姿态]  roll = {roll_deg:6.2f}°, pitch = {pitch_deg:6.2f}°, yaw = {yaw_deg:6.2f}°"
        )

    # ==================== 坐标转换 ====================
    def _pixel_to_world_absolute(self, u, v):
        """
        像素坐标 → 绝对世界坐标 (全部委托 MonocularPlaneMeasurer)
        """
        with self.state_lock:
            pos = self.drone_position
            quat = self.drone_attitude_quat
        if pos is None or quat is None:
            return None

        # 提取欧拉角
        roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)

        # 平面距离：无人机离地高度 - 相机偏移
        plane_distance = max(-pos[2] - CAMERA_OFFSET_Z, 0.1)

        # Step 1: 畸变校正 + 旋转解耦 + 像素→机体相对坐标 XY
        X, Y = self.measurer.pixel_to_world_with_decoupling(
            u, v,
            plane_distance,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            use_undistort=USE_UNDISTORT
        )

        # Step 2: yaw 旋转 + 叠加无人机位置 → 世界坐标
        W_X, W_Y = self.measurer.get_world_position(
            pos[0], pos[1],  # 无人机世界坐标 XY
            yaw_rad,
            X, Y
        )

        # Z: 目标在地面, NED 原点(起飞点)=0, 平坦地面假设下 Z≈0
        return W_X, W_Y, 0.0

    # ==================== 目标去重与计数 ====================
    def _check_and_add(self, class_name, position):
        with self.counts_lock:
            existing = self.class_positions.get(class_name, [])
            for rec in existing:
                # 2D 平面距离去重 (忽略 Z, 避免高度变化干扰)
                if np.linalg.norm(np.array(rec[:2]) - position[:2]) < DEDUP_THRESHOLD:
                    return False
            self.class_positions[class_name].append(tuple(map(float, position)))
            self.class_counts[class_name] = self.class_counts.get(class_name, 0) + 1
            return True

    def get_class_counts(self):
        with self.counts_lock:
            return dict(self.class_counts)

    # ==================== 主处理循环 ====================
    def _process_callback(self):
        with self.data_lock:
            data = self.latest_data
            self.latest_data = None

        if data is None:
            return

        detections = data.get('detections', [])
        if not detections:
            return

        world_results = []
        for det in detections:
            class_id = det.get('class_id', -1)
            class_name = CLASSES[class_id] if 0 <= class_id < len(CLASSES) else f"unknown({class_id})"

            center = det.get('center', None)
            if center is None:
                bbox = det.get('bbox', [0, 0, 0, 0])
                cu = (bbox[0] + bbox[2]) / 2.0
                cv = (bbox[1] + bbox[3]) / 2.0
            else:
                cu, cv = center[0], center[1]

            result = self._pixel_to_world_absolute(cu, cv)
            if result is None:
                continue

            W_X, W_Y, W_Z = result
            P_target_world = np.array([W_X, W_Y, W_Z])

            is_new = self._check_and_add(class_name, P_target_world)

            world_results.append({
                'class': class_name,
                'world_absolute': tuple(map(float, P_target_world)),
                'is_new': is_new
            })

        with self.results_lock:
            self.target_world_positions = world_results

        # ---- 发布世界坐标话题 (新增) ----
        if world_results:
            json_str = json.dumps(world_results, ensure_ascii=False)
            msg = String(data=json_str)
            self.world_pub.publish(msg)

        self._print_results(world_results)

    def _print_results(self, results):
        if not results:
            return

        current_time = time.time()
        if current_time - self._last_print_time < 0.5:
            return
        self._last_print_time = current_time

        with self.counts_lock:
            counts = dict(self.class_counts)

        lines = []
        lines.append(f"检测到 {len(results)} 个目标")
        lines.append(f"{'#':>3} {'类别':>10} {'绝对世界坐标 (m)':>32} {'新目标':>6} {'累计数量':>6}")
        lines.append("-" * 65)

        for i, r in enumerate(results):
            w = r['world_absolute']
            cls = r['class']
            flag = '✓' if r['is_new'] else ''
            lines.append(
                f"{i+1:>3} {cls:>10}  "
                f"({w[0]:>+8.3f}, {w[1]:>+8.3f}, {w[2]:>+8.3f})  "
                f"{flag:>6}  {counts.get(cls, 0):>6}"
            )

        lines.append("-" * 65)
        self._logger.info("\n".join(lines))

    # ==================== 公开接口 ====================
    def get_target_world_positions(self):
        with self.results_lock:
            return list(self.target_world_positions)

    def get_drone_state(self):
        with self.state_lock:
            return {
                'position': self.drone_position,
                'attitude_quat': self.drone_attitude_quat
            }

    # ==================== UDP 线程 ====================
    def start_udp(self):
        try:
            self.udp_socket.bind((UDP_HOST, UDP_PORT))
            self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self.udp_thread.start()
            self.get_logger().info(f'UDP接收已启动: {UDP_HOST}:{UDP_PORT}')
        except Exception as e:
            self.get_logger().error(f'UDP绑定失败: {e}')
            raise

    def _udp_loop(self):
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

    def stop(self):
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