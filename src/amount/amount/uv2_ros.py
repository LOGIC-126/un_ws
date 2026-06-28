"""
UDP接收检测结果 → 计算相机坐标和世界坐标 → 发布话题
基于 uv2.py + MP.py 的运算逻辑，新增话题发布功能。

发布话题:
  /detection/camera_coordinates  — 相机坐标系 (X左正, Y前正, Z=PLANE_DISTANCE)
  /detection/world_coordinates   — NED 世界坐标

数据流:
  rknn_yolo_detect.py --UDP--> 像素中心坐标
  ROS2 /fmu/out/vehicle_local_position_v1 --> 无人机世界坐标
  ROS2 /fmu/out/vehicle_attitude          --> 无人机姿态四元数
"""

import socket
import json
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, DistanceSensor
from std_msgs.msg import String

from amount.MonocularPlaneMeasurer import MonocularPlaneMeasurer


# ============ 配置 ============
UDP_HOST = '127.0.0.1'
UDP_PORT = 8888
BUFFER_SIZE = 4096
LASER_OFFSET = 0.06            # 激光雷达安装偏移 (米), 读数值减此偏移
FALLBACK_DISTANCE = 1.0        # 激光雷达未就绪时的备用高度 (米)
USE_UNDISTORT = True

CLASSES = ["elephant", "tiger", "wolf", "monkey", "peacock"]

CAMERA_MATRIX = np.array([
    [306.94344552, 0.0, 327.01283787],
    [0.0, 305.86926008, 236.35170675],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.array([0.07910935, -0.14167218, -0.0030752, -0.00107756, 0.05718679])


def quaternion_to_euler(q):
    x, y, z, w = q
    roll_rad  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw_rad   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll_rad), math.degrees(pitch_rad), yaw_rad


class Uv2PubNode(Node):

    def __init__(self):
        super().__init__('uv2_pub_node')

        self.measurer = MonocularPlaneMeasurer(CAMERA_MATRIX, DIST_COEFFS)

        # ---- 状态锁 ----
        self.state_lock = threading.Lock()
        self.drone_position = None           # (x, y, z) NED
        self.drone_attitude_quat = None      # (x, y, z, w)
        self.plane_distance = FALLBACK_DISTANCE  # 激光雷达实时高度
        self.receive_count = 0

        # ---- QoS ----
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅 ----
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            qos_profile
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile
        )
        self.distance_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self._distance_callback,
            qos_profile
        )

        # ---- 发布话题 ----
        # 相机坐标系坐标 (不含无人机位置)
        self.cam_pub = self.create_publisher(
            String, '/detection/camera_coordinates', 10
        )
        # NED 世界坐标 (原始)
        self.world_raw_pub = self.create_publisher(
            String, '/detection/world_coordinates_raw', 10
        )

        # 1 Hz 打印欧拉角
        self.euler_timer = self.create_timer(1.0, self._print_euler_callback)

        # ---- UDP ----
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)
        self.udp_running = True
        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(f'UDP监听: {UDP_HOST}:{UDP_PORT}, 激光雷达偏移={LASER_OFFSET}m')
        self.get_logger().info('发布: /detection/camera_coordinates (相机坐标)')
        self.get_logger().info('发布: /detection/world_coordinates_raw (世界坐标)')

    # ==================== 回调 ====================
    def _position_callback(self, msg):
        with self.state_lock:
            self.drone_position = (msg.x, msg.y, msg.z)

    def _attitude_callback(self, msg):
        w, x, y, z = msg.q
        with self.state_lock:
            self.drone_attitude_quat = (x, y, z, w)

    def _distance_callback(self, msg):
        """激光雷达竖直朝下, current_distance 减去偏移 = 相机到地面距离"""
        dist = msg.current_distance - LASER_OFFSET
        with self.state_lock:
            self.plane_distance = max(dist, 0.1)

    def _print_euler_callback(self):
        with self.state_lock:
            quat = self.drone_attitude_quat
        if quat is None:
            return
        roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(
            f"[姿态]  roll = {roll_deg:6.2f}°, pitch = {pitch_deg:6.2f}°, yaw = {yaw_deg:6.2f}°"
        )

    # ==================== UDP 接收与处理 ====================
    def _udp_loop(self):
        try:
            self.udp_socket.bind((UDP_HOST, UDP_PORT))
        except OSError as e:
            self.get_logger().error(f'UDP绑定失败: {e}')
            return

        while self.udp_running:
            try:
                data, _ = self.udp_socket.recvfrom(BUFFER_SIZE)
                msg = json.loads(data.decode('utf-8'))
                self._process_detections(msg)
                self.receive_count += 1
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                self.get_logger().debug('收到无效JSON')
            except OSError as e:
                if self.udp_running:
                    self.get_logger().error(f'UDP接收错误: {e}')
                break

    def _process_detections(self, data):
        detections = data.get('detections', [])
        if not detections:
            return

        # 获取位姿和高度
        with self.state_lock:
            pos = self.drone_position
            quat = self.drone_attitude_quat
            z = self.plane_distance

        if quat is not None:
            roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        else:
            roll_deg, pitch_deg, yaw_rad = 0.0, 0.0, 0.0

        cam_results = []
        raw_results = []

        for det in detections:
            class_id = det.get('class_id', -1)
            class_name = CLASSES[class_id] if 0 <= class_id < len(CLASSES) else f"unk({class_id})"

            # 中心像素坐标
            if 'center' in det:
                cx, cy = det['center']
            else:
                x1, y1, x2, y2 = det.get('bbox', (0, 0, 0, 0))
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            # ---- 相机坐标 (MP.py 翻转符号: X左正, Y前正) ----
            try:
                X_cam, Y_cam = self.measurer.pixel_to_world_with_decoupling(
                    cx, cy,
                    plane_distance=z,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    use_undistort=USE_UNDISTORT
                )
            except Exception as e:
                self.get_logger().error(f'像素→相机坐标失败: {e}')
                continue

            # ---- 相机坐标 ----
            cam_entry = {
                'class': class_name,
                'camera_relative': [round(X_cam, 4), round(Y_cam, 4), round(z, 4)]
            }
            cam_results.append(cam_entry)

            # ---- 世界坐标 (原始，不滤波) ----
            if pos is not None:
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                raw_x = pos[0] + Y_cam * cos_y + X_cam * sin_y
                raw_y = pos[1] + Y_cam * sin_y - X_cam * cos_y

                raw_entry = {
                    'class': class_name,
                    'world_absolute': [round(raw_x, 4), round(raw_y, 4), 0.0]
                }
                raw_results.append(raw_entry)

        # 发布
        if cam_results:
            self.cam_pub.publish(String(data=json.dumps(cam_results, ensure_ascii=False)))
        if raw_results:
            self.world_raw_pub.publish(String(data=json.dumps(raw_results, ensure_ascii=False)))

    # ==================== 关闭 ====================
    def stop(self):
        self.udp_running = False
        if self.udp_socket:
            self.udp_socket.close()
        self.udp_thread.join(timeout=2.0)
        self.get_logger().info('节点停止')


def main():
    rclpy.init()
    node = Uv2PubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
