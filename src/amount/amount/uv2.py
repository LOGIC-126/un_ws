"""
接收UDP检测结果，使用单目平面测量模块计算目标在相机坐标系下的位置，
同时读取并定期打印无人机欧拉角，平面距离硬编码为 0.55 米。
"""
import socket
import json
import threading
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude

# 导入单目测量模块（与代码1相同的路径）
from amount.MonocularPlaneMeasurer import MonocularPlaneMeasurer

# ============ 配置 ============
UDP_HOST = '127.0.0.1'
UDP_PORT = 8888
BUFFER_SIZE = 4096
PLANE_DISTANCE = 0.90          # 硬编码的相机到平面距离（米）
USE_UNDISTORT = True           # 是否进行畸变校正

CLASSES = ["elephant", "tiger", "wolf", "monkey", "peacock"]

# 相机内参（与代码1保持一致）
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


class MocMea_Node(Node):

    def __init__(self):
        super().__init__('monocular_measurement_node')

        # ---- 单目平面测量器 ----
        self.measurer = MonocularPlaneMeasurer(CAMERA_MATRIX, DIST_COEFFS)

        # ---- UDP ----
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)
        self.udp_running = True

        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        # ---- 姿态订阅 ----
        self.drone_attitude_quat = None   # 存储顺序: (x, y, z, w)
        self.attitude_lock = threading.Lock()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile
        )

        # 1 Hz 打印欧拉角定时器
        self.euler_timer = self.create_timer(1.0, self._print_euler_callback)

        self.get_logger().info(f'UDP监听启动: {UDP_HOST}:{UDP_PORT}')
        self.get_logger().info(f'平面距离硬编码: {PLANE_DISTANCE} m, 畸变校正: {USE_UNDISTORT}')
        self.get_logger().info('姿态订阅已启动，1Hz 欧拉角打印已就绪')

    # -------------------- 姿态回调 --------------------
    def _attitude_callback(self, msg):
        """
        接收 VehicleAttitude，内部存储为 (x, y, z, w)
        """
        w, x, y, z = msg.q
        with self.attitude_lock:
            self.drone_attitude_quat = (x, y, z, w)

    # -------------------- 定时打印欧拉角 --------------------
    def _print_euler_callback(self):
        """每秒打印一次当前欧拉角（roll, pitch, yaw）"""
        with self.attitude_lock:
            quat = self.drone_attitude_quat
        if quat is None:
            return

        roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(
            f"[姿态]  roll = {roll_deg:6.2f}°, pitch = {pitch_deg:6.2f}°, yaw = {yaw_deg:6.2f}°"
        )

    # -------------------- UDP 接收与处理 --------------------
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

        # 获取当前姿态（欧拉角）
        with self.attitude_lock:
            quat = self.drone_attitude_quat
        if quat is not None:
            roll_deg, pitch_deg, _ = quaternion_to_euler(quat)
        else:
            roll_deg, pitch_deg = 0.0, 0.0

        for det in detections:
            class_id = det.get('class_id', -1)
            if 0 <= class_id < len(CLASSES):
                class_name = CLASSES[class_id]
            else:
                class_name = f"unk({class_id})"

            # 中心像素坐标
            if 'center' in det:
                cx, cy = det['center']
            else:
                x1, y1, x2, y2 = det.get('bbox', (0, 0, 0, 0))
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            # ---------- 使用单目平面测量模块计算相机坐标系下的世界坐标 ----------
            try:
                X_cam, Y_cam = self.measurer.pixel_to_world_with_decoupling(
                    cx, cy,
                    plane_distance=PLANE_DISTANCE,
                    roll_deg=roll_deg,
                    pitch_deg=pitch_deg,
                    use_undistort=USE_UNDISTORT
                )
                coord_str = f"(X={X_cam:+.3f}, Y={Y_cam:+.3f}) m  (相机坐标系: X右 Y下)"
            except Exception as e:
                self.get_logger().error(f'坐标转换失败: {e}')
                coord_str = "计算失败"

            self.get_logger().info(
                f'{class_name} 像素({cx:.1f}, {cy:.1f}) → 相机坐标 {coord_str}'
            )

    # -------------------- 关闭 --------------------
    def stop(self):
        self.udp_running = False
        if self.udp_socket:
            self.udp_socket.close()
        self.udp_thread.join(timeout=2.0)
        self.get_logger().info('节点停止')


def main():
    rclpy.init()
    node = MocMea_Node()
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