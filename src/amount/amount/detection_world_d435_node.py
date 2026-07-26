"""
D435 深度世界坐标节点 — 对标 detection_world_node.py

订阅 D435 深度话题替代激光雷达距离传感器，每像素取深 → 反投影到三维空间。

数据流:
  realsense-ros driver (align_depth:=true)
    ├─ /camera/color/camera_info      → 提取内参
    ├─ /camera/depth/image_rect_raw   → 深度图 (mm)
  yolo_detector_node
    └─ /detections                     → Detection2DArray
  PX4
    ├─ /fmu/out/vehicle_local_position_v1 → 无人机 NED 位置
    └─ /fmu/out/vehicle_attitude          → 姿态四元数
        │
        ▼
  StereoDepthMeasurer.pixel_to_camera_3d(cx, cy) → (X, Y, Z)
        │
        ▼
  发布话题 (与 detection_world_node 同格式):
    /detection/camera_coordinates  — camera_link (X左正, Y前正, Z=深度值)
    /detection/world_coordinates   — map ENU (x东, y北, class_id|z, score|ow)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

from amount.StereoDepthMeasurer import StereoDepthMeasurer


# ============ 辅助函数（与 detection_world_node.py 相同） ============

def quaternion_to_euler(q):
    """四元数 → 欧拉角 (roll_deg, pitch_deg, yaw_rad)"""
    x, y, z, w = q
    roll_rad  = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch_rad = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw_rad   = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll_rad), math.degrees(pitch_rad), yaw_rad


# ============ 节点 ============

class DetectionWorldD435Node(Node):

    def __init__(self):
        super().__init__('detection_world_d435_node')

        self.bridge = CvBridge()
        self.measurer = None            # StereoDepthMeasurer, CameraInfo 到达后初始化
        self._info_received = False     # 标记已收到 CameraInfo

        # ---- 状态锁 ----
        self.state_lock = threading.Lock()
        self.drone_position = None          # (x, y, z) NED
        self.drone_attitude_quat = None     # (x, y, z, w)

        # ---- QoS (PX4 话题使用 BEST_EFFORT) ----
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- 订阅 D435 ----
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self._camera_info_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self._depth_callback,
            qos_profile
        )

        # ---- 订阅 YOLO 检测 ----
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detections',
            self._detection_callback,
            10
        )

        # ---- 订阅 PX4 状态 ----
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

        # ---- 发布话题 ----
        self.cam_pub = self.create_publisher(
            PoseArray, '/detection/camera_coordinates', 10
        )
        self.world_pub = self.create_publisher(
            PoseArray, '/detection/world_coordinates', 10
        )

        # 1 Hz 打印欧拉角
        self.euler_timer = self.create_timer(1.0, self._print_euler_callback)

        self.get_logger().info('D435 深度世界坐标节点启动')
        self.get_logger().info('等待 /camera/color/camera_info …')

    # ==================== 相机回调 ====================

    def _camera_info_callback(self, msg: CameraInfo):
        """首次收到 CameraInfo 时初始化 StereoDepthMeasurer，然后取消订阅"""
        try:
            new_measurer = StereoDepthMeasurer.from_camera_info(msg)
            with self.state_lock:
                # 保留可能已到达的深度图
                if self.measurer is not None and self.measurer.has_depth:
                    new_measurer.set_depth_image(self.measurer._depth_image)
                self.measurer = new_measurer

            self._info_received = True
            self.get_logger().info(
                f'相机内参已加载: fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, '
                f'cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}'
            )
        except Exception as e:
            self.get_logger().error(f'相机内参解析失败: {e}')
            return

        # 内参是静态的，首次成功后取消订阅
        self.destroy_subscription(self.camera_info_sub)

    def _depth_callback(self, msg: Image):
        """存入最新对齐深度图 (uint16 mm)"""
        try:
            # realsense-ros 发布 16UC1 编码，passthrough 保留原始类型
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            with self.state_lock:
                if self.measurer is not None:
                    self.measurer.set_depth_image(depth_img)
                # 若 measurer 尚未就绪 (CameraInfo 还未到达), 静默丢弃
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')

    # ==================== PX4 回调 ====================

    def _position_callback(self, msg):
        with self.state_lock:
            self.drone_position = (msg.x, msg.y, msg.z)

    def _attitude_callback(self, msg):
        w, x, y, z = msg.q
        with self.state_lock:
            self.drone_attitude_quat = (x, y, z, w)

    def _print_euler_callback(self):
        with self.state_lock:
            quat = self.drone_attitude_quat
        if quat is None:
            return
        roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        yaw_deg = math.degrees(yaw_rad)
        self.get_logger().info(
            f'[姿态]  roll = {roll_deg:6.2f}°, pitch = {pitch_deg:6.2f}°, yaw = {yaw_deg:6.2f}°'
        )

    # ==================== 检测回调 ====================

    def _detection_callback(self, msg: Detection2DArray):
        """接收 YOLO 检测结果，深度反投影，发布相机/世界坐标"""
        detections = msg.detections
        if not detections:
            return

        # ---- 获取共享状态 ----
        with self.state_lock:
            measurer = self.measurer
            pos = self.drone_position
            quat = self.drone_attitude_quat

        if measurer is None:
            self.get_logger().warning(
                '深度测量器未就绪（等待 /camera/color/camera_info）', throttle_duration_sec=5.0
            )
            return

        if quat is not None:
            roll_deg, pitch_deg, yaw_rad = quaternion_to_euler(quat)
        else:
            roll_deg, pitch_deg, yaw_rad = 0.0, 0.0, 0.0

        # ---- 构建 PoseArray ----
        stamp = self.get_clock().now().to_msg()

        cam_poses = PoseArray()
        cam_poses.header.stamp = stamp
        cam_poses.header.frame_id = 'camera_link'

        world_poses = PoseArray()
        world_poses.header.stamp = stamp
        world_poses.header.frame_id = 'map'

        for det in detections:
            # (1) 提取 bbox 中心像素坐标
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y

            # 提取类别和置信度
            class_id = ''
            score = 0.0
            if det.results:
                class_id = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score

            # (2) 深度反投影 —— 核心步骤
            try:
                X_cam, Y_cam, Z_cam = measurer.pixel_to_camera_3d(cx, cy)
            except Exception as e:
                self.get_logger().error(f'深度反投影失败: {e}')
                continue

            # (3) Z <= 0 → 无效深度，跳过
            if Z_cam <= 0.0:
                continue

            # (4) 构建相机坐标系 Pose
            #     与 detection_world_node.py 相同的编码:
            #     position.x = Y (前), position.y = -X (右), position.z = Z (深度)
            cam_pose = Pose()
            cam_pose.position.x = Y_cam
            cam_pose.position.y = -X_cam
            cam_pose.position.z = Z_cam
            cam_poses.poses.append(cam_pose)

            # (5) 世界坐标 (ENU)
            if pos is not None:
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)

                world_north = pos[0] - cam_pose.position.y * sin_y + cam_pose.position.x * cos_y
                world_east  = pos[1] + cam_pose.position.y * cos_y + cam_pose.position.x * sin_y

                cls_id = int(class_id) if class_id.isdigit() else -1

                world_pose = Pose()
                world_pose.position.x = world_north      # ENU x = 东
                world_pose.position.y = -world_east      # ENU y = 北
                world_pose.position.z = float(cls_id)    # 类别号
                world_pose.orientation.w = score         # 置信度
                world_poses.poses.append(world_pose)

        # ---- 发布 ----
        if cam_poses.poses:
            self.cam_pub.publish(cam_poses)
        if world_poses.poses:
            self.world_pub.publish(world_poses)


def main():
    rclpy.init()
    node = DetectionWorldD435Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()