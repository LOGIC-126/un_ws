"""
订阅 yolo_detector_node 发布的 Detection2DArray → 计算相机坐标和世界坐标 → 发布话题

与 uv2_ros.py 的区别：本节点不通过 UDP 接收检测结果，而是直接订阅
yolo_detector_node.cpp 发布的 vision_msgs/Detection2DArray 话题。

发布话题:
  /detection/camera_coordinates  — 相机坐标系 (X左正, Y前正, Z=PLANE_DISTANCE)
  /detection/world_coordinates   — NED 世界坐标

数据流:
  yolo_detector_node --ROS2 /detections--> Detection2DArray
  ROS2 /fmu/out/vehicle_local_position_v1 --> 无人机世界坐标
  ROS2 /fmu/out/vehicle_attitude          --> 无人机姿态四元数
"""

import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, DistanceSensor
from geometry_msgs.msg import PoseArray, Pose
from vision_msgs.msg import Detection2DArray

from amount.MonocularPlaneMeasurer import MonocularPlaneMeasurer


# ============ 配置 ============
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


class DetectionWorldNode(Node):

    def __init__(self):
        super().__init__('detection_world_node')

        self.measurer = MonocularPlaneMeasurer(CAMERA_MATRIX, DIST_COEFFS)

        # ---- 状态锁 ----
        self.state_lock = threading.Lock()
        self.drone_position = None           # (x, y, z) NED
        self.drone_attitude_quat = None      # (x, y, z, w)
        self.plane_distance = FALLBACK_DISTANCE  # 激光雷达实时高度

        # ---- QoS ----
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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
        self.distance_sub = self.create_subscription(
            DistanceSensor,
            '/fmu/out/distance_sensor',
            self._distance_callback,
            qos_profile
        )

        # ---- 订阅 YOLO 检测结果 (替代 UDP) ----
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detections',
            self._detection_callback,
            10
        )

        # ---- 发布话题 ----
        # 相机坐标系坐标 (不含无人机位置), frame_id: camera_optical_frame
        self.cam_pub = self.create_publisher(
            PoseArray, '/detection/camera_coordinates', 10
        )
        # NED 世界坐标, frame_id: map
        self.world_pub = self.create_publisher(
            PoseArray, '/detection/world_coordinates', 10
        )

        # 1 Hz 打印欧拉角
        self.euler_timer = self.create_timer(1.0, self._print_euler_callback)

        self.get_logger().info('检测世界坐标节点启动, 订阅 /detections (Detection2DArray)')
        self.get_logger().info(f'激光雷达偏移={LASER_OFFSET}m')
        self.get_logger().info('发布: /detection/camera_coordinates (相机坐标)')
        self.get_logger().info('发布: /detection/world_coordinates (世界坐标)')

    # ==================== PX4 回调 ====================
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

    # ==================== 检测回调 (替代 UDP) ====================
    def _detection_callback(self, msg: Detection2DArray):
        """接收 yolo_detector_node 发布的 Detection2DArray, 计算坐标并发布"""
        detections = msg.detections
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

        # ---- 构建 PoseArray 消息 ----
        stamp = self.get_clock().now().to_msg()

        cam_poses = PoseArray()
        cam_poses.header.stamp = stamp
        cam_poses.header.frame_id = "camera_optical_frame"

        world_poses = PoseArray()
        world_poses.header.stamp = stamp
        world_poses.header.frame_id = "map"

        for det in detections:
            # 中心像素坐标 (Detection2D 的 bbox center)
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y

            # ---- 相机坐标 ----
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

            # 相机坐标系下的 Pose
            cam_pose = Pose()
            cam_pose.position.x = X_cam
            cam_pose.position.y = Y_cam
            cam_pose.position.z = z
            cam_poses.poses.append(cam_pose)

            # ---- 世界坐标 (NED) ----
            if pos is not None:
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                world_x = pos[0] + Y_cam * cos_y + X_cam * sin_y
                world_y = pos[1] + Y_cam * sin_y - X_cam * cos_y

                world_pose = Pose()
                world_pose.position.x = world_x
                world_pose.position.y = world_y
                world_pose.position.z = 0.0
                world_poses.poses.append(world_pose)

        # 发布
        if cam_poses.poses:
            self.cam_pub.publish(cam_poses)
        if world_poses.poses:
            self.world_pub.publish(world_poses)


def main():
    rclpy.init()
    node = DetectionWorldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()