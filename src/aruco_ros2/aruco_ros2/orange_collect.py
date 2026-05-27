#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import cv2
from cv2 import aruco
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
import os
from scipy.spatial.transform import Rotation
from collections import deque
from scipy.signal import butter

# ---------- 巴特沃斯滤波器设计（与您的测试代码相同） ----------
def butter3_filter_design(cutoff, fs, order=3):
    """设计3阶巴特沃斯低通滤波器，返回 b, a 系数"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='lowpass', analog=False)
    return b, a

class Butter3:
    """3阶巴特沃斯低通滤波器，支持直接设置初始值以消除瞬态"""
    def __init__(self, b, a):
        self.B = b
        self.A = a
        self.X = np.zeros(4)
        self.Y = np.zeros(4)

    def set_initial_value(self, value):
        """将所有延迟单元设为同一常值，使滤波器立刻收敛于该值"""
        self.X.fill(value)
        self.Y.fill(value)

    def process(self, in_value):
        self.X[3] = in_value
        self.Y[3] = (self.B[0] * self.X[3] + self.B[1] * self.X[2] +
                     self.B[2] * self.X[1] + self.B[3] * self.X[0] -
                     self.A[1] * self.Y[2] - self.A[2] * self.Y[1] -
                     self.A[3] * self.Y[0])
        out_value = self.Y[3]
        self.X[0] = self.X[1]
        self.X[1] = self.X[2]
        self.X[2] = self.X[3]
        self.Y[0] = self.Y[1]
        self.Y[1] = self.Y[2]
        self.Y[2] = self.Y[3]
        return out_value

# ---------- 相机配置读取、显示等辅助函数（不变） ----------
def read_camera_config(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)
        return config
    except FileNotFoundError:
        print(f"Error: Cannot find file '{file_path}'")
        return None
    except yaml.YAMLError as e:
        print(f"Error: YAML Parse Error - {e}")
        return None

def display_camera_info(config):
    if not config:
        return
    print("\nCamera Info:")
    print(f"  Camera Name: {config['camera_name']}")
    print(f"  Image Size: {config['image_width']} x {config['image_height']} ")
    print("\n  Distortion Parameters:")
    dist = config['distortion_parameters']
    print(f"    k1: {dist['k1']}")
    print(f"    k2: {dist['k2']}")
    print(f"    p1: {dist['p1']}")
    print(f"    p2: {dist['p2']}")
    print(f"    k3: {dist['k3']}")
    print("\n  Projection Parameters:")
    proj = config['projection_parameters']
    print(f"    fx: {proj['fx']}")
    print(f"    fy: {proj['fy']}")
    print(f"    cx: {proj['cx']}")
    print(f"    cy: {proj['cy']}")
    print()

def ConfigureCamera(cap):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(3, 640)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_FPS, 120)

# ---------- 四元数低通（slerp） ----------
def slerp_quat(q0, q1, t):
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        result = q0 + t * (q1 - q0)
        result /= np.linalg.norm(result)
        return result

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0) + (s1 * q1)

class QuatLowPassFilter:
    def __init__(self, tau):
        self.tau = tau
        self.initialized = False
        self.q = np.array([0.0, 0.0, 0.0, 1.0])

    def update(self, q_new, dt):
        if not self.initialized:
            self.q = q_new
            self.initialized = True
            return self.q
        alpha = dt / (self.tau + dt)
        self.q = slerp_quat(self.q, q_new, alpha)
        return self.q

# ---------- 主节点 ----------
class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/camera/pose', 10)
        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 读取相机参数
        config_file = os.path.join(
            get_package_share_directory('aruco_ros2'), 'config', 'cam.yaml')
        camera_config = read_camera_config(config_file)
        if not camera_config:
            self.get_logger().error("Failed to load camera config")
            rclpy.shutdown()
            return
        display_camera_info(camera_config)

        cam_fx = camera_config['projection_parameters']['fx']
        cam_fy = camera_config['projection_parameters']['fy']
        cam_cx = camera_config['projection_parameters']['cx']
        cam_cy = camera_config['projection_parameters']['cy']
        cam_k1 = camera_config['distortion_parameters']['k1']
        cam_k2 = camera_config['distortion_parameters']['k2']
        cam_p1 = camera_config['distortion_parameters']['p1']
        cam_p2 = camera_config['distortion_parameters']['p2']
        cam_k3 = camera_config['distortion_parameters']['k3']
        self.cam_matrix = np.array([[cam_fx, 0, cam_cx],
                                    [0, cam_fy, cam_cy],
                                    [0, 0, 1]], dtype=np.float64)
        self.cam_dist = np.array([cam_k1, cam_k2, cam_p1, cam_p2, cam_k3],
                                 dtype=np.float64)

        # ---------- 位置滤波：中值 + 3阶巴特沃斯 ----------
        self.pos_median_window = deque(maxlen=7)          # 中值窗口（可调）
        fc_pos = 2.0                                      # 巴特沃斯截止频率 (Hz)，建议 1.5~2.5
        fs = 30.0                                         # 采样频率 (Hz)，匹配定时器频率
        b, a = butter3_filter_design(fc_pos, fs, order=3)
        self.butter_x = Butter3(b, a)
        self.butter_y = Butter3(b, a)
        self.butter_z = Butter3(b, a)
        self.butter_initialized = False

        # ---------- 姿态滤波：四元数低通（不变） ----------
        fc_rot = 4.0
        tau_rot = 1.0 / (2.0 * np.pi * fc_rot)
        self.quat_filter = QuatLowPassFilter(tau_rot)

        # ---------- ArUco 检测器（亚像素细化） ----------
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 30
        parameters.cornerRefinementMinAccuracy = 0.01
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 23
        parameters.adaptiveThreshWinSizeStep = 10
        self.detector = aruco.ArucoDetector(aruco_dict, parameters)

        self.marker_length = 190.0
        half = self.marker_length / 2.0
        self.obj_points = np.array([[-half, half, 0],
                                    [half, half, 0],
                                    [half, -half, 0],
                                    [-half, -half, 0]], dtype=np.float64)

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        ConfigureCamera(self.cap)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            rclpy.shutdown()
            return

        self.pose_list = []
        self.goal_frame_count = 200
        self.frame_count = 0
        self.last_time = time.perf_counter()

        # 帧率统计
        self._fps_count = 0
        self._fps_accum = 0.0

        self.get_logger().info("Aruco pose publisher started (Butter3 + median)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        current_time = time.perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time
        dt = min(dt, 0.1)

        # 帧率监控
        if dt > 0:
            self._fps_accum += 1.0 / dt
            self._fps_count += 1
            if self._fps_count >= 30:
                avg_fps = self._fps_accum / self._fps_count
                self.get_logger().info(f"Actual FPS: {avg_fps:.2f}")
                self._fps_count = 0
                self._fps_accum = 0.0

        corners, ids, _ = self.detector.detectMarkers(frame)
        if ids is not None and len(ids) > 0:
            marker_corners = corners[0][0]
            retval, rvec, tvec = cv2.solvePnP(
                self.obj_points, marker_corners,
                self.cam_matrix, self.cam_dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not retval:
                return

            # 可选 LM 精化
            rvec, tvec = cv2.solvePnPRefineLM(
                self.obj_points, marker_corners,
                self.cam_matrix, self.cam_dist, rvec, tvec)

            # 相机在标记系下的位置 (mm)
            rot_mat = cv2.Rodrigues(rvec)[0]
            rot_mat_T = np.transpose(rot_mat)
            camera_pos = -np.dot(rot_mat_T, tvec).flatten()

            # ---------- 中值滤波 ----------
            self.pos_median_window.append(camera_pos)
            if len(self.pos_median_window) == self.pos_median_window.maxlen:
                median_pos = np.median(self.pos_median_window, axis=0)
            else:
                median_pos = camera_pos

            # ---------- 巴特沃斯滤波 ----------
            if not self.butter_initialized:
                # 初始化滤波器状态为第一次中值，避免阶跃
                self.butter_x.set_initial_value(median_pos[0])
                self.butter_y.set_initial_value(median_pos[1])
                self.butter_z.set_initial_value(median_pos[2])
                self.butter_initialized = True
                pos_filtered = median_pos
            else:
                fx_mm = self.butter_x.process(median_pos[0])
                fy_mm = self.butter_y.process(median_pos[1])
                fz_mm = self.butter_z.process(median_pos[2])
                pos_filtered = np.array([fx_mm, fy_mm, fz_mm])

            # 转换米
            fx = pos_filtered[0] / 1000.0
            fy = pos_filtered[1] / 1000.0
            fz = pos_filtered[2] / 1000.0

            # ---------- 姿态四元数滤波 ----------
            raw_quat = Rotation.from_matrix(rot_mat_T).as_quat()
            filtered_quat = self.quat_filter.update(raw_quat, dt)

            # 发布
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "aruco_marker"
            pose_msg.pose.position.x = fx
            pose_msg.pose.position.y = fy
            pose_msg.pose.position.z = fz
            pose_msg.pose.orientation.x = filtered_quat[0]
            pose_msg.pose.orientation.y = filtered_quat[1]
            pose_msg.pose.orientation.z = filtered_quat[2]
            pose_msg.pose.orientation.w = filtered_quat[3]

            self.publisher_.publish(pose_msg)
            self.get_logger().info(
                f"Published: x={fx:.3f} y={fy:.3f} z={fz:.3f} (meters)")

            # 保存数据
            self.pose_list.append(
                np.array([current_time, fx, fy, fz,
                          filtered_quat[0], filtered_quat[1],
                          filtered_quat[2], filtered_quat[3]]))
            self.frame_count += 1
            if self.frame_count >= self.goal_frame_count:
                np.save("pose.npy", np.array(self.pose_list))
                self.get_logger().info(f"Saved {self.frame_count} frames to pose.npy")
                self.frame_count = 0

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            rclpy.shutdown()

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()