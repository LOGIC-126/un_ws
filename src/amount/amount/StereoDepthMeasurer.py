"""
D435 深度测量模块 — 对标 MonocularPlaneMeasurer

Pipeline:
  深度-彩色对齐（由 realsense-ros driver 的 align_depth:=true 完成）
  → 像素取深 (get_depth_at)
  → 反投影到相机三维坐标 (pixel_to_camera_3d)

与 MonocularPlaneMeasurer 的区别:
  - Z 从深度图逐像素取值，而非单一固定 plane_distance
  - 不做畸变校正和旋转解耦 — D435 driver 已输出矫正对齐后的深度图
"""

import numpy as np


class StereoDepthMeasurer:
    """
    D435 深度反投影测量器。

    输入:
      - 相机内参 (fx, fy, ppx, ppy)  — 从 CameraInfo.k 提取
      - 深度图 (numpy uint16, mm)     — 已对齐到彩色帧
      - 像素坐标 (u, v)               — 检测框中心

    输出:
      - 相机坐标系 (X, Y, Z) 米
        X = (ppx - u) / fx * Z   # 左正（与 MonocularPlaneMeasurer 同符号约定）
        Y = (ppy - v) / fy * Z   # 前正
        Z = depth / 1000.0       # mm → m
    """

    def __init__(self, fx, fy, ppx, ppy):
        self.fx = float(fx)
        self.fy = float(fy)
        self.ppx = float(ppx)       # 主点 x (cx)
        self.ppy = float(ppy)       # 主点 y (cy)
        self._depth_image = None    # numpy uint16 [mm], 对齐到彩色帧

    # ========== 深度图管理 ==========

    def set_depth_image(self, depth_image):
        """
        存入对齐后的深度图。

        参数:
          depth_image: numpy ndarray, uint16, 单位 mm
        """
        self._depth_image = depth_image

    @property
    def has_depth(self):
        """深度图是否已就绪"""
        return self._depth_image is not None

    # ========== 像素取深 ==========

    def get_depth_at(self, u, v):
        """
        获取像素 (u, v) 处的深度值。

        参数:
          u, v: 像素坐标 (浮点数, 内部会 round 到整数)
        返回:
          float — 深度值 (米), 无效时返回 0.0
        """
        if self._depth_image is None:
            return 0.0

        u_int, v_int = int(round(u)), int(round(v))
        h, w = self._depth_image.shape[:2]

        if not (0 <= u_int < w and 0 <= v_int < h):
            return 0.0

        depth_mm = self._depth_image[v_int, u_int]

        # D435 返回 0 表示该像素无有效深度数据
        if depth_mm == 0:
            return 0.0

        return float(depth_mm) / 1000.0

    # ========== 反投影 ==========

    def pixel_to_camera_3d(self, u, v):
        """
        像素坐标 → 相机坐标系三维坐标。

        参数:
          u, v: 像素坐标
        返回:
          (X, Y, Z) 米, 深度无效时返回 (0.0, 0.0, 0.0)
        """
        Z = self.get_depth_at(u, v)
        if Z <= 0.0:
            return 0.0, 0.0, 0.0

        X = (self.ppx - u) / self.fx * Z
        Y = (self.ppy - v) / self.fy * Z

        return X, Y, Z

    def pixel_to_camera_3d_batch(self, pixel_points):
        """
        批量像素坐标 → 相机坐标系三维坐标。

        参数:
          pixel_points: [(u1, v1), (u2, v2), ...]
        返回:
          [(X1, Y1, Z1), (X2, Y2, Z2), ...]
        """
        return [self.pixel_to_camera_3d(u, v) for u, v in pixel_points]

    # ========== 工厂方法 ==========

    @staticmethod
    def from_camera_info(camera_info):
        """
        从 ROS2 CameraInfo 消息创建实例。

        参数:
          camera_info: sensor_msgs.msg.CameraInfo
            k[0] = fx, k[4] = fy, k[2] = ppx, k[5] = ppy
        返回:
          StereoDepthMeasurer
        """
        return StereoDepthMeasurer(
            fx=camera_info.k[0],
            fy=camera_info.k[4],
            ppx=camera_info.k[2],
            ppy=camera_info.k[5],
        )