#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VFH+ 避障规划器 — 纯算法模块, 计算安全的 detour 位置。

由 click_navigate_node 在航点追踪循环中调用:
  - 无障碍 → 返回 None, 正常跟踪 A* 航点
  - 有障碍 → 返回 (detour_x, detour_y) map 帧坐标, 发布为 /uav/target_position
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class VFHParams:
    """VFH+ 可调参数。"""
    safety_distance: float = 0.5        # 最小安全距离 (m)
    robot_radius: float = 0.3           # 无人机半径 (m)
    sectors: int = 72                    # 极坐标扇区数 (5° 分辨率)
    detour_distance: float = 1.0        # 绕行距离 (m): 在安全方向上生成多远的目标点
    goal_weight: float = 2.0            # 目标方向权重
    smooth_weight: float = 1.5          # 平滑转向权重
    obstacle_threshold: float = 0.6     # 触发绕行: 前方扇区阻塞比例


class VFHPlanner:
    """VFH+ 位置级避障规划器。"""

    def __init__(self, params: VFHParams | None = None) -> None:
        self.p = params or VFHParams()
        self.angle_per_sector = 2.0 * math.pi / self.p.sectors
        self._robot_sectors = max(1, int(math.ceil(
            self.p.robot_radius /
            (self.p.safety_distance * math.tan(self.angle_per_sector / 2.0))
        )))
        self._prev_steer_sector: int | None = None

    def compute_detour(self, scan_ranges: list, scan_angle_min: float,
                       scan_angle_increment: float,
                       drone_yaw: float,
                       target_map_xy: Tuple[float, float],
                       drone_map_xy: Tuple[float, float],
                       ) -> Optional[Tuple[float, float]]:
        """核心入口: 如有障碍返回绕行点 (map 帧), 否则返回 None。

        Args:
            scan_ranges: LaserScan.ranges 列表
            scan_angle_min: LaserScan.angle_min
            scan_angle_increment: LaserScan.angle_increment
            drone_yaw: 无人机 NED 偏航 (rad)
            target_map_xy: 当前目标航点在 map 帧的坐标 (x,y)
            drone_map_xy: 无人机在 map 帧的坐标 (x,y)

        Returns:
            detour 目标点 (map 帧) 或 None (无障碍, 直行).
        """
        dx = target_map_xy[0] - drone_map_xy[0]
        dy = target_map_xy[1] - drone_map_xy[1]
        goal_dist = math.hypot(dx, dy)
        if goal_dist < 0.15:
            return None   # 已很接近, 不重新规划

        # 1. 极坐标直方图
        polar = self._build_polar(scan_ranges, scan_angle_min, scan_angle_increment)

        # 2. 二值化 + 掩码
        blocked = self._binary_mask(polar)

        # 3. 目标方向 → 机体帧角度
        goal_angle_body = self._ned_dir_to_body(dx, dy, drone_yaw)

        # 4. 判断是否需要绕行
        if not self._need_detour(blocked, goal_angle_body):
            self._prev_steer_sector = None
            return None

        # 5. 代价函数选最优扇区
        best = self._select_sector(blocked, goal_angle_body)
        if best is None:
            # 完全被围: 原地悬停 (目标 = 当前无人机位置)
            return drone_map_xy

        # 6. 扇区 → 机体帧方向 → map 帧 detour 点
        steer_angle = best * self.angle_per_sector
        body_vx = math.cos(steer_angle)
        body_vy = math.sin(steer_angle)
        # 机体 → NED (map 帧的约定: x=North, y=-East)
        ned_vx = body_vx * math.cos(drone_yaw) - body_vy * math.sin(drone_yaw)
        ned_vy = body_vx * math.sin(drone_yaw) + body_vy * math.cos(drone_yaw)

        detour_x = drone_map_xy[0] + ned_vx * self.p.detour_distance
        detour_y = drone_map_xy[1] - ned_vy   # map.y = -NED.east

        self._prev_steer_sector = best
        return (detour_x, detour_y)

    # ─── 直方图 ────────────────────────────────────────────

    def _build_polar(self, ranges: list, angle_min: float,
                     angle_inc: float) -> list:
        """每扇区最近距离, 无限远 → 99.0。"""
        hist = [99.0] * self.p.sectors
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = (angle_min + i * angle_inc) % (2.0 * math.pi)
            sec = int(angle / self.angle_per_sector) % self.p.sectors
            if r < hist[sec]:
                hist[sec] = r
        return hist

    def _binary_mask(self, polar: list) -> list:
        blocked = [polar[i] < self.p.safety_distance for i in range(self.p.sectors)]
        expanded = blocked[:]
        for i in range(self.p.sectors):
            if blocked[i]:
                for j in range(-self._robot_sectors, self._robot_sectors + 1):
                    expanded[(i + j) % self.p.sectors] = True
        return expanded

    def _need_detour(self, blocked: list, goal_body_angle: float) -> bool:
        gs = int(goal_body_angle / self.angle_per_sector) % self.p.sectors
        check = max(1, int(math.radians(30) / self.angle_per_sector))
        cnt = 0
        for j in range(-check, check + 1):
            if blocked[(gs + j) % self.p.sectors]:
                cnt += 1
        return cnt / (2 * check + 1) >= self.p.obstacle_threshold

    def _select_sector(self, blocked: list, goal_angle: float) -> int | None:
        gs = int(goal_angle / self.angle_per_sector) % self.p.sectors
        best, best_cost = None, float('inf')
        for i in range(self.p.sectors):
            if blocked[i]:
                continue
            # 目标偏差
            diff = abs(i - gs)
            if diff > self.p.sectors // 2:
                diff = self.p.sectors - diff
            goal_cost = self.p.goal_weight * diff / self.p.sectors
            # 平滑代价
            smooth_cost = 0.0
            if self._prev_steer_sector is not None:
                sd = abs(i - self._prev_steer_sector)
                if sd > self.p.sectors // 2:
                    sd = self.p.sectors - sd
                smooth_cost = self.p.smooth_weight * sd / self.p.sectors
            cost = goal_cost + smooth_cost
            if cost < best_cost:
                best_cost = cost
                best = i
        return best

    # ─── 坐标工具 ──────────────────────────────────────────

    @staticmethod
    def _ned_dir_to_body(dx_ned: float, dy_ned: float, yaw: float) -> float:
        """NED 方向 → 机体帧角度 (rad, [0, 2π))."""
        bx = dx_ned * math.cos(yaw) + dy_ned * math.sin(yaw)
        by = -dx_ned * math.sin(yaw) + dy_ned * math.cos(yaw)
        return math.atan2(by, bx) % (2.0 * math.pi)