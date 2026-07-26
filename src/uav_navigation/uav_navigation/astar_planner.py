#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A* 路径规划器, 在 Cartographer OccupancyGrid 上搜索最短路径。

输入: OccupancyGrid (/map), start (map 帧 x,y), goal (map 帧 x,y)
输出: 路径坐标列表 [(x,y), ...] (map 帧) 或空列表
"""

import heapq
import math
from typing import List, Tuple, Optional


class AStarPlanner:
    """8 邻域 A* 搜索, 带障碍物膨胀。"""

    def __init__(self, inflate_radius: float = 0.1) -> None:
        """
        Args:
            inflate_radius: 障碍物膨胀半径 (m), 默认 0.1m = 2 格 @0.05m.
        """
        self.inflate_radius = inflate_radius
        self._diag_cost = math.sqrt(2)

        # OccupancyGrid 参数缓存
        self.resolution: float = 0.0
        self.width: int = 0
        self.height: int = 0
        self.origin_x: float = 0.0
        self.origin_y: float = 0.0
        self._grid: List[List[int]] = []          # col-major 访问: grid[row][col]
        self._inflated: List[List[bool]] = []     # True = 不可通行

    # ─── 公共 API ───────────────────────────────────────────

    def update_grid(self, msg) -> None:
        """从 OccupancyGrid 消息解析网格并做障碍物膨胀。

        Args:
            msg: nav_msgs/OccupancyGrid
        """
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # 解析为 row-major 二维列表
        self._grid = [[0] * self.width for _ in range(self.height)]
        for row in range(self.height):
            base = row * self.width
            for col in range(self.width):
                self._grid[row][col] = msg.data[base + col]

        self._inflate_obstacles()

    def plan(self, start_xy: Tuple[float, float],
             goal_xy: Tuple[float, float]) -> List[Tuple[float, float]]:
        """A* 搜索。

        Returns:
            路径坐标列表 (map 帧), 空列表表示无路径。
        """
        sx, sy = start_xy
        gx, gy = goal_xy

        start = self._world_to_grid(sx, sy)
        goal = self._world_to_grid(gx, gy)

        if start is None or goal is None:
            return []

        if not self._is_free(goal[0], goal[1]):
            # goal 在障碍物内 — 尝试找最近的自由格
            alt = self._nearest_free(goal[0], goal[1])
            if alt is None:
                return []
            goal = alt

        if start == goal:
            return [start_xy]

        if not self._is_free(start[0], start[1]):
            return []

        return self._astar_search(start, goal)

    # ─── 障碍物膨胀 ────────────────────────────────────────

    def _inflate_obstacles(self) -> None:
        """将障碍物 (data=100) 按 inflate_radius 膨胀为不可通行区域。"""
        inflate_cells = max(1, int(math.ceil(self.inflate_radius / self.resolution)))

        # 收集原始障碍物坐标
        obstacles = []
        for row in range(self.height):
            for col in range(self.width):
                if self._grid[row][col] >= 90:       # 接近 100 就算障碍
                    obstacles.append((col, row))

        # 初始化膨胀网格
        self._inflated = [[False] * self.width for _ in range(self.height)]

        for oc, oy in obstacles:
            r_min = max(0, oy - inflate_cells)
            r_max = min(self.height - 1, oy + inflate_cells)
            c_min = max(0, oc - inflate_cells)
            c_max = min(self.width - 1, oc + inflate_cells)
            for r in range(r_min, r_max + 1):
                for c in range(c_min, c_max + 1):
                    self._inflated[r][c] = True

    # ─── 坐标转换 ──────────────────────────────────────────

    def _world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """map 帧坐标 → 网格 (col, row)。越界返回 None。"""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= col < self.width and 0 <= row < self.height:
            return (col, row)
        return None

    def _grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """网格中心 → map 帧坐标。"""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return (x, y)

    # ─── 网格查询 ──────────────────────────────────────────

    def _is_free(self, col: int, row: int) -> bool:
        """检查网格是否可通行 (未膨胀 且 非未知区域视为可通行)。"""
        if not (0 <= col < self.width and 0 <= row < self.height):
            return False
        if self._inflated[row][col]:
            return False
        # -1 (未知) 视为可通行; 0 (自由) 可通行; 100 (障碍) 已在膨胀中处理
        return self._grid[row][col] < 90

    def _nearest_free(self, col: int, row: int) -> Optional[Tuple[int, int]]:
        """BFS 搜索最近的自由格 (goal 被障碍物挡住时用)。"""
        from collections import deque
        seen = {(col, row)}
        q = deque([(col, row)])
        while q:
            c, r = q.popleft()
            if self._is_free(c, r):
                return (c, r)
            for dc, dr in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nc, nr = c + dc, r + dr
                if 0 <= nc < self.width and 0 <= nr < self.height and (nc, nr) not in seen:
                    seen.add((nc, nr))
                    q.append((nc, nr))
        return None

    # ─── A* 核心 ───────────────────────────────────────────

    def _astar_search(self, start: Tuple[int, int],
                      goal: Tuple[int, int]) -> List[Tuple[float, float]]:
        """标准 A*: 8 邻域, 欧几里得启发式。"""
        sx, sy = start
        gx, gy = goal

        # 优先队列: (f, tiebreaker, (col, row))
        counter = 0
        open_set = [(self._heuristic(sx, sy, gx, gy), counter, (sx, sy))]
        heapq.heapify(open_set)

        came_from = {(sx, sy): None}
        g_score = {(sx, sy): 0.0}

        # 8 邻域: (dc, dr, cost)
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, self._diag_cost), (1, -1, self._diag_cost),
            (-1, 1, self._diag_cost), (-1, -1, self._diag_cost),
        ]

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                return self._reconstruct_path(came_from, current)

            cx, cy = current
            for dc, dr, step_cost in neighbors:
                nx, ny = cx + dc, cy + dr
                if not self._is_free(nx, ny):
                    continue
                tent_g = g_score[current] + step_cost
                if tent_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tent_g
                    f = tent_g + self._heuristic(nx, ny, gx, gy)
                    counter += 1
                    heapq.heappush(open_set, (f, counter, (nx, ny)))

        return []      # 无路径可达

    @staticmethod
    def _heuristic(col: int, row: int, gx: int, gy: int) -> float:
        return math.hypot(gx - col, gy - row)

    def _reconstruct_path(self, came_from: dict,
                          current: Tuple[int, int]) -> List[Tuple[float, float]]:
        """回溯路径并转为 map 帧坐标列表。"""
        cells = []
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        return [self._grid_to_world(c, r) for c, r in cells]