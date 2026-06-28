#!/usr/bin/env python3
"""
世界坐标密度滤波器 (独立 ROS2 节点)

策略:
  1. 订阅 /detection/world_coordinates_raw，缓冲最近 N 个点
  2. 对每个新点，检查缓冲区内是否有 ≥5 个邻居在 7.5cm 以内
  3. 通过密度检验的点，取其邻域内所有点的中值作为滤波输出
  4. 发布到 /detection/world_coordinates (filtered)

话题:
  订阅: /detection/world_coordinates_raw  (uv2_ros 原始世界坐标)
  发布: /detection/world_coordinates       (密度滤波后)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import numpy as np
from collections import deque


# ============ 参数 ============
BUFFER_SIZE = 200          # 最多缓存多少个点
MIN_NEIGHBORS = 5          # 周围至少 N 个邻居才保留
NEIGHBOR_RADIUS = 0.075    # 邻居距离阈值 (7.5cm)


class DensityFilterNode(Node):

    def __init__(self):
        super().__init__('world_filter_node')

        self._buffer = deque(maxlen=BUFFER_SIZE)  # [(x, y, class), ...]
        self._total = 0
        self._passed = 0
        self._stats_t = self.get_clock().now()

        # 订阅原始世界坐标
        self.raw_sub = self.create_subscription(
            String,
            '/detection/world_coordinates_raw',
            self._raw_callback,
            10
        )

        # 发布滤波后世界坐标
        self.filtered_pub = self.create_publisher(
            String,
            '/detection/world_coordinates',
            10
        )

        self.get_logger().info(
            f'密度滤波器启动: min_neighbors={MIN_NEIGHBORS}, '
            f'radius={NEIGHBOR_RADIUS:.3f}m, buffer={BUFFER_SIZE}'
        )

    # ==================== 回调 ====================
    def _raw_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if isinstance(data, dict):
            detections = [data]
        elif isinstance(data, list):
            detections = data
        else:
            return

        filtered_results = []

        for det in detections:
            self._total += 1
            cls = det.get('class', 'unknown')
            world = det.get('world_absolute', [0, 0, 0])
            x, y = world[0], world[1]

            # 当前点加入缓冲区 (无论是否通过, 供后续点做密度检验)
            self._buffer.append((x, y, cls))

            # ---- 密度检验 ----
            neighbors = self._find_neighbors(x, y)

            if len(neighbors) < MIN_NEIGHBORS:
                continue

            self._passed += 1

            # ---- 中值平滑 ----
            nx = [n[0] for n in neighbors] + [x]
            ny = [n[1] for n in neighbors] + [y]
            fx = float(np.median(nx))
            fy = float(np.median(ny))

            filtered_results.append({
                'class': cls,
                'world_absolute': [round(fx, 4), round(fy, 4), 0.0]
            })

        # 发布
        if filtered_results:
            self.filtered_pub.publish(
                String(data=json.dumps(filtered_results, ensure_ascii=False))
            )

        # 定期统计
        now = self.get_clock().now()
        if (now - self._stats_t).nanoseconds > 10e9:
            self._stats_t = now
            rate = self._passed / max(self._total, 1) * 100
            self.get_logger().info(
                f"[密度滤波] 入{self._total} 出{self._passed} ({rate:.1f}%)  "
                f"缓冲区{len(self._buffer)}个点"
            )

    # ==================== 密度检测 ====================
    def _find_neighbors(self, x, y):
        """在缓冲区内查找距离 (x,y) 小于 NEIGHBOR_RADIUS 的邻居"""
        neighbors = []
        r2 = NEIGHBOR_RADIUS * NEIGHBOR_RADIUS
        for bx, by, _ in self._buffer:
            d2 = (x - bx)**2 + (y - by)**2
            if d2 < r2:
                neighbors.append((bx, by))
        return neighbors


def main(args=None):
    rclpy.init(args=args)
    node = DensityFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
