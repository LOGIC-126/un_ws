#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rviz2 点击导航节点: 订阅 /goal_pose → A* 路径规划 → 航点追踪 + VFH+ 避障.

状态机: IDLE → PLANNING → NAVIGATING → ARRIVED
所有控制通过 /uav/target_position (Pose, NED) 发布, 兼容现有 offboard_control 接口.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import TransformException

from .astar_planner import AStarPlanner
from .vfh_planner import VFHPlanner, VFHParams


# ─── 状态枚举 ──────────────────────────────────────────────

from enum import Enum


class NavState(Enum):
    IDLE = 0
    PLANNING = 1
    NAVIGATING = 2
    ARRIVED = 3


# ─── 颜色常量 ──────────────────────────────────────────────

GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0)
PURPLE = ColorRGBA(r=0.7, g=0.2, b=1.0, a=0.8)
GOLD = ColorRGBA(r=1.0, g=0.84, b=0.0, a=1.0)
GRAY = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)
WHITE = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.6)


class ClickNavigateNode(Node):
    """Rviz2 点击导航: A* 规划 + VFH+ 位置避障 + 可视化."""

    def __init__(self) -> None:
        super().__init__('click_navigate_node')

        # ── 参数 ──
        self.declare_parameter('takeoff_height', -1.2)       # NED 飞行高度
        self.declare_parameter('arrival_threshold', 0.2)     # 航点到达阈值 (m)
        self.declare_parameter('waypoint_spacing', 0.1)      # A* 路径航点最小间距 (m)
        self.declare_parameter('update_rate', 20.0)          # 控制循环 (Hz)
        self.declare_parameter('inflate_radius', 0.1)        # 障碍物膨胀半径 (m)
        self.declare_parameter('fixed_frame', 'map')         # Rviz2 Fixed Frame

        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.update_rate = self.get_parameter('update_rate').value
        self.fixed_frame = self.get_parameter('fixed_frame').value

        # ── QoS ──
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── 订阅 ──
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            rclpy.qos.qos_profile_sensor_data)

        # ── 发布 ──
        self.target_pub = self.create_publisher(Pose, '/uav/target_position', 10)
        self.path_pub = self.create_publisher(Path, '/click_navigate/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/click_navigate/markers', 10)

        # ── TF ──
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 规划器 ──
        self.astar = AStarPlanner(inflate_radius=self.get_parameter('inflate_radius').value)
        vfh_params = VFHParams()
        self.vfh = VFHPlanner(vfh_params)

        # ── 状态变量 ──
        self.state = NavState.IDLE
        self._latest_map: OccupancyGrid | None = None
        self._latest_scan: LaserScan | None = None
        self._path_waypoints: list = []          # [(map_x, map_y), ...]
        self._waypoint_index: int = 0
        self._goal_pose: PoseStamped | None = None
        self._drone_map_x: float = 0.0
        self._drone_map_y: float = 0.0
        self._drone_yaw: float = 0.0
        self._last_target_pose: Pose | None = None

        # ── 定时器 ──
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"点击导航节点已启动 | 固定帧={self.fixed_frame} | "
            f"飞行高度={self.takeoff_height}m | 到达阈值={self.arrival_threshold}m"
        )

    # ─── 订阅回调 ──────────────────────────────────────────

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.state in (NavState.NAVIGATING, NavState.PLANNING):
            self.get_logger().info("收到新目标, 取消当前路径")
        self._goal_pose = msg
        self.state = NavState.PLANNING

    def map_callback(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    # ─── 主循环 ────────────────────────────────────────────

    def timer_callback(self) -> None:
        self._update_drone_pose()
        if self.state == NavState.IDLE:
            self._publish_idle_markers()
        elif self.state == NavState.PLANNING:
            self._do_planning()
        elif self.state == NavState.NAVIGATING:
            self._do_navigating()
        elif self.state == NavState.ARRIVED:
            self._publish_arrived_markers()
            self._publish_idle_markers()

    # ─── TF 更新无人机位置 ─────────────────────────────────

    def _update_drone_pose(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform(
                self.fixed_frame, 'base_link',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            self._drone_map_x = t.transform.translation.x
            self._drone_map_y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._drone_yaw = math.atan2(siny, cosy)
        except TransformException:
            pass

    # ─── PLANNING ──────────────────────────────────────────

    def _do_planning(self) -> None:
        if self._latest_map is None:
            self.get_logger().warn("等待 /map ...", throttle_duration_sec=3.0)
            return

        goal = self._goal_pose
        if goal is None:
            return

        # 更新 A* 地图
        self.astar.update_grid(self._latest_map)

        # map 帧坐标
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        path = self.astar.plan(
            (self._drone_map_x, self._drone_map_y), (goal_x, goal_y))
        if not path:
            self.get_logger().error(
                f"A* 规划失败: ({self._drone_map_x:.2f},{self._drone_map_y:.2f}) → "
                f"({goal_x:.2f},{goal_y:.2f})")
            self.state = NavState.IDLE
            return

        # 降采样: 减少冗余航点
        self._path_waypoints = self._downsample(path)
        self._waypoint_index = 0

        # 可视化
        self._publish_path(path)
        self._publish_plan_markers()

        self.get_logger().info(
            f"规划完成: {len(self._path_waypoints)} 个航点 "
            f"({len(path)} A* 原始节点) → 目标 ({goal_x:.2f}, {goal_y:.2f})")
        self.state = NavState.NAVIGATING

    @staticmethod
    def _downsample(path: list, min_spacing: float = 0.1) -> list:
        if len(path) <= 2:
            return path
        result = [path[0]]
        last = path[0]
        for pt in path[1:-1]:
            if math.hypot(pt[0] - last[0], pt[1] - last[1]) >= min_spacing:
                result.append(pt)
                last = pt
        result.append(path[-1])
        return result

    # ─── NAVIGATING ────────────────────────────────────────

    def _do_navigating(self) -> None:
        if not self._path_waypoints:
            self.state = NavState.IDLE
            return

        # 当前航点
        idx = self._waypoint_index
        if idx >= len(self._path_waypoints):
            self.get_logger().info("抵达最终目标!")
            self.state = NavState.ARRIVED
            return

        waypoint = self._path_waypoints[idx]
        wx, wy = waypoint

        # 到达判断
        dist = math.hypot(self._drone_map_x - wx, self._drone_map_y - wy)
        if dist < self.arrival_threshold:
            self._waypoint_index += 1
            self.get_logger().info(
                f"航点 {idx + 1}/{len(self._path_waypoints)} 到达 "
                f"(剩余 {len(self._path_waypoints) - idx - 1})",
                throttle_duration_sec=1.0)
            if self._waypoint_index >= len(self._path_waypoints):
                self.state = NavState.ARRIVED
            return

        # ── VFH+ 避障检查 ──
        detour = None
        if self._latest_scan is not None:
            detour = self.vfh.compute_detour(
                scan_ranges=list(self._latest_scan.ranges),
                scan_angle_min=self._latest_scan.angle_min,
                scan_angle_increment=self._latest_scan.angle_increment,
                drone_yaw=self._drone_yaw,
                target_map_xy=(wx, wy),
                drone_map_xy=(self._drone_map_x, self._drone_map_y),
            )

        if detour is not None:
            tx, ty = detour
            self.get_logger().debug(
                f"VFH+ 绕行: → ({tx:.2f}, {ty:.2f})", throttle_duration_sec=0.5)
        else:
            tx, ty = wx, wy

        # map → NED 转换并发布
        self._publish_ned_target(tx, ty)

    # ─── 坐标转换与发布 ────────────────────────────────────

    def _publish_ned_target(self, map_x: float, map_y: float) -> None:
        """map 帧 → NED → /uav/target_position。"""
        msg = Pose()
        msg.position.x = map_x                              # NED north = map.x
        msg.position.y = -map_y                             # NED east = -map.y
        msg.position.z = float(self.takeoff_height)         # NED down
        msg.orientation.w = 1.0
        self.target_pub.publish(msg)

    # ─── 可视化 ────────────────────────────────────────────

    def _publish_path(self, path: list) -> None:
        """发布 nav_msgs/Path。"""
        msg = Path()
        msg.header.frame_id = self.fixed_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_plan_markers(self) -> None:
        """发布起点 + 终点 + 航点标记。"""
        arr = MarkerArray()

        if not self._path_waypoints:
            self.marker_pub.publish(arr)
            return

        # 起点 (绿色球)
        sx, sy = self._path_waypoints[0]
        arr.markers.append(self._make_marker(
            0, Marker.SPHERE, sx, sy, 0.0, GREEN, 0.15, 0.15, 0.15))

        # 终点 (红色箭头)
        gx, gy = self._path_waypoints[-1]
        arr.markers.append(self._make_marker(
            1, Marker.ARROW, gx, gy, 0.15, RED, 0.3, 0.08, 0.08))

        # 航点 (紫色小球)
        for i, (px, py) in enumerate(self._path_waypoints[1:-1]):
            arr.markers.append(self._make_marker(
                2 + i, Marker.SPHERE, px, py, 0.0, PURPLE, 0.06, 0.06, 0.06))

        self.marker_pub.publish(arr)

    def _publish_idle_markers(self) -> None:
        """空闲时清空标记。"""
        arr = MarkerArray()
        arr.markers.append(self._make_marker(
            99, Marker.CUBE, self._drone_map_x, self._drone_map_y,
            0.0, WHITE, 0.05, 0.05, 0.05))
        self.marker_pub.publish(arr)

    def _publish_arrived_markers(self) -> None:
        """到达目标: 金色星星。"""
        arr = MarkerArray()
        if self._path_waypoints:
            gx, gy = self._path_waypoints[-1]
            arr.markers.append(self._make_marker(
                100, Marker.SPHERE, gx, gy, 0.1, GOLD, 0.2, 0.2, 0.2))
        self.marker_pub.publish(arr)
        self.get_logger().info("已到达, 金色标记 (等待新目标)")

    def _make_marker(self, marker_id: int, shape: int,
                     x: float, y: float, z: float,
                     color: ColorRGBA,
                     sx: float, sy: float, sz: float,
                     lifetime_s: float = 0.0) -> Marker:
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'click_navigate'
        m.id = marker_id
        m.type = shape
        m.action = Marker.ADD
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = float(sx)
        m.scale.y = float(sy)
        m.scale.z = float(sz)
        m.color = color
        if lifetime_s > 0.0:
            m.lifetime = rclpy.duration.Duration(seconds=lifetime_s).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ClickNavigateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()