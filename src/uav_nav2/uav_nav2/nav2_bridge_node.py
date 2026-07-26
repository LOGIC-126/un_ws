#!/usr/bin/env python3
"""
Nav2 → offboard_control 桥接节点

将 Nav2 DWB 控制器输出的 /cmd_vel (body-frame Twist) 转换为
/uav/target_position (NED Pose), 保持固定高度。

数据流:
  /cmd_vel (Twist, body) → body→map 旋转变换 → +当前位置 → NED 转换 → /uav/target_position

坐标系转换 (与 ekf2_link_dds.py:67-75 一致):
  ROS map 帧: x=北/前, y=西/左, z=上, yaw=CCW+
  PX4 NED:    x=北,    y=东,    z=下, yaw=CW+
  转换: ned_x = ros_x,  ned_y = -ros_y,  ned_z = -ros_z,  ned_yaw = -ros_yaw
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import quaternion_from_euler


class Nav2BridgeNode(Node):
    """Nav2 桥接: /cmd_vel (body) → /uav/target_position (NED Pose)"""

    def __init__(self) -> None:
        super().__init__('nav2_bridge')

        # ——— 参数 ———
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('bridge_update_rate', 50.0)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('fixed_altitude', 1.0)
        self.declare_parameter('max_horizontal_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.update_rate = self.get_parameter('bridge_update_rate').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.fixed_altitude = self.get_parameter('fixed_altitude').value
        self.max_horizontal_speed = self.get_parameter('max_horizontal_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value

        # ——— TF ———
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ——— 状态 ———
        self._lock = threading.Lock()
        self.cmd_vel = Twist()       # 最新 cmd_vel (body 帧)
        self.last_cmd_vel_time = self.get_clock().now()
        self.active = False          # 导航活跃标志
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.has_pose = False        # 是否已获取过位姿

        # ——— QoS ———
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ——— 发布者 ———
        self.target_pub = self.create_publisher(Pose, '/uav/target_position', qos_profile)

        # ——— 订阅者 ———
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)

        # ——— Rviz2 点击目标 → Nav2 action ———
        self._goal_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, qos_profile)

        # ——— 定时器 ———
        self.timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            f"Nav2 bridge started: /cmd_vel → /uav/target_position "
            f"(fixed Z={self.fixed_altitude:.1f}m ENU, {self.update_rate:.0f} Hz) | "
            f"listening /goal_pose → /navigate_to_pose action"
        )

    # =================================================================
    # 回调
    # =================================================================

    def cmd_vel_callback(self, msg: Twist) -> None:
        """接收 Nav2 DWB 输出的 body-frame 速度指令"""
        with self._lock:
            self.cmd_vel = msg
            self.last_cmd_vel_time = self.get_clock().now()
            # 检测是否有非零速度 → 导航活跃
            if abs(msg.linear.x) > 0.001 or abs(msg.linear.y) > 0.001:
                self.active = True

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """Rviz2 Nav2 Goal 点击 → 调用 /navigate_to_pose action"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        if msg.header.frame_id == '':
            goal_msg.pose.header.frame_id = self.map_frame

        self.get_logger().info(
            f"Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) "
            f"→ sending NavigateToPose action"
        )

        send_goal_future = self._goal_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"NavigateToPose action failed: {e}")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("NavigateToPose goal rejected!")
            return
        self.get_logger().info("NavigateToPose goal accepted, Nav2 planning...")

    # =================================================================
    # 主循环 (50 Hz)
    # =================================================================

    def timer_callback(self) -> None:
        """50Hz 定时回调: 将 cmd_vel 转换为 NED 目标位置并发布"""

        # 1. 检查 cmd_vel 超时
        now = self.get_clock().now()
        with self._lock:
            elapsed = (now - self.last_cmd_vel_time).nanoseconds * 1e-9
            if elapsed > self.cmd_vel_timeout:
                self.cmd_vel = Twist()   # 超时 → 零速 (悬停)
                self.active = False

            vx_body = self.cmd_vel.linear.x
            vy_body = self.cmd_vel.linear.y
            vyaw_body = self.cmd_vel.angular.z

        # 2. 速度钳位
        vx_body = self._clamp(vx_body, -self.max_horizontal_speed, self.max_horizontal_speed)
        vy_body = self._clamp(vy_body, -self.max_horizontal_speed, self.max_horizontal_speed)
        vyaw_body = self._clamp(vyaw_body, -self.max_yaw_rate, self.max_yaw_rate)

        # 3. 获取当前位姿 (ROS map 帧)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            t = transform.transform.translation
            r = transform.transform.rotation

            self.current_x = t.x
            self.current_y = t.y
            self.current_z = t.z
            self.has_pose = True

            # 提取 yaw (ROS CCW+)
            siny = 2.0 * (r.w * r.z + r.x * r.y)
            cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            self.current_yaw = math.atan2(siny, cosy)

        except TransformException:
            if not self.has_pose:
                return  # 尚未获取到位姿, 不能发布目标

        # 4. body-frame 速度 → map-frame 位置偏移
        #    ROS: x=前, y=左
        #    vx_body: 前进速度, vy_body: 左移速度(全向)
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        map_vx = vx_body * cy - vy_body * sy
        map_vy = vx_body * sy + vy_body * cy

        # 5. 目标位置 (lookahead)
        target_ros_x = self.current_x + map_vx * self.lookahead_gain
        target_ros_y = self.current_y + map_vy * self.lookahead_gain
        target_ros_z = self.fixed_altitude                       # 固定高度
        target_ros_yaw = self.current_yaw + vyaw_body * self.lookahead_gain

        # 6. ROS → NED 转换 (与 ekf2_link_dds.py 一致)
        ned_x = target_ros_x                                     # NED 北
        ned_y = -target_ros_y                                    # NED 东 (翻转)
        ned_z = -target_ros_z                                    # NED 下 (翻转)
        ned_yaw = -target_ros_yaw                                # NED yaw (翻转方向)

        # 7. 目标 yaw → 四元数
        q_ned = quaternion_from_euler(0.0, 0.0, ned_yaw)

        # 8. 发布 /uav/target_position (NED Pose)
        pose = Pose()
        pose.position.x = ned_x
        pose.position.y = ned_y
        pose.position.z = ned_z
        pose.orientation.x = q_ned[0]
        pose.orientation.y = q_ned[1]
        pose.orientation.z = q_ned[2]
        pose.orientation.w = q_ned[3]
        self.target_pub.publish(pose)

        # 日志 (降低频率: 每 2 秒打印一次)
        if self._should_log():
            status = 'ACTIVE' if self.active else 'IDLE'
            self.get_logger().info(
                f"[{status}] ROS:({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) "
                f"→ NED:({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f}) | "
                f"cmd_vel:({vx_body:.2f}, {vy_body:.2f}, {vyaw_body:.2f})"
            )

    # =================================================================
    # 工具方法
    # =================================================================

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _should_log(self) -> bool:
        """降低日志频率: 每 2 秒打印一次"""
        now = self.get_clock().now()
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = now
            return True
        if (now - self._last_log_time).nanoseconds * 1e-9 >= 2.0:
            self._last_log_time = now
            return True
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
