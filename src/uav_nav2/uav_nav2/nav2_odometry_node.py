#!/usr/bin/env python3
"""
Nav2 里程计桥接节点

从 Cartographer 的 TF (map→base_link) 生成 Nav2 所需的 /odom 话题
和 odom→base_link TF 变换。Nav2 局部代价地图和 DWB 控制器依赖 /odom。

坐标系:
  map→odom: 静态 identity (在 launch 中发布)
  odom→base_link: 本节点从 map→base_link 直接映射并发布
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import quaternion_from_euler


class Nav2OdometryNode(Node):
    """TF → /odom 桥接: 从 Cartographer map→base_link 变换生成里程计消息"""

    def __init__(self) -> None:
        super().__init__('nav2_odometry_bridge')

        # 参数声明
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('publish_frequency', 50.0)
        self.declare_parameter('publish_tf', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.frequency = self.get_parameter('publish_frequency').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # TF 监听
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF 广播 (odom → base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 发布者: /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 定时器
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # 上一帧状态 (用于速度数值微分)
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_yaw = None
        self.last_time = None

        self.get_logger().info(
            f"Nav2 odometry bridge started: {self.map_frame}→{self.base_link_frame} → /odom "
            f"({self.frequency} Hz)"
        )

    def timer_callback(self) -> None:
        """50Hz 定时回调: 从 TF 提取位姿, 发布 Odometry"""
        try:
            now = self.get_clock().now()
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            t = transform.transform.translation
            r = transform.transform.rotation

            # 提取 yaw (ROS 约定: CCW+, x=前)
            siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
            cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # ——— 速度数值微分 ———
            dt = (now - self.last_time).nanoseconds * 1e-9 if self.last_time else 0.0
            vx = (t.x - self.last_x) / dt if self.last_x is not None and dt > 0.001 else 0.0
            vy = (t.y - self.last_y) / dt if self.last_y is not None and dt > 0.001 else 0.0
            vz = (t.z - self.last_z) / dt if self.last_z is not None and dt > 0.001 else 0.0
            vyaw = (yaw - self.last_yaw) / dt if self.last_yaw is not None and dt > 0.001 else 0.0

            # ——— 发布 Odometry ———
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_link_frame

            odom.pose.pose.position.x = t.x
            odom.pose.pose.position.y = t.y
            odom.pose.pose.position.z = t.z
            odom.pose.pose.orientation = r

            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.linear.z = vz
            odom.twist.twist.angular.z = vyaw

            # 协方差 (Cartographer 约 2cm 位置精度, 0.1 m/s 速度不确定性)
            odom.pose.covariance = [
                0.0004, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0004, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0004,
            ]
            odom.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
            ]

            self.odom_pub.publish(odom)

            # ——— 广播 odom→base_link TF ———
            if self.publish_tf:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = now.to_msg()
                tf_msg.header.frame_id = self.odom_frame
                tf_msg.child_frame_id = self.base_link_frame
                tf_msg.transform.translation.x = t.x
                tf_msg.transform.translation.y = t.y
                tf_msg.transform.translation.z = t.z
                tf_msg.transform.rotation = r
                self.tf_broadcaster.sendTransform(tf_msg)

            # 保存状态
            self.last_x = t.x
            self.last_y = t.y
            self.last_z = t.z
            self.last_yaw = yaw
            self.last_time = now

        except TransformException:
            pass  # TF 尚未就绪, 跳过本帧


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
