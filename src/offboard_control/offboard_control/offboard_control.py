import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        # 配置 QOS (保持原样)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 仅保留发布器和订阅器，删除了 Service Clients，因为我们现在是被动触发
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            qos_profile
        )
        
        self.state_sub = self.create_subscription(
            State, 
            'mavros/state', 
            self.state_cb, 
            qos_profile
        )
        
        self.current_state = None

        # 定时器：持续发布目标位置 (心跳，频率建议 >= 20Hz)
        # 注意：即使没有进入 OFFBOARD，也必须持续发心跳，否则切换模式会失败
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 目标位置
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.5
        
        self.get_logger().info("Offboard Control Node Started. Waiting for Manual Trigger...")

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        """主逻辑：根据飞控当前状态决定动作"""
        if self.current_state is None:
            return

        # 1. 始终发布目标位置 (心跳保证)
        # PX4 要求进入 OFFBOARD 之前必须收到流动的指令，且在模式中如果停止发送，会触发 Failsafe
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)

        # 2. 逻辑判断
        if not self.current_state.connected:
            self.get_logger().info("Waiting for FCU connection...", throttle_duration_sec=2.0)
            return

        # 检查是否手动解锁
        is_armed = self.current_state.armed
        # 检查是否手动切入 OFFBOARD
        is_offboard = (self.current_state.mode == "OFFBOARD")

        if not is_armed and not is_offboard:
            self.get_logger().info("Ready: Please ARM and set OFFBOARD mode via Remote Controller.", throttle_duration_sec=2.0)
        elif not is_armed:
            self.get_logger().info("Mode is OFFBOARD, waiting for Manual ARMing...", throttle_duration_sec=2.0)
        elif not is_offboard:
            self.get_logger().info("Vehicle ARMED, waiting for OFFBOARD mode switch...", throttle_duration_sec=2.0)
        else:
            # 只有当既解锁又是 OFFBOARD 模式时，才真正开始执行任务逻辑
            self.get_logger().info("System Active: Executing setpoint tracking.", throttle_duration_sec=5.0)
            # 在这里可以添加更复杂的航点切换逻辑

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()