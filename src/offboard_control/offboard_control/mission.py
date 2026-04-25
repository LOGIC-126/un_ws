import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from enum import Enum
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

# 1. 定义飞行状态枚举
class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    GOTOTAR = 2
    MISSION = 3
    LAND = 4
    DONE = 5

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        # 配置 QOS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 发布器：目标位置
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 
            'mavros/setpoint_position/local', 
            qos_profile
        )
        
        # 订阅器：飞控状态
        self.state_sub = self.create_subscription(
            State, 
            'mavros/state', 
            self.state_cb, 
            qos_profile
        )

        # 订阅器：当前本地位置 (用于计算距离)
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.local_pos_cb,
            qos_profile
        )
        
        # 变量初始化
        self.current_state = None
        self.current_local_pos = None
        self.state = FlightState.INIT
        self.current_mission_index = 0
        self.mission_complete = False
        self.High = 0.5  # 起飞高度
        self.wait_start_time = None  # Wait任务的开始时间戳

        # 任务点列表 (x, y, z, mission_type)
        self.mission_points = [
            (0.0, 0.0, 0.5, "wait"),
            (0.4, -0.4, 0.5, "scan"),
            (0.8, -0.8, 0.5, "wait"),
            (1.2, -1.2, 0.5, "wait"),
        ]

        # 目标位置缓存
        self.target_pose = PoseStamped()
        self.set_target_position(0.0, 0.0, 0.0) 

        # 定时器：20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Offboard State Machine Started (Readability Restored).")

    # --- 回调函数 ---
    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.current_local_pos = msg

    # --- 辅助工具函数 ---
    def set_target_position(self, x, y, z):
        """更新内部目标位置变量"""
        self.target_pose.pose.position.x = float(x)
        self.target_pose.pose.position.y = float(y)
        self.target_pose.pose.position.z = float(z)

    def check_distance(self, x, y, z, threshold):
        """计算当前位置与目标点的距离"""
        if self.current_local_pos is None:
            return False
        
        curr = self.current_local_pos.pose.position
        dist = math.sqrt((curr.x - x)**2 + (curr.y - y)**2 + (curr.z - z)**2)
        return dist < threshold

    def execute_mission(self, mission_type):
        """任务执行逻辑"""
        if mission_type == "wait":        
            return self.Wait()
        if mission_type == "pass":
            return self.Pass()
        if mission_type == "scan":
            return self.Scan()
        if mission_type == "flow":
            return self.Flow()
        self.get_logger().info(f"Executing: {mission_type}", throttle_duration_sec=2.0)
        return True

    def Wait(self, duration=5.0):
            now = self.get_clock().now()

            # 第一次进入，记录开始时间
            if self.wait_start_time is None:
                self.wait_start_time = now
                self.get_logger().info(f"开始倒计时: {duration}秒")
                return False

            # 计算流逝时间
            elapsed = (now - self.wait_start_time).nanoseconds / 1e9

            if elapsed < duration:
                # 时间还没到，返回 False 让状态机保持在 MISSION 状态
                # 由于 timer 还在跑，心跳发布指令依然在执行
                self.get_logger().info(f"正在等待... {elapsed:.1f}s", throttle_duration_sec=1.0)
                return False
            else:
                # 时间到了
                self.get_logger().info("等待结束")
                self.wait_start_time = None  # 清除标志位，供下一个任务点使用
                return True

    # --- 核心逻辑 ---
    def timer_callback(self):
        if self.current_state is None:
            return

        # 始终发布心跳，维持模式切换前提
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)

        if not self.current_state.connected:
            return

        # 安全保护：如果切出 OFFBOARD 或没解锁，状态机重置
        if not self.current_state.armed or self.current_state.mode != "OFFBOARD":
            self.get_logger().info("Wait for ARM and OFFBOARD...", throttle_duration_sec=5.0)
            self.state = FlightState.INIT
            self.state = FlightState.INIT
            self.wait_start_time = None  # 重置 Wait 任务的时间戳
            return

        self.run_state_machine()

    def run_state_machine(self):
        """飞行状态机主逻辑"""
        
        # 1. 初始化阶段
        if self.state == FlightState.INIT:
            self.get_logger().info("Status: READY -> TAKEOFF")
            self.state = FlightState.TAKEOFF

        # 2. 起飞阶段
        elif self.state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.High)
            if self.check_distance(0.0, 0.0, self.High, 0.15):
                self.get_logger().info("Takeoff Complete.")
                self.current_mission_index = 0
                self.state = FlightState.GOTOTAR
        
        # 3. 前往目标点阶段
        elif self.state == FlightState.GOTOTAR:
            if self.current_mission_index < len(self.mission_points):
                # 使用你习惯的元组解包方式
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, _ = point
                
                self.set_target_position(target_x, target_y, target_z)
                
                if self.check_distance(target_x, target_y, target_z, 0.15):
                    self.get_logger().info(f"Reached Target Point {self.current_mission_index + 1}")
                    self.state = FlightState.MISSION
        
        # 4. 执行任务阶段
        elif self.state == FlightState.MISSION:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                _, _, _, mission_type = point
                
                if self.execute_mission(mission_type):
                    self.get_logger().info(f"Mission at point {self.current_mission_index+1} finished")
                    
                    if self.current_mission_index >= len(self.mission_points) - 1:
                        self.get_logger().info("All missions done. Landing...")
                        self.state = FlightState.LAND
                    else:
                        self.current_mission_index += 1
                        self.state = FlightState.GOTOTAR
        
        # 5. 降落阶段
        elif self.state == FlightState.LAND:
            if not self.mission_complete:
                # 获取当前水平位置，原地下降
                curr_x = self.current_local_pos.pose.position.x if self.current_local_pos else 0.0
                curr_y = self.current_local_pos.pose.position.y if self.current_local_pos else 0.0
                self.set_target_position(curr_x, curr_y, 0.0)
                self.mission_complete = True
            
            if self.check_distance(self.target_pose.pose.position.x, 
                                 self.target_pose.pose.position.y, 0.0, 0.15):
                self.get_logger().info("Landed safely.")
                self.state = FlightState.DONE

        # 6. 结束阶段
        elif self.state == FlightState.DONE:
            pass

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