#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose
from px4_msgs.msg import (
    OffboardControlMode, 
    TrajectorySetpoint, 
    VehicleCommand, 
    VehicleLocalPosition, 
    VehicleStatus,
    BatteryStatus,
    VehicleLandDetected
)


class Land_Control(Node):
    """Node for controlling a vehicle in offboard mode with step-input smoothing."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2', self.vehicle_status_callback, qos_profile)
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', self.battery_status_callback, qos_profile)
        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.vehicle_land_detected_callback, qos_profile)
        
        # 修改：订阅外部目标位姿话题（包含位置和偏航角信息）
        self.target_position_subscriber = self.create_subscription(
            Pose, '/uav/target_position', self.target_position_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.battery_status = BatteryStatus()
        self.vehicle_land_detected = VehicleLandDetected()
        
        # 修改：初始化目标位姿（默认停留在原点，四元数 w=1 表示偏航角为 0）
        self.target_pose = Pose()
        self.target_pose.orientation.w = 1.0

        # 安全标志位
        self.has_target_altitude = False 

        # 安全步进参数配置
        self.max_speed = 0.4       # 限制最大期望移动速度 (米/秒)
        self.timer_period = 0.1    # 定时器周期 (秒)
        self.max_step = self.max_speed * self.timer_period 

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def battery_status_callback(self, battery_status):
        self.battery_status = battery_status

    def vehicle_land_detected_callback(self, vehicle_land_detected):
        self.vehicle_land_detected = vehicle_land_detected

    def target_position_callback(self, msg):
        """Callback to update target pose from external topic."""
        self.target_pose = msg
        self.has_target_altitude = True

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """修改：支持动态传入偏航角（Yaw）"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw  # 传入解析后的偏航角（弧度）
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info("Published position setpoint: [X: {:.2f}, Y: {:.2f}, Z: {:.2f}, Yaw: {:.2f} rad]".format(x, y, z, yaw))

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def quaternion_to_yaw(self, q) -> float:
        """核心新增：将 ROS2 Pose 中的四元数转换为 PX4 所需的航向角 Yaw (弧度)"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def path_planner(self, curr_x: float, curr_y: float, curr_z: float, 
                     tar_x: float, tar_y: float, tar_z: float):
        """优化需求1：独立的路径规划器函数，根据最大步长限制进行截断插值"""

        return tar_x, tar_y, tar_z

    def timer_callback(self) -> None:
        # 始终持续发布 Offboard 心跳信号
        self.publish_offboard_control_heartbeat_signal()

        battery_percent = self.battery_status.remaining * 100.0
        is_landed = self.vehicle_land_detected.landed
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

        # 打印无人机状态
        self.get_logger().info(
            f"--- UAV STATUS --- "
            f"Pos: [X: {self.vehicle_local_position.x:.2f}, Y: {self.vehicle_local_position.y:.2f}, Z: {self.vehicle_local_position.z:.2f}] | "
            f"Nav State: {self.vehicle_status.nav_state} | "
            f"Bat: {battery_percent:.1f}% | "
            f"Landed: {is_landed} | Armed: {is_armed}"
        )

        # 建立初始心跳流（PX4 要求切 Offboard 前必须有稳定的心跳）
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            return

        # 提取当前目标点的位置与偏航角
        tar_x = self.target_pose.position.x
        tar_y = self.target_pose.position.y
        tar_z = self.target_pose.position.z
        tar_yaw = self.quaternion_to_yaw(self.target_pose.orientation)

        # 自动化起飞与降落/上锁逻辑状态机
        if is_landed and not is_armed:
            # 在地面上锁时，若收到有效高度目标（非0），则触发自动起飞
            if self.has_target_altitude and abs(tar_z) > 0.1:
                self.get_logger().info("Ground & Disarmed. Valid altitude received, triggering Auto-Takeoff...")
                self.engage_offboard_mode()
                self.arm()
        
        elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # 如果在空中处于 Offboard 模式下，且收到 Z 轴目标为 0，则触发降落
            if abs(tar_z) < 0.1:
                self.get_logger().warn("Airborne & Target Z is close to 0. Triggering Land Mode...")
                self.land()
            else:
                # 正常 Offboard 连续飞行状态：运行路径规划器
                curr_x = self.vehicle_local_position.x
                curr_y = self.vehicle_local_position.y
                curr_z = self.vehicle_local_position.z

                # 调用独立的路径规划器
                cmd_x, cmd_y, cmd_z = self.path_planner(curr_x, curr_y, curr_z, tar_x, tar_y, tar_z)

                # 发布平滑处理后的设定点及外部更改后的偏航角
                self.publish_position_setpoint(cmd_x, cmd_y, cmd_z, tar_yaw)

        # 如果无人机已经着陆且依然处于解锁状态，则自动上锁
        if is_landed and is_armed and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Vehicle has touched down. Disarming automatically...")
            self.disarm()


def main(args=None) -> None:
    print('Starting optimized offboard control node...')
    rclpy.init(args=args)
    offboard_control = Land_Control()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)