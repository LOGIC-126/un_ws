#!/usr/bin/env python3
"""
双模式机载电脑控制节点

硬编码可选控制模式 (修改 CONTROL_MODE 常量即可切换):
  "position"     — 模式1: 接收期望位置 → 直接向飞控发送位置 setpoint (PX4 内部位置控制器)
  "velocity_pid" — 模式2: 接收期望位置 → 机载 PID 计算期望速度 → 向飞控发送速度 setpoint
                           接收期望速度 → 直接透传给飞控

两种模式共用:
  - /uav/target_position  (geometry_msgs/Pose)  期望位置 + 偏航
  - /uav/target_velocity  (geometry_msgs/Twist)  期望速度 + 偏航角速度
  - 自动起飞/降落/上锁状态机
  - 速度指令限幅保护
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    BatteryStatus,
    VehicleLandDetected
)

# ===========================================================================
# 硬编码控制模式选择
# ===========================================================================
CONTROL_MODE_POSITION = "position"           # PX4 内部位置控制
CONTROL_MODE_VELOCITY_PID = "velocity_pid"   # 机载 PID → 速度控制
# ===========================================================================


class Land_Control(Node):
    """双模式机载控制节点: 位置直传 / 速度PID"""

    # =======================================================================
    # 硬编码模式 — 修改此行切换控制模式
    # =======================================================================
    CONTROL_MODE = CONTROL_MODE_VELOCITY_PID
    # =======================================================================

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        self.get_logger().info(
            f"Control mode: {self.CONTROL_MODE} "
            f"({'FC position control' if self.CONTROL_MODE == CONTROL_MODE_POSITION else 'Onboard PID → velocity'})"
        )

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
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos_profile)
        self.battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1',
            self.battery_status_callback, qos_profile)
        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self.vehicle_land_detected_callback, qos_profile)

        # 外部目标位姿话题 (位置 + 偏航四元数)
        self.target_position_subscriber = self.create_subscription(
            Pose, '/uav/target_position', self.target_position_callback, qos_profile)

        # 速度控制接口
        self.target_velocity_subscriber = self.create_subscription(
            Twist, '/uav/target_velocity', self.target_velocity_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.battery_status = BatteryStatus()
        self.vehicle_land_detected = VehicleLandDetected()

        # 目标位姿 (默认原点, 四元数 w=1 表示偏航角为 0)
        self.target_pose = Pose()
        self.target_pose.orientation.w = 1.0
        self.target_yaw = 0.0  # 独立 yaw 变量 (弧度)

        # 安全标志位
        self.has_target_altitude = False

        # —— 速度控制状态 ——
        self.target_velocity = Twist()
        self.last_velocity_time = self.get_clock().now()
        self.velocity_timeout = 0.5  # 无新指令后回退到位置 PID (秒)
        self.was_velocity_active = False  # 位置→速度激活沿检测

        # 速度限幅
        self.max_horizontal_speed = 2.0   # m/s
        self.max_vertical_speed = 1.0     # m/s
        self.max_yaw_rate = 1.0           # rad/s

        # 安全步进参数配置 (position 模式用)
        self.max_speed = 0.4       # 限制最大期望移动速度 (米/秒)
        self.timer_period = 0.05    # 定时器周期 (秒)
        self.max_step = self.max_speed * self.timer_period

        # ===================================================================
        # PID 控制器参数 (velocity_pid 模式)
        # ===================================================================
        # X (NED North) 轴
        self.kp_x = 0.75
        self.ki_x = 0.07
        self.kd_x = 0.1

        # Y (NED East) 轴
        self.kp_y = 0.75
        self.ki_y = 0.07
        self.kd_y = 0.1

        # Z (NED Down / 高度) 轴 — 高度需更快响应
        self.kp_z = 1.2
        self.ki_z = 0.1
        self.kd_z = 0.15

        # Yaw (偏航) 轴
        self.kp_yaw = 1.0
        self.ki_yaw = 0.03
        self.kd_yaw = 0.05

        # PID 内部状态
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0

        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.last_error_yaw = 0.0

        # 积分饱和上限 (对称限幅)
        self.integral_max_x = self.max_horizontal_speed * 0.3
        self.integral_max_y = self.max_horizontal_speed * 0.3
        self.integral_max_z = self.max_vertical_speed * 0.3
        self.integral_max_yaw = self.max_yaw_rate * 0.3

        # PID 输出上限
        self.max_x_output = self.max_horizontal_speed
        self.max_y_output = self.max_horizontal_speed
        self.max_z_output = self.max_vertical_speed
        self.max_yaw_output = self.max_yaw_rate

        # Create a timer to publish control commands
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ===================================================================
    # 订阅回调 (与模式无关)
    # ===================================================================

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def battery_status_callback(self, battery_status):
        self.battery_status = battery_status

    def vehicle_land_detected_callback(self, vehicle_land_detected):
        self.vehicle_land_detected = vehicle_land_detected

    def target_position_callback(self, msg):
        """外部期望位姿回调."""
        self.target_pose = msg
        self.has_target_altitude = True

    def target_velocity_callback(self, msg):
        """外部期望速度回调 (含安全限幅)."""
        vx = msg.linear.x
        vy = msg.linear.y
        h_speed = math.sqrt(vx * vx + vy * vy)
        if h_speed > self.max_horizontal_speed:
            scale = self.max_horizontal_speed / h_speed
            vx *= scale
            vy *= scale

        vz = msg.linear.z
        if abs(vz) > self.max_vertical_speed:
            vz = self.max_vertical_speed if vz > 0 else -self.max_vertical_speed

        yr = msg.angular.z
        if abs(yr) > self.max_yaw_rate:
            yr = self.max_yaw_rate if yr > 0 else -self.max_yaw_rate

        self.target_velocity.linear.x = vx
        self.target_velocity.linear.y = vy
        self.target_velocity.linear.z = vz
        self.target_velocity.angular.z = yr
        self.last_velocity_time = self.get_clock().now()

    # ===================================================================
    # 指令方法 (与模式无关)
    # ===================================================================

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

    # ===================================================================
    # 心跳 & 发布 (模式感知)
    # ===================================================================

    def publish_offboard_control_heartbeat_signal(self):
        """发布 Offboard 心跳 — 根据 CONTROL_MODE 设置控制字."""
        msg = OffboardControlMode()
        if self.CONTROL_MODE == CONTROL_MODE_VELOCITY_PID:
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
        else:
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
        """向飞控发送位置 setpoint (position 模式)."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float,
                                   yawspeed: float = 0.0):
        """向飞控发送速度 setpoint (velocity_pid 模式)."""
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float(yawspeed)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

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

    # ===================================================================
    # PID 控制器 (velocity_pid 模式)
    # ===================================================================

    def _reset_pid(self):
        """重置所有 PID 状态 (在模式切换或长时间不发指令时调用)."""
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.last_error_yaw = 0.0

    @staticmethod
    def _yaw_error_wrap(target: float, current: float) -> float:
        """偏航角误差, 归一化到 [-pi, pi]."""
        err = target - current
        while err > math.pi:
            err -= 2.0 * math.pi
        while err < -math.pi:
            err += 2.0 * math.pi
        return err

    def _pid_step(self, axis: str, setpoint: float, measurement: float,
                  dt: float) -> float:
        """单轴 PID 步进.

        Args:
            axis: 'x', 'y', 'z', 或 'yaw'.
            setpoint: 期望值.
            measurement: 当前测量值.
            dt: 时间步长 (秒).

        Returns:
            控制输出 (速度或偏航角速度), 已限幅.
        """
        # 偏航轴需要角度 wrapping
        if axis == 'yaw':
            error = self._yaw_error_wrap(setpoint, measurement)
        else:
            error = setpoint - measurement

        # 读取增益
        kp = getattr(self, f'kp_{axis}')
        ki = getattr(self, f'ki_{axis}')
        kd = getattr(self, f'kd_{axis}')

        # 比例项
        p_out = kp * error

        # 积分项 (抗饱和)
        integral = getattr(self, f'integral_{axis}')
        integral += error * dt
        integral_max = getattr(self, f'integral_max_{axis}')
        integral = max(-integral_max, min(integral_max, integral))
        setattr(self, f'integral_{axis}', integral)
        i_out = ki * integral

        # 微分项 (误差微分)
        last_error = getattr(self, f'last_error_{axis}')
        derivative = (error - last_error) / dt if dt > 0.0 else 0.0
        setattr(self, f'last_error_{axis}', error)
        d_out = kd * derivative

        # PID 合成 + 输出限幅
        output = p_out + i_out + d_out
        max_out = getattr(self, f'max_{axis}_output')
        output = max(-max_out, min(max_out, output))

        return output

    # ===================================================================
    # 辅助方法
    # ===================================================================

    def quaternion_to_yaw(self, q) -> float:
        """四元数 → 偏航角 (弧度)."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def path_planner(self, curr_x: float, curr_y: float, curr_z: float,
                     tar_x: float, tar_y: float, tar_z: float):
        """路径规划器 (position 模式用), 根据最大步长进行截断插值."""
        # 当前为直接透传, 可在此处实现步进限幅
        return tar_x, tar_y, tar_z

    # ===================================================================
    # 主定时器回调
    # ===================================================================

    def timer_callback(self) -> None:
        battery_percent = self.battery_status.remaining * 100.0
        is_landed = self.vehicle_land_detected.landed
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        # 判断速度控制是否激活 (最近 timeout 秒内收到过 /uav/target_velocity)
        now = self.get_clock().now()
        velocity_active = (
            in_offboard and is_armed and
            (now - self.last_velocity_time).nanoseconds * 1e-9 < self.velocity_timeout
        )

        # 发布模式感知心跳
        self.publish_offboard_control_heartbeat_signal()

        # 状态日志
        mode_label = 'Vel-PID' if self.CONTROL_MODE == CONTROL_MODE_VELOCITY_PID else 'Pos'
        cmd_label = 'DirectVel' if velocity_active else mode_label
        self.get_logger().info(
            f"--- UAV STATUS --- "
            f"Pos: [X: {self.vehicle_local_position.x:.2f}, "
            f"Y: {self.vehicle_local_position.y:.2f}, "
            f"Z: {self.vehicle_local_position.z:.2f}] | "
            f"Hdg: {self.vehicle_local_position.heading:.2f} | "
            f"Nav State: {self.vehicle_status.nav_state} | "
            f"Bat: {battery_percent:.1f}% | "
            f"Landed: {is_landed} | Armed: {is_armed} | "
            f"Ctrl: {cmd_label}"
        )

        # 建立初始心跳流 (PX4 要求切 Offboard 前有稳定心跳)
        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            return

        # 根据模式分发
        if self.CONTROL_MODE == CONTROL_MODE_VELOCITY_PID:
            self._timer_velocity_pid_mode(
                is_landed, is_armed, in_offboard, velocity_active)
        else:
            self._timer_position_mode(
                is_landed, is_armed, in_offboard, velocity_active)

    # ===================================================================
    # 模式: velocity_pid — 机载 PID → 速度 setpoint
    # ===================================================================

    def _timer_velocity_pid_mode(self, is_landed: bool, is_armed: bool,
                                  in_offboard: bool, velocity_active: bool):
        """Velocity PID 模式定时器逻辑."""
        tar_z = self.target_pose.position.z
        dt = self.timer_period

        # —— 状态机: 自动起飞 / 降落 / 上锁 ——
        if is_landed and not is_armed:
            # 地面未解锁: 收到有效高度 → 自动起飞
            if self.has_target_altitude and abs(tar_z) > 0.1:
                self.get_logger().info(
                    "Ground & Disarmed. Valid altitude → Auto-Takeoff...")
                self._reset_pid()
                self.engage_offboard_mode()
                self.arm()

        elif in_offboard:
            if abs(tar_z) < 0.05:
                # 目标 Z≈0 → 降落
                self.get_logger().warn(
                    "Airborne & Target Z≈0. Triggering Land Mode...")
                self.land()
            else:
                # —— 核心控制 ——
                if velocity_active:
                    # ★ 直接速度透传
                    if not self.was_velocity_active:
                        self._reset_pid()  # 切到透传时清零积分
                        self.was_velocity_active = True

                    self.publish_velocity_setpoint(
                        self.target_velocity.linear.x,
                        self.target_velocity.linear.y,
                        self.target_velocity.linear.z,
                        self.target_velocity.angular.z)

                else:
                    # ★ 位置 PID → 速度
                    self.was_velocity_active = False

                    curr_x = self.vehicle_local_position.x
                    curr_y = self.vehicle_local_position.y
                    curr_z = self.vehicle_local_position.z
                    curr_yaw = self.vehicle_local_position.heading

                    tar_x = self.target_pose.position.x
                    tar_y = self.target_pose.position.y
                    tar_yaw = self.quaternion_to_yaw(self.target_pose.orientation)
                    self.target_yaw = tar_yaw

                    # PID 计算各轴期望速度
                    vx = self._pid_step('x', tar_x, curr_x, dt)
                    vy = self._pid_step('y', tar_y, curr_y, dt)
                    vz = self._pid_step('z', tar_z, curr_z, dt)
                    yawspeed = self._pid_step('yaw', tar_yaw, curr_yaw, dt)

                    self.publish_velocity_setpoint(vx, vy, vz, yawspeed)

        # 着陆后自动上锁
        if is_landed and is_armed and not in_offboard:
            self.get_logger().info("Touched down. Disarming...")
            self._reset_pid()
            self.disarm()

    # ===================================================================
    # 模式: position — 向飞控直发位置 setpoint (原有逻辑)
    # ===================================================================

    def _timer_position_mode(self, is_landed: bool, is_armed: bool,
                              in_offboard: bool, velocity_active: bool):
        """Position 模式定时器逻辑 (保持原有行为)."""
        tar_x = self.target_pose.position.x
        tar_y = self.target_pose.position.y
        tar_z = self.target_pose.position.z
        tar_yaw = self.quaternion_to_yaw(self.target_pose.orientation)
        self.target_yaw = tar_yaw  # 同步 yaw

        # —— 速度控制: Python 侧积分速度 → 位置 setpoint ——
        if velocity_active:
            dt = self.timer_period

            # 位置→速度激活瞬间: 同步 target_pose 到当前位置
            if not self.was_velocity_active:
                self.target_pose.position.x = self.vehicle_local_position.x
                self.target_pose.position.y = self.vehicle_local_position.y
                self.target_pose.position.z = self.vehicle_local_position.z
                self.target_yaw = self.quaternion_to_yaw(
                    self.target_pose.orientation)
                self.was_velocity_active = True

            # 积分速度 → 位置 (NED)
            self.target_pose.position.x += self.target_velocity.linear.x * dt
            self.target_pose.position.y += self.target_velocity.linear.y * dt
            self.target_pose.position.z += self.target_velocity.linear.z * dt
            self.target_yaw += self.target_velocity.angular.z * dt

            self.publish_position_setpoint(
                self.target_pose.position.x,
                self.target_pose.position.y,
                self.target_pose.position.z,
                self.target_yaw)
            return

        self.was_velocity_active = False

        # —— 位置控制 (原有逻辑) ——
        # 自动起飞/降落/上锁状态机
        if is_landed and not is_armed:
            if self.has_target_altitude and abs(tar_z) > 0.1:
                self.get_logger().info(
                    "Ground & Disarmed. Valid altitude → Auto-Takeoff...")
                self.engage_offboard_mode()
                self.arm()

        elif in_offboard:
            if abs(tar_z) < 0.05:
                self.get_logger().warn(
                    "Airborne & Target Z≈0. Triggering Land Mode...")
                self.land()
            else:
                curr_x = self.vehicle_local_position.x
                curr_y = self.vehicle_local_position.y
                curr_z = self.vehicle_local_position.z
                cmd_x, cmd_y, cmd_z = self.path_planner(
                    curr_x, curr_y, curr_z, tar_x, tar_y, tar_z)
                self.publish_position_setpoint(cmd_x, cmd_y, cmd_z, tar_yaw)

        if is_landed and is_armed and not in_offboard:
            self.get_logger().info("Touched down. Disarming...")
            self.disarm()


def main(args=None) -> None:
    print(f'Starting offboard control node (mode: {Land_Control.CONTROL_MODE})...')
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
