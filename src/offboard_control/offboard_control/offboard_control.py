#!/usr/bin/env python3
"""
双模式机载电脑控制节点 (已集成速度期望平滑滤波器)

硬编码可选控制模式 (修改 CONTROL_MODE 常量即可切换):
  "position"     — 模式1: 接收期望位置 → 直接向飞控发送位置 setpoint (PX4 内部位置控制器)
  "velocity_pid" — 模式2: 接收期望位置 → 机载 PID 计算期望速度 → 向飞控发送速度 setpoint
                           接收期望速度 → 直接透传给飞控
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
    """双模式机载控制节点: 位置直传 / 速度PID (含一阶低通滤波与加速度限幅)"""

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

        # Configure QoS profile
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

        self.target_position_subscriber = self.create_subscription(
            Pose, '/uav/target_position', self.target_position_callback, qos_profile)
        self.target_velocity_subscriber = self.create_subscription(
            Twist, '/uav/target_velocity', self.target_velocity_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.battery_status = BatteryStatus()
        self.vehicle_land_detected = VehicleLandDetected()

        self.target_pose = Pose()
        self.target_pose.orientation.w = 1.0
        self.target_yaw = 0.0

        self.has_target_altitude = False

        # —— 速度控制状态 ——
        self.target_velocity = Twist()
        self.last_velocity_time = self.get_clock().now()
        self.velocity_timeout = 0.5
        self.was_velocity_active = False

        # 速度极限幅值
        self.max_horizontal_speed = 4.0   # m/s
        self.max_vertical_speed = 3.0     # m/s
        self.max_yaw_rate = 1.0           # rad/s

        self.max_speed = 0.4
        self.timer_period = 0.02          # 50Hz 控制周期
        self.max_step = self.max_speed * self.timer_period

        # ===================================================================
        # 【新增】平滑滤波器与物理限制参数 (解决蓝线剧烈抖动与尖峰)
        # ===================================================================
        # 1. 一阶低通滤波器截止频率 (Hz)，值越小滤波越强，但会带来微小延迟。建议 2.0 - 5.0 Hz
        self.lpf_cutoff_freq = 6.0 
        
        # 2. 最大允许加速度限制 (斜率限制)，防止出现瞬间垂直跳变的尖峰指令
        self.max_accel_horizontal = 5.0   # m/s^2 
        self.max_accel_vertical = 4.0     # m/s^2
        self.max_accel_yaw = 1.5          # rad/s^2

        # 3. 滤波器上一次的输出状态值 (初始化为0)
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0
        self.filtered_yawspeed = 0.0

        # ===================================================================
        # PID 控制器参数 (velocity_pid 模式)
        # ===================================================================
        self.kp_x = 1.2
        self.ki_x = 0.05
        self.kd_x = 0.45

        self.kp_y = 1.2
        self.ki_y = 0.05
        self.kd_y = 0.45

        self.kp_z = 1.2
        self.ki_z = 0.1
        self.kd_z = 0.15

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

        # 积分饱和上限
        self.integral_max_x = self.max_horizontal_speed * 0.3
        self.integral_max_y = self.max_horizontal_speed * 0.3
        self.integral_max_z = self.max_vertical_speed * 0.3
        self.integral_max_yaw = self.max_yaw_rate * 0.3

        self.max_x_output = self.max_horizontal_speed
        self.max_y_output = self.max_horizontal_speed
        self.max_z_output = self.max_vertical_speed
        self.max_yaw_output = self.max_yaw_rate

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ===================================================================
    # 订阅回调与基础指令方法 (保持不变)
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
        self.target_pose = msg
        self.has_target_altitude = True

    def target_velocity_callback(self, msg):
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

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
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
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yawspeed: float = 0.0):
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
    # PID 控制器与核心平滑滤波算法
    # ===================================================================

    def _reset_pid(self):
        """重置所有 PID 状态与滤波器历史状态."""
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.last_error_yaw = 0.0
        
        # 同步清空滤波器状态，防止切回时产生跃变
        self.filtered_vx = self.vehicle_local_position.vx if hasattr(self.vehicle_local_position, 'vx') else 0.0
        self.filtered_vy = self.vehicle_local_position.vy if hasattr(self.vehicle_local_position, 'vy') else 0.0
        self.filtered_vz = self.vehicle_local_position.vz if hasattr(self.vehicle_local_position, 'vz') else 0.0
        self.filtered_yawspeed = 0.0

    @staticmethod
    def _yaw_error_wrap(target: float, current: float) -> float:
        err = target - current
        while err > math.pi: err -= 2.0 * math.pi
        while err < -math.pi: err += 2.0 * math.pi
        return err

    def _pid_step(self, axis: str, setpoint: float, measurement: float, dt: float) -> float:
        if axis == 'yaw':
            error = self._yaw_error_wrap(setpoint, measurement)
        else:
            error = setpoint - measurement

        kp = getattr(self, f'kp_{axis}')
        ki = getattr(self, f'ki_{axis}')
        kd = getattr(self, f'kd_{axis}')

        p_out = kp * error

        integral = getattr(self, f'integral_{axis}')
        integral += error * dt
        integral_max = getattr(self, f'integral_max_{axis}')
        integral = max(-integral_max, min(integral_max, integral))
        setattr(self, f'integral_{axis}', integral)
        i_out = ki * integral

        last_error = getattr(self, f'last_error_{axis}')
        derivative = (error - last_error) / dt if dt > 0.0 else 0.0
        setattr(self, f'last_error_{axis}', error)
        d_out = kd * derivative

        output = p_out + i_out + d_out
        max_out = getattr(self, f'max_{axis}_output')
        output = max(-max_out, min(max_out, output))

        return output

    # ===================================================================
    # 【新增】双重平滑处理函数 (RC低通滤波 + 斜率限制)
    # ===================================================================
    def _smooth_velocity_setpoints(self, raw_vx: float, raw_vy: float, raw_vz: float, raw_yawspeed: float, dt: float):
        """
        对输入的原始速度期望值进行斜率限制（加速度限制）和一阶低通滤波
        """
        if dt <= 0.0:
            return raw_vx, raw_vy, raw_vz, raw_yawspeed

        # ---- 1. 斜率限制 (Slew Rate Limit / 加速度限制) ----
        max_step_h = self.max_accel_horizontal * dt
        max_step_v = self.max_accel_vertical * dt
        max_step_y = self.max_accel_yaw * dt

        # 对一阶输入做跨步限制，消除断点引发的巨大突变脉冲
        limited_vx = max(self.filtered_vx - max_step_h, min(self.filtered_vx + max_step_h, raw_vx))
        limited_vy = max(self.filtered_vy - max_step_h, min(self.filtered_vy + max_step_h, raw_vy))
        limited_vz = max(self.filtered_vz - max_step_v, min(self.filtered_vz + max_step_v, raw_vz))
        limited_yawspeed = max(self.filtered_yawspeed - max_step_y, min(self.filtered_yawspeed + max_step_y, raw_yawspeed))

        # ---- 2. 一阶低通滤波器 (Low-Pass Filter) ----
        # 计算滤波系数 alpha = dt / (RC + dt) = dt / (1/(2*pi*fc) + dt)
        rc = 1.0 / (2.0 * math.pi * self.lpf_cutoff_freq)
        alpha = dt / (rc + dt)
        
        # 严格限制 alpha 在合理的开区间
        alpha = max(0.01, min(1.0, alpha))

        # 迭代滤波状态，消除高频锯齿
        self.filtered_vx = (1.0 - alpha) * self.filtered_vx + alpha * limited_vx
        self.filtered_vy = (1.0 - alpha) * self.filtered_vy + alpha * limited_vy
        self.filtered_vz = (1.0 - alpha) * self.filtered_vz + alpha * limited_vz
        self.filtered_yawspeed = (1.0 - alpha) * self.filtered_yawspeed + alpha * limited_yawspeed

        return self.filtered_vx, self.filtered_vy, self.filtered_vz, self.filtered_yawspeed

    def quaternion_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def path_planner(self, curr_x: float, curr_y: float, curr_z: float,
                     tar_x: float, tar_y: float, tar_z: float):
        return tar_x, tar_y, tar_z

    # ===================================================================
    # 主定时器回调与分发
    # ===================================================================
    def timer_callback(self) -> None:
        battery_percent = self.battery_status.remaining * 100.0
        is_landed = self.vehicle_land_detected.landed
        is_armed = self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        in_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        now = self.get_clock().now()
        velocity_active = (
            in_offboard and is_armed and
            (now - self.last_velocity_time).nanoseconds * 1e-9 < self.velocity_timeout
        )

        self.publish_offboard_control_heartbeat_signal()

        mode_label = 'Vel-PID' if self.CONTROL_MODE == CONTROL_MODE_VELOCITY_PID else 'Pos'
        cmd_label = 'DirectVel' if velocity_active else mode_label
        self.get_logger().info(
            f"--- UAV STATUS --- "
            f"Pos: [X: {self.vehicle_local_position.x:.2f}, Y: {self.vehicle_local_position.y:.2f}, Z: {self.vehicle_local_position.z:.2f}] | "
            f"Nav State: {self.vehicle_status.nav_state} | "
            f"Ctrl: {cmd_label}"
        )

        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            return

        if self.CONTROL_MODE == CONTROL_MODE_VELOCITY_PID:
            self._timer_velocity_pid_mode(is_landed, is_armed, in_offboard, velocity_active)
        else:
            self._timer_position_mode(is_landed, is_armed, in_offboard, velocity_active)

    # ===================================================================
    # 核心修改：在发送给飞控前，对计算出的速度进行平滑拦截
    # ===================================================================
    def _timer_velocity_pid_mode(self, is_landed: bool, is_armed: bool,
                                  in_offboard: bool, velocity_active: bool):
        tar_z = self.target_pose.position.z
        dt = self.timer_period

        if is_landed and not is_armed:
            if self.has_target_altitude and abs(tar_z) > 0.1:
                self.get_logger().info("Ground & Disarmed. Valid altitude → Auto-Takeoff...")
                self._reset_pid()
                self.engage_offboard_mode()
                self.arm()

        elif in_offboard:
            if abs(tar_z) < 0.05:
                self.get_logger().warn("Airborne & Target Z≈0. Triggering Land Mode...")
                self.land()
            else:
                # —— 核心控制分发 ——
                if velocity_active:
                    # 1. 外部直接速度透传模式
                    if not self.was_velocity_active:
                        self._reset_pid()
                        self.was_velocity_active = True

                    raw_vx = self.target_velocity.linear.x
                    raw_vy = self.target_velocity.linear.y
                    raw_vz = self.target_velocity.linear.z
                    raw_yr = self.target_velocity.angular.z
                else:
                    # 2. 机载位置 PID 计算期望速度模式
                    self.was_velocity_active = False

                    curr_x = self.vehicle_local_position.x
                    curr_y = self.vehicle_local_position.y
                    curr_z = self.vehicle_local_position.z
                    curr_yaw = self.vehicle_local_position.heading

                    tar_x = self.target_pose.position.x
                    tar_y = self.target_pose.position.y
                    tar_yaw = self.quaternion_to_yaw(self.target_pose.orientation)
                    self.target_yaw = tar_yaw

                    # 原始突变/带噪 PID 期望输出
                    raw_vx = self._pid_step('x', tar_x, curr_x, dt)
                    raw_vy = self._pid_step('y', tar_y, curr_y, dt)
                    raw_vz = self._pid_step('z', tar_z, curr_z, dt)
                    raw_yr = self._pid_step('yaw', tar_yaw, curr_yaw, dt)

                # ===========================================================
                # 【拦截点】不管哪种速度源，发送前均进行物理平滑与滤波处理
                # ===========================================================
                smooth_vx, smooth_vy, smooth_vz, smooth_yr = self._smooth_velocity_setpoints(
                    raw_vx, raw_vy, raw_vz, raw_yr, dt
                )

                # 发布平滑后的蓝线指令给飞控
                self.publish_velocity_setpoint(smooth_vx, smooth_vy, smooth_vz, smooth_yr)

        if is_landed and is_armed and not in_offboard:
            self.get_logger().info("Touched down. Disarming...")
            self._reset_pid()
            self.disarm()

    def _timer_position_mode(self, is_landed: bool, is_armed: bool,
                             in_offboard: bool, velocity_active: bool):
        # 保持原有行为不变
        tar_x = self.target_pose.position.x
        tar_y = self.target_pose.position.y
        tar_z = self.target_pose.position.z
        tar_yaw = self.quaternion_to_yaw(self.target_pose.orientation)
        self.target_yaw = tar_yaw

        if velocity_active:
            dt = self.timer_period
            if not self.was_velocity_active:
                self.target_pose.position.x = self.vehicle_local_position.x
                self.target_pose.position.y = self.vehicle_local_position.y
                self.target_pose.position.z = self.vehicle_local_position.z
                self.target_yaw = self.quaternion_to_yaw(self.target_pose.orientation)
                self.was_velocity_active = True

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

        if is_landed and not is_armed:
            if self.has_target_altitude and abs(tar_z) > 0.1:
                self.get_logger().info("Ground & Disarmed. Valid altitude → Auto-Takeoff...")
                self.engage_offboard_mode()
                self.arm()

        elif in_offboard:
            if abs(tar_z) < 0.05:
                self.get_logger().warn("Airborne & Target Z≈0. Triggering Land Mode...")
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