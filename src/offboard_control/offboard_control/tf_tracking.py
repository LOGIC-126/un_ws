#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF 小车追踪节点 (模式1 + 模式2 非阻塞等待)

通过 TF2 监听小车 map→car_base_link 变换，计算小车相对无人机的 NED 偏移，
驱动无人机追踪小车并执行任务。

状态机:
  模式1: INIT → TAKEOFF → WAIT ⇄ TRACK ⇄ LOST → DROP → RTH → LAND → DONE
  模式2: INIT → (FS_M2.TAKEOFF) → TRACKLAND → FLIGHT_AGINE → LEAVE_CAR → RTH → LAND → DONE
"""

import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum

import tf2_ros
from tf2_ros import TransformException
from tf2_msgs.msg import TFMessage

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose


# ====== 状态枚举 ======
class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    WAIT = 2
    TRACK = 3
    LOST = 4
    DROP = 5
    RTH = 6
    LAND = 7
    DONE = 8

class FS_M2(Enum):
    TAKEOFF = 0
    TRACKLAND = 1
    FLIGHT_AGINE = 2
    LEAVE_CAR = 3
    RTH = 4
    LAND = 5


class TFTrackingNode(Node):
    def __init__(self):
        super().__init__('tf_tracking_node')

        # ====== 参数 ======
        self.declare_parameter('takeoff_height', -1.2)
        self.declare_parameter('arrival_threshold', 0.05)
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('lost_timeout', 2.0)
        self.declare_parameter('search_timeout', 10.0)
        self.declare_parameter('drop_distance_threshold', 0.5)
        self.declare_parameter('drop_dwell_time', 3.0)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('drone_frame', 'base_link')
        self.declare_parameter('car_frame', 'base_footprint')

        self.declare_parameter('car.offset_x', 0.6)
        self.declare_parameter('car.offset_y', -0.36)

        self.declare_parameter('max_distance', 15.0)
        self.declare_parameter('close_descent', 0.25)
        self.declare_parameter('rth_offset_x', -0.15)
        self.declare_parameter('rth_offset_y', 0.0)

        # 读取参数
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.confirm_frames = self.get_parameter('confirm_frames').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.drop_distance_threshold = self.get_parameter('drop_distance_threshold').value
        self.drop_dwell_time = self.get_parameter('drop_dwell_time').value
        self.close_descent = self.get_parameter('close_descent').value
        self.rth_offset_x = self.get_parameter('rth_offset_x').value
        self.rth_offset_y = self.get_parameter('rth_offset_y').value

        self.drop_height = -0.5
        self.rth_delay = 5.0
        self.land_low_height = -0.3

        self.map_frame = self.get_parameter('map_frame').value
        self.drone_frame = self.get_parameter('drone_frame').value
        self.car_frame = self.get_parameter('car_frame').value
        self.car_offset_x = self.get_parameter('car.offset_x').value
        self.car_offset_y = self.get_parameter('car.offset_y').value
        self.max_distance = self.get_parameter('max_distance').value

        # TF2 监听
        self.tf_buffer = tf2_ros.Buffer()
        for topic in ('/tf', '/tf_static', '/car/tf', '/car/tf_static'):
            self.create_subscription(
                TFMessage, topic,
                lambda msg: [self.tf_buffer.set_transform(t, 'default_authority')
                             for t in msg.transforms],
                100)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 发布器
        self.target_position_pub = self.create_publisher(Pose, '/uav/target_position', qos)
        self.drop_complete_pub = self.create_publisher(Int32, '/car/drop_complete', 10)
        self.car_resume_pub = self.create_publisher(Int32, '/car/resume', 10)

        # 订阅器
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v2',
            self.vehicle_status_callback, qos)

        self._car_mode = 0
        self.car_trigger_sub = self.create_subscription(
            Int32, '/car/trigger', self._car_trigger_callback, 10)

        # 状态变量
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.state = FlightState.INIT
        self._car_ned = None
        self._car_detection_count = 0
        self._locked_target = None
        self._lost_start_time = None
        self._search_start_time = None
        self._wait_start_time = None
        self._drop_start_time = None
        self._drop_enter_time = None
        self._drop_servo_done = False
        self._land_stage = 0
        self._car_resume_sent = False

        # 模式2 专用
        self._flight_again_start = None      # 非阻塞等待计时器

        # 舵机
        self.servo_serial = None
        try:
            self.servo_serial = serial.Serial('/dev/ttyS0', 115200, timeout=0.1)
            self.get_logger().info('舵机串口 /dev/ttyS0@115200 已打开')
        except serial.SerialException as e:
            self.get_logger().warn(f'舵机串口打开失败: {e}')

        self._hover_x = 0.0
        self._hover_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("TF追踪节点已启动 | 模式1+模式2 | 非阻塞等待已启用")

    # ==================== 回调 ====================
    def vehicle_local_position_callback(self, msg): self.vehicle_local_position = msg
    def vehicle_status_callback(self, msg): self.vehicle_status = msg

    def _car_trigger_callback(self, msg):
        if msg.data == 1 and self._car_mode == 0:
            self._car_mode = 1
            self.get_logger().info('收到触发 → 模式1（跟踪抛投）')
        elif msg.data == 2 and self._car_mode == 0:
            self._car_mode = 2
            self.state = FS_M2.TAKEOFF        # 立即切换到模式2初始状态
            self.get_logger().info('收到触发 → 模式2（跟踪起降）')

    # ==================== 舵机 ====================
    def _servo_cmd(self, data):
        if self.servo_serial is None or not self.servo_serial.is_open: return
        try:
            self.servo_serial.write(data)
            self.get_logger().info(f'舵机: {data.hex(" ").upper()}')
        except serial.SerialException as e:
            self.get_logger().warn(f'舵机失败: {e}')

    def _servo_drop(self):
        self.get_logger().info('抛投: 正向+60°')
        self._servo_cmd(bytes([0xA5, 0x01, 0xA6]))
        time.sleep(0.5)
        self.get_logger().info('抛投: 反向-60°')
        self._servo_cmd(bytes([0xA5, 0x02, 0xA7]))
        time.sleep(0.5)
        self.get_logger().info('抛投完成')

    # ==================== TF → NED ====================
    def _lookup_car_ned(self):
        try:
            drone_tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.drone_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            drone_map_x = drone_tf.transform.translation.x
            drone_map_y = drone_tf.transform.translation.y
            car_tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.car_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            car_map_x = car_tf.transform.translation.x + self.car_offset_x
            car_map_y = car_tf.transform.translation.y + self.car_offset_y
        except TransformException as e:
            self.get_logger().warn(f"TF查询失败: {e}", throttle_duration_sec=3.0)
            return None
        delta_x = car_map_x - drone_map_x
        delta_y = car_map_y - drone_map_y
        drone = self.vehicle_local_position
        if math.isnan(drone.x) or math.isnan(drone.y): return None
        return (drone.x + delta_x, drone.y - delta_y)

    def _get_target_ned(self):
        car = self._lookup_car_ned()
        if car is None: return None
        drone = self.vehicle_local_position
        if math.isnan(drone.x) or math.isnan(drone.y): return None
        dx = car[0] - drone.x; dy = car[1] - drone.y
        dist = math.hypot(dx, dy)
        if dist > self.max_distance: return None
        return (car[0], car[1], dist)

    # ==================== 发布 ====================
    def publish_target_position(self):
        msg = Pose()
        msg.position.x = float(self.target_x)
        msg.position.y = float(self.target_y)
        msg.position.z = float(self.target_z)
        msg.orientation.w = math.cos(self.target_yaw / 2.0)
        msg.orientation.z = math.sin(self.target_yaw / 2.0)
        self.target_position_pub.publish(msg)

    def set_target_position(self, x, y, z, yaw=0.0):
        fx, fy, fz, fyaw = float(x), float(y), float(z), float(yaw)
        if fx == self.target_x and fy == self.target_y and fz == self.target_z and fyaw == self.target_yaw: return
        self.target_x, self.target_y, self.target_z, self.target_yaw = fx, fy, fz, fyaw
        self.publish_target_position()

    def check_arrived(self, x, y, z):
        pos = self.vehicle_local_position
        if math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z): return False
        return math.hypot(pos.x - x, pos.y - y, pos.z - z) < self.arrival_threshold

    def check_arrived_2d(self, x, y):
        pos = self.vehicle_local_position
        if math.isnan(pos.x) or math.isnan(pos.y): return False
        return math.hypot(pos.x - x, pos.y - y) < self.arrival_threshold

    # ==================== 辅助 ====================
    def _reset_tracking_counters(self):
        self._car_detection_count = 0; self._lost_start_time = None; self._search_start_time = None

    def _record_hover_position(self):
        drone = self.vehicle_local_position
        if not math.isnan(drone.x) and not math.isnan(drone.y): self._hover_x, self._hover_y = drone.x, drone.y

    # ==================== 主循环 ====================
    def timer_callback(self):
        self.run_state_machine()
        self._print_status()

    def _print_status(self):
        now = self.get_clock().now()
        if not hasattr(self, '_last_log_time'): self._last_log_time = now; return
        if (now - self._last_log_time).nanoseconds * 1e-9 < 2.0: return
        self._last_log_time = now
        drone = self.vehicle_local_position
        state_name = self.state.name if hasattr(self.state, 'name') else str(self.state)
        car = self._lookup_car_ned()
        if car is not None:
            dx = car[0] - drone.x; dy = car[1] - drone.y; dist = math.hypot(dx, dy)
            self.get_logger().info(
                f"[{state_name}] 无人机({drone.x:.2f},{drone.y:.2f},{drone.z:.2f}) | "
                f"小车({car[0]:.2f},{car[1]:.2f}) | 距离={dist:.2f}m")
        else:
            self.get_logger().info(
                f"[{state_name}] 无人机({drone.x:.2f},{drone.y:.2f},{drone.z:.2f}) | 小车:未检测到")

    # ==================== 状态机顶层分发 ====================
    def run_state_machine(self):
        # 公共状态：INIT
        if self.state == FlightState.INIT:
            self.set_target_position(0.0, 0.0, 0.0)
            if self._car_mode == 1:
                self.get_logger().info("INIT → TAKEOFF (模式1)")
                self.set_target_position(0.0, 0.0, self.takeoff_height)
                self.state = FlightState.TAKEOFF
            elif self._car_mode == 2:
                # 模式2已在回调中设置 FS_M2.TAKEOFF，这里无需处理
                pass
            else:
                self.get_logger().info("等待小车触发...", throttle_duration_sec=3.0)

        # 公共状态：DONE
        elif self.state == FlightState.DONE:
            self._car_mode = 0
            self.get_logger().info('[DONE] 等待锁桨...', throttle_duration_sec=3.0)
            self.set_target_position(self.rth_offset_x, self.rth_offset_y, 0.0)

        else:
            if self._car_mode == 1 and isinstance(self.state, FlightState):
                self._run_mode1_state_machine()
            elif self._car_mode == 2 and isinstance(self.state, FS_M2):
                self._run_mode2_state_machine()
            else:
                self.get_logger().warn('状态异常，强制回 INIT')
                self.state = FlightState.INIT

    # ==================== 模式1 状态机（不变） ====================
    def _run_mode1_state_machine(self):
        if self.state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.takeoff_height)
            if self.check_arrived(0.0, 0.0, self.takeoff_height):
                self.get_logger().info("TAKEOFF → WAIT (悬停2s)")
                self._record_hover_position()
                self._reset_tracking_counters()
                self._locked_target = None
                self._wait_start_time = self.get_clock().now()
                self.car_resume_pub.publish(Int32(data=0))
                self.state = FlightState.WAIT

        elif self.state == FlightState.WAIT:
            self.set_target_position(self._hover_x, self._hover_y, self.takeoff_height)
            if self._wait_start_time is not None:
                hover_elapsed = (self.get_clock().now() - self._wait_start_time).nanoseconds * 1e-9
                if hover_elapsed < 2.0:
                    self.get_logger().info(f'[WAIT] 悬停 {hover_elapsed:.1f}s/2.0s...', throttle_duration_sec=1.0)
                    return
                self._wait_start_time = None
            target = self._get_target_ned()
            self._car_detection_count = self._car_detection_count + 1 if target else 0
            if self._car_detection_count >= self.confirm_frames:
                target = self._get_target_ned()
                if target is not None:
                    ned_x, ned_y, dist = target
                    self._locked_target = (ned_x, ned_y)
                    self.get_logger().info(f"WAIT → TRACK (锁定 dist={dist:.2f}m)")
                    self._reset_tracking_counters()
                    self._car_resume_sent = False
                    self.state = FlightState.TRACK

        elif self.state == FlightState.TRACK:
            target = self._get_target_ned()
            if target is not None:
                ned_x, ned_y, dist = target
                self._locked_target = (ned_x, ned_y)
                if self._lost_start_time is not None:
                    self.get_logger().info("目标重新出现"); self._lost_start_time = None
                close_ratio = max(0.0, 1.0 - dist / self.drop_distance_threshold)
                track_z = self.takeoff_height + self.close_descent * close_ratio
                self.set_target_position(ned_x, ned_y, track_z)
                if dist < self.drop_distance_threshold:
                    if self._drop_start_time is None:
                        self._drop_start_time = self.get_clock().now()
                        if not self._car_resume_sent:
                            self.car_resume_pub.publish(Int32(data=1))
                            self._car_resume_sent = True
                            self.get_logger().info(f'模式1 接近小车 dist={dist:.2f}m: 发car_resume=1')
                    else:
                        dwell = (self.get_clock().now() - self._drop_start_time).nanoseconds * 1e-9
                        if dwell >= self.drop_dwell_time:
                            self.get_logger().info(f'TRACK → DROP')
                            self.car_resume_pub.publish(Int32(data=2))
                            self._drop_start_time = None
                            self._drop_enter_time = self.get_clock().now()
                            self.state = FlightState.DROP; return
                else:
                    self._drop_start_time = None
            else:
                now = self.get_clock().now()
                if self._lost_start_time is None: self._lost_start_time = now
                if (now - self._lost_start_time).nanoseconds * 1e-9 >= self.lost_timeout:
                    self.get_logger().info("TRACK → LOST")
                    self._reset_tracking_counters(); self._search_start_time = now; self.state = FlightState.LOST
                elif self._locked_target is not None:
                    self.set_target_position(*self._locked_target, self.takeoff_height)

        elif self.state == FlightState.LOST:
            if self._locked_target is not None:
                self.set_target_position(*self._locked_target, self.takeoff_height)
            target = self._get_target_ned(); now = self.get_clock().now()
            if target is not None:
                self._car_detection_count += 1
                if self._car_detection_count >= self.confirm_frames:
                    ned_x, ned_y, dist = target
                    self.get_logger().info(f"LOST → TRACK")
                    self._locked_target = (ned_x, ned_y); self._reset_tracking_counters()
                    self.state = FlightState.TRACK; return
            else: self._car_detection_count = 0
            if self._search_start_time and (now - self._search_start_time).nanoseconds * 1e-9 >= self.search_timeout:
                self.get_logger().info("LOST → WAIT")
                self._record_hover_position(); self._reset_tracking_counters()
                self._locked_target = None; self.state = FlightState.WAIT

        elif self.state == FlightState.DROP:
            if self._drop_enter_time is None: self._drop_enter_time = self.get_clock().now()
            target = self._get_target_ned()
            if target is not None:
                self._locked_target = (target[0], target[1])
                self.set_target_position(target[0], target[1], self.drop_height)
            elif self._locked_target is not None:
                self.set_target_position(*self._locked_target, self.drop_height)
            curr_z = self.vehicle_local_position.z
            if not self._drop_servo_done and abs(curr_z - self.drop_height) < 0.2:
                self._drop_servo_done = True
                self.get_logger().info(f'到达抛投高度 Z={curr_z:.2f}m')
                self._servo_drop(); self._drop_enter_time = self.get_clock().now()
            if self._drop_servo_done:
                elapsed = (self.get_clock().now() - self._drop_enter_time).nanoseconds * 1e-9
                if elapsed >= self.rth_delay:
                    self.get_logger().info('DROP → RTH')
                    self.drop_complete_pub.publish(Int32(data=1))
                    self._drop_enter_time = None; self.state = FlightState.RTH; return
            else:
                self.get_logger().info(f'[DROP] 降高中 Z={curr_z:.2f}→{self.drop_height:.1f}...', throttle_duration_sec=1.0)

        elif self.state == FlightState.RTH:
            rth_x, rth_y = self.rth_offset_x, self.rth_offset_y
            self.set_target_position(rth_x, rth_y, self.takeoff_height)
            if self.check_arrived(rth_x, rth_y, self.takeoff_height):
                self.get_logger().info('RTH → LAND')
                self.car_resume_pub.publish(Int32(data=3))
                self._land_stage = 0
                self.state = FlightState.LAND

        elif self.state == FlightState.LAND:
            land_x, land_y = self.rth_offset_x, self.rth_offset_y
            if self._land_stage == 0:
                self.set_target_position(land_x, land_y, self.land_low_height)
                if self.check_arrived(land_x, land_y, self.land_low_height):
                    self.get_logger().info('LAND 阶段0→1'); self._land_stage = 1
            else:
                self.set_target_position(land_x, land_y, 0.0)
                if self.vehicle_local_position.z >= -0.15:
                    self.get_logger().info('LAND → DONE')
                    self.car_resume_pub.publish(Int32(data=5))
                    self._car_mode = 0
                    self.state = FlightState.DONE

    # ==================== 模式2 状态机（非阻塞等待） ====================
    def _run_mode2_state_machine(self):
        if self.state == FS_M2.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.takeoff_height)
            if self.check_arrived(0.0, 0.0, self.takeoff_height):
                self.get_logger().info("TAKEOFF → TRACKLAND")
                self.state = FS_M2.TRACKLAND
                self.target_z = self.takeoff_height + 0.2

        elif self.state == FS_M2.TRACKLAND:
            target = self._get_target_ned()
            self.set_target_position(target[0], target[1], self.target_z)
            if self.check_arrived_2d(target[0], target[1]):
                self.get_logger().info("DOWN !!")
                if self.vehicle_local_position.z < 0.01:   # 尚未触地
                    self.target_z += 0.2                  # 下降 (NED 中值增大)
                else:
                    self.get_logger().info("TRACKLAND → FLIGHT_AGINE")
                    self.state = FS_M2.FLIGHT_AGINE
                    self.target_z = self.takeoff_height
                    self._flight_again_start = None       # 重置等待计时器

        elif self.state == FS_M2.FLIGHT_AGINE:
            # 非阻塞等待 3 秒，保持在小车水平位置、巡航高度
            target = self._get_target_ned()
            if self._flight_again_start is None:
                self._flight_again_start = self.get_clock().now()
            elapsed = (self.get_clock().now() - self._flight_again_start).nanoseconds * 1e-9
            if elapsed < 3.0:
                self.set_target_position(target[0], target[1], self.target_z)
                self.get_logger().info(f'[FLIGHT_AGINE] 等待 {elapsed:.1f}s/3.0s', throttle_duration_sec=1.0)
            else:
                self.get_logger().info('FLIGHT_AGINE → RTH')
                self.state = FS_M2.RTH

        elif self.state == FS_M2.RTH:
            rth_x, rth_y = self.rth_offset_x, self.rth_offset_y
            self.set_target_position(rth_x, rth_y, self.takeoff_height)
            if self.check_arrived(rth_x, rth_y, self.takeoff_height):
                self.get_logger().info('RTH → LAND')
                self.state = FS_M2.LAND

        elif self.state == FS_M2.LAND:
            land_x, land_y = self.rth_offset_x, self.rth_offset_y
            self.set_target_position(land_x, land_y, 0.0)
            # 直接判断高度触地，避免水平位置无法精准到达
            if self.vehicle_local_position.z >= -0.15:
                self.get_logger().info('LAND → DONE')
                self._car_mode = 0
                self.state = FlightState.DONE


def main(args=None):
    rclpy.init(args=args)
    node = TFTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("终止.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()