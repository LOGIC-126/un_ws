#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: RZR
@说明: 飞行任务控制 — 电赛特供 (控制解耦版)
       控制链路: 发布 Pose 到 /uav/target_position，由 offboard_control 桥接 PX4
       路径规划: GridSolver 根据障碍物数据求解 TSP 全遍历最短路径
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from enum import Enum
import math

# PX4 反馈 (仅状态读取)
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, ManualControlSetpoint

# 通用控制消息
from geometry_msgs.msg import Pose
from std_msgs.msg import String

# 路径规划
from .build_road import GridSolver


class FlightState(Enum):
    INIT = 0
    TAKEOFF = 1
    GOTOTAR = 2
    MISSION = 3
    LAND = 4
    DONE = 5


class CompetitionMissionNode(Node):
    # ====== 硬编码安全开关 ======
    ENABLE_AUTO_TAKEOFF = True

    # ====== 任务参数 ======
    # 障碍物数据: A/B 网格坐标，每组三个连续障碍格
    # A1-A7 对应网格第1-7行 (y方向), B1-B9 对应网格第1-9列 (x方向)
    # 格式: [('A行号', 'B列号'), ...], 共3组
    DEMO_BARRIERS = [
        ('A7', 'B3'),
        ('A7', 'B4'),
        ('A7', 'B5'),
    ]

    TAKE_HEIGHT = -1.2    # 起飞/任务高度 (NED, m)
    GRID_SIZE = 0.5       # 每格尺寸 (m)

    # 动物类型 (与 animal_detection_sim 中 ANIMAL_TYPES 顺序一致)
    ANIMAL_TYPES = ['elephant', 'tiger', 'wolf', 'monkey', 'peacock']

    def __init__(self):
        super().__init__('competition_mission_node')

        # 可配置参数: vehicle_status 话题后缀 (v2=真机旧版PX4, v4=SITL 1.18+)
        self.declare_parameter('vehicle_status_suffix', 'v4')
        # 航点插值: 每周期目标点向航点移动步长 (m)
        self.declare_parameter('interp_step', 0.05)
        self.declare_parameter('interp_enabled', True)

        # ====== RC 一键启动参数 (通道9→aux1) ======
        # ManualControlSetpoint 的字段名: aux1~aux6
        self.declare_parameter('rc_trigger_aux', 'aux1')
        # 触发起动阈值: 字段值 > threshold 时触发 (范围 [-1, +1], 拨杆上=+1)
        self.declare_parameter('rc_trigger_threshold', 0.5)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----- 发布器 (解耦) -----
        self.target_position_pub = self.create_publisher(
            Pose, '/uav/target_position', qos_profile)

        # ----- 订阅器 (仅反馈) -----
        self.vehicle_local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f'/fmu/out/vehicle_status_{self.get_parameter("vehicle_status_suffix").value}',
            self.vehicle_status_callback, qos_profile)
        # 动物检测日志: "AnBm,animal_type,count"
        self.detection_log_sub = self.create_subscription(
            String, '/uav/detection_log',
            self.detection_log_callback, 10)
        # 动物相对位置 (NED 差值) + 动物类型 ID
        self.animal_detection_sub = self.create_subscription(
            Pose, '/uav/animal_detection',
            self.animal_detection_callback, 10)
        # RC 一键启动 (aux1, 通道9映射)
        self.manual_control_sub = self.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint',
            self.manual_control_callback, qos_profile)

        # ----- 路径规划器 -----
        self.solver = GridSolver()

        # ----- 内部变量 -----
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()

        self.state = FlightState.INIT
        self.current_mission_index = 0
        self.wait_start_time = None

        # RC 一键启动: 上升沿触发, 触发后不再受开关影响
        self.rc_triggered = False
        self._last_rc_trigger_raw = False

        # 由 solver 动态生成的任务点列表 [(x, y, z, mission_type), ...]
        self.mission_points = []

        # 已检测到动物的方格: {grid_code: [animal_type, ...]}
        self.detected_grids = {}

        # 动物的世界坐标: {(grid_code, animal_type): (world_x, world_y)}
        self.animal_positions = {}

        # 已追踪标记，防止重复插入: set of (grid_code, animal_type)
        self.tracked_animals = set()

        # 目标缓存 (NED)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0

        # 插值: 目标点(buffer) 和 期望航点(waypoint)
        self._buf_x = 0.0
        self._buf_y = 0.0
        self._buf_z = 0.0
        self._wp_x = 0.0
        self._wp_y = 0.0
        self._wp_z = 0.0

        # 根据开关一次性发布初始目标位姿
        if self.ENABLE_AUTO_TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)

        self.timer = self.create_timer(0.05, self.timer_callback)

        mode_str = "自主起飞" if self.ENABLE_AUTO_TAKEOFF else "手动解锁 (等待外部指令)"
        self.get_logger().info(
            f"电赛任务节点已启动 (控制解耦模式: {mode_str})."
        )

    # --- 1. 订阅回调 ---
    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def manual_control_callback(self, msg: ManualControlSetpoint) -> None:
        """RC 一键启动: 检测 aux1 上升沿 (拨杆低→高) 触发任务"""
        if not msg.valid:
            return
        aux_name = self.get_parameter('rc_trigger_aux').value
        val = getattr(msg, aux_name, 0.0)
        raw = (val > self.get_parameter('rc_trigger_threshold').value)
        # 上升沿检测: 上次为低, 本次为高 → 触发
        if raw and not self._last_rc_trigger_raw:
            self.rc_triggered = True
            self.get_logger().info(
                f"RC 一键启动触发! ({aux_name}={val:.2f})")
        self._last_rc_trigger_raw = raw

    # --- 2. 插值与控制发布 ---
    def _interp_tick(self) -> None:
        """每周期将 target (buffer) 向 waypoint 移动 interp_step 米"""
        step = self.get_parameter('interp_step').value
        dx = self._wp_x - self._buf_x
        dy = self._wp_y - self._buf_y
        dz = self._wp_z - self._buf_z
        dist = math.hypot(dx, dy, dz)
        if dist <= step:
            self._buf_x = self._wp_x
            self._buf_y = self._wp_y
            self._buf_z = self._wp_z
        else:
            frac = step / dist
            self._buf_x += dx * frac
            self._buf_y += dy * frac
            self._buf_z += dz * frac

    def _sync_target_to_buffer(self) -> None:
        """将 target 同步到 buffer 并发布"""
        if (self.target_x == self._buf_x and self.target_y == self._buf_y and
            self.target_z == self._buf_z):
            return
        self.target_x = self._buf_x
        self.target_y = self._buf_y
        self.target_z = self._buf_z
        self._publish_target()

    def _publish_target(self) -> None:
        msg = Pose()
        msg.position.x = float(self.target_x)
        msg.position.y = float(self.target_y)
        msg.position.z = float(self.target_z)
        msg.orientation.w = math.cos(self.target_yaw / 2.0)
        msg.orientation.z = math.sin(self.target_yaw / 2.0)
        self.target_position_pub.publish(msg)

    # --- 3. 状态机辅助 ---
    def set_waypoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """设定期望航点: 若插值关闭则直接跳到航点"""
        self._wp_x = float(x)
        self._wp_y = float(y)
        self._wp_z = float(z)
        self.target_yaw = float(yaw)
        if not self.get_parameter('interp_enabled').value:
            self._buf_x = self._wp_x
            self._buf_y = self._wp_y
            self._buf_z = self._wp_z
        self._sync_target_to_buffer()
        # 首次调用强制发布 (timer 可能尚未启动)
        self._publish_target()

    def set_target_position(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """兼容旧接口: 同 set_waypoint"""
        self.set_waypoint(x, y, z, yaw)

    def check_distance(self, x: float, y: float, z: float, threshold: float = 0.15) -> bool:
        pos = self.vehicle_local_position
        dist = math.sqrt((pos.x - x)**2 + (pos.y - y)**2 + (pos.z - z)**2)
        return dist < threshold

    # --- 4. 路径生成与网格工具 ---
    def detection_log_callback(self, msg: String) -> None:
        """接收动物检测日志 'AnBm,animal_type,count'"""
        try:
            parts = msg.data.split(',')
            if len(parts) >= 2:
                grid_code = parts[0]
                animal_type = parts[1]
                if grid_code not in self.detected_grids:
                    self.detected_grids[grid_code] = []
                if animal_type not in self.detected_grids[grid_code]:
                    self.detected_grids[grid_code].append(animal_type)
        except Exception:
            pass

    def animal_detection_callback(self, msg: Pose) -> None:
        """接收动物相对位置，换算为世界 NED 坐标并缓存"""
        try:
            type_id = int(msg.orientation.w)
            type_name = self.ANIMAL_TYPES[type_id]
        except (IndexError, ValueError):
            return

        # NED 相对 → 世界
        world_x = self.vehicle_local_position.x + msg.position.x
        world_y = self.vehicle_local_position.y + msg.position.y

        grid_code = self._world_to_grid_code(world_x, world_y)
        self.animal_positions[(grid_code, type_name)] = (world_x, world_y)

    @staticmethod
    def _world_to_grid_code(x: float, y: float) -> str:
        """世界 NED 坐标 → 方格代码 'AnBm'"""
        n = round(9 - x / 0.5)
        m = round(1 + y / 0.5)
        n = max(1, min(9, n))
        m = max(1, min(7, m))
        return f"A{n}B{m}"

    @staticmethod
    def _grid_code_to_center(code: str):
        """方格代码 'AnBm' → 格心世界 NED 坐标 (x, y)"""
        a_idx = code.index('A')
        b_idx = code.index('B')
        n = int(code[a_idx+1:b_idx])
        m = int(code[b_idx+1:])
        x = (9 - n) * 0.5 + 0.25
        y = (m - 1) * 0.5 + 0.25
        return x, y

    def generate_mission_points(self) -> list:
        """
        根据障碍物数据调用 GridSolver 生成全遍历最短路径 (TSP 闭环)。
        scan 段: 跟随 solver 网格路径遍历所有格子
        back 段: 从最后访问格直线返航，45° 平滑下滑至原点
        """
        try:
            barriers = [self.solver.ab_to_coordinate(a, b) for a, b in self.DEMO_BARRIERS]
            self.solver.set_barriers(barriers)
            result = self.solver.solve()

            return_idx = result['return_start_index']
            world_path = result['world_path']

            self.get_logger().info(
                f"路径规划完成: 总步数={result['total_steps']}, "
                f"航点数={len(world_path)}, 效率={result['efficiency']:.1f}%, "
                f"返航段起始索引={return_idx}"
            )

            # scan 段: solver 使用 NED y 负方向, 转换为正 y 朝下
            points = [(x, -y, self.TAKE_HEIGHT, "scan")
                      for x, y in world_path[:return_idx + 1]]

            # back 段: 从最后一个 scan 点直线 45° 下滑到原点
            last_x, raw_y = world_path[return_idx]
            last_y = -raw_y
            back_points = self._generate_back_path(
                last_x, last_y, self.TAKE_HEIGHT, 0.0, 0.0, 0.0
            )
            points.extend(back_points)

            self.get_logger().info(
                f"任务点: scan={return_idx + 1}, back={len(back_points)}"
            )
            return points

        except Exception as e:
            self.get_logger().error(f"路径规划失败: {str(e)}，使用默认单点任务.")
            return [(2.0, -1.5, self.TAKE_HEIGHT, "scan")]

    def _generate_back_path(self, start_x, start_y, start_z,
                            end_x, end_y, end_z, step=0.25) -> list:
        """
        生成平滑 45° 下滑返航路径 (直线插值)。

        从起点沿直线飞向终点:
        - 水平距离 > 下降高度时: 先平飞, 至剩余距离=高度时开始下滑
        - 水平距离 ≤ 下降高度时: 全程按比例下滑
        - 中间点 z 始终远离 0 (|z| >= 0.06), 避免误触发 offboard land()
        - 仅终点处 z = end_z (地面)
        """
        SAFE_Z = -0.06  # 避开 offboard_control 的 |z|<0.05 触发 land()

        dx = end_x - start_x
        dy = end_y - start_y
        total_hdist = math.hypot(dx, dy)

        if total_hdist < 0.01:
            return [(end_x, end_y, end_z, "back")]

        descent_needed = abs(start_z - end_z)

        if total_hdist > descent_needed:
            level_dist = total_hdist - descent_needed
            descent_dist = descent_needed
        else:
            level_dist = 0.0
            descent_dist = total_hdist

        total_len = level_dist + descent_dist
        n_steps = max(2, int(total_len / step))
        points = []

        for i in range(1, n_steps + 1):
            frac = i / n_steps
            px = start_x + dx * frac
            py = start_y + dy * frac
            path_dist = total_len * frac

            if path_dist <= level_dist:
                pz = start_z
            else:
                t = (path_dist - level_dist) / descent_dist
                pz = start_z + t * (end_z - start_z)

            # 中间航点必须高于 land 触发阈值, 仅终点接地
            if i < n_steps and pz > SAFE_Z:
                pz = SAFE_Z

            points.append((px, py, pz, "back"))

        return points

    # --- 5. 任务执行 ---
    def execute_mission(self, mission_type: str) -> bool:
        if mission_type == "wait":
            return self.Wait()
        if mission_type == "pass":
            return True
        if mission_type == "scan":
            return self.Scan()
        if mission_type == "back":
            return self.Back()
        return True

    def Back(self) -> bool:
        """返航任务: 直接通过，45° 下滑由航点坐标编码"""
        self.get_logger().info("返航中... (45° 下滑)", throttle_duration_sec=1.0)
        return True

    def Wait(self, duration: float = 5.0) -> bool:
        now = self.get_clock().now()
        if self.wait_start_time is None:
            self.wait_start_time = now
            self.get_logger().info(f"悬停等待 {duration} 秒...")
            return False
        elapsed = (now - self.wait_start_time).nanoseconds / 1e9
        if elapsed < duration:
            self.get_logger().info(f"等待中... {elapsed:.1f}s", throttle_duration_sec=1.0)
            return False
        self.get_logger().info("等待结束")
        self.wait_start_time = None
        return True

    def Scan(self) -> bool:
        self.get_logger().info("执行扫描任务...", throttle_duration_sec=0.5)
        # if self.current_mission_index >= len(self.mission_points):
        #     return True

        # # 当前航点方格
        # cp = self.mission_points[self.current_mission_index]
        # current_grid = self._world_to_grid_code(cp[0], cp[1])

        # # 下一航点方格
        # next_grid = None
        # if self.current_mission_index + 1 < len(self.mission_points):
        #     np_ = self.mission_points[self.current_mission_index + 1]
        #     next_grid = self._world_to_grid_code(np_[0], np_[1])

        # # 查找有未追踪动物的方格
        # target_grid = None
        # for grid in (current_grid, next_grid):
        #     if grid is None:
        #         continue
        #     if grid not in self.detected_grids:
        #         continue
        #     for animal_type in self.detected_grids[grid]:
        #         if (grid, animal_type) not in self.tracked_animals:
        #             target_grid = grid
        #             break
        #     if target_grid is not None:
        #         break

        # if target_grid is None:
        #     return True  # 无目标，跳过

        # # 获取该方格中未追踪的动物
        # untracked = [
        #     a for a in self.detected_grids.get(target_grid, [])
        #     if (target_grid, a) not in self.tracked_animals
        # ]

        # for animal_type in untracked:
        #     # 优先取 /uav/animal_detection 累积的世界坐标，fallback 到格心
        #     key = (target_grid, animal_type)
        #     if key in self.animal_positions:
        #         animal_x, animal_y = self.animal_positions[key]
        #     else:
        #         animal_x, animal_y = self._grid_code_to_center(target_grid)

        #     self.insert_mission_point_after_current(
        #         animal_x, animal_y, self.TAKE_HEIGHT, "pass"
        #     )
        #     self.tracked_animals.add(key)
        #     self.get_logger().info(
        #         f"[Scan] {target_grid} {animal_type} → "
        #         f"追踪坐标 ({animal_x:.2f}, {animal_y:.2f}) (世界 NED)"
        #     )

        return True

    def insert_mission_point_after_current(self, x, y, z, mission_type):
        """
        在当前任务点之后插入一个新的任务点
        
        参数:
            x, y, z: 新任务点的坐标
            mission_type: 任务类型
        """
        # 创建新的任务点元组
        new_point = (x, y, z, mission_type)
        
        # 在当前任务索引之后插入新任务点
        insert_index = self.current_mission_index + 1
        
        # 插入到任务列表中
        self.mission_points.insert(insert_index, new_point)
        
        self.get_logger().info(f"已插入新任务点: 位置[{x}, {y}, {z}], 类型:{mission_type}")

    def insert_mission_point_at_end(self, x, y, z, mission_type):
        """
        在任务列表末尾插入一个新的任务点
        
        参数:
            x, y, z: 新任务点的坐标
            mission_type: 任务类型
        """
        # 创建新的任务点元组
        new_point = (x, y, z, mission_type)
        
        # 添加到任务列表末尾
        self.mission_points.append(new_point)
        
        self.get_logger().info(f"已添加新任务点到末尾: 位置[{x}, {y}, {z}], 类型:{mission_type}")

    def insert_mission_point_at_index(self, index, x, y, z, mission_type):
        """
        在指定索引位置插入一个新的任务点
        
        参数:
            index: 插入位置的索引
            x, y, z: 新任务点的坐标
            mission_type: 任务类型
        """
        # 创建新的任务点元组
        new_point = (x, y, z, mission_type)
        
        # 插入到指定位置
        if 0 <= index <= len(self.mission_points):
            self.mission_points.insert(index, new_point)
            self.get_logger().info(f"已在索引{index}插入新任务点: 位置[{x}, {y}, {z}], 类型:{mission_type}")
        else:
            self.get_logger().warn(f"索引{index}无效，任务点列表长度为{len(self.mission_points)}")


    # --- 6. 主循环 ---
    def timer_callback(self) -> None:
        # A. 插值: 始终运行, 确保 target 持续发布 (offboard_control 需要)
        if self.get_parameter('interp_enabled').value:
            self._interp_tick()
            self._sync_target_to_buffer()
        else:
            self._publish_target()

        # B. 安全防护
        is_armed = (self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        is_offboard = (self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        if self.state not in [FlightState.LAND, FlightState.DONE]:
            if not is_armed or not is_offboard:
                if self.state != FlightState.INIT:
                    self.get_logger().warn("掉出 Offboard / 上锁状态，退回待命.")
                    self.state = FlightState.INIT
                    self.wait_start_time = None
                return

        # C. 运行状态机
        self.run_state_machine()

    def run_state_machine(self) -> None:
        if self.state == FlightState.INIT:
            if not self.mission_points:
                self.mission_points = self.generate_mission_points()

            # 一键启动: 等待 RC 通道 9 上升沿触发
            if not self.rc_triggered:
                self.set_target_position(0.0, 0.0, 0.0)
                self.get_logger().info(
                    "等待 RC 一键启动 (aux1 拨杆高位)...",
                    throttle_duration_sec=3.0,
                )
                return

            if self.ENABLE_AUTO_TAKEOFF:
                self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            else:
                self.set_target_position(0.0, 0.0, 0.0)
                self.get_logger().info(
                    "自主起飞已禁用，保持 z=0 待命。请手动解锁并切 OFFBOARD...",
                    throttle_duration_sec=3.0,
                )
                return

            self.get_logger().info(
                f"RC 触发成功！路径已生成，共 {len(self.mission_points)} 个航点。状态: INIT -> TAKEOFF"
            )
            self.state = FlightState.TAKEOFF

        elif self.state == FlightState.TAKEOFF:
            self.set_target_position(0.0, 0.0, self.TAKE_HEIGHT)
            if self.check_distance(0.0, 0.0, self.TAKE_HEIGHT):
                self.get_logger().info("起飞完成，开始航点遍历.")
                self.current_mission_index = 0
                self.state = FlightState.GOTOTAR

        elif self.state == FlightState.GOTOTAR:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, _ = point
                self.set_target_position(target_x, target_y, target_z)

                if self.check_distance(target_x, target_y, target_z):
                    self.get_logger().info(
                        f"到达航点 {self.current_mission_index + 1}/{len(self.mission_points)}"
                    )
                    self.state = FlightState.MISSION

        elif self.state == FlightState.MISSION:
            if self.current_mission_index < len(self.mission_points):
                point = self.mission_points[self.current_mission_index]
                target_x, target_y, target_z, mission_type = point
                self.set_target_position(target_x, target_y, target_z)

                if self.execute_mission(mission_type):
                    self.get_logger().info(
                        f"航点 {self.current_mission_index + 1} [{mission_type}] 完成"
                    )
                    if self.current_mission_index >= len(self.mission_points) - 1:
                        self.get_logger().info("全部航点遍历完毕，准备降落.")
                        self.state = FlightState.LAND
                    else:
                        self.current_mission_index += 1
                        self.state = FlightState.GOTOTAR

        elif self.state == FlightState.LAND:
            self.set_target_position(0.0, 0.0, 0.0)
            if self.vehicle_local_position.z >= -0.15:
                self.get_logger().info("已着陆，任务结束.")
                self.state = FlightState.DONE

        elif self.state == FlightState.DONE:
            self.get_logger().info("电赛任务完成，关闭节点.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CompetitionMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("用户强行终止节点.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
