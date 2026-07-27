# EVA0N — Embedded Versatile Autonomy Drone 

基于 ROS 2 Humble + PX4 + RK3588 NPU 的无人机目标检测追踪系统。

## 硬件平台

| 组件 | 型号 |
|---|---|
| 机载计算机 | Orange Pi 5 Pro (RK3588) |
| 飞控 | Pixhawk (PX4) |
| 雷达 | 雷神 LSN10P |
| 摄像头 | USB 单目 + Intel RealSense D435(可选) |
| 通信 | 串口 (DDS 直连) |

## 架构概览

```
雷达 ──→ Cartographer SLAM ──→ ekf2_trans ──→ PX4 EKF2 (里程计)
                                               ↑
摄像头 ──→ rknn_yolo (NPU) ──→ amount ──→ offboard_control ──→ PX4 (板载控制)
                                  (坐标变换)    (追踪/任务/PID)
```

## 包结构

| 包 | 类型 | 功能 |
|---|---|---|
| `rknn_yolo` | C++ | RK3588 NPU 上运行 YOLOv8 目标检测，摄像头采集 |
| `amount` | Python | 像素坐标 → 相机坐标 → 世界坐标变换 |
| `offboard_control` | Python | PX4 板载控制、PID 速度控制、YOLO 追踪、竞赛任务状态机 |
| `ekf2_trans` | Python | Cartographer TF → PX4 视觉里程计，IMU 坐标系转换 |
| `cartographer` | Launch | 2D SLAM 建图，提供 map→base_link TF |
| `aruco_ros2` | Python | ArUco 标记位姿估计与滤波 |
| `lslidar_driver` | C++ | 雷神雷达驱动 |
| `lslidar_msgs` | CMake | 雷达消息定义 |
| `record` | Python | 数据记录（CSV） |

## 节点与数据流（以 offboard_control 为中心）

```
                            ┌─────────────────────┐
                            │   lslidar_driver     │
                            │   → /scan           │
                            └──────────┬──────────┘
                                       ↓
                            ┌─────────────────────┐
                            │   Cartographer       │
                            │   → TF map→base_link │
                            └──────────┬──────────┘
                                       ↓
┌──────────────┐             ┌─────────────────────┐
│ camera_node  │             │   ekf2_link_dds      │
│ → /image_raw │             │   TF→视觉里程计       │
└──────┬───────┘             │   → /fmu/in/vehicle_ │
       ↓                     │     visual_odometry  │
┌──────────────┐             └──────────────────────┘
│ yolo_        │                        ↓
│ detector     │                (PX4 EKF2 融合)
│ (RKNN NPU)   │
│ → /detections│
└──────┬───────┘
       ↓
┌──────────────┐
│ detection_   │
│ world_node   │
│ (amount)     │
│ 像素→相机→世界│
│ → /detection/│
│   world_     │
│   coordinates│
└──────┬───────┘
       ↓
┌──────────────────────────────────────────────┐
│              offboard_control                 │
│                                               │
│  ┌─────────────────┐  ┌────────────────────┐ │
│  │ yolo_tracking   │  │ offboard_control   │ │
│  │ 状态机:          │  │ PID 速度控制        │ │
│  │ INIT→TAKEOFF    │  │                     │ │
│  │ →WAIT⇄TRACK⇄LOST│  │ → /fmu/in/          │ │
│  │                 │  │   trajectory_setpoint│ │
│  │ → /uav/target_  │  │   offboard_control_ │ │
│  │   position      │──│   mode              │ │
│  └─────────────────┘  └────────────────────┘ │
│                                               │
│  ┌──────────────────────────────────────────┐ │
│  │ competition_mission                       │ │
│  │ 7×9 网格 TSP 路径规划 + 动物检测            │ │
│  │ → /uav/target_position                   │ │
│  └──────────────────────────────────────────┘ │
│                                               │
│  输入: /detection/world_coordinates           │
│       /fmu/out/vehicle_local_position_v1      │
│       /fmu/out/vehicle_attitude               │
│       /fmu/out/vehicle_status_v2              │
│       /fmu/out/manual_control_setpoint (RC)   │
│                                               │
│  输出: /uav/target_position (Pose)            │
│       /fmu/in/trajectory_setpoint             │
│       /fmu/in/offboard_control_mode           │
└──────────────────────────────────────────────┘
```

## 编译

### 依赖

```bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

RKNN 依赖：需要 RK3588 NPU 运行时库 (`librknnrt.so`)，CMake 中引用路径为 `/home/orangepi/rknn-toolkit2/rknpu2/runtime/`，此路径可按实际安装位置调整。

### 构建

```bash
cd ~/un_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 1. 建图

```bash
ros2 launch cartographer cartographer_imu_dds.launch.py
```

### 2. 板载控制

输入 `/uav/target_position` (`geometry_msgs/msg/Pose`)，
输出 `/fmu/in/trajectory_setpoint`、`/fmu/in/offboard_control_mode`。

```bash
ros2 run offboard_control node_offboard_control \
    --ros-args --params-file install/offboard_control/share/offboard_control/config/pid_params.yaml
```

### 3. 检测 + 追踪

```bash
ros2 run rknn_yolo camera_publisher_node
ros2 run rknn_yolo yolo_detector_node \
    --ros-args --params-file install/rknn_yolo/share/rknn_yolo/config/yolo_detector_params.yaml
ros2 run amount detection_world_node
ros2 run offboard_control node_yolo_tracking
```

### 4. 2025 年电赛 H 题飞行部分

7×9 网格 TSP 全覆盖路径规划，逐格扫描识别地面动物目标。

```bash
ros2 run offboard_control node_competition_mission
```

## 关键配置

| 参数 | 位置 | 说明 |
|---|---|---|
| `camera_device` | `yolo_detector_params.yaml` | 摄像头 by-id 稳定路径 |
| `rknn_model` | `yolo_detector_params.yaml` | RKNN 模型路径 |
| `pid_params` | `pid_params.yaml` | PID 增益 (Kp/Ki/Kd)，最大速度 2m/s |
| `tracking_params` | `tracking_params.yaml` | 追踪限速 0.3m/s |
| LiDAR 串口 | `lsn10p.yaml` | `/dev/ttyS3` |
| PX4 串口 | `eva0n_dds.sh` | `/dev/ttyS6:921600` |

## 工具脚本

| 脚本 | 用途 |
|---|---|
| `eva0n_dds.sh` | 一键启动 MicroXRCEAgent + Cartographer + ekf2 |
| `record_bag.sh` | 按话题组选择性录制 ros2 bag |
| `bag_to_csv.py` | bag 离线导出 CSV |
