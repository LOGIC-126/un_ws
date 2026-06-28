#!/usr/bin/env python3
"""
可视化脚本：uv2 记录数据（相机坐标 + 世界坐标 + 无人机位姿）
用法：
  python3 visualize_uv2.py record_uv2_YYYYMMDD_HHMMSS.csv
输出：同名的 .png 图片
"""

import csv
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_csv(csv_path):
    secs, nsecs = [], []
    dxs, dys, dzs = [], [], []
    rolls, pitches, yaws = [], [], []
    class_names = []
    cam_xs, cam_ys, cam_zs = [], [], []
    world_xs, world_ys, world_zs = [], [], []
    raw_xs, raw_ys = [], []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            secs.append(float(row['timestamp_sec']))
            nsecs.append(float(row['timestamp_nanosec']))
            dxs.append(float(row['drone_x']))
            dys.append(float(row['drone_y']))
            dzs.append(float(row['drone_z']))
            rolls.append(float(row['roll_deg']))
            pitches.append(float(row['pitch_deg']))
            yaws.append(float(row['yaw_deg']))
            class_names.append(row['class_name'])
            cam_xs.append(float(row['cam_x']))
            cam_ys.append(float(row['cam_y']))
            cam_zs.append(float(row['cam_z']))
            world_xs.append(float(row['world_x']))
            world_ys.append(float(row['world_y']))
            world_zs.append(float(row['world_z']))
            raw_xs.append(float(row.get('raw_x', row['world_x'])))
            raw_ys.append(float(row.get('raw_y', row['world_y'])))

    timestamps = np.array(secs) + np.array(nsecs) * 1e-9
    rel_time = timestamps - timestamps[0]

    return (rel_time,
            np.array(dxs), np.array(dys), np.array(dzs),
            np.array(rolls), np.array(pitches), np.array(yaws),
            class_names,
            np.array(cam_xs), np.array(cam_ys), np.array(cam_zs),
            np.array(world_xs), np.array(world_ys), np.array(world_zs),
            np.array(raw_xs), np.array(raw_ys))


def visualize(rel_time, dx, dy, dz, roll, pitch, yaw,
              classes, cx, cy, cz, wx, wy, wz, rx, ry, output_png):
    n = len(rel_time)
    print(f"共 {n} 条数据")
    if n == 0:
        print("无数据，跳过绘图")
        return

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    # axes layout:
    #   [0,0] 姿态       [0,1] 高度       [0,2] 世界XY散点
    #   [1,0] 无人机位置  [1,1] 相机相对    [1,2] (空, 隐藏)

    # ===== 子图1: 姿态 — 时间 =====
    ax = axes[0, 0]
    ax.plot(rel_time, roll, label='Roll', color='tab:red', linewidth=0.8)
    ax.plot(rel_time, pitch, label='Pitch', color='tab:green', linewidth=0.8)
    ax.plot(rel_time, yaw, label='Yaw', color='tab:blue', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title('1. Drone Attitude')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.legend(loc='best')

    # ===== 子图2: 高度 — 时间 =====
    ax = axes[0, 1]
    ax.plot(rel_time, -dz, color='tab:blue', linewidth=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Height (m)')
    ax.set_title('2. Drone Height')
    ax.grid(True, linestyle=':', alpha=0.5)

    # 按类别分配标记
    unique_classes = sorted(set(classes))
    markers = ['o', 's', '^', 'D', 'v', 'p', '*', 'x'][:len(unique_classes)]
    colors = plt.cm.tab10(np.linspace(0, 1, len(unique_classes)))
    cls_to_style = {c: (m, col) for c, m, col in zip(unique_classes, markers, colors)}

    # ===== 子图3: 物体世界坐标 X-Y (滤波后) =====
    ax = axes[0, 2]
    for cls in unique_classes:
        mask = np.array([c == cls for c in classes])
        m, col = cls_to_style[cls]
        ax.scatter(wx[mask], wy[mask], marker=m, color=col, s=15, alpha=0.7, label=cls)
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    ax.set_title('3. Filtered World X-Y')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.axis('equal')
    ax.legend(loc='best', markerscale=2)

    # ===== 子图4: 无人机位置 — 时间 =====
    ax = axes[1, 0]
    ax.plot(rel_time, dx, label='Drone X (北)', color='tab:red', linewidth=0.8)
    ax.plot(rel_time, dy, label='Drone Y (东)', color='tab:green', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('World Coord (m)')
    ax.set_title('4. Drone World Position')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.legend(loc='best')

    # ===== 子图5: 相机相对位置 X-Y =====
    ax = axes[1, 1]
    ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
    ax.axvline(x=0, color='gray', linewidth=0.5, linestyle='--')
    ax.plot(0, 0, 'k+', markersize=10)
    for cls in unique_classes:
        mask = np.array([c == cls for c in classes])
        m, col = cls_to_style[cls]
        ax.scatter(cx[mask], cy[mask], marker=m, color=col, s=15, alpha=0.7, label=cls)
    ax.set_xlabel('Cam X (m) ← 左正')
    ax.set_ylabel('Cam Y (m) → 前正')
    ax.set_title('5. Camera-Relative Target Positions')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.axis('equal')
    ax.legend(loc='best', markerscale=2)

    # ===== 子图6: 原始世界坐标 X-Y (未滤波) =====
    ax = axes[1, 2]
    for cls in unique_classes:
        mask = np.array([c == cls for c in classes])
        m, col = cls_to_style[cls]
        ax.scatter(rx[mask], ry[mask], marker=m, color=col, s=15, alpha=0.7, label=cls)
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    ax.set_title('6. Raw World X-Y (unfiltered)')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.axis('equal')
    ax.legend(loc='best', markerscale=2)

    plt.suptitle('UV2 Recorder — Camera & World Coordinates', fontsize=14)
    plt.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"图片已保存为: {output_png}")


def main():
    if len(sys.argv) < 2:
        print("用法: python3 visualize_uv2.py <record_uv2_*.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在: {csv_path}")
        sys.exit(1)

    (rel_time, dx, dy, dz, roll, pitch, yaw,
     classes, cx, cy, cz, wx, wy, wz, rx, ry) = load_csv(csv_path)
    output_png = csv_path.rsplit('.', 1)[0] + '.png'
    visualize(rel_time, dx, dy, dz, roll, pitch, yaw,
              classes, cx, cy, cz, wx, wy, wz, rx, ry, output_png)


if __name__ == '__main__':
    main()
