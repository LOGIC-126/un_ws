#!/usr/bin/env python3
"""
从 ros2 bag 导出数据到 CSV，方便离线查看和用 visualize_uv2.py 绘图。

用法:
  python3 bag_to_csv.py <bag_path>                   # 导出全部
  python3 bag_to_csv.py <bag_path> -t /scan          # 只导出指定话题
  python3 bag_to_csv.py <bag_path> -o my_output_dir  # 指定输出目录
  python3 bag_to_csv.py <bag_path> --list            # 列出 bag 中的话题

依赖: pip install rosbags (不需要 ROS 环境!)
"""

import sys
import os
import csv
import json
import argparse
import math
from collections import defaultdict
from pathlib import Path


def quaternion_to_euler(w, x, y, z):
    """四元数 → 欧拉角 (度)"""
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class BagExporter:
    """使用 rosbags 库读取 ros2 bag (不需要 ROS 环境)"""

    def __init__(self, bag_path, output_dir="."):
        self.bag_path = Path(bag_path)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 各话题的 CSV writer
        self.writers = {}
        self.files = {}
        self.csv_writers = {}

        # 合并导出 (uv2 格式) 的缓冲区
        self.merged_rows = []
        self.drone_pos = None
        self.drone_att = None
        self.latest_cam = None
        self.latest_raw = None
        self.latest_world = None

    def _get_writer(self, topic_name):
        """为每个话题创建独立的 CSV 文件"""
        if topic_name in self.csv_writers:
            return self.csv_writers[topic_name]

        safe_name = topic_name.strip('/').replace('/', '_')
        csv_path = self.output_dir / f"{safe_name}.csv"
        f = open(csv_path, 'w', newline='', encoding='utf-8')
        self.files[topic_name] = f

        # 根据话题类型预设列名
        if 'attitude' in topic_name:
            columns = ['timestamp', 'w', 'x', 'y', 'z',
                       'roll_deg', 'pitch_deg', 'yaw_deg']
        elif 'local_position' in topic_name:
            columns = ['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz']
        elif topic_name == '/scan':
            columns = ['timestamp', 'angle_min', 'angle_max', 'angle_increment',
                       'range_min', 'range_max', 'ranges_count', 'ranges']
        elif topic_name == '/imu':
            columns = ['timestamp',
                       'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                       'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
                       'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z']
        else:
            columns = ['timestamp', 'data']

        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        self.csv_writers[topic_name] = writer
        return writer

    def export(self, selected_topics=None):
        try:
            from rosbags.rosbag2 import Reader
            from rosbags.typesys import Stores, get_typestore
        except ImportError:
            print("错误: 需要安装 rosbags 库")
            print("  pip install rosbags")
            sys.exit(1)

        typestore = get_typestore(Stores.ROS2_HUMBLE)

        print(f"读取 bag: {self.bag_path}")
        reader = Reader(self.bag_path)
        reader.open()

        all_topics = set()
        for conn in reader.connections:
            all_topics.add(conn.topic)

        print(f"发现话题 ({len(all_topics)}):")
        for t in sorted(all_topics):
            print(f"  {t}  [{reader.connections}]")

        # ... but we need per-connection info
        print(f"\n话题详情:")
        for conn in reader.connections:
            print(f"  {conn.topic}  ← {conn.msgtype}")

        count = 0
        for conn, timestamp, rawdata in reader.messages():
            topic = conn.topic

            if selected_topics and topic not in selected_topics:
                continue

            count += 1
            try:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                self._write_row(topic, timestamp, msg, conn.msgtype)
            except Exception as e:
                # 某些自定义消息可能无法反序列化，跳过
                pass

            if count % 10000 == 0:
                print(f"  已处理 {count} 条消息...")

        reader.close()

        # 写入合并 CSV
        self._write_merged()

        # 关闭所有文件
        for f in self.files.values():
            f.close()

        print(f"\n导出完成! 共处理 {count} 条消息")
        print(f"输出目录: {self.output_dir}")
        for topic in self.csv_writers:
            safe_name = topic.strip('/').replace('/', '_')
            print(f"  {safe_name}.csv")

    def _write_row(self, topic, timestamp, msg, msgtype):
        """将一条消息转为 CSV 行"""
        writer = self._get_writer(topic)
        t = timestamp / 1e9  # ns → seconds

        row = {'timestamp': t}

        try:
            if 'VehicleAttitude' in msgtype:
                q = msg.q
                if hasattr(q, '__iter__'):
                    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
                else:
                    w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
                roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
                row.update({'w': w, 'x': x, 'y': y, 'z': z,
                            'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': yaw})

            elif 'VehicleLocalPosition' in msgtype:
                row.update({'x': float(msg.x), 'y': float(msg.y), 'z': float(msg.z),
                            'vx': float(getattr(msg, 'vx', 0)),
                            'vy': float(getattr(msg, 'vy', 0)),
                            'vz': float(getattr(msg, 'vz', 0))})

            elif 'LaserScan' in msgtype:
                ranges_str = json.dumps(list(msg.ranges)[:10])  # 只存前10个避免过大
                row.update({'angle_min': float(msg.angle_min),
                            'angle_max': float(msg.angle_max),
                            'angle_increment': float(msg.angle_increment),
                            'range_min': float(msg.range_min),
                            'range_max': float(msg.range_max),
                            'ranges_count': len(msg.ranges),
                            'ranges': ranges_str})

            elif 'Imu' in msgtype:
                row.update({
                    'orientation_x': float(msg.orientation.x),
                    'orientation_y': float(msg.orientation.y),
                    'orientation_z': float(msg.orientation.z),
                    'orientation_w': float(msg.orientation.w),
                    'angular_velocity_x': float(msg.angular_velocity.x),
                    'angular_velocity_y': float(msg.angular_velocity.y),
                    'angular_velocity_z': float(msg.angular_velocity.z),
                    'linear_acceleration_x': float(msg.linear_acceleration.x),
                    'linear_acceleration_y': float(msg.linear_acceleration.y),
                    'linear_acceleration_z': float(msg.linear_acceleration.z),
                })

            elif 'Detection2DArray' in msgtype:
                dets = []
                for d in msg.detections:
                    dets.append({
                        'class': d.results[0].hypothesis.class_id if d.results else -1,
                        'score': float(d.results[0].hypothesis.score) if d.results else 0,
                        'x': float(d.bbox.center.x),
                        'y': float(d.bbox.center.y),
                        'w': float(d.bbox.size_x),
                        'h': float(d.bbox.size_y),
                    })
                row['data'] = json.dumps(dets, ensure_ascii=False)

            elif 'String' in msgtype:
                row['data'] = str(msg.data)

            else:
                # 通用: 尝试序列化
                row['data'] = str(msg)

        except Exception as e:
            row['data'] = f"SERIALIZE_ERROR: {e}"

        writer.writerow(row)

    def _write_merged(self):
        """暂未实现合并CSV (需要跨话题时间对齐)"""
        pass


def list_topics(bag_path):
    """列出 bag 中的所有话题"""
    try:
        from rosbags.rosbag2 import Reader
    except ImportError:
        print("错误: 需要 rosbags 库: pip install rosbags")
        sys.exit(1)

    reader = Reader(bag_path)
    reader.open()
    print(f"Bag: {bag_path}")
    print(f"话题列表:")
    for conn in reader.connections:
        print(f"  {conn.topic:50s} ← {conn.msgtype}")
    reader.close()


def main():
    parser = argparse.ArgumentParser(description="从 ros2 bag 导出 CSV")
    parser.add_argument('bag_path', help='ros2 bag 路径')
    parser.add_argument('-o', '--output', default='.', help='输出目录 (默认: 当前目录)')
    parser.add_argument('-t', '--topics', help='指定话题 (逗号分隔)')
    parser.add_argument('--list', action='store_true', help='仅列出话题')
    args = parser.parse_args()

    if args.list:
        list_topics(args.bag_path)
        return

    selected = None
    if args.topics:
        selected = set(args.topics.split(','))

    exporter = BagExporter(args.bag_path, args.output)
    exporter.export(selected)


if __name__ == '__main__':
    main()
