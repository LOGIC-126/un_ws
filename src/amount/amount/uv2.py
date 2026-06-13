"""
接收UDP检测结果并打印中心像素坐标（单目测量节点）
"""

import socket
import json
import threading

import rclpy
from rclpy.node import Node

# ============ 配置 ============
UDP_HOST = '127.0.0.1'
UDP_PORT = 8888
BUFFER_SIZE = 4096

CLASSES = ["elephant", "tiger", "wolf", "monkey", "peacock"]


class MocMea_Node(Node):

    def __init__(self):
        super().__init__('monocular_measurement_node')

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(1.0)
        self.udp_running = True

        self.udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.udp_thread.start()

        self.get_logger().info(f'UDP监听启动: {UDP_HOST}:{UDP_PORT}')

    def _udp_loop(self):
        try:
            self.udp_socket.bind((UDP_HOST, UDP_PORT))
        except OSError as e:
            self.get_logger().error(f'UDP绑定失败: {e}')
            return

        while self.udp_running:
            try:
                data, _ = self.udp_socket.recvfrom(BUFFER_SIZE)
                msg = json.loads(data.decode('utf-8'))
                self._process_detections(msg)
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                self.get_logger().debug('收到无效JSON')
            except OSError as e:
                if self.udp_running:
                    self.get_logger().error(f'UDP接收错误: {e}')
                break

    def _process_detections(self, data):
        detections = data.get('detections', [])
        if not detections:
            return

        for det in detections:
            class_id = det.get('class_id', -1)
            if 0 <= class_id < len(CLASSES):
                class_name = CLASSES[class_id]
            else:
                class_name = f"unk({class_id})"

            # 中心坐标：优先使用 center，否则从 bbox 计算
            if 'center' in det:
                cx, cy = det['center']
            else:
                x1, y1, x2, y2 = det.get('bbox', (0, 0, 0, 0))
                cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            self.get_logger().info(f'{class_name} ({cx:.1f}, {cy:.1f})')

    def stop(self):
        self.udp_running = False
        if self.udp_socket:
            self.udp_socket.close()
        self.udp_thread.join(timeout=2.0)
        self.get_logger().info('节点停止')


def main():
    rclpy.init()
    node = MocMea_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()