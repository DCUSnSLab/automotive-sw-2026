#!/usr/bin/env python3
"""[6주차 단계 2] scan_watcher — /scan 전처리(창·필터·최솟값) 후 전방 최소거리를 출력한다.

    ros2 run basics scan_watcher
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data          # 센서 구독의 생존 규칙
from sensor_msgs.msg import LaserScan


class ScanWatcher(Node):
    def __init__(self):
        super().__init__('scan_watcher')
        self.front = None
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        c = len(msg.ranges) // 2                                     # 정면 인덱스 (360)
        w = int(math.radians(15.0) / msg.angle_increment)           # ±15° → 30개
        window = msg.ranges[c - w: c + w + 1]                        # ① 전방 창 자르기 (61개)
        valid = []                                                   # ② 유효값 필터
        for r in window:
            if math.isfinite(r) and r > msg.range_min:               #    inf 아니고, 너무 가깝지 않은 값만
                valid.append(r)
        if valid:
            self.front = min(valid)                                  # ③ 최솟값 (과제: 하위 5개 평균)
            self.get_logger().info(
                'front: %.2f m  (유효 %d/%d점)' % (self.front, len(valid), len(window)))
        else:
            self.front = None
            self.get_logger().info('front: ---  (전방 ±15° 에 유효한 점 없음)')


def main():
    rclpy.init()
    node = ScanWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
