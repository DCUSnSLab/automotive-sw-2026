#!/usr/bin/env python3
"""[7주차 단계 2] wall_finder — 오른쪽 벽 거리 측정 (제어 없음, '보기' 전용).

6주차 scan_watcher 의 전방 ±15° 창을 오른쪽 −90° 로 옮긴 것. 새 문법 없음.
    ros2 run basics wall_finder
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def sector_min(msg, center_deg, half_deg):
    """center_deg 방향 ±half_deg 창에서 유효한 거리값의 최솟값. 유효값이 없으면 -1.0 을 돌려준다."""
    center = len(msg.ranges) // 2 + int(math.radians(center_deg) / msg.angle_increment)
    half = int(math.radians(half_deg) / msg.angle_increment)
    best = -1.0                                   # -1.0 = 아직 유효값 없음
    for i in range(center - half, center + half + 1):
        r = msg.ranges[i]
        if not math.isfinite(r):                  # inf / NaN 은 버린다
            continue
        if r <= msg.range_min:                    # 센서 자체 노이즈(너무 가까운 값)도 버린다
            continue
        if best < 0.0 or r < best:
            best = r
    return best


class WallFinder(Node):
    def __init__(self):
        super().__init__('wall_finder')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        right = sector_min(msg, -90.0, 4.0)       # 오른쪽 연석: 정면(360) − 180 = 인덱스 180 주변
        front = sector_min(msg, 0.0, 10.0)        # 전방
        self.get_logger().info('right: %.2f m   front: %.2f m' % (right, front),
                               throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = WallFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
