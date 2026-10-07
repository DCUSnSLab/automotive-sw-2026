#!/usr/bin/env python3
"""LaserScan 해부 — /scan 한 프레임을 받아 메타데이터와 네 방향의 거리를 출력한다.

    python3 scan_dump.py        (한 프레임 출력 후 종료)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data          # 센서 구독의 생존 규칙
from sensor_msgs.msg import LaserScan


class ScanDump(Node):
    def __init__(self):
        super().__init__('scan_dump')
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        # 1) 메타데이터
        print('점 개수        :', len(msg.ranges))
        print('angle_min (rad):', msg.angle_min)
        print('angle_increment:', msg.angle_increment)
        print('range_min/max  :', msg.range_min, '/', msg.range_max)

        # 2) 네 방향의 거리 — 인덱스를 직접 넣어 본다
        print('뒤쪽   i=0   :', round(msg.ranges[0], 2), 'm')
        print('오른쪽 i=180 :', round(msg.ranges[180], 2), 'm')
        print('정면   i=360 :', round(msg.ranges[360], 2), 'm')
        print('왼쪽   i=540 :', round(msg.ranges[540], 2), 'm')

        # 3) i=360 이 왜 정면인가 — 각도 = angle_min + i * angle_increment
        angle = msg.angle_min + 360 * msg.angle_increment
        print('i=360 의 각도  :', round(math.degrees(angle), 1), '도')

        raise SystemExit                                  # 한 프레임만 보고 끝


def main():
    rclpy.init()
    node = ScanDump()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
