#!/usr/bin/env python3
"""[6주차 단계 4] AEB v1 — 거리 기반 긴급제동.

    규칙: 전방 최소거리 < stop_dist 이면 정지, 아니면 speed 로 전진
    ros2 run basics aeb_distance
    ros2 param set /aeb_distance stop_dist 3.0
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AebDistance(Node):
    def __init__(self):
        super().__init__('aeb_distance')
        self.declare_parameter('stop_dist', 5.0)      # [m]  정지 거리
        self.declare_parameter('speed', 1.0)          # [m/s] 순항 속도
        self.front = None
        self.recent = deque(maxlen=2)   # 최근 2프레임의 최솟값 — 1프레임 튀는 점(self-hit 글리치) 무시
        self.stopped = False
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 제어는 10 Hz 타이머

    def on_scan(self, msg):                               # 인지는 콜백
        c = len(msg.ranges) // 2
        w = int(math.radians(15.0) / msg.angle_increment)
        valid = []
        for r in msg.ranges[c - w: c + w + 1]:                     # 전방 ±15° 창
            if math.isfinite(r) and r > msg.range_min:            # 유효값만
                valid.append(r)
        if not valid:
            self.front = None
            return
        self.recent.append(min(valid))
        self.front = max(self.recent)      # 2프레임 연속으로 가까워야 '가깝다'고 믿는다

    def tick(self):
        stop = self.get_parameter('stop_dist').value     # tick 마다 읽어야 param set 이 반영된다
        msg = Twist()
        if self.front is not None and self.front < stop:
            msg.linear.x = 0.0
            if not self.stopped:
                self.get_logger().warn('STOP  front=%.2f m < stop_dist=%.1f m' % (self.front, stop))
            self.stopped = True
        else:
            msg.linear.x = self.get_parameter('speed').value
            self.stopped = False
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = AebDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())                         # 종료 시 정지 명령 1회
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
