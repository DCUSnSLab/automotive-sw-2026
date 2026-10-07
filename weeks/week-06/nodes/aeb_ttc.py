#!/usr/bin/env python3
"""[6주차 단계 5] AEB v2 — TTC(Time-To-Collision) 기반 긴급제동.

    접근속도 v = (지난 거리 - 지금 거리) / dt  → EMA 필터 → TTC = 거리 / v
    규칙: v > 0.05 이고 TTC < ttc_threshold 이면 정지
    ros2 run basics aeb_ttc
    ros2 param set /aeb_ttc ttc_threshold 1.0
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AebTtc(Node):
    def __init__(self):
        super().__init__('aeb_ttc')
        self.declare_parameter('ttc_threshold', 2.0)   # [s]  임계 TTC
        self.declare_parameter('speed', 1.0)           # [m/s] 순항 속도
        self.declare_parameter('alpha', 0.3)           # EMA 계수 (작을수록 부드럽고 느림)
        self.declare_parameter('hold_stop', True)      # 한 번 멈추면 유지 (False: 채터링 체험용)
        self.declare_parameter('release_dist', 6.0)    # [m]  이보다 멀어지면 정지 해제 (리셋·박스 제거)
        self.front = None
        self.recent = deque(maxlen=2)   # 최근 2프레임의 최솟값 — 1프레임 튀는 점(self-hit 글리치) 무시
        self.prev = None          # 지난 스캔의 전방 거리
        self.prev_t = None        # 지난 스캔 시각
        self.v = 0.0              # 필터된 접근속도 [m/s]
        self.stopped = False
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.log_timer = self.create_timer(0.5, self.report)

    def on_scan(self, msg):
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
        d = max(self.recent)               # 2프레임 연속으로 가까워야 '가깝다'고 믿는다
        now = self.get_clock().now()
        if self.prev is not None:
            dt = (now - self.prev_t).nanoseconds * 1e-9
            if dt > 1e-3:
                v_raw = (self.prev - d) / dt                      # 접근속도 (+: 가까워짐)
                a = self.get_parameter('alpha').value
                self.v = a * v_raw + (1.0 - a) * self.v           # EMA
        self.prev, self.prev_t = d, now
        self.front = d

    def ttc(self):
        if self.front is None or self.v <= 0.05:                  # 멀어지거나 정지 → 위험 없음
            return math.inf
        return self.front / self.v

    def tick(self):
        ttc_th = self.get_parameter('ttc_threshold').value
        msg = Twist()
        ttc = self.ttc()
        # 정지 유지(latch): 멈추면 v→0, TTC→∞ 가 되어 바로 재출발하는 채터링을 막는다.
        # 전방이 release_dist 보다 멀어질 때(리셋·박스 제거)만 해제 — '3초 유지' 는 과제 ③
        if self.stopped and self.get_parameter('hold_stop').value:
            if self.front is not None and self.front <= self.get_parameter('release_dist').value:
                self.pub.publish(msg)
                return
            self.get_logger().info('RELEASE  front=%s m > release_dist' % ('---' if self.front is None else '%.2f' % self.front))
            self.stopped = False
        if ttc < ttc_th:
            msg.linear.x = 0.0
            if not self.stopped:
                self.get_logger().warn('STOP  TTC=%.2f s < %.1f s  (front=%.2f m, v=%.2f m/s)'
                                       % (ttc, ttc_th, self.front, self.v))
            self.stopped = True
        else:
            msg.linear.x = self.get_parameter('speed').value
            self.stopped = False
        self.pub.publish(msg)

    def report(self):
        if self.front is None:
            return
        ttc = self.ttc()
        self.get_logger().info('front=%5.2f m  v=%5.2f m/s  TTC=%s' %
                               (self.front, self.v, 'inf' if math.isinf(ttc) else '%.2f s' % ttc))


def main():
    rclpy.init()
    node = AebTtc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
