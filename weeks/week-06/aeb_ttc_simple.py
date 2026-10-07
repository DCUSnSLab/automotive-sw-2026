#!/usr/bin/env python3
"""17~21p 를 한 파일로 — 거리 전처리(18p) → 접근속도·EMA(21p) → TTC(19p) → 판단·제어(19p)

    python3 aeb_ttc_simple.py        (시뮬레이터 + 전방 15 m 박스를 놓고 실행)

이론 슬라이드의 코드 조각이 실제로 어떻게 한 노드 안에서 연결되는지 보여 주는 최소 버전.
(파라미터·정지 해제 등은 뺐다 — 실습용 완성본은 basics/aeb_ttc.py)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

TTC_THRESHOLD = 2.0     # [s]   20p: 반응 지연 0.3 s + 제동 1.0 s + 안전 여유
SPEED = 1.0             # [m/s] 순항 속도
ALPHA = 0.3             # 21p: EMA 계수 (작을수록 부드럽지만 느리다)


class AebTtcSimple(Node):
    def __init__(self):
        super().__init__('aeb_ttc_simple')
        # 인지(구독) · 제어(발행) · 제어 주기(타이머)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)          # 10 Hz

        self.front = None       # 전방 최소거리 [m]        (18p)
        self.last_min = None    # 바로 전 프레임의 최솟값  (14p: 1프레임 튀는 점 무시용)
        self.prev = None        # 지난 스캔의 전방 거리    (21p)
        self.prev_t = None      # 지난 스캔의 시각         (21p)
        self.v = 0.0            # 접근속도 [m/s], + 면 가까워지는 중 (21p)
        self.stopped = False    # 한 번 멈추면 유지        (22p 채터링)

    # ---------------- 인지: /scan 콜백, 20 Hz ----------------
    def on_scan(self, msg):
        # 18p ① 전방 ±15° 창  ② 유효값  ③ 최솟값
        c = len(msg.ranges) // 2                                 # 정면 인덱스 360
        w = int(math.radians(15) / msg.angle_increment)          # 15° = 30칸
        valid = []
        for r in msg.ranges[c - w: c + w + 1]:                   # 61개
            if math.isfinite(r) and r > msg.range_min:
                valid.append(r)
        if len(valid) == 0:
            self.front = None
            return
        m = min(valid)

        # 14p 센서 데이터는 더럽다: 두 프레임 연속으로 가까워야 믿는다 (1프레임 글리치 무시)
        if self.last_min is None:
            d = m
        else:
            d = max(m, self.last_min)
        self.last_min = m

        # 21p 접근속도: 차분 → EMA
        now = self.get_clock().now()
        if self.prev is not None:                                # 첫 스캔은 비교 대상이 없다
            dt = (now - self.prev_t).nanoseconds * 1e-9          # 경과 시간 ≈ 0.05 s
            v_raw = (self.prev - d) / dt                         # 차분: + 면 가까워지는 중
            self.v = ALPHA * v_raw + (1 - ALPHA) * self.v        # EMA
        self.prev = d
        self.prev_t = now
        self.front = d

    # ---------------- 19p TTC = 거리 / 접근속도 ----------------
    def ttc(self):
        if self.front is None or self.v <= 0.05:                 # 멀어지거나 사실상 정지 → 위험 없음
            return math.inf
        return self.front / self.v

    # ---------------- 판단 + 제어: 10 Hz 타이머 ----------------
    def tick(self):
        cmd = Twist()
        if self.stopped:                                         # 22p: 멈춘 뒤 v→0, TTC→∞ 로 재출발하지 않도록 유지
            cmd.linear.x = 0.0
        elif self.ttc() < TTC_THRESHOLD:                         # 19p 판단 규칙
            cmd.linear.x = 0.0
            self.stopped = True
            self.get_logger().warn('STOP  front=%.2f m  v=%.2f m/s  TTC=%.2f s'
                                   % (self.front, self.v, self.ttc()))
        else:
            cmd.linear.x = SPEED
            if self.front is not None:
                self.get_logger().info('front=%.2f m  v=%.2f m/s  TTC=%s'
                                       % (self.front, self.v,
                                          'inf' if math.isinf(self.ttc()) else '%.2f s' % self.ttc()),
                                       throttle_duration_sec=0.5)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AebTtcSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())                                # 종료 시 정지 명령 1회
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
