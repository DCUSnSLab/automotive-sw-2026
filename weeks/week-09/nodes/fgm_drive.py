#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[9주차 빈칸 파일] fgm_drive — Follow the Gap 주행.

'# TODO' 표시된 곳만 채운다. 번호는 이론 슬라이드의 6단계와 같다.
    TODO ① 전처리  ② 최근접점  ④ gap 탐색   → 단계 1 의 gap_finder 에서 그대로 복사
    TODO ③ 안전 거품 (이론 14~15p)   ⑤ 목표각 (이론 18p)   ⑥ 조향·감속 (이론 21p·25p)
뼈대(파라미터·전방 정지·결승선 FINISH·10 Hz 타이머)는 완성되어 있다.

    ros2 run basics fgm_drive
    ros2 param set /fgm_drive threshold 3.0          # 실행 중 튜닝

파라미터:
  threshold      : gap 판정 거리 t [m]
  bubble_radius  : 안전 거품 반경 r_b [m] (차 반폭 + 여유)
  min_gap_beams  : 통과 가능 최소 gap 폭 n [빔]
  fov_deg        : 전방 창 반각 [deg]
  steer_gain     : 목표각 → 조향 민감도
  speed          : 기본 전진 속도 [m/s] (조향 비례 감속)
  front_stop     : 전방 정지 거리 [m] (안전용)
  finish_dist    : 출발점에서 이 거리만큼 가면 FINISH [m] (결승선)
  body_min       : 이보다 가까운 값은 차체 반사로 보고 무시 [m]
  log_csv        : 경로를 주면 t,aim,steer,speed 기록
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

MAX_STEER = 0.40     # 조향 클램프 [rad] — 7주차와 동일


class FgmDrive(Node):
    def __init__(self):
        super().__init__('fgm_drive')
        self.declare_parameter('threshold', 2.5)
        self.declare_parameter('bubble_radius', 0.5)
        self.declare_parameter('min_gap_beams', 30)
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('steer_gain', 1.0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('front_stop', 1.0)
        self.declare_parameter('finish_dist', 28.0)
        self.declare_parameter('body_min', 0.8)
        self.declare_parameter('log_csv', '')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz
        # ---- 상태 변수 (전부 미리 만들어 둠 — TODO 에서 값을 채운다) ----
        self.aim = None          # 목표각 [rad] (정면 기준, + = 왼쪽). None = 지나갈 틈 없음   (TODO ⑤)
        self.front = None        # 전방 ±10° 최소 거리 [m]                                   (완성)
        self.got_scan = False    # 첫 스캔 수신 여부
        self.x = None            # 현재 위치 (odom)
        self.y = None
        self.x0 = None           # 출발 위치 (첫 주행 명령 직후)
        self.y0 = None
        self.t_start = None      # 첫 주행 명령 시각 (기록 계측)
        self.finished = False
        self.log = None
        p = self.get_parameter('log_csv').value
        if p:
            self.log = open(p, 'w')
            self.log.write('t,aim,steer,speed\n')

    # ---------- 인지: 스캔 1장 → 목표각 1개 ----------
    def on_scan(self, msg):
        self.got_scan = True
        body = self.get_parameter('body_min').value
        c = len(msg.ranges) // 2                      # 정면 = 360
        fov = math.radians(self.get_parameter('fov_deg').value)
        w = int(fov / msg.angle_increment)
        lo = c - w

        # 전방 정지용 최소 거리 (±10°) — 6주차 AEB 와 같은 창 (완성)
        wf = int(math.radians(10.0) / msg.angle_increment)
        self.front = None
        for i in range(c - wf, c + wf + 1):
            r = msg.ranges[i]
            if math.isfinite(r) and r > body:
                if self.front is None or r < self.front:
                    self.front = r

        # TODO ① 전처리 — gap_finder 의 ① 을 그대로 복사 (inf·차체 반사 → range_max)
        rng = []

        # TODO ② 최근접점 — gap_finder 의 ② 를 그대로 복사 (i_min, d_min)
        #    단, d_min 이 0.1 보다 작으면 0.1 로 (③ 의 atan2 에서 0 나누기 방지)
        i_min = 0
        d_min = 1.0

        # TODO ③ 안전 거품 — 최근접점 주변 반각 half = int(atan2(r_b, d_min) / angle_increment) 빔을 거리 0 으로
        #    r_b = bubble_radius 파라미터. 범위는 0 ~ len(rng)-1 로 잘라낸다 (if 두 개).
        #    힌트: start = i_min - half, end = i_min + half → for i in range(start, end + 1): rng[i] = 0.0
        r_b = self.get_parameter('bubble_radius').value

        # TODO ④ gap 탐색 — gap_finder 의 ④ 를 그대로 복사 (best_len, best_s, best_e)
        t = self.get_parameter('threshold').value
        best_len = 0
        best_s = 0
        best_e = 0

        # TODO ⑤ 목표각 — best_len 이 n(min_gap_beams) 미만이면 self.aim = None
        #    아니면 mid = (best_s + best_e) // 2 의 각도: self.aim = (lo + mid - c) * msg.angle_increment
        n = self.get_parameter('min_gap_beams').value
        self.aim = None

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.t_start is not None and self.x0 is None:
            self.x0 = self.x                          # 출발점 (첫 주행 명령 직후 위치)
            self.y0 = self.y

    def dist_from_start(self):
        if self.x0 is None or self.x is None:
            return 0.0
        dx = self.x - self.x0
        dy = self.y - self.y0
        return math.sqrt(dx * dx + dy * dy)

    # ---------- 판단 + 제어: 10 Hz ----------
    def tick(self):
        msg = Twist()
        if self.finished:
            self.pub.publish(msg)                     # 결승 후 정지 유지
            return
        if not self.got_scan:
            return                                    # 아직 스캔 없음
        if self.t_start is not None and \
                self.dist_from_start() >= self.get_parameter('finish_dist').value:
            self.pub.publish(msg)                     # 결승선 통과 → 정지
            dt = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
            self.get_logger().info('FINISH  %.1f s' % dt)
            self.finished = True
            return
        if self.front is not None and \
                self.front < self.get_parameter('front_stop').value:
            self.pub.publish(msg)                     # 전방 정지 (안전)
            self.get_logger().info('FRONT STOP  %.2f m' % self.front,
                                   throttle_duration_sec=1.0)
            return
        if self.aim is None:
            self.pub.publish(msg)                     # gap 없음 — 정지
            self.get_logger().info('NO GAP — 정지', throttle_duration_sec=1.0)
            return

        # TODO ⑥ 조향 = steer_gain × self.aim, ±MAX_STEER 로 클램프 (if / elif)
        #    감속: ratio = |steer| / MAX_STEER, sp = speed × (1 − 0.5 × ratio)  — 꺾을수록 천천히
        steer = 0.0
        sp = 0.0

        msg.linear.x = sp
        msg.angular.z = steer
        if self.t_start is None:
            self.t_start = self.get_clock().now()     # 계측 시작
            self.x0 = self.x                          # 출발점 (odom 전이면 None → on_odom 에서 채움)
            self.y0 = self.y
        self.pub.publish(msg)
        if self.log:
            dt = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
            self.log.write('%.2f,%.3f,%.3f,%.2f\n' % (dt, self.aim, steer, sp))
            self.log.flush()


def main(args=None):
    rclpy.init(args=args)
    node = FgmDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.pub.publish(Twist())                     # 종료 시 정지 명령 1회
    if node.log:
        node.log.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
