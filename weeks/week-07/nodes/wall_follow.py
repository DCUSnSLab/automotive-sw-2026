#!/usr/bin/env python3
"""[7주차 단계 3~5] wall_follow — 오른쪽 벽 추종 (P / PD 제어) + 전방 정지.  ★ 빈칸 버전 ★

6주차 aeb_distance 의 뼈대(파라미터 → 콜백 → 10 Hz 타이머)는 완성되어 있다.
'# ★ TODO' 표시된 곳만 단계별로 채운다.  (완성본은 다음 주 공개)

    단계 3 (P)   : TODO ① 오른쪽 거리   ② 오차   ③ 조향·클램프
    단계 4 (D)   : TODO ④ 오차 변화율(차분 → EMA)
    단계 5 (안전): TODO ⑤-a 전방 거리 (on_scan)   ⑤-b 전방 정지 (tick)  — 6주차 본인 AEB 이식

    ros2 run basics wall_follow --ros-args -p kp:=0.2 -p kd:=0.0      # 단계 3: P 만
    ros2 param set /wall_follow kp 0.6                               # 실행 중 튜닝
    ros2 param set /wall_follow kd 1.5                               # 단계 4: D 항

부호 규약: e = 오른쪽거리 − target.  멀면(e > 0) 오른쪽(−) 으로 꺾어야 한다.
⚠ 단계 5 전에는 브레이크가 없다 — 연석에 가까워지면 Ctrl+C 후 차량 리셋.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

MAX_STEER = 0.40      # 조향 클램프 [rad] — 물리 한계(0.46)보다 안쪽
EMA_ALPHA = 0.3       # 미분항 저역 필터 계수 — 6주차 TTC 와 같은 처방


def sector_min(msg, center_deg, half_deg):
    """center_deg 방향 ±half_deg 창에서 유효한 거리값의 최솟값. 유효값이 없으면 -1.0  (단계 2와 동일)"""
    center = len(msg.ranges) // 2 + int(math.radians(center_deg) / msg.angle_increment)
    half = int(math.radians(half_deg) / msg.angle_increment)
    best = -1.0
    for i in range(center - half, center + half + 1):
        r = msg.ranges[i]
        if not math.isfinite(r):
            continue
        if r <= msg.range_min:
            continue
        if best < 0.0 or r < best:
            best = r
    return best


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        # ---- 파라미터: 6주차와 같은 방식 (선언 → tick 마다 읽기) ----
        self.declare_parameter('speed', 1.0)          # [m/s] 순항 속도
        self.declare_parameter('front_stop', 2.5)     # [m]   전방 정지 거리 (단계 5에서 사용)
        self.declare_parameter('target_dist', 1.5)    # [m]   오른쪽 벽과의 목표 거리
        self.declare_parameter('kp', 0.2)             # 비례 게인   [rad/m]
        self.declare_parameter('kd', 0.0)             # 미분 게인 (0 이면 순수 P)
        self.declare_parameter('log_csv', '')         # 과제 1: 기록 파일 경로 (비우면 기록 안 함)

        # ---- 상태 변수 (전부 미리 만들어 둠 — TODO 에서 값을 채운다) ----
        self.front = 0.0                              # 전방 거리 [m]                 (단계 5)
        self.front_ok = False                         # 전방 거리가 유효한가          (단계 5)
        self.front_last = -1.0                        # 바로 전 프레임의 전방 최솟값  (단계 5, 6주차 2프레임 필터)
        self.right = 0.0                              # 오른쪽 벽 거리 [m]            (단계 3)
        self.right_ok = False                         # 오른쪽 거리가 유효한가        (단계 3)
        self.prev_e = 0.0                             # 지난 tick 의 오차             (단계 4)
        self.have_prev = False                        # 지난 오차가 있는가            (단계 4)
        self.de_f = 0.0                               # 필터된 오차 변화율            (단계 4)

        # ---- 인지(구독) · 제어(발행) · 제어 주기(타이머): 6주차 그대로 ----
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

        # ---- 과제 1 기록용 (수업 중에는 신경 쓰지 않아도 됨) ----
        self.t0 = self.get_clock().now()
        self.log = None
        path = self.get_parameter('log_csv').value
        if path != '':
            self.log = open(path, 'w')
            self.log.write('t,d,e,steer\n')

    # ---------------- 인지: /scan 콜백 (20 Hz) ----------------
    def on_scan(self, msg):
        # ★ TODO ① [단계 3] 오른쪽 벽 거리
        #    오른쪽 −90° ±4° 창의 최솟값을 self.right 에, 유효(0 이상) 여부를 self.right_ok 에.
        #    힌트: 단계 2 wall_finder 의 sector_min() 한 줄 + 비교 한 줄


        # ★ TODO ⑤-a [단계 5] 전방 거리 — 6주차 aeb_distance 의 on_scan 이식
        #    전방 0° ±10° 창의 최솟값 → self.front, self.front_ok
        #    (6주차 2프레임 필터: 직전 값 self.front_last 가 더 크면 그 값을 쓴다 — 1프레임 튀는 점 무시)
        pass

    # ---------------- 판단 + 제어: 10 Hz 타이머 ----------------
    def tick(self):
        speed = self.get_parameter('speed').value      # tick 마다 읽어야 param set 이 반영된다
        front_stop = self.get_parameter('front_stop').value
        target = self.get_parameter('target_dist').value
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        msg = Twist()                                  # 기본값: 속도 0, 조향 0

        # ★ TODO ⑤-b [단계 5] 전방 정지 — 6주차 aeb_distance 의 tick 규칙 이식
        #    전방 거리가 유효하고 front_stop 보다 가까우면: 정지 명령(msg 그대로) 발행 후 return


        if not self.right_ok:                          # 오른쪽 벽이 안 보이면 멈춘다
            self.pub.publish(msg)
            return

        # ★ TODO ② [단계 3] 오차 — 멀면 +, 가까우면 −
        e = 0.0

        # ★ TODO ④ [단계 4] 오차 변화율 de (D 항)
        #    차분 (e − self.prev_e) / 0.1 → EMA 필터로 self.de_f 갱신 → de 에 대입
        #    첫 tick 은 비교 대상이 없으므로 self.have_prev 로 구분, 끝에서 prev_e·have_prev 갱신
        de = 0.0

        # ★ TODO ③ [단계 3] 조향 = −(Kp·e + Kd·de)  (오른쪽 벽: 멀면 오른쪽(−) 으로)
        #    그리고 ±MAX_STEER 로 클램프 (if / elif)
        steer = 0.0

        msg.linear.x = speed
        msg.angular.z = steer
        self.pub.publish(msg)

        if self.log is not None:
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            self.log.write('%.2f,%.3f,%.3f,%.3f\n' % (t, self.right, e, steer))
            self.log.flush()


def main():
    rclpy.init()
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())                          # 종료 시 정지 명령 1회
    if node.log is not None:
        node.log.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
