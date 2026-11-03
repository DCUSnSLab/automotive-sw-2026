#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[9주차 빈칸 파일] gap_finder — 조향 전에 gap 을 눈으로.

전방 ±fov_deg 창에서 threshold 를 넘는 연속 구간(gap)을 찾아
가장 넓은 gap 의 폭·중심 각도·가장 가까운 점을 0.5 s 마다 로그로 출력.
(제어 없음 — 7주차 wall_finder 와 같은 '보기' 노드)

'# TODO' 표시된 곳만 채운다. 번호는 이론 슬라이드의 6단계와 같다.
    TODO ① 전처리      (이론 12p)   TODO ② 최근접점 (이론 13p)   TODO ④ gap 탐색 (이론 16~17p)
    (③ 안전 거품 ⑤ 목표각 ⑥ 조향 은 fgm_drive 에서)

    ros2 run basics gap_finder
    ros2 param set /gap_finder threshold 3.0        # 실행 중 튜닝
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class GapFinder(Node):
    def __init__(self):
        super().__init__('gap_finder')
        self.declare_parameter('threshold', 2.5)     # gap 판정 거리 t [m]
        self.declare_parameter('fov_deg', 60.0)      # 전방 창 반각 [deg]
        self.declare_parameter('body_min', 0.8)      # 이보다 가까운 값 = 자기 차체 반사
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        t = self.get_parameter('threshold').value
        fov = math.radians(self.get_parameter('fov_deg').value)
        body = self.get_parameter('body_min').value
        c = len(msg.ranges) // 2                      # 정면 = 360
        w = int(fov / msg.angle_increment)           # 60° = 120빔
        lo = c - w                                    # 창의 첫 빔 = 240

        # TODO ① 전처리 — 전방 창(lo ~ c+w)을 rng 에 복사한다.
        #    inf(측정 범위 밖 = 뚫림) 는 msg.range_max 로, body 보다 가까운 값(차체 반사)도 msg.range_max 로.
        #    힌트: for i in range(lo, c + w + 1): r = msg.ranges[i] ... rng.append(r)
        rng = []

        # TODO ② 최근접점 — rng 에서 가장 짧은 빔의 번호 i_min, 거리 d_min
        #    힌트: i_min = 0 에서 시작해 for 문으로 더 짧은 빔이 나오면 i_min 갱신
        i_min = 0
        d_min = 0.0
        a_min = math.degrees((lo + i_min - c) * msg.angle_increment)

        # TODO ④ gap 탐색 — rng[i] > t 인 연속 구간 중 가장 긴 것
        #    결과: n_gap(구간 개수), best_len(최장 폭[빔]), best_s(시작 번호), best_e(끝 번호)
        #    힌트: s = -1 (구간 없음) 로 시작. free 이고 s < 0 이면 시작, free 가 아니고 s >= 0 이면 끝(길이 비교).
        #          for i in range(len(rng) + 1) 로 한 칸 더 돌아 마지막 열린 구간을 닫는다 (이론 17p 추적표).
        n_gap = 0
        best_len = 0
        best_s = 0
        best_e = 0

        if best_len > 0:
            mid = (best_s + best_e) // 2
            a_mid = math.degrees((lo + mid - c) * msg.angle_increment)
            self.get_logger().info(
                'gap %d개 | 최대 %d빔(%.0f°) 중심 %+.0f° | 최근접 %.2f m @ %+.0f°'
                % (n_gap, best_len, best_len * math.degrees(msg.angle_increment),
                   a_mid, d_min, a_min),
                throttle_duration_sec=0.5)
        else:
            self.get_logger().info(
                'gap 없음 | 최근접 %.2f m @ %+.0f°' % (d_min, a_min),
                throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = GapFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
