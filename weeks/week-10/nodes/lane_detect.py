#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[10주차 빈칸 파일] lane_detect — 카메라 영상 1장 → 차로 중심 offset 1개.

① 축소·HSV 노란색 마스크 → ② 조감도(BEV) 변환 → ③ 행 스캔으로 좌·우 차선 x
→ ④ 차로 중심과 offset[m] (+ = 차로 중심이 차보다 왼쪽).
③ 은 조감도의 가까운 행 3개에서 '직전 차로 중심'부터 오른쪽·왼쪽으로 훑어 처음 만나는 노란 픽셀을 차선으로 본다.
④ 는 오른쪽 차선 우선: 중심 = 오른쪽 x - 차로 폭/2 (곡선에서는 오른쪽 실선만 보이는 일이 많다 — 7주차 오른쪽 벽 기준과 같은 생각).

'# TODO' 표시된 곳만 채운다. 번호는 이론 슬라이드의 처리 단계와 같다.
    TODO ① 노란색 마스크   TODO ② 조감도 변환   TODO ③ 행 스캔   TODO ④ 차로 중심과 offset
    ros2 run basics lane_detect
    ros2 run rqt_image_view rqt_image_view /lane/debug      # 조감도·찾은 차선 확인

발행:
  /lane/offset  std_msgs/Float32  차로 중심 - 차 중심 [m] (+ 왼쪽). 차선을 못 찾으면 NaN
  /lane/debug   sensor_msgs/Image BEV 마스크 + 찾은 차선·중심 표시 (rqt_image_view 로 확인, 5 Hz)

파라미터:
  h_lo, h_hi, s_lo, v_lo : 노란색 HSV 범위 (OpenCV H 0~179)
  lane_width            : 두 차선 안쪽 가장자리 사이 [m] (시험장 실측 3.1 m)
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

PW, PH = 320, 180            # 처리 해상도 (카메라가 더 크면 먼저 줄인다)
BW, BH = 200, 200            # 조감도 크기 [px] — 앞 2~8 m, 좌우 ±3 m
M_PER_PX = 6.0 / BW          # 조감도 가로 1 px = 0.03 m
# 조감도 4점: 지면 사각형(앞 8 m·2 m, 좌우 ±3 m)이 320x180 영상에 비치는 위치 (카메라 높이 0.81 m, 아래로 0.1 rad)
SRC = [[109.9, 90.1], [210.1, 90.1], [354.5, 129.3], [-34.5, 129.3]]
DST = [[0, 0], [BW - 1, 0], [BW - 1, BH - 1], [0, BH - 1]]
ROWS = [170, 150, 130]       # 훑을 조감도 행 = 앞 약 2.9 / 3.5 / 4.1 m (조감도 세로 1 px = 0.03 m)


class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')
        self.declare_parameter('h_lo', 20)
        self.declare_parameter('h_hi', 35)
        self.declare_parameter('s_lo', 80)
        self.declare_parameter('v_lo', 120)
        self.declare_parameter('lane_width', 3.1)     # 두 차선 안쪽 가장자리 사이 [m]
        self.bridge = CvBridge()
        self.M = cv2.getPerspectiveTransform(np.float32(SRC), np.float32(DST))
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, '/lane/offset', 10)
        self.pub_dbg = self.create_publisher(Image, '/lane/debug', 1)
        self.center = BW / 2.0       # 직전 차로 중심 [조감도 px] — 다음 프레임 행 스캔의 시작점
        self.count = 0

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[1] != PW:
            img = cv2.resize(img, (PW, PH))

        # TODO ① 노란색 마스크 — img 를 HSV 로 바꾼 뒤(cv2.cvtColor) 노란색 범위만 255 로(cv2.inRange)
        #    범위: 아래 (h_lo, s_lo, v_lo) ~ 위 (h_hi, 255, 255) — 파라미터에서 읽는다
        mask = np.zeros((PH, PW), np.uint8)

        # TODO ② 조감도 — mask 를 self.M 으로 원근 변환해 BW x BH 크기의 bev 로 (cv2.warpPerspective)
        bev = np.zeros((BH, BW), np.uint8)

        # TODO ③ 행 스캔 — ROWS 의 행 y 마다, 시작점 start(직전 차로 중심)에서
        #    오른쪽(x 증가)으로 훑어 처음 만나는 노란 픽셀(bev[y, x] > 0) → r_sum 에 x 를 더하고 r_cnt += 1
        #    왼쪽(x 감소)으로 훑어 처음 만나는 노란 픽셀 → l_sum, l_cnt
        #    시작점이 선 위(bev[y, start] > 0)면 그 행은 건너뛴다.  2개 행 이상에서 찾았을 때만 평균을 차선 x 로
        start = int(self.center)
        if start < 1:
            start = 1
        if start > BW - 2:
            start = BW - 2
        r_sum = 0
        r_cnt = 0
        l_sum = 0
        l_cnt = 0
        right_x = -1.0
        left_x = -1.0

        # TODO ④ 차로 중심 — 오른쪽 차선 우선: right_x 가 있으면 center_x = right_x - half_px,
        #    없고 left_x 만 있으면 center_x = left_x + half_px, 둘 다 없으면 -1 (아래에서 NaN 발행)
        half_px = self.get_parameter('lane_width').value / 2.0 / M_PER_PX
        center_x = -1.0
        out = Float32()
        if center_x < 0:
            out.data = float('nan')
        else:
            self.center = center_x
            out.data = (BW / 2.0 - center_x) * M_PER_PX           # 왼쪽(+)
        self.pub.publish(out)

        # 디버그 영상 (5 Hz 정도로 솎아서)
        self.count += 1
        if self.count % 3 == 0 and self.pub_dbg.get_subscription_count() > 0:
            dbg = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
            for y in ROWS:
                cv2.line(dbg, (0, y), (BW - 1, y), (80, 80, 80), 1)
            if left_x >= 0:
                cv2.circle(dbg, (int(left_x), ROWS[1]), 5, (255, 0, 0), -1)
            if right_x >= 0:
                cv2.circle(dbg, (int(right_x), ROWS[1]), 5, (0, 0, 255), -1)
            if center_x >= 0:
                cv2.circle(dbg, (int(center_x), ROWS[1]), 5, (0, 255, 0), -1)
            cv2.circle(dbg, (BW // 2, ROWS[1]), 5, (255, 255, 255), 1)
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetect()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
