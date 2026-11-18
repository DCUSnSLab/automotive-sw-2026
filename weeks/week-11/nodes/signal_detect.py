#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[11주차 빈칸 파일] signal_detect — 카메라 영상 1장 → 신호등 색 1개.

① 관심 영역(ROI) 자르기 → ② HSV 변환 → ③ 색 3개 마스크와 픽셀 수 → ④ 판정(빨강 우선) → ⑤ 연속 3프레임 확인.
켜진 불은 '선명하고(S 큼) 밝은(V 큼)' 픽셀이다. 그런 픽셀을 색상(H)으로 나눠 세고, 가장 위험한 색부터 본다.

'# TODO' 표시된 곳만 채운다. 번호는 이론 슬라이드의 처리 단계와 같다.
    TODO ① 관심 영역   TODO ② HSV 변환   TODO ③ 색 3개 마스크와 픽셀 수   TODO ④ 판정
    (⑤ 연속 3프레임 확인과 디버그 영상은 완성되어 있다)
    ros2 run basics signal_detect
    ros2 run rqt_image_view rqt_image_view /signal/debug      # 관심 영역·판정 확인

발행:
  /signal/state  std_msgs/String  'RED' / 'YELLOW' / 'GREEN' / 'NONE'
  /signal/size   std_msgs/Int32   이번 프레임에서 본 불의 픽셀 수 (가까울수록 큼, 안 보이면 0)
  /signal/debug  sensor_msgs/Image 관심 영역 표시 + 판정 결과 (rqt_image_view 로 확인, 4 Hz)

파라미터:
  s_lo, v_lo : 켜진 불의 채도·밝기 하한 (OpenCV S·V 0~255)
  min_px     : 이 픽셀 수 이상이어야 그 색 불이 켜진 것으로 본다
  hold       : 같은 결과가 이 프레임 수만큼 이어져야 발행 값을 바꾼다
"""
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

PW, PH = 640, 360                 # 처리 해상도 (카메라가 더 크면 먼저 줄인다)
ROI_X0, ROI_X1 = 270, 370         # 관심 영역 열: 영상 가운데 (정면 신호등만)
ROI_Y0, ROI_Y1 = 55, 150          # 관심 영역 행: 영상 위쪽 (지평선보다 위)


class SignalDetect(Node):
    def __init__(self):
        super().__init__('signal_detect')
        self.declare_parameter('s_lo', 120)
        self.declare_parameter('v_lo', 150)
        self.declare_parameter('min_px', 4)
        self.declare_parameter('hold', 3)
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.pub_state = self.create_publisher(String, '/signal/state', 10)
        self.pub_size = self.create_publisher(Int32, '/signal/size', 10)
        self.pub_dbg = self.create_publisher(Image, '/signal/debug', 1)
        self.state = 'NONE'          # 발행 중인 판정
        self.cand = 'NONE'           # 이번에 본 판정 (후보)
        self.cand_n = 0              # 후보가 연속으로 나온 프레임 수
        self.count = 0

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[1] != PW:
            img = cv2.resize(img, (PW, PH), interpolation=cv2.INTER_AREA)

        # TODO ① 관심 영역 — img 에서 행 ROI_Y0~ROI_Y1, 열 ROI_X0~ROI_X1 만 잘라 roi 로 (행 먼저, 열 다음)
        roi = img

        # TODO ② HSV 변환 — roi 를 HSV 로 (cv2.cvtColor, cv2.COLOR_BGR2HSV)
        hsv = roi

        # TODO ③ 색 3개 마스크와 픽셀 수 — cv2.inRange 로 마스크를 만들고 cv2.countNonZero 로 센다
        #    S·V 하한은 세 색 모두 (s_lo, v_lo), 상한은 (255, 255).  H 범위:
        #      빨강 0~8 과 172~179 (마스크 2개를 cv2.bitwise_or 로 합친다) / 노랑 18~35 / 초록 45~90
        s_lo = self.get_parameter('s_lo').value
        v_lo = self.get_parameter('v_lo').value
        n_red = 0
        n_yellow = 0
        n_green = 0

        # TODO ④ 판정 — min_px 이상인 색 중 빨강 → 노랑 → 초록 순서로 고른다 (if / elif)
        #    고른 색 이름을 seen 에 ('RED' / 'YELLOW' / 'GREEN'), 그 색의 픽셀 수를 size 에. 없으면 그대로 둔다
        min_px = self.get_parameter('min_px').value
        seen = 'NONE'
        size = 0

        # ⑤ 연속 hold 프레임 같은 결과일 때만 발행 값을 바꾼다 (한 프레임짜리 오인식 거르기)
        if seen == self.cand:
            self.cand_n = self.cand_n + 1
        else:
            self.cand = seen
            self.cand_n = 1
        if self.cand_n >= self.get_parameter('hold').value:
            self.state = self.cand

        out = String()
        out.data = self.state
        self.pub_state.publish(out)
        num = Int32()
        num.data = size
        self.pub_size.publish(num)

        # ---- 디버그 영상 (3프레임에 1번) ----
        self.count = self.count + 1
        if self.count % 3 == 0:
            self.publish_debug(img, n_red, n_yellow, n_green)

    def publish_debug(self, img, n_red, n_yellow, n_green):
        dbg = img.copy()
        cv2.rectangle(dbg, (ROI_X0, ROI_Y0), (ROI_X1, ROI_Y1), (255, 255, 255), 1)
        big = cv2.resize(img[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
        dbg[PH - big.shape[0]:PH, 0:big.shape[1]] = big          # 왼쪽 아래에 관심 영역 3배 확대
        color = (200, 200, 200)
        if self.state == 'RED':
            color = (0, 0, 255)
        elif self.state == 'YELLOW':
            color = (0, 220, 255)
        elif self.state == 'GREEN':
            color = (0, 220, 0)
        cv2.putText(dbg, self.state, (ROI_X1 + 10, ROI_Y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        cv2.putText(dbg, 'R %d  Y %d  G %d' % (n_red, n_yellow, n_green), (ROI_X1 + 10, ROI_Y0 + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 2)
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = SignalDetect()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
