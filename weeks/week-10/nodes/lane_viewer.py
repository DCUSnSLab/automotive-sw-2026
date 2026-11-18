#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[10주차 완성본] lane_viewer — 카메라 영상 보기 + 클릭한 픽셀의 BGR·HSV 값 출력.

창에서 차선(노란색)·노면 글자(흰색)·도로(회색)를 클릭해 HSV 값을 직접 읽고,
lane_detect 의 노란색 범위(h_lo, h_hi, s_lo, v_lo)를 정하는 데 쓴다.
    ros2 run basics lane_viewer
    창 위에서 왼쪽 클릭 → 터미널에 (x, y) BGR HSV 출력,  s 키 → 현재 프레임을 frame_NN.png 로 저장,  q 키 → 종료
"""
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

PW, PH = 320, 180            # lane_detect 와 같은 처리 해상도로 줄여서 본다
SCALE = 2                    # 창은 2배로 확대해 표시 (640x360)


class LaneViewer(Node):
    def __init__(self):
        super().__init__('lane_viewer')
        self.bridge = CvBridge()
        self.img = None
        self.saved = 0
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        cv2.namedWindow('lane_viewer')
        cv2.setMouseCallback('lane_viewer', self.on_mouse)

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[1] != PW:
            img = cv2.resize(img, (PW, PH))
        self.img = img

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.img is None:
            return
        px = x // SCALE
        py = y // SCALE
        b, g, r = self.img[py, px]
        hsv = cv2.cvtColor(self.img[py:py + 1, px:px + 1], cv2.COLOR_BGR2HSV)[0, 0]
        self.get_logger().info('(x=%3d, y=%3d)  BGR (%3d, %3d, %3d)  HSV (%3d, %3d, %3d)'
                               % (px, py, b, g, r, hsv[0], hsv[1], hsv[2]))

    def show(self):
        if self.img is None:
            return True
        big = cv2.resize(self.img, (PW * SCALE, PH * SCALE), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('lane_viewer', big)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            name = 'frame_%02d.png' % self.saved
            cv2.imwrite(name, self.img)
            self.get_logger().info('저장: %s' % name)
            self.saved += 1
        return key != ord('q')


def main(args=None):
    rclpy.init(args=args)
    node = LaneViewer()
    try:
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.03)
            running = node.show()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
