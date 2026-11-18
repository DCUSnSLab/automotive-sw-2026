#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[11주차 빈칸 파일] signal_drive — 신호등 색에 따라 정지·출발 (직진 주행).

signal_detect 가 발행한 색(/signal/state)과 크기(/signal/size)를 받아 10 Hz 로 속도를 정한다.
신호등은 가까울수록 크게 보인다 → 픽셀 수로 '정지선까지 얼마나 남았는지' 가늠한다.

    크기 < near_px            아직 멀다        → 주행
    near_px ≤ 크기 < pass_px  정지 구간        → 빨강·노랑이면 대기 시작 (정지)
    크기 ≥ pass_px            정지선을 지났다   → 주행 (교차로 안에서는 멈추지 않는다)
한번 대기를 시작하면 초록을 볼 때까지 정지를 유지한다 (크기가 프레임마다 1~2픽셀 흔들려도 멈췄다 가다 하지 않게).

'# TODO' 표시된 곳만 채운다 — TODO ⑥ 정지 판단.  나머지(직진·FINISH)는 완성.
    ros2 run basics signal_drive
    ros2 param set /signal_drive near_px 22            # 실행 중 튜닝

파라미터:
  speed     : 주행 속도 [m/s]
  near_px   : 정지 구간이 시작되는 신호등 크기 [픽셀 수]
  pass_px   : 정지선을 지난 것으로 보는 신호등 크기 [픽셀 수]
  run_dist  : 출발점에서 이 거리만큼 가면 FINISH 후 정지 [m]
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String


class SignalDrive(Node):
    def __init__(self):
        super().__init__('signal_drive')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('near_px', 20)
        self.declare_parameter('pass_px', 28)
        self.declare_parameter('run_dist', 36.0)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_state = self.create_subscription(String, '/signal/state', self.on_state, 10)
        self.sub_size = self.create_subscription(Int32, '/signal/size', self.on_size, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.1, self.tick)      # 10 Hz
        self.state = None            # 신호등 색. None = 아직 받지 못함
        self.size = 0                # 신호등 크기 [픽셀 수]
        self.x = None                # 현재 위치 (odom)
        self.y = None
        self.x0 = None               # 출발 위치
        self.y0 = None
        self.waiting = False         # True = 정지 구간에서 빨강·노랑을 보고 대기 중
        self.finished = False

    def on_state(self, msg):
        self.state = msg.data

    def on_size(self, msg):
        self.size = msg.data

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.x0 is None:
            self.x0 = self.x
            self.y0 = self.y

    def tick(self):
        cmd = Twist()                                        # 기본값 = 정지
        if self.state is None or self.x0 is None or self.finished:
            self.pub.publish(cmd)                            # 신호 판정·위치를 받기 전, 결승 후에는 정지
            return
        dx = self.x - self.x0
        dy = self.y - self.y0
        dist = math.sqrt(dx * dx + dy * dy)
        if dist >= self.get_parameter('run_dist').value:
            self.finished = True
            self.pub.publish(cmd)
            self.get_logger().info('FINISH  %.1f m' % dist)
            return

        # TODO ⑥ 정지 판단 — self.waiting 을 정한다 (True = 대기 중)
        #    색이 'RED' 또는 'YELLOW' 이고 크기가 near_px 이상 pass_px 미만이면 self.waiting = True
        #    색이 'GREEN' 이면 self.waiting = False.  그 밖에는 그대로 둔다
        near_px = self.get_parameter('near_px').value
        pass_px = self.get_parameter('pass_px').value
        stop = self.waiting

        if stop:
            self.get_logger().info('STOP  %s  size %d' % (self.state, self.size), throttle_duration_sec=1.0)
        else:
            cmd.linear.x = self.get_parameter('speed').value
            self.get_logger().info('GO    %s  size %d' % (self.state, self.size), throttle_duration_sec=1.0)
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SignalDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rclpy.ok():
        node.pub.publish(Twist())                            # 종료 시 정지 명령 1회
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
