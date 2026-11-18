#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[10주차 빈칸 파일] lane_follow — offset 1개 → 조향 (7주차 PD 와 같은 구조).

/lane/offset (차로 중심 - 차 중심 [m], + 왼쪽) 을 오차 e 로 보고 조향 = Kp·e + Kd·de.
offset 이 NaN(차선 놓침)이면 직전 조향을 lost_hold 초 동안 유지하고, 그래도 없으면 정지.
주행 거리(/ground_truth/odom 누적)가 finish_dist 가 되면 FINISH 기록 후 정지 — 곡선이 있어 직선거리 대신 누적 거리.

파라미터: kp, kd, speed, lost_hold, finish_dist

'# TODO' 표시된 곳만 채운다 — TODO ⑤ 조향(7주차 PD 와 같은 식).  나머지(차선 놓침 정지·FINISH)는 완성.
    ros2 run basics lane_follow
    ros2 param set /lane_follow kp 0.3                 # 실행 중 튜닝
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

MAX_STEER = 0.40


class LaneFollow(Node):
    def __init__(self):
        super().__init__('lane_follow')
        self.declare_parameter('kp', 0.2)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('lost_hold', 0.5)
        self.declare_parameter('finish_dist', 42.0)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/lane/offset', self.on_offset, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.1, self.tick)     # 10 Hz
        self.offset = 0.0
        self.have_offset = False
        self.t_seen = None          # 마지막으로 차선을 본 시각
        self.prev_e = 0.0
        self.have_prev = False
        self.steer = 0.0
        self.x = None
        self.y = None
        self.travel = 0.0           # 출발 후 누적 주행 거리 [m]
        self.t_start = None
        self.finished = False

    def on_offset(self, msg):
        if math.isnan(msg.data):
            return                  # 차선 놓침 — 직전 값 유지
        self.offset = msg.data
        self.have_offset = True
        self.t_seen = self.get_clock().now()

    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.t_start is not None and self.x is not None:
            dx = x - self.x
            dy = y - self.y
            self.travel += math.sqrt(dx * dx + dy * dy)
        self.x = x
        self.y = y

    def tick(self):
        msg = Twist()
        if self.finished:
            self.pub.publish(msg)
            return
        if not self.have_offset:
            return                  # 아직 차선을 한 번도 못 봄
        if self.t_start is not None and self.travel >= self.get_parameter('finish_dist').value:
            self.pub.publish(msg)
            dt = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
            self.get_logger().info('FINISH  %.1f s  (%.1f m)' % (dt, self.travel))
            self.finished = True
            return
        lost = (self.get_clock().now() - self.t_seen).nanoseconds * 1e-9
        if lost > self.get_parameter('lost_hold').value:
            self.pub.publish(msg)   # 차선을 오래 놓침 — 정지
            self.get_logger().info('LANE LOST — 정지', throttle_duration_sec=1.0)
            return

        # TODO ⑤ 조향 — e = self.offset (차로 중심이 왼쪽이면 + → 왼쪽(+)으로 꺾는다)
        #    de = (e - self.prev_e) / 0.1  (첫 tick 은 self.have_prev 로 구분, 끝에서 prev_e·have_prev 갱신)
        #    steer = kp·e + kd·de,  ±MAX_STEER 로 클램프 (if / elif)
        steer = 0.0
        ratio = abs(steer) / MAX_STEER
        msg.linear.x = self.get_parameter('speed').value * (1.0 - 0.5 * ratio)
        msg.angular.z = steer
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollow()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rclpy.ok():
        node.pub.publish(Twist())
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
