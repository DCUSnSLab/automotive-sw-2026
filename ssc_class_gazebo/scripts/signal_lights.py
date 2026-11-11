#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""코스의 신호등을 시뮬레이션 시간에 맞춰 바꾼다 (초록 → 노랑 → 빨강 주기).

license_course.launch.py 가 course:= 에 신호가 있을 때 띄운다 — 직접 실행할 일은 없다.
    signal_lights.py <계획 파일(json)>

계획 파일: {"signals": [{"id", "name", "green_s", "yellow_s", "red_s", "offset_s", "lenses": {색: [x, y, z, …]}}]}
켜진 렌즈는 색마다 따로 모델(lb_signal_<신호등>_<색>)이다. 켤 색을 렌즈 자리로 올리고 나머지를 땅속으로 내린다
(/lb/set_entity_state). 채점 서버에서는 채점기가 같은 일을 하고, 학생 코드는 이 서비스를 쓸 수 없다 —
지금 무슨 불인지는 카메라로 본다.
"""
import json
import sys

import rclpy
import rclpy.utilities
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

COLORS = ('red', 'yellow', 'green')
KOREAN = {'red': '빨강', 'yellow': '노랑', 'green': '초록'}
OFF_Z = -30.0


def state_at(sig, t):
    g, y, r = sig['green_s'], sig['yellow_s'], sig['red_s']
    p = (max(t, 0.0) + sig.get('offset_s', 0.0)) % (g + y + r)
    return 'green' if p < g else ('yellow' if p < g + y else 'red')


class SignalLights(Node):
    def __init__(self, plan):
        super().__init__('signal_lights')
        self.signals = plan['signals']
        self.shown = {s['id']: state_at(s, 0.0) for s in self.signals}   # 월드에 처음 켜 둔 색
        self.client = self.create_client(SetEntityState, '/lb/set_entity_state')
        # Gazebo 의 /clock 은 best effort 로 나온다 — 기본(reliable) 구독으로는 한 건도 받지 못한다
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if not self.client.service_is_ready():
            return
        for s in self.signals:
            want = state_at(s, t)
            if self.shown[s['id']] == want:
                continue
            self.shown[s['id']] = want
            for color in COLORS:
                self.move(s, color, color == want)
            self.get_logger().info(f"{s['name']} → {KOREAN[want]} (t = {t:.1f} s)")

    def move(self, sig, color, on):
        x, y, z = sig['lenses'][color][:3]
        req = SetEntityState.Request()
        req.state.name = f"lb_signal_{sig['id']}_{color}"
        req.state.pose.position.x, req.state.pose.position.y = float(x), float(y)
        req.state.pose.position.z = float(z) if on else OFF_Z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        self.client.call_async(req).add_done_callback(lambda f: self.done(f, sig['id']))

    def done(self, fut, light):
        try:
            ok = bool(fut.result().success)
        except Exception:  # noqa: BLE001
            ok = False
        if not ok:   # 다음 /clock 에 다시 맞춘다
            self.shown[light] = None


def main():
    args = rclpy.utilities.remove_ros_args(sys.argv)
    if len(args) < 2:
        sys.exit('사용: signal_lights.py <계획 파일(json)>')
    with open(args[1], encoding='utf-8') as fh:
        plan = json.load(fh)
    rclpy.init()
    node = SignalLights(plan)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
