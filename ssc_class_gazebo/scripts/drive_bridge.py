#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AckermannDriveStamped(/drive) -> Twist(/cmd_vel) 브릿지.

학생 코드가 표준 메시지(AckermannDriveStamped)만 쓰도록 하는 추상화 레이어.
시뮬레이터를 교체해도(예: 2027년 Gazebo Harmonic 이관) 이 노드만 바꾸면 된다.

주의(TA 검증 항목): gazebo_ros_ackermann_drive는 cmd_vel의 angular.z를
'조향각 목표'로 해석한다(요레이트 아님). 실차/타 시뮬레이터 이관 시
angular_mode 파라미터로 전환할 것.

W2에서는 사용하지 않음 — W3부터 학생 배포용.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from ackermann_msgs.msg import AckermannDriveStamped
except ImportError:  # ros-humble-ackermann-msgs 미설치
    AckermannDriveStamped = None


class DriveBridge(Node):
    def __init__(self):
        super().__init__('drive_bridge')
        self.declare_parameter('wheelbase', 0.65)
        # 'steer'  : angular.z = 조향각 [rad]  (gazebo_ros_ackermann_drive 기본)
        # 'yawrate': angular.z = 요레이트 [rad/s] (Twist 표준 해석)
        self.declare_parameter('angular_mode', 'steer')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            AckermannDriveStamped, 'drive', self.on_drive, 10)
        self.get_logger().info('drive_bridge: /drive -> /cmd_vel')

    def on_drive(self, msg):
        speed = msg.drive.speed
        steer = msg.drive.steering_angle
        out = Twist()
        out.linear.x = speed
        if self.get_parameter('angular_mode').value == 'yawrate':
            L = self.get_parameter('wheelbase').value
            out.angular.z = speed * math.tan(steer) / L
        else:
            out.angular.z = steer
        self.pub.publish(out)


def main():
    if AckermannDriveStamped is None:
        raise SystemExit('ackermann_msgs 미설치: sudo apt install ros-humble-ackermann-msgs')
    rclpy.init()
    node = DriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
