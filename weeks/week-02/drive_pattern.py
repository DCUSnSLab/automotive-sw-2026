#!/usr/bin/env python3
"""/cmd_vel 을 직접 publish 해 교육용 차량을 '직진 → 우회전' 패턴으로 반복 주행시키는 예제 (rclpy).

    python3 drive_pattern.py      (Ctrl+C 로 종료하면 정지 명령을 한 번 보내고 끝난다)

ssc_class_car 의 /cmd_vel 은 linear.x = 목표 속도[m/s], angular.z = 목표 조향각[rad] (최대 ±0.461) 이다.
"""
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist


class DrivePattern(Node):
    def __init__(self):
        super().__init__('drive_pattern')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz 로 계속 보낸다
        self.t = 0.0

    def tick(self):
        msg = Twist()
        phase = self.t % 8.0                 # 8초 주기: 5초 직진 + 3초 우회전
        if phase < 5.0:
            msg.linear.x = 1.0               # 전진 속도 [m/s]
        else:
            msg.linear.x = 0.6
            msg.angular.z = -0.3             # 조향각 [rad] ≈ -17° (음수 = 우회전; 출발선에서 좌회전하면 연석에 걸린다)
        self.pub.publish(msg)
        self.t += 0.1


def main():
    # rclpy 기본 SIGINT 처리기는 Ctrl+C 때 컨텍스트를 먼저 닫아 그 뒤의 publish 가 실패한다.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DrivePattern()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())                # 종료 시 정지 (모든 값 0)
    node.get_logger().info('stop command sent - bye')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
