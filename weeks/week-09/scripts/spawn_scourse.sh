#!/bin/bash
# [9주차 도전 실습] S자 라바콘 통로 설치 — 콘 두 줄(wall_L, wall_R)로 만든 폭 2.6 m 통로
# 기준: 우측 출발선 start:=right (11.0305, 51.2384, yaw -2.9665). 모델 원점 = 출발점.
#   통로 중심선: c(s) = 0.5 + 1.0*sin(2π(s-2)/12) m  (s = 출발점에서의 거리, + = 왼쪽)
#   폭 2.6 m (차 폭 0.64 m), 콘 간격 0.6 m, s = 2~30 m, 한 벽에 콘 47개
#   결승선: 출발점에서 28 m (fgm_drive 의 finish_dist)
# 설치는 Gazebo 자체 명령(gz model)으로 — ROS 서비스(/spawn_entity)는 GUI 부하 시 응답을 잃어 멈출 수 있음.
D="$(cd "$(dirname "$0")" && pwd)"
source ~/ssc_ws/install/setup.bash

spawn() {   # spawn <이름> <sdf>  (같은 이름이 이미 있으면 Gazebo가 무시함)
  gz model -f "$2" -m "$1" -x 11.0305 -y 51.2384 -z 0 -Y -2.9665 && echo "  $1: 설치"
}

spawn wall_L "$D/wall_L.sdf"
spawn wall_R "$D/wall_R.sdf"
echo "라바콘 통로 설치 완료 (wall_L, wall_R). 확인: gz model -m wall_L -p"
