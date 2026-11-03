#!/bin/bash
# [9주차 기본 실습] 라바콘 4개 설치 — 차가 실제로 지나는 자리에 차례로
# 기준: 우측 출발선 start:=right (11.0305, 51.2384, yaw -2.9665)
#   번호   전방거리   좌우 오프셋(+ = 왼쪽)   월드 좌표         위치
#   cone1    6 m     +0.3 m                 ( 5.17, 49.90)    차 정면 — 직진하면 충돌
#   cone2   11 m     +3.0 m                 ( 0.72, 46.37)    반대 차로 (콘1을 왼쪽으로 피한 뒤의 진행선)
#   cone3   16 m     +2.6 m                 (-4.27, 45.89)    반대 차로 (콘2를 지난 뒤 내려오는 진행선)
#   cone4   21 m     +0.3 m                 (-9.60, 47.28)    우측 차로
#   결승선: 출발점에서 28 m (fgm_drive 의 finish_dist)
# 콘을 추가·이동하려면 아래 줄을 복사해 번호·좌표만 바꾼다 (과제).
#   좌표 계산: X = 11.0305 - 0.9847*d + 0.1743*lat,  Y = 51.2384 - 0.1743*d - 0.9847*lat
# 설치는 Gazebo 자체 명령(gz model)으로 — ROS 서비스(/spawn_entity)는 GUI 부하 시 응답을 잃어 멈출 수 있음.
D="$(cd "$(dirname "$0")" && pwd)"
source ~/ssc_ws/install/setup.bash

spawn() {   # spawn <이름> <x> <y>  (같은 이름이 이미 있으면 Gazebo가 무시함)
  gz model -f "$D/cone.sdf" -m "$1" -x "$2" -y "$3" -z 0 && echo "  $1: 설치 ($2, $3)"
}

spawn cone1  5.17 49.90
spawn cone2  0.72 46.37
spawn cone3 -4.27 45.89
spawn cone4 -9.60 47.28
echo "라바콘 4개 설치 완료 (cone1~4). 확인: gz model -m cone1 -p"
