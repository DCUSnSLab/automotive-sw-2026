# -*- coding: utf-8 -*-
"""운전면허 시험장 월드 — 저메모리(rip) 버전 실행.

license_course.launch.py 와 인자·토픽이 모두 같고, 월드와 차량 모델만 저메모리 버전으로 바꿔 띄운다.
16 GB·iGPU(공유 메모리) 노트북처럼 메모리가 부족한 장치용. 사용법은 원본과 동일:

    ros2 launch ssc_class_gazebo license_course_rip.launch.py
    ros2 launch ssc_class_gazebo license_course_rip.launch.py start:=right
    ros2 launch ssc_class_gazebo license_course_rip.launch.py software_gl:=true lidar_viz:=true

원본과 다른 점 (자세한 근거·수치는 패키지 README):
    - worlds/classroom_rip.world : 정적 모델 215개를 모델 1개(course_static)로 병합하고, 실습에 불필요한
      장식 모델 39개(대기 쉘터·공사차량·철길·표지판·노면 문자·녹지 등)를 뺀 월드. 차선·연석·라바콘·정지선은 그대로.
    - models/ssc_class_car_rip : 차량 메시를 DAE 75k → STL 15k 삼각형으로 줄이고, 카메라를 1280x720 → 800x450.
      (라이다 720 샘플·20 Hz, /cmd_vel·/odom 등 토픽과 플러그인은 원본과 동일)
    - gzserver 메모리 약 1,200 MB → 약 650 MB (GPU, GUI 없음 기준). 라이다 스캔은 쉘터가 빠진 구간 외에 동일.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('ssc_class_gazebo')
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'license_course.launch.py')),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'classroom_rip.world'),
            'car_sdf': os.path.join(pkg_share, 'models', 'ssc_class_car_rip', 'model.sdf'),
        }.items(),
    )
    # start / x·y·yaw / gui / rviz / software_gl / lidar_viz 등 나머지 인자는 명령줄에서 그대로 전달된다
    return LaunchDescription([base])
