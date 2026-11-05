# -*- coding: utf-8 -*-
"""기말 프로젝트 연습용 실행 — 채점 서버와 같은 조건 (저메모리 월드 · 센서 12 Hz · 우측 출발).

    ros2 launch ssc_class_gazebo final_course.launch.py                 # 채점 서버와 같은 조건
    ros2 launch ssc_class_gazebo final_course.launch.py gui:=false      # 화면 없이
    ros2 launch ssc_class_gazebo final_course.launch.py x:=… y:=… yaw:=… start:=left   # 다른 출발점
    ros2 launch ssc_class_gazebo final_course.launch.py course:=$PWD/course.json        # 리더보드 코스 자료 그대로
                                                                                         # (출발 자리·장애물)

license_course_rip.launch.py 에 기본값만 바꿔 넘긴다. 인자는 license_course.launch.py 와 같다.
채점 서버는 /ground_truth/odom 과 Gazebo 서비스를 학생 코드에 주지 않는다 — 연습할 때도 쓰지 말 것.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DEFAULTS = {
    'lidar_hz': '12',
    'camera_hz': '12',
    'start': 'right',
    'gui': 'true',
    'course': '',
}


def generate_launch_description():
    pkg_share = get_package_share_directory('ssc_class_gazebo')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()]
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'license_course_rip.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in DEFAULTS}.items(),
    )
    return LaunchDescription(args + [base])
