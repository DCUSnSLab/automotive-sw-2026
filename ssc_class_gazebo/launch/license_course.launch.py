# -*- coding: utf-8 -*-
"""운전면허 시험장 월드 + 실습 차량 실행.

사용:
    ros2 launch ssc_class_gazebo license_course.launch.py
    ros2 launch ssc_class_gazebo license_course.launch.py x:=1.0 y:=-18.0 yaw:=1.57
    ros2 launch ssc_class_gazebo license_course.launch.py rviz:=true     # rviz2 함께
    ros2 launch ssc_class_gazebo license_course.launch.py software_gl:=true  # Gazebo 화면이 검거나 죽을 때만

토픽 (차량 스폰 후):
    구독  /cmd_vel            geometry_msgs/Twist   (linear.x 속도, angular.z 조향)
    발행  /scan               sensor_msgs/LaserScan (720pt, 20 Hz)
    발행  /camera/image_raw   sensor_msgs/Image     (1280x720, 30 Hz)
    발행  /odom               nav_msgs/Odometry     (+ TF odom->base_link)
    발행  /ground_truth/odom  nav_msgs/Odometry     (채점용 정답 위치)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ssc_class_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'classroom.world')
    models_path = os.path.join(pkg_share, 'models')
    car_sdf = os.path.join(models_path, 'ssc_class_car', 'model.sdf')

    # 출발선 좌표 (2026-09-22 확정): 시험장 출발 표시 위, -x 방향을 보고 선다
    args = [
        DeclareLaunchArgument('x', default_value='10.5379', description='spawn x [m]'),
        DeclareLaunchArgument('y', default_value='47.6657', description='spawn y [m]'),
        DeclareLaunchArgument('z', default_value='0.15', description='spawn z [m]'),
        DeclareLaunchArgument('yaw', default_value='-2.9741', description='spawn yaw [rad]'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI 표시'),
        DeclareLaunchArgument('rviz', default_value='false', description='rviz2 (LaserScan·Odometry·카메라) 함께 실행'),
        DeclareLaunchArgument('software_gl', default_value='false',
                              description='GPU 대신 소프트웨어 렌더링 (화면이 검을 때만; 카메라가 약 2 Hz로 느려짐)'),
    ]

    # 온라인 모델 DB 조회를 끈다 — 켜 두면 첫 실행 GUI가 약 2분간 스플래시에서 멈춘다 (모델은 모두 로컬)
    no_model_db = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    # software_gl:=true 일 때만, 이번 실행에 한해 적용 (~/.bashrc 영구 등록 금지 — 이후 주차 카메라 속도 저하)
    software_gl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                                         condition=IfCondition(LaunchConfiguration('software_gl')))

    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', models_path)
    # /usr/share/gazebo/setup.sh 를 빠뜨려도 gzserver 가 죽지 않도록 리소스 경로를 보강한다
    env_extra = []
    if os.path.isdir('/usr/share/gazebo-11'):
        env_extra.append(AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_path,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'ssc_class_car',
            '-file', car_sdf,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # 센서 프레임 TF (model.sdf의 센서 부착 위치와 일치해야 함)
    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='lidar_tf',
        arguments=['--x', '-0.10', '--y', '0', '--z', '0.46',
                   '--frame-id', 'base_link', '--child-frame-id', 'roof_lidar_link'],
    )
    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_tf',
        arguments=['--x', '0.35', '--y', '0', '--z', '0.82', '--pitch', '0.1',
                   '--frame-id', 'base_link', '--child-frame-id', 'front_camera_link'],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'class_car.rviz')],
    )

    # 스폰 직후 정지 명령을 한 번 보낸다 — 첫 /cmd_vel 전까지 차가 아주 느리게 미끄러지는 것을 막는다
    stop_once = TimerAction(period=6.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist', '{}'], output='log')])

    return LaunchDescription(args + env_extra + [no_model_db, software_gl, set_model_path, gazebo, spawn_car, lidar_tf, camera_tf, rviz, stop_once])
