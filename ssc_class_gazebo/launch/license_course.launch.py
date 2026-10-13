# -*- coding: utf-8 -*-
"""운전면허 시험장 월드 + 실습 차량 실행.

사용:
    ros2 launch ssc_class_gazebo license_course.launch.py
    ros2 launch ssc_class_gazebo license_course.launch.py x:=1.0 y:=-18.0 yaw:=1.57
    ros2 launch ssc_class_gazebo license_course.launch.py rviz:=true     # rviz2 함께
    ros2 launch ssc_class_gazebo license_course.launch.py software_gl:=true  # Gazebo 화면이 검거나 죽을 때만
    ros2 launch ssc_class_gazebo license_course.launch.py camera_width:=1280 camera_height:=720
                                                           # 고해상도 카메라가 꼭 필요할 때 (메모리·CPU 더 씀)

토픽 (차량 스폰 후):
    구독  /cmd_vel            geometry_msgs/Twist   (linear.x 속도, angular.z 조향)
    발행  /scan               sensor_msgs/LaserScan (720pt, 20 Hz; lidar_samples 인자로 변경)
    발행  /camera/image_raw   sensor_msgs/Image     (800x450, 30 Hz; camera_* 인자로 변경)
    발행  /odom               nav_msgs/Odometry     (+ TF odom->base_link)
    발행  /ground_truth/odom  nav_msgs/Odometry     (채점용 정답 위치)

메모리 절감 (2026-10-13, 16 GB·iGPU 노트북 대상):
    - worlds/classroom.world 는 정적 모델 215개를 모델 1개로 합친 결과물이다(tools/merge_static_models.py).
      편집용 원본은 worlds/classroom_authoring.world — 원본을 고친 뒤 스크립트로 다시 생성할 것.
    - 차량 메시는 DAE 75k 삼각형 → 바이너리 STL 15k 삼각형, 텍스처는 절반 해상도.
    - 카메라 해상도를 1280x720 → 800x450 으로 낮췄다(주기 30 Hz 는 그대로). 인자로 되돌릴 수 있다.
      ⚠ 일부 해상도(640x360·640x480·960x540 실측)는 소프트웨어 렌더링(llvmpipe)에서 카메라가 0.3~0.6 Hz 로
        급락한다. 800x450·1280x720 은 10 Hz 이상으로 정상. 해상도를 바꾸면 software_gl:=true 로 한 번 확인할 것.
"""
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _render_car_sdf(context, car_sdf):
    """model.sdf 의 센서 수치를 launch 인자 값으로 치환한 임시 SDF 경로를 돌려준다.

    인자가 model.sdf 기본값과 같으면 원본 파일을 그대로 쓴다.
    """
    width = context.launch_configurations['camera_width']
    height = context.launch_configurations['camera_height']
    fps = context.launch_configurations['camera_fps']
    samples = context.launch_configurations['lidar_samples']
    with open(car_sdf, encoding='utf-8') as f:
        sdf = f.read()

    def sub(pattern, value, text):
        new, n = re.subn(pattern, lambda m: m.group(1) + value + m.group(3), text, count=1, flags=re.S)
        if n != 1:
            raise RuntimeError(f'model.sdf 에서 {pattern!r} 를 찾지 못했다 — 센서 블록이 바뀌었는지 확인')
        return new

    cam_block = r'(<sensor name="front_camera".*?<image>\s*<width>)(\d+)(</width>)'
    out = sub(cam_block, width, sdf)
    out = sub(r'(<sensor name="front_camera".*?<image>.*?<height>)(\d+)(</height>)', height, out)
    out = sub(r'(<sensor name="front_camera".*?<update_rate>)([\d.]+)(</update_rate>)', fps, out)
    out = sub(r'(<sensor name="roof_lidar".*?<horizontal>\s*<samples>)(\d+)(</samples>)', samples, out)
    if out == sdf:
        return car_sdf
    tmp = tempfile.NamedTemporaryFile('w', suffix='_ssc_class_car.sdf', delete=False, encoding='utf-8')
    tmp.write(out)
    tmp.close()
    return tmp.name


def _spawn_car(context, car_sdf):
    sdf_path = _render_car_sdf(context, car_sdf)
    return [Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'ssc_class_car',
            '-file', sdf_path,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )]


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
                              description='GPU 대신 소프트웨어 렌더링 (화면이 검을 때만; 카메라가 느려짐)'),
        # 센서 수치 — 기본값은 models/ssc_class_car/model.sdf 와 같다. 바꾸면 임시 SDF 를 만들어 스폰한다.
        DeclareLaunchArgument('camera_width', default_value='800', description='카메라 가로 해상도 [px]'),
        DeclareLaunchArgument('camera_height', default_value='450', description='카메라 세로 해상도 [px]'),
        DeclareLaunchArgument('camera_fps', default_value='30', description='카메라 발행 주기 [Hz]'),
        DeclareLaunchArgument('lidar_samples', default_value='720', description='2D 라이다 1회전 샘플 수'),
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

    spawn_car = OpaqueFunction(function=_spawn_car, args=[car_sdf])

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
