# -*- coding: utf-8 -*-
"""운전면허 시험장 월드 + 실습 차량 실행.

사용:
    ros2 launch ssc_class_gazebo license_course.launch.py
    ros2 launch ssc_class_gazebo license_course.launch.py x:=1.0 y:=-18.0 yaw:=1.57
    ros2 launch ssc_class_gazebo license_course.launch.py rviz:=true     # rviz2 함께
    ros2 launch ssc_class_gazebo license_course.launch.py software_gl:=true  # Gazebo 화면이 검거나 죽을 때만
    ros2 launch ssc_class_gazebo license_course.launch.py lidar_viz:=true    # Gazebo 안에 LiDAR 광선을 그린다 (6주차)
    ros2 launch ssc_class_gazebo license_course.launch.py lidar_hz:=12 camera_hz:=12  # 센서 주기 변경 (기말 프로젝트·리더보드 기준)
    ros2 launch ssc_class_gazebo license_course_rip.launch.py                # 저메모리(rip) 월드 — 인자는 전부 동일
    ros2 launch ssc_class_gazebo final_course.launch.py course:=$PWD/course.json  # 리더보드 코스 자료 그대로

토픽 (차량 스폰 후):
    구독  /cmd_vel            geometry_msgs/Twist   (linear.x 속도, angular.z 조향)
    발행  /scan               sensor_msgs/LaserScan (720pt, 20 Hz — lidar_hz 로 변경)
    발행  /camera/image_raw   sensor_msgs/Image     (1280x720, 30 Hz — camera_hz 로 변경)
    발행  /odom               nav_msgs/Odometry     (+ TF odom->base_link)
    발행  /ground_truth/odom  nav_msgs/Odometry     (채점용 정답 위치)
"""
import json
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---- 리더보드 코스 자료 (course.json) ------------------------------------------
# 리더보드 채점 서버와 같은 장애물 모델 — 한쪽을 바꾸면 채점 서버 쪽도 같이 바꾼다.
CONE_MESH = 'model://yongin_license_course_runtime/meshes/traffic_cone_{part}.stl'
CONE_W, CONE_H = 0.48, 0.94          # 시험장 라바콘 충돌체(반지름 0.24, 높이 0.94)
ORANGE = '0.94118 0.41569 0.10196 1'
WHITE = '0.93333 0.93333 0.91765 1'


def _material(rgba):
    return f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'


def obstacle_model(o):
    name = f"lb_obstacle_{o['id']}"
    w, d, h = float(o['w']), float(o['d']), float(o['h'])
    pose = f"{float(o['x']):.4f} {float(o['y']):.4f} 0 0 0 {float(o.get('yaw', 0)):.4f}"
    if o['type'] == 'cone':
        sx, sz = w / CONE_W, h / CONE_H
        visuals = ''.join(
            f"""<visual name="{part}"><geometry><mesh><uri>{CONE_MESH.format(part=part)}</uri>
              <scale>{sx:.4f} {sx:.4f} {sz:.4f}</scale></mesh></geometry>{_material(color)}</visual>"""
            for part, color in (('lower', ORANGE), ('stripe', WHITE), ('upper', ORANGE)))
        base = f'{w + 0.02:.3f} {w + 0.02:.3f} 0.06'
        visuals += f"""<visual name="base"><pose>0 0 0.03 0 0 0</pose><geometry><box><size>{base}</size>
              </box></geometry>{_material('0.145 0.145 0.145 1')}</visual>"""
        collision = f"""<collision name="collision"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><cylinder>
              <radius>{w / 2:.3f}</radius><length>{h:.3f}</length></cylinder></geometry></collision>"""
    else:
        size = f'{w:.3f} {d:.3f} {h:.3f}'
        stripe = f'{w + 0.004:.3f} {d + 0.004:.3f} {max(0.04, h * 0.15):.3f}'
        visuals = f"""<visual name="body"><pose>0 0 {h / 2:.3f} 0 0 0</pose>
              <geometry><box><size>{size}</size></box></geometry>{_material(ORANGE)}</visual>
            <visual name="stripe"><pose>0 0 {h * 0.7:.3f} 0 0 0</pose>
              <geometry><box><size>{stripe}</size></box></geometry>{_material(WHITE)}</visual>"""
        collision = f"""<collision name="collision"><pose>0 0 {h / 2:.3f} 0 0 0</pose><geometry><box><size>{size}</size>
              </box></geometry></collision>"""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        {collision}
        {visuals}
      </link>
    </model>"""


def add_obstacles(world_sdf, obstacles):
    i = world_sdf.rfind('</world>')
    if i < 0:
        raise RuntimeError('월드 파일에 </world> 가 없습니다')
    return world_sdf[:i] + ''.join(obstacle_model(o) for o in obstacles) + '\n  ' + world_sdf[i:]


def load_course(context):
    """course:=… 가 있으면 읽는다 (없으면 None)."""
    path = LaunchConfiguration('course').perform(context).strip()
    if not path:
        return None
    path = os.path.abspath(os.path.expanduser(path))
    try:
        with open(path, encoding='utf-8') as fh:
            course = json.load(fh)
    except (OSError, ValueError) as e:
        raise RuntimeError(f'course:={path} 를 읽을 수 없습니다 — 리더보드에서 받은 course.json 경로인지 확인 ({e})')
    if not isinstance(course.get('spawn'), dict):
        raise RuntimeError(f'course:={path} 에 spawn 이 없습니다 — 리더보드 코스 자료의 course.json 이 맞나요?')
    return course


def generate_launch_description():
    pkg_share = get_package_share_directory('ssc_class_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'classroom.world')
    models_path = os.path.join(pkg_share, 'models')
    car_sdf = os.path.join(models_path, 'ssc_class_car', 'model.sdf')

    # 출발선 좌표 (2026-09-22 확정): 시험장 출발 표시 위, -x 방향을 보고 선다
    # start:=right (7주차 이후 기본 사용): 도로 우측 포즈 — 조교 확정 좌표 (2026-10-13, automotive-sw-2026 week-07)
    args = [
        # world / car_sdf 는 license_course_rip.launch.py 가 저메모리 월드·차량으로 바꿔 넣는 자리 — 직접 쓸 일은 드물다
        DeclareLaunchArgument('world', default_value=world_path, description='월드 파일 경로'),
        DeclareLaunchArgument('car_sdf', default_value=car_sdf, description='실습 차량 model.sdf 경로'),
        DeclareLaunchArgument('start', default_value='left',
                              description="출발 프리셋: left(기존, 왼쪽 연석 2.05 m) / right(도로 우측) — right면 x·y·yaw 인자 무시"),
        DeclareLaunchArgument('x', default_value='10.5379', description='spawn x [m]'),
        DeclareLaunchArgument('y', default_value='47.6657', description='spawn y [m]'),
        DeclareLaunchArgument('z', default_value='0.15', description='spawn z [m]'),
        DeclareLaunchArgument('yaw', default_value='-2.9741', description='spawn yaw [rad]'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI 표시'),
        DeclareLaunchArgument('rviz', default_value='false', description='rviz2 (LaserScan·Odometry·카메라) 함께 실행'),
        DeclareLaunchArgument('software_gl', default_value='false',
                              description='GPU 대신 소프트웨어 렌더링 (화면이 검을 때만; 카메라가 약 2 Hz로 느려짐)'),
        DeclareLaunchArgument('lidar_viz', default_value='false',
                              description='Gazebo 화면에 LiDAR 광선(파란 선)을 그린다 — 수업용, 렌더링 부하 약간 증가'),
        DeclareLaunchArgument('lidar_hz', default_value='',
                              description='LiDAR 주기 [Hz] — 비우면 model.sdf 값(20). 기말 프로젝트·리더보드는 12'),
        DeclareLaunchArgument('camera_hz', default_value='',
                              description='카메라 주기 [Hz] — 비우면 model.sdf 값(30). 기말 프로젝트·리더보드는 12'),
        DeclareLaunchArgument('course', default_value='',
                              description='리더보드 코스 자료의 course.json — 출발 자리·장애물을 채점 서버와 같게 '
                                          '(start·x·y·yaw 무시)'),
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

    def make_gazebo(context):
        # course:= 에 장애물이 있으면 월드 사본에 넣어 띄운다 (원본은 그대로) — 채점 서버도 같은 방식
        world = LaunchConfiguration('world').perform(context)
        course = load_course(context)
        notes = []
        if course:
            want = course.get('world')
            if want and os.path.basename(world) != want:
                notes.append(LogInfo(msg=f'[course] 이 코스는 {want} 기준인데 {os.path.basename(world)} 를 띄웁니다 '
                                         '— final_course.launch.py 로 띄우세요'))
            obstacles = course.get('obstacles') or []
            if obstacles:
                with open(world, encoding='utf-8') as fh:
                    txt = add_obstacles(fh.read(), obstacles)
                tmp = tempfile.NamedTemporaryFile('w', suffix='_course.world', delete=False, encoding='utf-8')
                tmp.write(txt); tmp.close()
                world = tmp.name
            sp = course['spawn']
            notes.append(LogInfo(msg=f"[course] {course.get('name', '')} ({course.get('key', '')} 판 "
                                     f"{course.get('version', '')}) — 출발 ({sp['x']}, {sp['y']}, {sp['yaw']}), "
                                     f'장애물 {len(obstacles)}개'))
        return notes + [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world, 'gui': LaunchConfiguration('gui')}.items(),
        )]
    gazebo = OpaqueFunction(function=make_gazebo)

    def set_in_sensor(txt, sensor, old_tag_re, new_tag):
        # <sensor name="sensor" ...> 블록 안의 첫 태그 하나만 바꾼다 (다른 센서·플러그인의 같은 태그는 그대로)
        head, sep, tail = txt.partition(f'<sensor name="{sensor}"')
        if not sep:
            raise RuntimeError(f'car_sdf 에 센서 {sensor} 가 없습니다')
        body, end, rest = tail.partition('</sensor>')
        return head + sep + re.sub(old_tag_re, new_tag, body, count=1) + end + rest

    def make_spawn(context):
        # lidar_viz·lidar_hz·camera_hz 가 지정되면 model.sdf 를 고친 사본을 만들어 스폰한다 (원본은 그대로)
        sdf = LaunchConfiguration('car_sdf').perform(context)
        viz = LaunchConfiguration('lidar_viz').perform(context).lower() in ('true', '1')
        lidar_hz = LaunchConfiguration('lidar_hz').perform(context).strip()
        camera_hz = LaunchConfiguration('camera_hz').perform(context).strip()
        if viz or lidar_hz or camera_hz:
            with open(sdf, encoding='utf-8') as fh:
                txt = fh.read()
            if viz:
                txt = set_in_sensor(txt, 'roof_lidar', r'<visualize>false</visualize>', '<visualize>true</visualize>')
            if lidar_hz:
                txt = set_in_sensor(txt, 'roof_lidar', r'<update_rate>[^<]*</update_rate>',
                                    f'<update_rate>{float(lidar_hz):g}</update_rate>')
            if camera_hz:
                txt = set_in_sensor(txt, 'front_camera', r'<update_rate>[^<]*</update_rate>',
                                    f'<update_rate>{float(camera_hz):g}</update_rate>')
            tmp = tempfile.NamedTemporaryFile('w', suffix='_ssc_class_car.sdf', delete=False, encoding='utf-8')
            tmp.write(txt); tmp.close()
            sdf = tmp.name
        # start:=right — 도로 우측 포즈. 조교 좌표(11.0967, 50.8642)를 연석 쪽으로 0.38 m 보정 →
        # 오른쪽 연석 2.05 m (실측 2026-10-13). 2.43 m 에서는 초기 오차가 커서 P 단독 주행이 즉시 전방 정지에 걸림.
        sx = LaunchConfiguration('x').perform(context)
        sy = LaunchConfiguration('y').perform(context)
        syaw = LaunchConfiguration('yaw').perform(context)
        course = load_course(context)
        if course:  # 리더보드 코스 자료의 출발 자리 — start·x·y·yaw 보다 앞선다
            sx, sy, syaw = (str(float(course['spawn'][k])) for k in ('x', 'y', 'yaw'))
        elif LaunchConfiguration('start').perform(context).lower() == 'right':
            sx, sy, syaw = '11.0305', '51.2384', '-2.9665'
        return [Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', 'ssc_class_car',
                '-file', sdf,
                '-x', sx,
                '-y', sy,
                '-z', LaunchConfiguration('z'),
                '-Y', syaw,
                '-timeout', '300',
            ],
        )]
    spawn_car = OpaqueFunction(function=make_spawn)

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
