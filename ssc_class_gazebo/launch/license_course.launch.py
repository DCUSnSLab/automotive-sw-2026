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
    ros2 launch ssc_class_gazebo final_course.launch.py course:=$PWD/course.json signal_offset:=0  # 신호 시작 시점 고정

토픽 (차량 스폰 후):
    구독  /cmd_vel            geometry_msgs/Twist   (linear.x 속도, angular.z 조향)
    발행  /scan               sensor_msgs/LaserScan (720pt, 20 Hz — lidar_hz 로 변경)
    발행  /camera/image_raw   sensor_msgs/Image     (1280x720, 30 Hz — camera_hz 로 변경)
    발행  /odom               nav_msgs/Odometry     (+ TF odom->base_link)
    발행  /ground_truth/odom  nav_msgs/Odometry     (채점용 정답 위치)
"""
import json
import os
import random
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


# ---- 신호등 (course.json 의 signals) --------------------------------------------
# 월드의 신호등 렌즈(traffic_light_NN__lens_<색>)는 한 모델의 visual 이라 주행 도중에 색을 바꿀 수 없다. 그래서
# 코스가 쓰는 신호등은 렌즈를 모두 꺼 두고, 켜진 렌즈를 색마다 따로 고정 모델(lb_signal_<신호등>_<색>)로 둔다 —
# 켜진 색은 렌즈 자리에, 나머지는 땅속(SIGNAL_OFF_Z)에. scripts/signal_lights.py 가 시뮬레이션 시간에 맞춰
# /lb/set_entity_state 로 올리고 내린다. 코스가 쓰지 않는 신호등은 그대로 둔다.
# 채점 서버와 같은 모델 — 한쪽을 바꾸면 채점 서버 쪽도 같이 바꾼다.
SIGNAL_COLORS = ('red', 'yellow', 'green')
SIGNAL_LIT = {'red': '1.0 0.10 0.06 1', 'yellow': '1.0 0.78 0.05 1', 'green': '0.08 0.92 0.25 1'}
SIGNAL_DARK = {'red': '0.0246 0.0036 0.0024 1', 'yellow': '0.0285 0.0216 0.0024 1',
               'green': '0.0024 0.0246 0.006 1', 'green_arrow_symbol': '0.0012 0.03 0.0042 1'}
SIGNAL_OFF_Z = -30.0
SIGNAL_STATE_PLUGIN = """
    <plugin name="lb_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/lb</namespace></ros>
      <update_rate>1.0</update_rate>
    </plugin>"""


def signal_model_name(light, color):
    return f'lb_signal_{light}_{color}'


def _signal_lens(world_sdf, light, part):
    return re.search(rf'<visual name="{re.escape(light)}__lens_{part}">(.*?)</visual>', world_sdf, re.S)


def signal_lenses(world_sdf, light):
    """신호등 렌즈 자세 {색: [x, y, z, roll, pitch, yaw]} — 월드 좌표."""
    out = {}
    for color in SIGNAL_COLORS:
        m = _signal_lens(world_sdf, light, color)
        pose = re.search(r'<pose>([^<]*)</pose>', m.group(1)) if m else None
        if pose is None or len(pose.group(1).split()) != 6:
            raise RuntimeError(f'월드 파일에 신호등 {light} 의 {color} 렌즈가 없습니다')
        out[color] = [float(v) for v in pose.group(1).split()]
    return out


def signal_lit_model(light, color, pose, on):
    # 모델 자세에는 위치만 두고 방향은 visual 에 — 올리고 내릴 때 위치만 바꾸면 된다
    x, y, z, roll, pitch, yaw = pose
    rgba = SIGNAL_LIT[color]
    return f"""
    <model name="{signal_model_name(light, color)}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} {z if on else SIGNAL_OFF_Z:.6f} 0 0 0</pose>
      <link name="link">
        <visual name="lit">
          <pose>0 0 0 {roll:.6f} {pitch:.6f} {yaw:.6f}</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><cylinder><radius>0.165</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive>
            <specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>"""


def add_signals(world_sdf, lights):
    """lights = {신호등 id: 처음 켤 색}. 그 신호등의 렌즈를 끄고, 켜진 렌즈 모델과 상태 플러그인을 넣는다."""
    if not lights:
        return world_sdf
    models = []
    for light, shown in lights.items():
        if shown not in SIGNAL_COLORS:
            raise RuntimeError(f'신호 색은 {SIGNAL_COLORS} 중 하나')
        lenses = signal_lenses(world_sdf, light)
        for part, rgba in SIGNAL_DARK.items():
            m = _signal_lens(world_sdf, light, part)
            if not m:
                continue
            body = re.sub(r'<(ambient|diffuse)>[^<]*</\1>', rf'<\1>{rgba}</\1>', m.group(1))
            body = re.sub(r'<emissive>[^<]*</emissive>', '<emissive>0 0 0 1</emissive>', body)
            world_sdf = world_sdf[:m.start(1)] + body + world_sdf[m.end(1):]
        models += [signal_lit_model(light, c, lenses[c], c == shown) for c in SIGNAL_COLORS]
    i = world_sdf.rfind('</world>')
    if i < 0:
        raise RuntimeError('월드 파일에 </world> 가 없습니다')
    return world_sdf[:i] + SIGNAL_STATE_PLUGIN + ''.join(models) + '\n  ' + world_sdf[i:]


def signal_state(sig, t):
    """출발(t = 0)부터 t 초 뒤의 불빛 — 초록 → 노랑 → 빨강 주기, offset_s 만큼 앞서 시작한다."""
    g, y, r = (float(sig[k]) for k in ('green_s', 'yellow_s', 'red_s'))
    p = (max(t, 0.0) + float(sig.get('offset_s') or 0.0)) % (g + y + r)
    return 'green' if p < g else ('yellow' if p < g + y else 'red')


def signal_plan(world_sdf, signals, offset=None):
    """신호마다 이번 실행의 시작 시점(offset_s)과 렌즈 자리를 붙인다. offset 이 없으면 주기 안에서 무작위."""
    plan = []
    for s in signals:
        cycle = sum(float(s[k]) for k in ('green_s', 'yellow_s', 'red_s'))
        off = float(offset) % cycle if offset is not None else round(random.uniform(0.0, cycle), 1) % cycle
        plan.append({'id': s['id'], 'name': s.get('name') or s['id'],
                     'green_s': float(s['green_s']), 'yellow_s': float(s['yellow_s']), 'red_s': float(s['red_s']),
                     'offset_s': round(off, 1), 'lenses': signal_lenses(world_sdf, s['id'])})
    return plan


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
                              description='리더보드 코스 자료의 course.json — 출발 자리·장애물·신호를 채점 서버와 같게 '
                                          '(start·x·y·yaw 무시)'),
        DeclareLaunchArgument('signal_offset', default_value='',
                              description='course 에 신호가 있을 때 주기의 시작 시점 [s] — 비우면 띄울 때마다 무작위 '
                                          '(채점 서버도 주행마다 무작위)'),
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
        # course:= 에 장애물·신호가 있으면 월드 사본에 넣어 띄운다 (원본은 그대로) — 채점 서버도 같은 방식
        world = LaunchConfiguration('world').perform(context)
        course = load_course(context)
        notes = []
        if course:
            want = course.get('world')
            if want and os.path.basename(world) != want:
                notes.append(LogInfo(msg=f'[course] 이 코스는 {want} 기준인데 {os.path.basename(world)} 를 띄웁니다 '
                                         '— final_course.launch.py 로 띄우세요'))
            obstacles = course.get('obstacles') or []
            signals = course.get('signals') or []
            if obstacles or signals:
                with open(world, encoding='utf-8') as fh:
                    txt = fh.read()
                if signals:
                    offset = LaunchConfiguration('signal_offset').perform(context).strip()
                    plan = signal_plan(txt, signals, float(offset) if offset else None)
                    txt = add_signals(txt, {s['id']: signal_state(s, 0.0) for s in plan})
                    pf = tempfile.NamedTemporaryFile('w', suffix='_signals.json', delete=False, encoding='utf-8')
                    json.dump({'signals': plan}, pf); pf.close()
                    # 시뮬레이션 시간에 맞춰 불빛을 바꾸는 노드 — 채점 서버는 채점기가 같은 일을 한다
                    notes.append(Node(package='ssc_class_gazebo', executable='signal_lights.py', output='screen',
                                      arguments=[pf.name]))
                    for s in plan:
                        notes.append(LogInfo(msg=f"[course] 신호 {s['name']} — 초록 {s['green_s']:g}초 → 노랑 "
                                                 f"{s['yellow_s']:g}초 → 빨강 {s['red_s']:g}초, 시작 시점 "
                                                 f"{s['offset_s']:g}초 (signal_offset:= 로 고정)"))
                txt = add_obstacles(txt, obstacles) if obstacles else txt
                tmp = tempfile.NamedTemporaryFile('w', suffix='_course.world', delete=False, encoding='utf-8')
                tmp.write(txt); tmp.close()
                world = tmp.name
            sp = course['spawn']
            notes.append(LogInfo(msg=f"[course] {course.get('name', '')} ({course.get('key', '')} 판 "
                                     f"{course.get('version', '')}) — 출발 ({sp['x']}, {sp['y']}, {sp['yaw']}), "
                                     f'장애물 {len(obstacles)}개, 신호 {len(signals)}개'))
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
