# ssc_class_gazebo — 운전면허 시험장 월드 + 실습 차량

ROS 2 Humble + Gazebo Classic 11. 실행 방법·인자·토픽은 `launch/license_course.launch.py` 머리말 참고.

## 저메모리(rip) 버전 — `license_course_rip.launch.py`

RAM 16 GB 에 iGPU 가 시스템 메모리를 나눠 쓰는 노트북(WSL2 포함)에서는 원본 월드의 gzserver 가 1.1 GB 를 넘겨
전체가 느려진다. 이런 장치는 **원본 대신 rip 런치를 쓰면 된다**. 인자·토픽·플러그인은 원본과 완전히 같다.

```bash
ros2 launch ssc_class_gazebo license_course_rip.launch.py                 # 원본: license_course.launch.py
ros2 launch ssc_class_gazebo license_course_rip.launch.py start:=right    # 7주차 이후 우측 출발
ros2 launch ssc_class_gazebo license_course_rip.launch.py software_gl:=true lidar_viz:=true
```

rip 런치는 원본 런치를 그대로 include 하면서 `world`·`car_sdf` 두 인자만 저메모리 파일로 바꿔 넣는다.
원본 월드·차량 파일은 손대지 않으므로 두 버전이 한 패키지 안에 공존한다.

| | 원본 | rip |
|---|---|---|
| 월드 | `worlds/classroom.world` — 모델 215개 | `worlds/classroom_rip.world` — 정적 모델 177개를 모델 1개(`course_static`)로 병합, 장식 모델 39개 삭제 |
| 차량 | `models/ssc_class_car` — COLLADA 75k 삼각형, 카메라 1280×720 | `models/ssc_class_car_rip` — 바이너리 STL 15k 삼각형, 카메라 **800×450** (30 Hz) |
| 라이다·플러그인·토픽 | 720 샘플 20 Hz, /cmd_vel·/odom·/scan·/camera/* | 동일 |

### 측정 (521-0, Gazebo 11.10.2, week-07 기준, GPU·GUI 없음, 스폰 후 정지 상태)

| 런치 | gzserver RSS | 카메라 발행률 | 우측 출발 시 오른쪽 연석 거리 |
|---|---|---|---|
| `license_course.launch.py start:=right` | 1,140 MB | 14 Hz (1280×720) | 2.054 m |
| `license_course_rip.launch.py start:=right` | **636 MB** | 18 Hz (800×450) | 2.054 m |

좌측 출발(기본)도 왼쪽 연석 2.053 m 로 같다. 소프트웨어 렌더링(llvmpipe) + GUI 조건에서는 gzserver+gzclient 합계 PSS 가
약 1,560 MB → 930 MB 로 줄었다(2026-10-13 실측). 라이다 스캔은 출발선 옆 대기 쉘터가 빠진 방향 외에는 원본과 0.6 mm 이내로 같다.

### 왜 모델 병합이 핵심인가

Gazebo Classic 은 월드의 **모델 1개마다 약 1.7 MB, 링크 1개마다 약 0.65 MB** 를 gzserver 에서 고정으로 소비한다
(비주얼 1개는 약 0.08 MB; 2026-10-13 실측). 시험장 월드는 차선 하나하나가 별도 모델이라 모델 수만 215개 →
이 오버헤드만 ~360 MB 였다. 정적 물체는 움직이지 않으므로 하나의 정적 모델에 모든 visual/collision 을
월드 좌표로 합쳐 넣어도 물리·센서 결과가 같다. 바뀌는 점은 **GUI 모델 트리에서 차선·콘을 개별 선택/이동할 수 없다**는 것뿐.

### rip 월드 — 무엇을 남기고 무엇을 지웠나

자율주행 실습(벽 추종·차선 추종·장애물 회피·정지선/신호 인식·주차)에 필요한 것만 남겼다.

| 남김 | 이유 |
|---|---|
| 바닥·도로면(아스팔트 텍스처) | 주행면, 카메라 차선 인식 배경 |
| 황색 차선 45개, 정지선 10, 횡단보도 6, 노면 화살표 21, 주차구역 | 차선·정지선 인식, 주차 실습 |
| 바깥 연석 + 연석 링 9조(상·측면) | 코스 경계 — 벽 추종의 라이다 반사면이자 물리 충돌체 |
| 라바콘 68개 | 라이다 장애물 |
| 신호등 4개 | 신호 인식 |

| 지움 | 비고 |
|---|---|
| 대기 쉘터(지붕·기둥·유리·벤치) | 출발선 옆 장식 |
| 공사차량 `lcs_5004`(VMS 포함) | 장식 (`<include>`) |
| 철길 판·홈·레일, 속도 표지판(20), 노면 문자, 녹지 섬 9개, 마름모 6, 빗금 6 | 장식·면허시험용 요소 |

rip 월드는 원본 모델 폴더(`models/yongin_license_course_runtime`)의 메시를 그대로 참조하므로 파일 삭제는 없다.

### 월드를 고쳤을 때 rip 월드 다시 만들기

원본 `worlds/classroom.world` 를 편집한 뒤 아래 명령으로 `classroom_rip.world` 를 재생성한다
(`tools/merge_static_models.py`: 정적 모델 병합 + `--drop` 정규식에 맞는 모델/`<include>` 제거).

```bash
cd ssc_class_gazebo
python3 tools/merge_static_models.py worlds/classroom.world worlds/classroom_rip.world \
  --drop '^course_waiting_shelter' --drop '^lane_control_vehicle$' --drop '^course_railroad' \
  --drop '^sign_20' --drop '^text_marking' --drop '^green_island' --drop '^mark_diamond' --drop '^hatch_yellow_lines'
```

라바콘(`course_fixed_cone*`)은 원본에서 `static=false` 지만 고정물이라 함께 병합된다. 움직이는 콘이 필요하면
`--also-merge ''` 로 제외하거나 `--keep '<이름 정규식>'` 으로 뺀다. 플러그인·조인트·센서가 있는 모델과 `<include>` 는
자동으로 병합 대상에서 제외된다. 특정 장식을 되살리려면 해당 `--drop` 만 빼고 다시 생성하면 된다.

### 차량(rip) 카메라 해상도

`models/ssc_class_car_rip/model.sdf` 의 `<sensor name="front_camera">` 안 `<width>`/`<height>` 를 고친다.
⚠ 소프트웨어 렌더링(llvmpipe)에서는 640×360·640×480·960×540 이 카메라를 0.3~0.6 Hz 로 급락시킨다(실측).
800×450·1280×720 은 10 Hz 이상으로 정상 — 해상도를 바꾸면 `software_gl:=true` 로 한 번 확인할 것.
