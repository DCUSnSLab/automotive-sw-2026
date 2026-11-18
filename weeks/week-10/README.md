# 10주차 — 인지 2: 카메라와 차선 검출 + 차선 추종 주행

> 6~9주차의 라이다는 높이 있는 물체(연석·라바콘)만 봄 → 이번 주는 **카메라로 바닥의 노란 차선**을 찾아 따라 달림.
> 처리 5단계: ① 노란색 마스크(HSV) → ② 조감도(Bird's-Eye View) → ③ 행 스캔 → ④ 차로 중심·offset → ⑤ 조향(7주차 PD 재사용).
> OpenCV·numpy 는 영상 처리 함수 호출(cvtColor·inRange·warpPerspective)에만 사용. 판단 로직은 6~9주차와 같은 for/if.
> 코드는 **basics 패키지에 추가**. [nodes/](nodes/)의 `lane_viewer.py` 는 완성본, 나머지 두 파일은 뼈대가 완성된 **빈칸 파일** — `# TODO` 자리만 채움. TODO 번호 = 이론 슬라이드의 5단계 번호. 완성본은 다음 주 공개.
>
> | 파일 | 내용 | 채울 TODO | 단계 |
> |---|---|---|---|
> | `lane_viewer.py` | 카메라 보기 + 클릭한 픽셀의 BGR·HSV 출력 (완성본) | — | 단계 1 |
> | `lane_detect.py` | 카메라 → `/lane/offset`, `/lane/debug` | ① 마스크 · ② 조감도 · ③ 행 스캔 · ④ 차로 중심 | 단계 2 |
> | `lane_follow.py` | `/lane/offset` → `/cmd_vel`, 42 m 에서 FINISH | ⑤ 조향 | 단계 3 |
> | `lane_course.json` (week-10 폴더) | 출발 자리 (차선 주행 구간 앞) — 시뮬레이터 실행 때 `course:=` 로 지정 | — | 단계 0 |
> | `cv_basics.py` (week-10 폴더) | (선택) OpenCV 맛보기 — ROS 없이 파이썬만으로 실행 | — | 단계 1 |

## 실습 로드맵

0. 주차 루틴 — week-10 받기 + 시뮬레이터 재빌드 1회 + `course:=lane_course.json` 실행 + 파일 3개 추가
1. 카메라 확인 — 토픽 주기, `lane_viewer` 로 노란 차선·도로의 HSV 값 읽기
2. `lane_detect` — TODO ①~④ → `rqt_image_view` 로 `/lane/debug` 확인
3. `lane_follow` — TODO ⑤ → 직선 20 m → 왼쪽 90° 곡선 → 직선 → FINISH
4. 튜닝 — Kp·Kd·speed, HSV 범위, lane_width
5. (도전) 정지선 검출 — 흰 가로선 앞 2초 정지 후 출발

---

## [단계 0] 주차 루틴

### 시뮬레이터 (터미널 1)

```bash
cd ~/ssc_ws/src/ssc_class
git pull
git remote update
git checkout week-10
cd ~/ssc_ws
colcon build --packages-select ssc_class_gazebo
source ~/ssc_ws/install/setup.bash
ros2 launch ssc_class_gazebo final_course.launch.py course:=$HOME/ssc_ws/src/ssc_class/weeks/week-10/lane_course.json
```

- `course:=` 로 코스 파일을 주면 **출발 자리**가 정해짐 (기말 프로젝트와 같은 실행 방식) — `lane_course.json` = 차선 주행 구간 앞, 오른쪽 차로 중심
- 이번 주부터 `final_course.launch.py` 를 사용 → **재빌드 1회 필요** (`colcon build --packages-select ssc_class_gazebo`)
- 카메라: 800x450 · 12 Hz (기말 채점 서버와 같은 설정). 노드는 320x180 으로 줄여 처리 — `lane_detect` CPU 약 5 % (실측)
- 출발 후 코스: 직선 약 20 m → 왼쪽 90° 곡선(차로 중심 반경 약 6.5 m) → 남쪽 직선 약 11 m
- 차선은 전부 **노란색** (오른쪽 바깥 실선 + 중앙선). 흰색은 노면 표시(글자·마름모·횡단보도·정지선)
- PC 가 느리면 끝에 `gui:=false` 를 붙여 Gazebo 화면 없이 실행 (카메라·rqt 는 그대로 동작)

### 리셋

- 차량만 출발점으로: `ros2 service call /reset_world std_srvs/srv/Empty`
- 시뮬레이터 전체 초기화: `gz world -r`
- 손으로 몰기(텔레옵): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### 파일 3개 추가 (터미널 2)

```bash
cd ~/ssc_ws/src/ssc_class/weeks/week-10/nodes
cp lane_viewer.py lane_detect.py lane_follow.py ~/basics_ws/src/basics/basics/
```

파일 구조:

```
~/basics_ws/src/basics/
├── setup.py              ← entry_points 에 세 줄 추가
└── basics/
    ├── (6·7·9주차 파일들)
    ├── lane_viewer.py   ← 완성본: 카메라 보기 + 클릭 HSV       [단계 1]
    ├── lane_detect.py   ← 빈칸: TODO ①~④ → /lane/offset       [단계 2]
    └── lane_follow.py   ← 빈칸: TODO ⑤ → /cmd_vel             [단계 3]
```

`setup.py` 의 `console_scripts` 에 추가:

```python
'lane_viewer = basics.lane_viewer:main',
'lane_detect = basics.lane_detect:main',
'lane_follow = basics.lane_follow:main',
```

빌드:

```bash
cd ~/basics_ws
colcon build --packages-select basics
source ~/basics_ws/install/setup.bash
```

> 환경이 깨졌으면 [3주차 복구 절차](../week-03/README.md) 먼저.

## [단계 1] 카메라 확인

### 토픽 주기

```bash
ros2 topic hz /camera/image_raw
```

- 12 Hz 근처 (실측 11.9 Hz)
- 크게 낮으면 다른 프로그램을 닫고 `gui:=false` 로 시뮬레이터 재실행

### lane_viewer — 클릭해서 HSV 값 읽기

```bash
ros2 run basics lane_viewer
```

- 창 = 카메라 영상을 320x180 으로 줄인 뒤 2배 확대 (노드가 실제로 처리하는 크기)
- 왼쪽 클릭 → 터미널에 `(x, y) BGR (...) HSV (...)` 출력
- `s` = 현재 프레임 저장(frame_00.png …), `q` = 종료
- 확인할 것
  - 노란 차선: H 약 30, S·V 큼 → 마스크 범위 (20~35, 80 이상, 120 이상)
  - 회색 도로: S 작음 → 걸러짐
  - 흰 글자: S 거의 0 → 걸러짐

### (선택) OpenCV 맛보기 — cv_basics.py

- OpenCV 가 처음이면 ROS 없이 파이썬만으로 먼저 확인 (이론 1 의 맛보기 슬라이드와 같은 코드)
- 준비: lane_viewer 창에서 `s` → `frame_00.png` 가 lane_viewer 를 실행한 폴더에 저장됨

```bash
cd ~                                                    # lane_viewer 를 실행한 폴더
cp ~/ssc_ws/src/ssc_class/weeks/week-10/cv_basics.py .
python3 cv_basics.py
```

- 출력: 픽셀 1개의 B G R, 노란 픽셀 수 — for 루프(약 100 ms) vs OpenCV 함수(약 0.2 ms) 시간 비교
- 저장: `out.png`(빨간 원), `mask.png`(노란색 마스크)
- 프레임마다 값은 다름 — (99, 106) 이 차선이 아니면 다른 값이 나옴

## [단계 2] lane_detect — TODO ①~④

`basics/basics/lane_detect.py` — 구독·발행·조감도 행렬·디버그 영상·NaN 발행은 완성. TODO 4곳만 채움.

| TODO | 이론 | 내용 |
|---|---|---|
| ① 노란색 마스크 | [①] | `cv2.cvtColor(img, cv2.COLOR_BGR2HSV)` → `cv2.inRange(hsv, (h_lo, s_lo, v_lo), (h_hi, 255, 255))` |
| ② 조감도 | [②] | `cv2.warpPerspective(mask, self.M, (BW, BH))` — 앞 2~8 m, 좌우 ±3 m, 1 px = 0.03 m |
| ③ 행 스캔 | [③] | 행 170·150·130 마다 시작점(직전 차로 중심)에서 오른쪽·왼쪽으로 첫 노란 픽셀, 2개 행 이상이면 평균 |
| ④ 차로 중심 | [④] | 오른쪽 우선 — `right_x - half_px`, 오른쪽이 없으면 `left_x + half_px`, 둘 다 없으면 -1 |

- 발행 토픽
  - `/lane/offset` (std_msgs/Float32) — 차로 중심의 옆 위치 [m], **+ = 차로 중심이 왼쪽**, 차선 없으면 NaN
  - `/lane/debug` (sensor_msgs/Image) — 조감도 + 찾은 점 (구독자가 있을 때만, 3프레임마다)
- 파라미터: `h_lo 20 / h_hi 35 / s_lo 80 / v_lo 120 / lane_width 3.1`

실행·확인:

```bash
ros2 run basics lane_detect
ros2 run rqt_image_view rqt_image_view /lane/debug
ros2 topic echo /lane/offset
```

- `/lane/debug`: 빨간 점 = 오른쪽 차선, 파란 점 = 왼쪽 차선, 초록 점 = 차로 중심, 흰 원 = 차 중심
- 출발 위치(차로 중심)에서 offset 이 0 근처면 정상

## [단계 3] lane_follow — TODO ⑤ + 첫 주행

`basics/basics/lane_follow.py` — offset 구독·차선 놓침 정지·감속·FINISH 는 완성. 조향 식만 채움 (7주차 PD 와 같은 모양):

```
e  = offset                       (차로 중심이 왼쪽이면 + → 왼쪽(+)으로 꺾음)
de = (e - prev_e) / 0.1           (10 Hz, 첫 tick 은 0)
steer = kp·e + kd·de,  ±0.40 rad 클램프
speed = speed·(1 − 0.5·|steer|/0.40)   (완성)
```

```bash
ros2 run basics lane_follow
```

- 기본값: `kp 0.2 / kd 0.05 / speed 1.0 / lost_hold 0.5 / finish_dist 42.0`
- 결승: `/ground_truth/odom` 으로 누적한 주행 거리 42 m 에서 자동 정지 + 기록 출력 (곡선이 있어 직선거리 대신 누적 거리)
- 실측(기본값, 800x450 · 12 Hz): `FINISH  45.4 s  (42.1 m)` — 차로 중심과의 평균 거리 직선 0.03 m · 곡선 0.22 m · 남쪽 직선 0.07 m
- 차선 놓침: NaN 이 들어오면 직전 offset 유지, `lost_hold` 0.5 s 넘게 못 보면 정지 (`LANE LOST — 정지`) → 다시 보이면 자동 재출발
  - PC 가 잠깐 멈춰 메시지가 끊겨도 같은 규칙으로 정지
- 다시 달리기: Ctrl+C → `ros2 service call /reset_world std_srvs/srv/Empty` → 재실행

## [단계 4] 튜닝

노드를 켠 채 `param set` (7주차와 동일):

```bash
ros2 param set /lane_follow kp 0.3
ros2 param set /lane_detect s_lo 60
```

| 증상 | 처방 |
|---|---|
| 곡선 안쪽으로 파고듦 | `kp` ↓ |
| 곡선에서 바깥으로 밀림·늦게 꺾음 | `kp` ↑ |
| 직선에서 좌우로 흔들림 | `kd` ↑ 또는 `kp` ↓ |
| 직선에서 한쪽으로 치우침 | `lane_width` 확인 (`/lane/debug` 의 두 차선 간격) |
| LANE LOST 자주 | `h_lo`·`h_hi`·`s_lo`·`v_lo` 범위 확인 (`lane_viewer` 로 HSV 다시 읽기) |

게인 비교 실측 (곡선 / 남쪽 직선의 차로 중심과 평균 거리):

| kp | 곡선 | 남쪽 직선 |
|---|---|---|
| 0.6 | 0.49 m | 0.26 m |
| 0.3 | 0.36 m | 0.03 m |
| 0.2 (기본) | 0.21 m | 0.05 m |

## [단계 5] (도전) 정지선 검출

- 출발: `start:=right` — 약 13 m 앞에 흰 정지선
- 새 노드 `stopline_detect` (직접 작성)
  - 흰색 마스크 `cv2.inRange(hsv, (0, 0, 200), (179, 40, 255))` → 같은 조감도 행렬로 변환
  - 조감도 아래(가까운 쪽) 행부터 위로 → 차 앞 ±40 px(±1.2 m) 창의 흰 픽셀이 70 % 이상인 첫 행 = 정지선
  - 거리 = `2.0 + (199 − 행) × 0.03` m → `/lane/stopline` (Float32, 없으면 NaN)
  - 노면 글자·마름모·횡단보도 줄무늬는 가로로 70 % 를 못 채워 걸러짐
- `lane_follow` 수정
  - `/lane/stopline` 이 2.6 m 이하 → 2초 정지(`STOP LINE`) → 출발(`GO`)
  - 출발 후 4초 동안은 정지선 무시 (같은 선에서 다시 서지 않도록)
- 실측: 정지선 2곳에서 2.57 m · 2.51 m 에 정지 → 재출발

```bash
ros2 launch ssc_class_gazebo license_course.launch.py start:=right
```

## 자주 하는 실수

- 마스크가 비어 있음 → BGR 을 HSV 로 안 바꾸고 `inRange` (또는 `imgmsg_to_cv2(msg, 'rgb8')` 사용)
- H 범위를 0~360 으로 생각 → OpenCV 의 H 는 0~179 (노란색 약 30)
- `img[x][y]` 로 읽음 → `img[y][x]` (행이 먼저)
- 조향 부호 반대 → offset 이 + 면 왼쪽(+)으로
- 곡선에서 차선을 놓침 → 행 스캔 시작점을 직전 차로 중심이 아닌 화면 가운데로 고정함
- 노드가 느림 → 320x180 으로 줄이지 않음

## 과제

1. 차선 주행 리포트 — kp 2개 이상 조합의 FINISH 기록·관찰 비교 표 + 최종 `ros2 param dump /lane_follow`
2. HSV 범위 실험 — `s_lo`·`v_lo` 를 바꿔 마스크 변화 캡처 2장 + 한 줄 해석
3. (도전) 정지선 검출 — 정지선 앞 정지 후 출발하는 터미널 로그·영상 캡처
4. (도전) 양쪽 평균 방식으로 바꿔 곡선 조향 그래프 비교 — 오른쪽 우선과 차이 설명

제출: `.py` + 캡처, LMS — 다음 수업 전날 23:59
