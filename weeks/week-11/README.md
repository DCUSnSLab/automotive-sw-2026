# 11주차 — 인지 3: 신호등 인식 + 신호 정지·출발

> 10주차는 카메라로 바닥의 노란 차선을 찾아 조향 → 이번 주는 **카메라로 신호등 불빛 색**을 알아내 정지·출발.
> 처리 6단계: ① 관심 영역(ROI) → ② HSV 변환 → ③ 색 3개 마스크·픽셀 수 → ④ 판정(빨강 우선) → ⑤ 연속 3프레임 확인 → ⑥ 정지 판단.
> 새로 쓰는 OpenCV 함수는 `cv2.countNonZero`·`cv2.bitwise_or` 2개. 나머지는 10주차 도구(자르기·cvtColor·inRange)와 if 문.
> 코드는 **basics 패키지에 추가**. [nodes/](nodes/)의 두 파일은 뼈대가 완성된 **빈칸 파일** — `# TODO` 자리만 채움. TODO 번호 = 이론 슬라이드의 단계 번호. 완성본은 다음 주 공개.
>
> | 파일 | 내용 | 채울 TODO | 단계 |
> |---|---|---|---|
> | `signal_course.json` | 출발 자리 + 신호 주기 (초록 12 s → 노랑 3 s → 빨강 12 s) | — | 단계 0 |
> | `nodes/signal_detect.py` | 카메라 → `/signal/state`, `/signal/size`, `/signal/debug` | ① 관심 영역 · ② HSV · ③ 마스크·픽셀 수 · ④ 판정 | 단계 1 |
> | `nodes/signal_drive.py` | `/signal/state`·`/signal/size` → `/cmd_vel`, 36 m 에서 FINISH | ⑥ 정지 판단 | 단계 2 |

## 실습 로드맵

0. 주차 루틴 — week-11 받기 + 시뮬레이터(`course:=signal_course.json`) + 신호 확인 + 파일 2개 추가
1. `signal_detect` — TODO ①~④ → `rqt_image_view` 로 `/signal/debug` 확인
2. `signal_drive` — TODO ⑥ → 빨강·노랑에 정지선 앞 정지 → 초록에 출발 → FINISH
3. 튜닝 — `near_px`·`pass_px`·`speed`

---

## [단계 0] 주차 루틴

### 시뮬레이터 (터미널 1)

```bash
cd ~/ssc_ws/src/ssc_class
git pull
git remote update
git checkout week-11
cd ~/ssc_ws
colcon build --packages-select ssc_class_gazebo
source ~/ssc_ws/install/setup.bash
ros2 launch ssc_class_gazebo final_course.launch.py course:=$HOME/ssc_ws/src/ssc_class/weeks/week-11/signal_course.json
```

- `course:=` 로 코스 파일을 주면 **출발 자리와 신호 주기**가 정해짐 (기말 프로젝트와 같은 실행 방식)
- 출발 자리: 남쪽 도로의 오른쪽 차로, 북쪽을 보고 정지선 약 17 m 앞
- 정면 신호등: 정지선에서 16 m 뒤(교차로 건너편) · 높이 3.5 m — 초록 12 s → 노랑 3 s → 빨강 12 s 반복
- 신호가 바뀔 때마다 터미널 1 에 한 줄 출력: `[signal_lights]: 북행 신호 → 빨강 (t = 25.0 s)`
- 신호 시작 시점은 실행할 때마다 무작위. 고정하려면 끝에 `signal_offset:=0` 추가
- 카메라: 800x450 · 12 Hz (기말 채점 서버와 같은 설정)

### 신호 확인

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

- 영상 위쪽 가운데의 작은 신호등이 정면 신호. 왼쪽·오른쪽의 신호등은 다른 방향 신호(색이 바뀌지 않음)
- (선택) 10주차 `lane_viewer` 로 켜진 불을 클릭해 HSV 값 확인 — 빨강 H 3 · 노랑 H 30 · 초록 H 72, S 200 이상 · V 255

### 다시 달리기

- 터미널 1 의 시뮬레이터를 Ctrl+C 로 끄고 다시 실행 (차·신호가 처음 상태로)
- 손으로 몰기(텔레옵): `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### 파일 2개 추가 (터미널 2)

```bash
cd ~/ssc_ws/src/ssc_class/weeks/week-11/nodes
cp signal_detect.py signal_drive.py ~/basics_ws/src/basics/basics/
```

파일 구조:

```
~/basics_ws/src/basics/
├── setup.py              ← entry_points 에 두 줄 추가
└── basics/
    ├── (6·7·9·10주차 파일들)
    ├── signal_detect.py  ← 빈칸: TODO ①~④ → /signal/state, /signal/size   [단계 1]
    └── signal_drive.py   ← 빈칸: TODO ⑥ → /cmd_vel                        [단계 2]
```

`setup.py` 의 `entry_points` 에 추가:

```python
            'signal_detect = basics.signal_detect:main',
            'signal_drive = basics.signal_drive:main',
```

빌드:

```bash
cd ~/basics_ws
colcon build --packages-select basics
source ~/basics_ws/install/setup.bash
```

> 빈칸 파일은 채우기 전에도 실행됨 — 판정은 항상 `NONE`, 차는 신호와 관계없이 직진.
> 환경이 깨졌으면 [3주차 복구 절차](../week-03/README.md) 먼저.

---

## [단계 1] signal_detect — TODO ①~④

`basics/basics/signal_detect.py` — 카메라 영상 1장 → 신호등 색 1개. 영상은 640x360 으로 줄여 처리.

| TODO | 이론 | 내용 |
|---|---|---|
| ① 관심 영역 | [①] | `roi = img[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1]` — 행 55~150, 열 270~370 (행 먼저, 열 다음) |
| ② HSV 변환 | [②] | `cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)` |
| ③ 마스크·픽셀 수 | [③] | `cv2.inRange` 4번(빨강 2개 + 노랑 + 초록) → 빨강은 `cv2.bitwise_or` 로 합침 → `cv2.countNonZero` |
| ④ 판정 | [④] | `min_px` 이상인 색을 빨강 → 노랑 → 초록 순서로 (`if / elif`) → `seen`, `size` |

- 색 범위: S·V 하한은 세 색 공통 `(s_lo, v_lo)` = (120, 150). H: 빨강 0~8 과 172~179 · 노랑 18~35 · 초록 45~90
- ⑤ 연속 3프레임 확인(`hold`)과 디버그 영상은 완성되어 있음
- 발행: `/signal/state` (String: `RED`/`YELLOW`/`GREEN`/`NONE`) · `/signal/size` (Int32: 이번 프레임에서 본 불의 픽셀 수)

실행·확인:

```bash
ros2 run basics signal_detect
ros2 run rqt_image_view rqt_image_view /signal/debug
ros2 topic echo /signal/state
```

- `/signal/debug`: 흰 사각형 = 관심 영역, 왼쪽 아래 = 관심 영역 3배 확대, 글자 = 판정과 `R / Y / G` 픽셀 수
- 확인: 터미널 1 의 신호 로그와 판정이 같은가 · `R / Y / G` 중 하나만 0 이 아닌가
- 실측(관심 영역 픽셀 수): 출발 자리 6~7 · 정지선 11 m 앞 8 · 정지선 24~25. 영상 전체에서 세면 빨강 79 · 노랑 3382 · 초록 188 (정면 신호 빨강일 때) — 관심 영역이 필요한 이유

## [단계 2] signal_drive — TODO ⑥

`basics/basics/signal_drive.py` — 직진·FINISH 는 완성. 정지 판단만 채움:

| 크기(`/signal/size`) | 구간 | 동작 |
|---|---|---|
| `near_px`(20) 미만 | 멀다 | 어떤 색이든 주행 |
| `near_px` 이상 `pass_px`(28) 미만 | 정지 구간 (길이 약 3.5 m) | 빨강·노랑 → 대기 시작(정지), 초록 → 주행 |
| `pass_px` 이상 | 정지선을 지났다 | 어떤 색이든 주행 (교차로 안에서 멈추지 않음) |

- 한번 대기를 시작하면(`self.waiting = True`) 초록을 볼 때까지 정지 유지 — 크기가 프레임마다 1~2픽셀 흔들려도 멈췄다 가다 하지 않음

```bash
ros2 run basics signal_drive
```

- 기본값: `speed 1.0 / near_px 20 / pass_px 28 / run_dist 36.0`
- 결승: `/odom` 기준 출발점에서 36 m (교차로를 건넌 뒤) 에서 자동 정지 + `FINISH` 출력
- 실측(기본값, 신호 시점을 고정한 3회)
  - 빨강에 도착 → 정지선 1.9 m 앞(차 중심)에 정지 → 초록으로 바뀐 뒤 0.4 s 에 출발
  - 정지 구간에서 노랑으로 바뀜 → 정지선 1.1 m 앞에 정지 → 다음 초록에 출발
  - 정지선을 지난 뒤 노랑으로 바뀜 → 멈추지 않고 통과

## [단계 3] 튜닝

노드를 켠 채 `param set` (7주차와 동일):

```bash
ros2 param set /signal_drive near_px 22
ros2 param set /signal_drive speed 1.5
```

- `near_px` 를 키우면 정지선에 더 가까이 가서 정지. `pass_px` 보다 크면 정지 구간이 없어짐
- 속도를 올리면 정지 거리가 늘어남 → `near_px` 를 줄임

## 자주 하는 실수

- `img[x, y]` 순서로 자름 → 엉뚱한 곳을 봄. 영상은 `img[행(y), 열(x)]`
- 빨강 범위를 0~8 하나만 → 시뮬레이터에서는 되지만 실제 카메라의 빨강(H 172~179)을 놓침
- S·V 하한을 0 으로 → 꺼진 렌즈·하늘까지 세어 항상 초록 또는 빨강
- 판정 순서를 초록부터 → 빨강과 초록이 함께 보일 때 통과해 버림
- `near_px` 를 `pass_px` 보다 크게 → 멈추지 않음

## 과제

1. 신호 주행 리포트 — 빨강에 정지 → 초록에 출발 → FINISH 까지의 `signal_drive` 로그 캡처
2. `near_px` 를 16 · 20 · 24 로 바꿔 정지 위치가 어떻게 달라지는지 기록
3. (도전) 차선 주행과 합치기 — 10주차 `lane_follow` 에 `/signal/state`·`/signal/size` 구독과 ⑥ 규칙 추가
4. (도전) 정지선으로 정확히 서기 — 크기 대신 10주차 도전의 정지선(흰 가로선) 검출로 정지 위치 결정
5. (도전) 속도 2.0 m/s 에서도 정지선을 넘지 않는 `near_px` 찾기

제출: `.py` + 캡처, LMS — 다음 수업 전날 23:59

## 한계

- 픽셀 수 거리 눈금은 거친 방법 (오차 약 1 m). 신호등 크기나 카메라 해상도가 바뀌면 `near_px`·`pass_px` 를 다시 재야 함
- 관심 영역이 고정 → 차가 차로 가운데에서 정면을 볼 때만 정면 신호등이 그 안에 들어옴
