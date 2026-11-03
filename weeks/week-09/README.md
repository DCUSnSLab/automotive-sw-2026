# 9주차 — 판단 2: FGM(Follow the Gap) + 미니 타임어택

> 벽을 따라가는 차(7주차)에서 **장애물을 피해 달리는 차**로.
> 막힌 곳을 세는 대신 **뚫린 곳(gap)** 을 찾아 그 가운데로 조향합니다.
> 이번 주 새 개념은 사실상 하나, **gap**. 창·전처리·조향·클램프·파라미터는 전부 6·7주차 도구입니다.
> 코드는 **basics 패키지에 추가**합니다. [nodes/](nodes/)의 두 파일은 7주차처럼 뼈대가 완성된 **빈칸 파일**이며, `# TODO` 자리만 채웁니다. TODO 번호는 이론 슬라이드의 6단계(① 전처리 ② 최근접점 ③ 안전 거품 ④ gap 탐색 ⑤ 목표각 ⑥ 조향)와 같습니다. 완성본은 다음 주 공개.
>
> | 파일 | 채울 TODO | 단계 |
> |---|---|---|
> | `gap_finder.py` | ① 전처리 · ② 최근접점 · ④ gap 탐색 | 단계 1 |
> | `fgm_drive.py` | ①②④ (gap_finder에서 복사) · ③ 안전 거품 · ⑤ 목표각 · ⑥ 조향·감속 | 단계 2 |

## 오늘 실습의 전체 로드맵 — 두 주제

**실습 1 (기본) — 라바콘 4개 슬라럼**: 차 정면의 콘을 피해 결승선까지

0. 주차 루틴 — week-09 받기 + 시뮬레이터 + **라바콘 4개 설치**
1. `gap_finder` — 조향 전에 gap을 눈으로
2. `fgm_drive` 첫 주행 — 기본 파라미터로 슬라럼 → 결승선 28 m에서 FINISH
3. 파라미터 튜닝 — threshold·bubble·steer_gain

**실습 2 (도전) — S자 라바콘 통로**: 콘 두 줄 사이의 폭 2.6 m 통로를 따라 완주

4. 통로 설치 + 완주 + 속도 올리기
5. 미니 타임어택 — 통로 완주 최고 기록 경쟁

---

## 실습 1 (기본) — 라바콘 4개 슬라럼

### [단계 0] 주차 루틴 + 라바콘 4개 설치

터미널 1 — 저장소 갱신 + 시뮬레이터:

```bash
cd ~/ssc_ws/src/ssc_class
git pull
git remote update
git checkout week-09
source ~/ssc_ws/install/setup.bash
ros2 launch ssc_class_gazebo license_course.launch.py start:=right
```

> 출발 위치는 7주차와 동일하게 **도로 우측**(`start:=right`)입니다.

터미널 2 — 직선 구간에 라바콘 4개 설치:

```bash
bash ~/ssc_ws/src/ssc_class/weeks/week-09/scripts/spawn_cones.sh
```

| 콘 | 전방 거리 | 좌우 오프셋(+ = 왼쪽) | 위치 |
|---|---|---|---|
| cone1 | 6 m | +0.3 m | 차 정면 — **직진하면 충돌** (콘 가장자리 0.15 m < 차 반폭 0.32 m) |
| cone2 | 11 m | +3.0 m | 콘1을 왼쪽으로 피한 뒤 차가 실제로 지나는 반대 차로 자리 |
| cone3 | 16 m | +2.6 m | 반대 차로에서 내려오는 진행선 |
| cone4 | 21 m | +0.3 m | 우측 차로로 복귀한 자리 — 다시 정면 |

- 콘은 **차가 실제로 지나는 자리**에 놓여 있습니다. FGM은 차선을 모르고 가장 넓은 빈 공간 쪽으로 피하므로, 콘1을 왼쪽(반대 차로)으로 피한 뒤의 진행선을 실측해 콘2·3을 놓았습니다
- 결승선 = 출발점에서 **28 m** (`fgm_drive`의 `finish_dist`)
- 다시 깔끔하게: `bash .../scripts/delete_cones.sh`
- 차량 리셋: `ros2 service call /reset_world std_srvs/srv/Empty` (콘은 그대로, 차만 출발점으로)

터미널 3 — 내 작업 공간:

```bash
source ~/basics_ws/install/setup.bash
```

> 환경이 깨졌으면 [3주차 복구 절차](../week-03/README.md) 먼저.

### [단계 1] gap_finder — 조향 전에 '보기'부터

`basics/basics/gap_finder.py` — [nodes/gap_finder.py](nodes/gap_finder.py)(빈칸 파일)를 복사해 TODO ①②④ 를 채웁니다. 각 TODO 는 이론 슬라이드의 코드(12p·13p·16~17p)와 같은 for/if 문입니다.

- 전방 ±60° 창 = 720빔 중 240~480 (정면 360, 0.5°/빔 — 6주차 공식)
- **inf는 range_max로** 바꿔 둔다 — AEB에서는 inf를 버렸지만(멀면 안전), FGM에서는 inf가 가장 좋은 소식(뚫린 곳)
- **0.8 m 미만도 range_max로** — 시뮬레이터 라이다가 자기 차체 모서리(앞 0.63 m, 옆 0.33 m)를 가끔 읽습니다. FGM은 최근접점에 반응하므로 걸러야 합니다 (`body_min` 파라미터)
- gap = `threshold`(기본 2.5 m)를 넘는 연속 빔 구간

setup.py 등록(`'gap_finder = basics.gap_finder:main',`) → 재빌드 → 실행:

```bash
ros2 run basics gap_finder
```

확인:

- 출발선: 정면이 통째로 하나의 넓은 gap (`gap 1개 | 최대 ...빔`)
- 텔레옵으로 콘1에 2.5 m 안까지 다가가면 — gap이 둘로 갈라지는가, 최근접 거리·각도가 콘1을 가리키는가

### [단계 2] fgm_drive — 첫 슬라럼

`basics/basics/fgm_drive.py` — [nodes/fgm_drive.py](nodes/fgm_drive.py)(빈칸 파일). ①②④ 는 gap_finder 에서 그대로 복사하고, ③ 안전 거품(이론 14~15p) · ⑤ 목표각(18p) · ⑥ 조향·감속(21p·25p)을 채웁니다. 전방 정지·결승선 FINISH·10 Hz 타이머는 완성되어 있습니다.

구조는 7주차와 동일(인지는 콜백, 제어는 10 Hz 타이머, 전방 정지 내장). 판단만 바뀝니다. 결승선은 `/ground_truth/odom`으로 출발점에서의 거리를 재서 판정합니다:

```
① 안전 거품: 가장 가까운 점 주변 반각 atan(bubble_radius/거리)만큼 거리 0 처리
② gap 찾기: threshold 초과 최장 연속 구간 (min_gap_beams 미만이면 정지)
③ 조향: gap 중심 빔 → 각도 × steer_gain (클램프 ±0.40)
④ 감속: speed × (1 − 0.5·|steer|/0.40) — 꺾을수록 천천히
```

등록(`'fgm_drive = basics.fgm_drive:main',`) → 재빌드 → 실행:

```bash
ros2 run basics fgm_drive
```

기본값: `threshold 2.5 / bubble_radius 0.5 / min_gap_beams 30 / steer_gain 1.0 / speed 1.0 / front_stop 1.0 / finish_dist 28.0`

차가 콘 4개를 피해 28 m를 가면 자동 정지하며 기록을 출력합니다 (실측 2회, 기본값: 41.6 s / 51.7 s, 콘 4개 무접촉 — 콘 중심과 최소 0.99 m):

```
[fgm_drive]: FINISH  41.6 s
```

궤적: 콘1 왼쪽 → 반대 차로에서 콘2·콘3 → 우측 차로로 복귀해 콘4 오른쪽 → 결승선. 기록은 시뮬레이터 속도(RTF)에 따라 달라지므로 **같은 PC·같은 조건**에서만 비교합니다.

(출발~결승선까지 노드가 스스로 계측 — 타임어택의 공식 기록. 다시 달리기: Ctrl+C → `/reset_world` → 재실행)

### [단계 3] 튜닝 — 세 손잡이의 성격

노드를 켠 채 `param set` (7주차에서 익힌 그 손맛):

```bash
ros2 param set /fgm_drive threshold 3.0
ros2 param set /fgm_drive bubble_radius 0.7
ros2 param set /fgm_drive steer_gain 0.8
```

| 증상 | 처방 |
|---|---|
| 콘을 스친다 | `bubble_radius` ↑ |
| 멀쩡한 길에서 멈춘다 (NO GAP) | `threshold` ↓ 또는 `min_gap_beams` ↓ |
| 좌우로 춤춘다 | `steer_gain` ↓ (7주차 Kp 진동의 재방문) |
| 콘 정면 돌진 | inf 처리 확인 — 0이 아니라 **range_max**로 |

## 실습 2 (도전) — S자 라바콘 통로

### [단계 4] 통로 설치 + 완주 + 속도 올리기

기본 콘을 지우고 통로를 설치합니다 (콘 두 줄 `wall_L`·`wall_R`, 모델 2개 = 콘 94개):

```bash
bash ~/ssc_ws/src/ssc_class/weeks/week-09/scripts/delete_cones.sh
bash ~/ssc_ws/src/ssc_class/weeks/week-09/scripts/spawn_scourse.sh
```

- 통로 중심선 `c(s) = 0.5 + 1.0·sin(2π(s−2)/12)` m (s = 출발점에서의 거리, + = 왼쪽), 폭 2.6 m, 콘 간격 0.6 m, s = 2~30 m
- 직진하면 s ≈ 4 m에서 오른쪽 콘 벽에 닿습니다 — 조향 없이는 통과 불가
- 통로 안에서는 가장 넓은 gap = 통로 앞쪽이므로 차가 통로 중심을 따라갑니다 (F1TENTH 트랙과 같은 상황)
- 왜 한 줄짜리 콘 게이트가 아닌가: FGM은 **거리 > threshold인 빔**을 빈 공간으로 보므로, 비스듬히 3~5 m 떨어진 콘 줄은 '비어 있음'으로 판정되어 차가 콘 줄을 따라 미끄러집니다
- 실측(기본값) 2회: FINISH 38.2 s / 55.2 s, 콘 무접촉 (콘 중심과 최소 1.07 m)

완주가 되면 속도를 올립니다:

```bash
ros2 param set /fgm_drive speed 1.5
```

빨라질수록 같은 거품도 '늦게' 도착합니다 — 반응 거리가 짧아지므로,
속도를 올리면 `threshold`·`bubble_radius`도 함께 키우는 것이 정석입니다.

최종 설정 저장(타임어택 제출물):

```bash
ros2 param dump /fgm_drive
```

### [단계 5] 미니 타임어택 — 규칙

**미션: 출발선 → S자 라바콘 통로 통과 → 결승선 28 m 자동 정지. 기록은 FINISH 로그.**

- 출발: `fgm_drive` 시작 = 계측 시작 (노드가 스스로 잰다)
- 콘 접촉 1회당 **+2초**, 연석 접촉은 실격
- 파라미터는 자유 — 단 `param dump`를 기록과 함께 제출
- 수업 마지막 20분: 기록 공유 — 상위 기록의 파라미터를 함께 열어 봅니다

---

## 과제 (LMS, 다음 수업 전날 23:59)

| # | 내용 | 제출물 |
|---|---|---|
| 1 | **기본 코스 리포트** — 파라미터 2조합 이상 기록 비교 표 + 해석 한 줄 | 표 + 최고 기록의 `param dump` |
| 2 | **S자 통로 완주** — 수업 중 미완이면 과제로 | FINISH 캡처 + 설정 |
| 3 (도전) | **deepest gap** — '가장 넓은' 대신 '가장 깊은'(gap 안 최대 거리 빔 포함) gap 선택으로 변경, 어느 상황에서 유리한지 한 줄 | 수정 .py |
| 4 (도전) | **콘 1개 추가** — 기본 코스에 콘 하나를 더 놓고도 완주하는 설정 찾기 (`spawn_cones.sh`의 줄 복사, 좌표 계산식은 주석) | 수정 .sh + 설정 + 기록 캡처 |

## 자주 하는 실수

| 증상 | 해결 |
|---|---|
| 콘 정면 돌진 | inf를 0으로 바꿈 — **range_max**로 바꿀 것 (뚫린 곳이 막힌 곳이 된다) |
| 출발부터 NO GAP | threshold가 너무 큼 (연석까지 거리보다 크면 전부 '막힘') |
| 콘을 스침 | bubble_radius 부족 — 차 반폭 + 여유 |
| 좌우로 춤 | steer_gain 과대 |
| 느닷없는 급선회 | min_gap_beams 너무 작음 — 좁은 가짜 틈에 반응 |
| param 반영 안 됨 | get_parameter를 콜백/tick마다 읽는지 |

## 실측 자료 ([img/](img/))

| 파일 | 내용 |
|---|---|
| `basic_layout.png` / `basic_traj.png` | 기본 코스 콘 4개 배치도 / 기본 파라미터 실측 궤적 (FINISH 41.6 s, 콘 무접촉) |
| `scourse_layout.png` / `scourse_traj.png` | S자 통로 배치도 / 실측 궤적 (FINISH 55.2 s, 콘 무접촉) |
| `term_gap_finder.png` / `term_finish.png` | `gap_finder` 로그 (콘1 접근 시 gap 분열) / `fgm_drive` FINISH 로그 |
| `basic_run.mp4` / `scourse_run.mp4` | 두 코스 주행 영상 (조감 카메라, 학생 노드 그대로, 기본 파라미터) |

> 통로 모델(`wall_L.sdf`, `wall_R.sdf`)은 [scripts/gen_walls.py](scripts/gen_walls.py)로 생성합니다 (중심선·폭·간격 변경 시 재생성).
>
> ⚠ 콘 설치·삭제 스크립트는 ROS 서비스(`/spawn_entity` 등) 대신 `gz model`을 씁니다. GUI가 켜진 상태에서 ROS 서비스는 응답을 잃어 명령이 멈추는 일이 잦았습니다. 차량 리셋이 `/reset_world`로 멈추면 `gz world -r`을 쓰십시오.
