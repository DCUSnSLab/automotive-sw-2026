# ssc_class_gazebo — 운전면허 시험장 월드 + 실습 차량

ROS 2 Humble + Gazebo Classic 11. 실행 방법과 토픽은 `launch/license_course.launch.py` 머리말 참고.

## 저메모리 구성 (2026-10-13)

수업용 노트북(RAM 16 GB, iGPU 가 시스템 메모리를 공유)에서 Gazebo 가 메모리를 과도하게 쓰는 문제를
줄이기 위해 다음을 적용했다. **차량 동작·충돌·라이다·카메라 시야는 원본과 같다.**

| 항목 | 원본 | 변경 | 효과 |
|---|---|---|---|
| 월드 모델 수 | 정적 모델 215개 (차선·콘·표지…) | 모델 1개(`course_static`)·링크 1개로 병합 | gzserver −330 MB |
| 차량 메시 | COLLADA 75k 삼각형 (4.6 MB) | 바이너리 STL 15k 삼각형 (0.8 MB) | gzserver −60 MB, gzclient 감소 |
| 코스 메시 | ASCII STL 121개 (7.9 MB) | 바이너리 STL (1.7 MB) | 로드 시간·디스크 |
| 텍스처 | 1024² / 1024×512 | 절반 해상도 | GPU(=시스템) 메모리 |
| 불필요 모델 | 쉘터·공사차량·철길·표지판·노면문자·녹지·마름모·빗금 (39개) | **삭제 (rip)** | gzserver −60 MB, 파일 56개 삭제 |
| 카메라 | 1280×720 @ 30 Hz | **800×450 @ 30 Hz** (launch 인자로 복원 가능; 640×360·640×480·960×540 은 llvmpipe 에서 카메라가 0.3~0.6 Hz 로 급락하므로 피할 것) | 영상 버퍼·렌더 타깃 |

### 왜 모델 병합이 핵심인가

Gazebo Classic 은 월드의 **모델 1개마다 약 1.7 MB, 링크 1개마다 약 0.65 MB** 를 gzserver 에서
고정으로 소비한다(비주얼 1개는 약 0.08 MB; 2026-10-13 실측). 시험장 월드는 차선 하나하나가 별도 모델이라
모델 수만 215개 → 이 오버헤드만 ~360 MB 였다. 정적 물체는 움직이지 않으므로 하나의 정적 모델에 모든
visual/collision 을 월드 좌표로 합쳐 넣어도 물리·센서 결과가 같다.

바뀌는 점은 **GUI 모델 트리에서 차선·콘을 개별 선택/이동할 수 없다**는 것뿐이다. 월드를 편집하려면
원본(`worlds/classroom_authoring.world`)을 고친 뒤 다시 생성한다:

```bash
python3 tools/merge_static_models.py worlds/classroom_authoring.world worlds/classroom.world
```

라바콘(`course_fixed_cone*`)은 원본에서 `static=false` 였지만 이름대로 고정물이라 함께 병합했다.
움직이는 콘이 필요하면 `--also-merge ''` 로 제외하거나 `--keep '<이름 정규식>'` 으로 빼면 된다.
`<include>`(공사 차량 `lcs_5004`)·플러그인·조인트·센서가 있는 모델은 자동으로 병합 대상에서 제외된다.

### rip 버전 — 무엇을 남기고 무엇을 지웠나

자율주행 실습(차선 추종·장애물 회피·정지선/신호 인식·주차)에 필요한 것만 남겼다.

| 남김 | 이유 |
|---|---|
| 바닥·도로면(아스팔트 텍스처) | 주행면, 카메라 차선 인식 배경 |
| 황색 차선 45개, 정지선 10, 횡단보도 6, 노면 화살표 21, 주차구역 | 차선·정지선 인식, 주차 실습 |
| 바깥 연석 + 연석 링 9조(상·측면) | 코스 경계 — 라이다 반사면이자 물리 충돌체 |
| 라바콘 68개 | 라이다 장애물 |
| 신호등 4개 | 신호 인식 |

| 지움 | 비고 |
|---|---|
| 대기 쉘터(지붕·기둥·유리·벤치) | 출발선 옆 장식 |
| 공사차량 `lcs_5004`(VMS 포함) | 장식, 모델 폴더째 삭제 |
| 철길 판·홈·레일, 속도 표지판(20), 노면 문자(출발 등), 녹지 섬 9개, 마름모 6, 빗금 6 | 장식·면허시험용 요소 |

생성 명령(위 명령에 `--drop` 을 붙인 것):

```bash
python3 tools/merge_static_models.py worlds/classroom_authoring.world worlds/classroom.world \
  --drop '^course_waiting_shelter' --drop '^lane_control_vehicle$' --drop '^course_railroad' \
  --drop '^sign_20' --drop '^text_marking' --drop '^green_island' --drop '^mark_diamond' --drop '^hatch_yellow_lines'
```

지운 모델의 메시·텍스처 파일도 함께 삭제했으므로, 되살리려면 `git show main:ssc_class_gazebo/models/<경로>` 로
파일을 복원한 뒤 해당 `--drop` 을 빼고 다시 생성한다.

### 센서 수치 되돌리기

```bash
ros2 launch ssc_class_gazebo license_course.launch.py camera_width:=1280 camera_height:=720
ros2 launch ssc_class_gazebo license_course.launch.py lidar_samples:=360   # 라이다 1° 간격으로 (−30 MB)
```

### 측정 (521-0, i7-13700, Gazebo 11.10.2, 월드 로드 + 차량 스폰 후 정지 상태)

소프트웨어 렌더링(llvmpipe, GUI 포함) — iGPU 노트북과 가장 비슷한 조건:

| | gzserver RSS | gzclient RSS | 합계 PSS |
|---|---|---|---|
| 원본 | 1,185 MB | 594 MB | 1,559 MB |
| 저메모리(병합·메시·카메라) | 753 MB | 526 MB | 1,050 MB |
| + rip (장식 모델 삭제) | **678 MB** | **487 MB** | **932 MB** |

GPU(RTX 2080 Ti, GUI 없음) gzserver: 1,199 MB → 711 MB → rip 641 MB. 라이다 20 Hz·RTF 1.0 은 원본과 같고,
카메라 발행률은 오히려 올랐다(GPU 17 → 29 Hz, llvmpipe 5.7 → 10.4 Hz). 출발 위치에서 캡처한 카메라 영상은 원본과 평균 0.85/255 차이,
라이다 스캔은 최대 0.6 mm 차이(720점 모두 일치).

### 디스크

패키지 `models/` 17 MB → 9.5 MB → rip **4.3 MB** (파일 56개 삭제, `lcs_5004` 폴더 제거).
