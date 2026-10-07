# 오토모티브SW프로그래밍 — 실습 리포지토리

대구가톨릭대학교 오토모티브SW프로그래밍 실습 코드입니다.
ROS 2 Humble + Gazebo Classic 11, 운전면허 시험장 시뮬레이션 환경을 사용합니다.

## 시작하기 (최초 1회)

```bash
mkdir -p ~/ssc_ws/src && cd ~/ssc_ws/src
git clone https://github.com/DCUSnSLab/automotive-sw-2026.git ssc_class
```

## 매주 수업 시작 시 — 이번 주 브랜치로 전환

```bash
cd ~/ssc_ws/src/ssc_class
git pull
git remote update
git checkout week-NN        # NN = 이번 주차 번호 (예: 3주차 → week-03)
```

**이후 절차는 checkout한 주차의 가이드를 따라가세요: `weeks/week-NN/README.md`**
(예: week-03 브랜치라면 `weeks/week-03/README.md`)

현재 공개된 브랜치 확인:

```bash
git branch -r
```

## 주차별 브랜치

| 브랜치 | 내용 | 공개 시점 |
|---|---|---|
| `week-02` | 개발환경 구축 & 첫 주행 | 2주차 |
| `week-03` | ROS 2 기초 1 — 노드와 토픽 | 3주차 |
| `week-06` | 인지 1 — LiDAR와 긴급제동(AEB) | 해당 수업 주 |
| `week-NN-solution` | NN주차 과제 정답 | NN+1주차 |

- 브랜치는 **해당 주차가 되어야 공개**됩니다 (목록에 없으면 아직 공개 전)
- 브랜치는 누적 방식입니다 — 최신 주차 브랜치에 이전 주차 자료가 모두 포함되어 있습니다

## 문의

- 실습 중 문제: 수업 시간 조교 / LMS Q&A
- 환경 구축 실패: 오류 화면 캡처와 함께 LMS로
