# 오토모티브SW프로그래밍 — 실습 리포지토리

대구가톨릭대학교 오토모티브SW프로그래밍 실습 코드입니다.
ROS 2 Humble + Gazebo Classic 11, 운전면허 시험장 시뮬레이션 환경을 사용합니다.

## 시작하기

```bash
mkdir -p ~/ssc_ws/src && cd ~/ssc_ws/src
git clone https://github.com/DCUSnSLab/automotive-sw-2026.git ssc_class
cd ssc_class && git checkout week-02
```

이후 절차는 [weeks/week-02/README.md](weeks/week-02/README.md) 를 따라가세요.

## 주차별 브랜치

| 브랜치 | 내용 | 공개 시점 |
|---|---|---|
| `week-02` | 개발환경 구축 & 첫 주행 | 2주차 |
| `week-03` | ROS 2 기초 1 — 노드와 토픽 | 3주차 |
| `week-N-solution` | N주차 과제 정답 | N+1주차 |

브랜치는 **해당 주차가 되어야 공개**됩니다. 매주 시작 시:

```bash
cd ~/ssc_ws/src/ssc_class
git pull
git checkout week-N
```

## 문의

- 실습 중 문제: 수업 시간 조교 / LMS Q&A
- 환경 구축 실패: 오류 화면 캡처와 함께 LMS로
