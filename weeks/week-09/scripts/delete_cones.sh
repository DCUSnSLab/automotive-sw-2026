#!/bin/bash
# [9주차] 라바콘·통로 제거 (cone1~8, wall_L, wall_R) — gz model 사용 (없는 모델은 조용히 넘어감)
source ~/ssc_ws/install/setup.bash
for n in cone1 cone2 cone3 cone4 cone5 cone6 cone7 cone8 wall_L wall_R; do
  gz model -m "$n" -d > /dev/null 2>&1
done
echo "라바콘 제거 완료"
