#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV 맛보기 — lane_viewer 에서 s 키로 저장한 frame_00.png 로 실행.

    python3 cv_basics.py
결과: 터미널 출력 + out.png(빨간 원) + mask.png(노란색 마스크)
"""
import time
import cv2

img = cv2.imread('frame_00.png')            # 파일 → 배열 (B, G, R 순서)
H = img.shape[0]                            # 세로(행 수)
W = img.shape[1]                            # 가로(열 수)
print('크기 W x H =', W, 'x', H, '  채널', img.shape[2], '  자료형', img.dtype)

# 1) 픽셀 하나 읽기 — img[y][x], 행(y) 먼저
b, g, r = img[106][99]
print('픽셀 (x=99, y=106)  B G R =', b, g, r)

# 2) 노란 픽셀 세기 — for 와 if 로 직접
t0 = time.time()
count = 0
for y in range(H):
    for x in range(W):
        b, g, r = img[y][x]
        if r > 200 and g > 200 and b < 150:
            count += 1
t1 = time.time()
print('for 루프   : 노란 픽셀 %5d 개, %.1f ms' % (count, (t1 - t0) * 1000))

# 3) 같은 일을 OpenCV 함수로
t0 = time.time()
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, (20, 80, 120), (35, 255, 255))
count = cv2.countNonZero(mask)
t1 = time.time()
print('OpenCV 함수: 노란 픽셀 %5d 개, %.1f ms' % (count, (t1 - t0) * 1000))

# 4) 그리기와 저장 — 그리기 함수의 좌표는 (x, y) 순서
cv2.circle(img, (99, 106), 6, (0, 0, 255), 2)     # 중심 (x, y), 반지름, 색 (B, G, R), 두께
cv2.imwrite('out.png', img)
cv2.imwrite('mask.png', mask)
print('저장: out.png, mask.png')
