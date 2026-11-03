# -*- coding: utf-8 -*-
"""S자 라바콘 통로 모델 생성: wall_L.sdf / wall_R.sdf (+ course.py 용 좌표 표).
통로 중심 c(s) = C0 + A*sin(2π (s - S0) / P), 폭 W. 콘 간격 PITCH. 모델 원점 = 출발점, x축 = 진행 방향, y축 = 왼쪽."""
import io, math, os, sys
C0, A, P, W, PITCH = 0.5, 1.0, 12.0, 2.6, 0.6
S0, S1 = 2.0, 30.0
OUT = os.path.dirname(os.path.abspath(__file__))


def center(s):
    if s < S0:
        return C0
    return C0 + A * math.sin(2 * math.pi * (s - S0) / P)


def sdf(name, side):
    links = ''
    pts = []
    k = 0
    s = S0
    while s <= S1 + 1e-6:
        lat = center(s) + side * W / 2
        pts.append((round(s, 2), round(lat, 2)))
        k += 1
        links += ('    <link name="c%d"><pose>%.2f %.2f 0.5 0 0 0</pose>\n'
                  '      <collision name="c"><geometry><cylinder><radius>0.15</radius><length>1.0</length></cylinder></geometry></collision>\n'
                  '      <visual name="v"><geometry><cylinder><radius>0.15</radius><length>1.0</length></cylinder></geometry>\n'
                  '        <material><ambient>1 0.35 0 1</ambient><diffuse>1 0.35 0 1</diffuse></material></visual></link>\n') % (k, s, lat)
        s += PITCH
    body = ('<?xml version="1.0"?>\n'
            '<!-- [9주차] %s: S자 라바콘 통로의 %s 벽. 모델 원점 = 우측 출발점, x = 진행 방향, y = 왼쪽.\n'
            '     중심선 c(s) = %.1f + %.1f*sin(2π(s-%.0f)/%.0f) m, 폭 %.1f m, 콘 간격 %.1f m, s = %.0f~%.0f m (콘 %d개) -->\n'
            '<sdf version="1.6">\n  <model name="%s">\n    <static>true</static>\n%s  </model>\n</sdf>\n'
            % (name, '왼쪽' if side > 0 else '오른쪽', C0, A, S0, P, W, PITCH, S0, S1, k, name, links))
    io.open(os.path.join(OUT, name + '.sdf'), 'w', encoding='utf-8', newline='\n').write(body)
    return pts


L = sdf('wall_L', +1)
R = sdf('wall_R', -1)
io.open(os.path.join(OUT, 'walls.py'), 'w', encoding='utf-8', newline='\n').write(
    '# 자동 생성(gen_walls.py): 통로 콘 좌표 (s, lat)\nC0, A, P, W = %r, %r, %r, %r\nWALL_L = %r\nWALL_R = %r\n' % (C0, A, P, W, L, R))
print('cones per wall', len(L), 'lat range L %.2f..%.2f R %.2f..%.2f' % (
    min(p[1] for p in L), max(p[1] for p in L), min(p[1] for p in R), max(p[1] for p in R)))
