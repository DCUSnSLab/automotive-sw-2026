#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정적 모델 병합 — Gazebo Classic 메모리 절감용 월드 변환기.

Gazebo Classic 11 은 월드의 **모델 1개당 약 1.7 MB, 링크 1개당 약 0.65 MB** 의 고정 메모리를
gzserver 에서 소비한다(2026-10-13 실측, 비주얼 1개는 약 0.08 MB). 운전면허 시험장 월드는
차선·라바콘·표지 등 215개 모델로 이뤄져 있어 이 오버헤드만 ~360 MB 에 달한다.

이 스크립트는 `<static>true</static>` 인 모델(및 옵션으로 라바콘처럼 움직일 필요 없는 모델)을
**하나의 정적 모델 / 하나의 링크** 로 합친다. 각 visual·collision 의 pose 는
model pose ∘ link pose ∘ visual pose 로 합성해 월드 좌표상 위치는 그대로다.
물리(정적 충돌)·라이다 반사·카메라 영상은 변하지 않고, 바뀌는 것은 GUI 모델 트리에서
개별 차선/콘을 따로 선택·이동할 수 없다는 점만이다.

사용:
    python3 merge_static_models.py <in.world> <out.world> [--merge-name NAME]
                                   [--also-merge REGEX] [--keep REGEX] [--drop REGEX ...]

    --drop        실습에 불필요한 모델을 아예 제거(rip 버전). <include> 는 <name> 으로 매칭
    --also-merge  static=false 이지만 병합할 모델 이름 정규식 (기본: course_fixed_cone.*)
    --keep        병합하지 않고 남길 모델 이름 정규식 (여러 번 지정 가능)
<include> 요소와 플러그인·조인트·센서가 있는 모델은 항상 그대로 둔다.
"""
import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET


# ---------- pose 합성 (Gazebo: 회전 = Rz(yaw)·Ry(pitch)·Rx(roll)) ----------
def parse_pose(text):
    if text is None or not text.strip():
        return [0.0] * 6
    v = [float(x) for x in text.split()]
    if len(v) != 6:
        raise ValueError(f'pose must have 6 numbers: {text!r}')
    return v


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_to_rpy(m):
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    if abs(abs(sp) - 1.0) < 1e-9:  # gimbal lock(pitch = ±90°): roll 을 0 으로 두고 yaw 에 몰아준다
        # p = ±π/2 이면 R[0][1] = -sin(r∓y), R[1][1] = cos(r∓y) 이므로 r = 0 으로 두면 두 경우 식이 같다
        r = 0.0
        y = math.atan2(-m[0][1], m[1][1])
    else:
        r = math.atan2(m[2][1], m[2][2])
        y = math.atan2(m[1][0], m[0][0])
    return r, p, y


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def compose(parent, child):
    """parent ∘ child (child 는 parent 프레임 기준) → 월드 기준 pose."""
    pr = rpy_to_mat(*parent[3:])
    cr = rpy_to_mat(*child[3:])
    t = [parent[i] + x for i, x in enumerate(matvec(pr, child[:3]))]
    r, p, y = mat_to_rpy(matmul(pr, cr))
    return t + [r, p, y]


def fmt_pose(v):
    def f(x):
        s = f'{x:.6f}'
        return '0.000000' if s in ('-0.000000',) else s
    return ' '.join(f(x) for x in v)


# ---------- 병합 ----------
def is_mergeable(model, also_merge, keep):
    name = model.get('name', '')
    if any(k.search(name) for k in keep):
        return False
    if model.find('joint') is not None or model.find('plugin') is not None:
        return False
    if any(True for _ in model.iter('sensor')):
        return False
    if model.find('model') is not None or model.find('include') is not None:
        return False
    static = (model.findtext('static') or 'false').strip().lower() in ('1', 'true')
    if static:
        return True
    return also_merge is not None and also_merge.search(name) is not None


def drop_models(world, drop):
    """이름이 drop 정규식 중 하나에 맞는 <model>/<include> 를 월드에서 제거한다."""
    removed = []
    for el in list(world):
        if el.tag == 'model':
            name = el.get('name', '')
        elif el.tag == 'include':
            name = (el.findtext('name') or el.findtext('uri') or '').strip()
        else:
            continue
        if any(d.search(name) for d in drop):
            world.remove(el)
            removed.append(name)
    return removed


def merge(world, merge_name, also_merge, keep):
    merged = ET.Element('model', {'name': merge_name})
    ET.SubElement(merged, 'static').text = 'true'
    link = ET.SubElement(merged, 'link', {'name': 'link'})
    n_models = n_vis = n_col = 0
    first_index = None
    children = list(world)
    for idx, model in enumerate(children):
        if model.tag != 'model' or not is_mergeable(model, also_merge, keep):
            continue
        if first_index is None:
            first_index = idx
        mpose = parse_pose(model.findtext('pose'))
        for lnk in model.findall('link'):
            lpose = compose(mpose, parse_pose(lnk.findtext('pose')))
            for el in list(lnk):
                if el.tag not in ('visual', 'collision'):
                    continue  # inertial/gravity/self_collide 등은 정적 링크에 의미 없음
                new = ET.Element(el.tag, {'name': f"{model.get('name')}__{el.get('name')}"})
                pose_el = el.find('pose')
                world_pose = compose(lpose, parse_pose(pose_el.text if pose_el is not None else None))
                p = ET.SubElement(new, 'pose')
                p.text = fmt_pose(world_pose)
                for child in el:
                    if child.tag == 'pose':
                        continue
                    new.append(child)
                link.append(new)
                if el.tag == 'visual':
                    n_vis += 1
                else:
                    n_col += 1
        world.remove(model)
        n_models += 1
    if n_models == 0:
        return 0, 0, 0
    # 원래 첫 정적 모델 자리에 끼워 넣어 파일 순서를 유지한다
    insert_at = min(first_index, len(list(world)))
    world.insert(insert_at, merged)
    return n_models, n_vis, n_col


def indent(elem, level=0):
    pad = '\n' + '  ' * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + '  '
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = pad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--merge-name', default='course_static')
    ap.add_argument('--also-merge', default=r'^course_fixed_cone',
                    help="static=false 라도 병합할 모델 이름 정규식 ('' 이면 없음)")
    ap.add_argument('--keep', action='append', default=[], help='병합 제외 모델 이름 정규식')
    ap.add_argument('--drop', action='append', default=[],
                    help='월드에서 아예 제거할 모델/<include> 이름 정규식 (rip 버전용, 여러 번 지정 가능)')
    a = ap.parse_args()

    tree = ET.parse(a.src)
    world = tree.getroot().find('world')
    if world is None:
        sys.exit('no <world> element')
    removed = drop_models(world, [re.compile(d) for d in a.drop])
    if removed:
        print(f'dropped {len(removed)} models: {", ".join(removed)}')
    also = re.compile(a.also_merge) if a.also_merge else None
    keep = [re.compile(k) for k in a.keep]
    n_models, n_vis, n_col = merge(world, a.merge_name, also, keep)
    remaining = len(world.findall('model'))
    indent(tree.getroot())
    tree.write(a.dst, encoding='unicode', xml_declaration=True)
    with open(a.dst, 'a') as f:
        f.write('\n')
    print(f'merged {n_models} models ({n_vis} visuals, {n_col} collisions) into <model name="{a.merge_name}">; '
          f'{remaining} models remain in world')


if __name__ == '__main__':
    main()
