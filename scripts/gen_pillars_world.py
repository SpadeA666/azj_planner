#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""azj_pillars.world 生成器 —— 随机密林场景。

设计约束（都来自实际需求）：
  · **无围墙**：只靠柱子当障碍，避免"边界把路径封死"干扰判断
  · **随机半径/高度**：0.15~0.50m 半径、1.2~3.2m 高，
    模拟真实环境里粗细高矮不一的柱子
  · **随机间隙**：每对相邻柱子的缝宽不同（0.35~1.3m），
    有的能过、有的过不去 —— 这才是真实避障
  · **机头正对柱林**：起飞点 (0,0)、yaw=0 机头朝 +X，
    柱阵沿 +X 方向铺开（x ∈ [2.5, 22]），起飞后正对林子
  · **起飞区净空**：起点 2.5m 半径内不放柱子，保证能起飞

生成算法：**拒绝采样（rejection sampling）**
  随机撒点 → 检查与已放置柱子的距离 >= r1 + r2 + gap → 通过则接受。
  比格子排布自然，又能保证任意两柱之间至少留 gap。

用法：
    python3 gen_pillars_world.py                 # 默认种子，生成到 PX4 worlds
    python3 gen_pillars_world.py --seed 42       # 换一个随机布局
    python3 gen_pillars_world.py --n 200 --out /tmp/x.world
"""

import argparse
import math
import random
import xml.etree.ElementTree as ET

# ---------------- 场景参数 ----------------
X_RANGE = (2.5, 26.0)      # 柱阵 x 范围（+X = 机头方向）
Y_RANGE = (-11.0, 11.0)    # 柱阵 y 范围
CLEAR_START_R = 2.5        # 起飞点净空半径
R_PILLAR = (0.15, 0.50)    # 柱半径范围
H_PILLAR = (1.2, 3.2)      # 柱高范围
# 拒绝采样的最小间隙（净宽，不含半径）。
#
# ★ 这个值是"能否通过"的决定性参数：
#   柱子两两之间的缝 >= 这个值。robot_radius 0.30 → 需要 >= 0.60 才过得去。
#   设 0.35 会生成"到处都是 0.35m 死缝"的林子 —— 实测 300 根时
#   最近邻缝宽中位只有 0.39m，全图基本不可通行，测试不出东西。
#
#   取 0.72：留 0.12m 余量。这样**任何两根柱子之间都过得去**，
#   但柱子足够密，路径必须不断左右穿插 —— 这才是有效的密集避障测试。
# ★ 0.72 太密（258 根，Gazebo 卡）。改成 1.20m：
#   柱子少一半以上，缝更宽，物理引擎负担明显降低。
#   robot_radius 0.30 → 直径 0.60，1.20m 缝仍有 0.60m 余量，够用且不挑战极限。
GAP_BETWEEN = (1.20, 1.20)

# 与机身的对比：robot_radius 0.30 → 直径 0.60
#   gap < 0.60 的缝过不去（硬约束会拒绝）
#   gap 0.60~0.80 勉强能过，必须居中
#   gap > 0.90 宽松


def gen(seed, target_n, max_tries=400000):
    rnd = random.Random(seed)
    placed = []          # [(x, y, r, h)]
    tries = 0
    while len(placed) < target_n and tries < max_tries:
        tries += 1
        x = rnd.uniform(*X_RANGE)
        y = rnd.uniform(*Y_RANGE)
        # 起飞区净空
        if math.hypot(x, y) < CLEAR_START_R:
            continue
        r = rnd.uniform(*R_PILLAR)
        h = rnd.uniform(*H_PILLAR)
        # 拒绝采样：与所有已有柱子的间距必须 >= r1+r2+gap
        ok = True
        for (px, py, pr, _ph) in placed:
            need = pr + r + rnd.uniform(*GAP_BETWEEN)
            # 注意：每次比较用新随机 gap 会让结果不稳定，
            # 改成用固定下限再单独抽一次。这里用最小 gap 保证收敛。
            if math.hypot(x - px, y - py) < pr + r + GAP_BETWEEN[0]:
                ok = False
                break
        if ok:
            placed.append((x, y, r, h))
    return placed, tries


def build_world(pillars, out_path):
    world = ET.Element('sdf', {'version': '1.6'})
    w = ET.SubElement(world, 'world', {'name': 'azj_pillars'})

    sun = ET.SubElement(w, 'light', {'name': 'sun', 'type': 'directional'})
    ET.SubElement(sun, 'cast_shadows').text = '1'
    ET.SubElement(sun, 'pose').text = '5 0 20 0 0 0'
    ET.SubElement(sun, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(sun, 'specular').text = '0.1 0.1 0.1 1'
    ET.SubElement(sun, 'direction').text = '-0.3 0.1 -1'

    scene = ET.SubElement(w, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.55 0.55 0.55 1'
    ET.SubElement(scene, 'background').text = '0.75 0.82 0.92 1'
    ET.SubElement(scene, 'shadows').text = '1'

    ph = ET.SubElement(w, 'physics', {'name': 'default_physics', 'default': '0', 'type': 'ode'})
    ET.SubElement(ph, 'max_step_size').text = '0.004'
    ET.SubElement(ph, 'real_time_factor').text = '1.0'
    ET.SubElement(ph, 'real_time_update_rate').text = '250'

    # 地面（无围墙）
    g = ET.SubElement(w, 'model', {'name': 'ground'})
    ET.SubElement(g, 'static').text = 'true'
    gl = ET.SubElement(g, 'link', {'name': 'link'})
    gc = ET.SubElement(gl, 'collision', {'name': 'collision'})
    ET.SubElement(ET.SubElement(gc, 'geometry'), 'plane').extend([
        ET.Element('normal', {}), ET.Element('size', {})])
    gc.find('geometry/plane/normal').text = '0 0 1'
    gc.find('geometry/plane/size').text = '80 80'

    # 柱子
    for i, (x, y, r, h) in enumerate(pillars):
        m = ET.SubElement(w, 'model', {'name': 'pillar_%03d' % i})
        ET.SubElement(m, 'static').text = 'true'
        ET.SubElement(m, 'pose').text = '%.3f %.3f %.3f 0 0 0' % (x, y, h / 2.0)
        ln = ET.SubElement(m, 'link', {'name': 'link'})
        col = ET.SubElement(ln, 'collision', {'name': 'collision'})
        cg = ET.SubElement(col, 'geometry')
        cyl = ET.SubElement(cg, 'cylinder')
        ET.SubElement(cyl, 'radius').text = '%.3f' % r
        ET.SubElement(cyl, 'length').text = '%.3f' % h
        vis = ET.SubElement(ln, 'visual', {'name': 'visual'})
        vg = ET.SubElement(vis, 'geometry')
        vcyl = ET.SubElement(vg, 'cylinder')
        ET.SubElement(vcyl, 'radius').text = '%.3f' % r
        ET.SubElement(vcyl, 'length').text = '%.3f' % h
        # 按高度上色：高的偏红、矮的偏蓝，便于肉眼判断
        t = (h - H_PILLAR[0]) / max(1e-6, H_PILLAR[1] - H_PILLAR[0])
        mat = ET.SubElement(vis, 'material')
        ET.SubElement(mat, 'ambient').text = '%.2f %.2f %.2f 1' % (0.25 + 0.5 * t, 0.45, 0.85 - 0.5 * t)
        ET.SubElement(mat, 'diffuse').text = '%.2f %.2f %.2f 1' % (0.35 + 0.5 * t, 0.55, 0.95 - 0.5 * t)

    # Python 3.8 没有 ET.indent（3.9+ 才有），手动缩进
    def _indent(elem, level=0, space='  '):
        pad = "\n" + space * level
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = pad + space
            for child in elem:
                _indent(child, level + 1, space)
            if not child.tail or not child.tail.strip():
                child.tail = pad
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad
    _indent(world)
    tree = ET.ElementTree(world)
    tree.write(out_path, encoding='utf-8', xml_declaration=True)

    # 统计：用**最近邻缝宽**（每个柱子到最近邻的净距），这才反映稠密度。
    # 之前统计"所有柱对"会把相距 20m 的也算进去，中位数毫无意义。
    nn = []
    for i, a in enumerate(pillars):
        best = 1e9
        for j, b in enumerate(pillars):
            if i == j:
                continue
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            g = d - a[2] - b[2]
            if g < best:
                best = g
        if best < 1e8:
            nn.append(best)
    nn.sort()
    gaps = nn
    n_pass = sum(1 for g in gaps if g >= 0.60)     # robot_radius 0.30 → 直径 0.60
    n_tight = sum(1 for g in gaps if 0.45 <= g < 0.60)
    return {
        "n": len(pillars),
        "x": (min(p[0] for p in pillars), max(p[0] for p in pillars)),
        "y": (min(p[1] for p in pillars), max(p[1] for p in pillars)),
        "r": (min(p[2] for p in pillars), max(p[2] for p in pillars)),
        "h": (min(p[3] for p in pillars), max(p[3] for p in pillars)),
        "gap_min": gaps[0] if gaps else 0,
        "gap_med": gaps[len(gaps) // 2] if gaps else 0,
        "gap_max": gaps[-1] if gaps else 0,
        "n_pass": n_pass, "n_tight": n_tight,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', '-s', type=int, default=7)
    ap.add_argument('--n', type=int, default=90)
    ap.add_argument('--out', default='/home/spadea/PX4_Firmware/Tools/sitl_gazebo/worlds/azj_pillars.world')
    a = ap.parse_args()

    pillars, tries = gen(a.seed, a.n)
    st = build_world(pillars, a.out)

    print("生成 %s   (seed=%d, 尝试 %d 次)" % (a.out, a.seed, tries))
    print("  柱子数      : %d" % st["n"])
    print("  x 范围      : %.2f ~ %.2f   (机头 +X 方向)" % st["x"])
    print("  y 范围      : %.2f ~ %.2f" % st["y"])
    print("  半径        : %.2f ~ %.2f m" % st["r"])
    print("  高度        : %.2f ~ %.2f m" % st["h"])
    print("  最近邻缝宽  : min %.2f / 中位 %.2f / max %.2f m"
          % (st["gap_min"], st["gap_med"], st["gap_max"]))
    print("  柱子的最近邻缝 >=0.60m (可过)   : %d 根" % st["n_pass"])
    print("  柱子的最近邻缝 0.45~0.60m(勉强) : %d 根" % st["n_tight"])
    print("  起飞点 (0,0) 净空半径 %.1fm，机头 yaw=0 朝 +X 正对柱林" % CLEAR_START_R)


if __name__ == '__main__':
    main()
