#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""穿框测试：证明 2D 单层规划能穿过框孔，柱状投影不能。

这是用户指出的核心问题：
  框的孔在规划层（z=0.8）是空的，但柱状投影把框的上下沿压到同一平面，
  孔就变成实心的 —— "本来有空间却没有空间了"。

场景：
  一堵墙 x=0，墙上开一个方孔：
    孔中心 (0, 0, 0.8)，孔高 z∈[0.55, 1.05]，孔宽 y∈[-0.6, 0.6]
  飞机在 z=0.8 飞行 —— 正好从孔中心穿过。

期望：
  单层 ESDF（plan_2d_slab_half=0）→ 能穿过，路径经过 x=0 附近 y≈0
  柱状投影（slab_half=0.5）        → 穿不过（孔被压扁）

用法：python3 box_gap_test.py [single|slab]
"""
import math
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

MODE = sys.argv[1] if len(sys.argv) > 1 else "single"
PLANE_Z = 0.80

# 孔
GAP_Y = 0.60          # 半宽
GAP_Z_LO, GAP_Z_HI = 0.55, 1.05


def build_wall():
    """一堵墙 + 一个方孔。点云必须密集，否则 rog_map 建不出孔沿。"""
    pts = []
    res = 0.04
    # 墙：x∈[-0.05,0.05]，y∈[-2,2]，z∈[0,2]
    for y in np.arange(-2.0, 2.0, res):
        for z in np.arange(0.0, 2.0, res):
            in_gap = (abs(y) < GAP_Y - res) and (GAP_Z_LO + res < z < GAP_Z_HI - res)
            if in_gap:
                continue
            pts.append((-0.05, y, z))
            pts.append((0.05, y, z))
    # 地板
    for x in np.arange(-2, 2, res * 2):
        for y in np.arange(-2, 2, res * 2):
            pts.append((x, y, 0.0))
    return pts


def main():
    rospy.init_node("boxgap", anonymous=True)
    op = rospy.Publisher("/gazebo_truth/odom", Odometry, queue_size=10)
    cp = rospy.Publisher("/super_cloud", PointCloud2, queue_size=5)
    gp = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    last = [None]
    rospy.Subscriber("/azj/path", Path, lambda m: last.__setitem__(0, m), queue_size=10)

    pts = build_wall()
    print("模式=%s  点云 %d 点（墙 + 方孔 y±%.2f z[%.2f,%.2f]）"
          % (MODE, len(pts), GAP_Y, GAP_Z_LO, GAP_Z_HI))

    start = (-1.2, 0.0, PLANE_Z)
    goal = (1.2, 0.0, PLANE_Z)
    r = rospy.Rate(10)

    def feed(n):
        for _ in range(n):
            now = rospy.Time.now()
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = "map"
            o.pose.pose.position.x = start[0]
            o.pose.pose.position.y = start[1]
            o.pose.pose.position.z = start[2]
            o.pose.pose.orientation.w = 1.0
            op.publish(o)
            cp.publish(pc2.create_cloud_xyz32(
                rospy.Header(frame_id="map", stamp=now), pts))
            r.sleep()

    print("建图 8s ...")
    feed(80)

    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "map"
    g.pose.position.x, g.pose.position.y, g.pose.position.z = goal
    g.pose.orientation.w = 1.0
    gp.publish(g)

    import time
    t0 = time.time()
    while time.time() - t0 < 15 and last[0] is None:
        feed(5)

    if last[0] is None:
        print("RESULT=NO_PATH（框孔被堵死，穿不过去）")
        return 1

    p = [(q.pose.position.x, q.pose.position.y, q.pose.position.z)
         for q in last[0].poses]
    L = sum(math.dist(p[i], p[i - 1]) for i in range(1, len(p)))
    straight = math.dist(start, goal)
    crossed = any(abs(v[0]) < 0.10 for v in p)
    end = p[-1]
    d = math.dist(end, goal)
    # 穿框时路径在 x≈0 处的 y 值
    y_at_wall = [v[1] for v in p if abs(v[0]) < 0.15]

    print("路径 %d 点  L=%.2fm  直线=%.2fm  绕行比=%.2f" % (len(p), L, straight, L / straight))
    print("  穿过 x=0: %s   终点距目标 %.3f m" % ("是" if crossed else "否", d))
    if y_at_wall:
        print("  穿框处 y = %.2f ~ %.2f（孔半宽 %.2f）"
              % (min(y_at_wall), max(y_at_wall), GAP_Y))

    ok = crossed and d < 0.4 and L < straight * 2.5
    print("RESULT=" + ("PASS（穿过框孔）" if ok else "FAIL（没穿过去）"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
