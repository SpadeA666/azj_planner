#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证"优先走大空间"：一侧空旷、一侧点云密集，路径该走空旷侧。

复现用户报的问题：
  "路径规划不走空闲区大的地方，要从点云密集处规划"

根因：UNKNOWN 格子的 clearance 被 unknown_clearance_cap 一刀切成 0.32，
      而点云密集处（已知区）保留真实 ESDF（可能 0.5~2.0）
      → A* 认为墙边比空旷区更宽敞 → 专门往墙边走。

场景：
  · 起点 (0,0)、目标 (0,6)，直线南向
  · 西侧 x∈[-0.5,-0.2] 有一面点云墙（**点云密集**）
  · 东侧 x∈[0.2,3.0] 完全空（**无点云 → UNKNOWN**）
  · 期望：路径往东偏（走空旷区），而不是贴着西侧点云墙

若修复前，A* 会贴着西墙走（clearance 真实 0.5+）；
修复后，东侧 UNKNOWN 的真实 ESDF 也很大（离墙远），路径应当东偏或居中。
"""
import math
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

PLANE_Z = 1.20            # 飞高一点，让地面(z=0)不限制最大 clearance
START = (0.0, 0.0, PLANE_Z)
GOAL = (0.0, 6.0, PLANE_Z)


def build():
    """只有西侧一面点云墙 + 地面。东侧完全空。"""
    P = []
    res = 0.05
    # 西墙：x∈[-0.6,-0.35]，y∈[-1,8]，z∈[0,3]
    # 起点 (0,0) 到墙面 x=-0.35 只有 0.35m（刚好过 robot_radius 0.3）；
    # 往东走到 x=1.5 就是 1.85m，差异明显。
    for x in np.arange(-0.6, -0.35, res):
        for y in np.arange(-1.0, 8.0, res):
            for z in np.arange(0.0, 3.0, res):
                on_surf = (abs(x + 0.6) < res or abs(x + 0.35) < res or
                           abs(y + 1.0) < res or abs(y - 8.0) < res or
                           abs(z) < res or abs(z - 3.0) < res)
                if on_surf:
                    P.append((x, y, z))
    # 地面（覆盖整个区域，否则下方是 UNKNOWN）
    for x in np.arange(-1.0, 4.0, res * 2):
        for y in np.arange(-1.0, 8.0, res * 2):
            P.append((x, y, 0.0))
    return np.array(P, dtype=np.float32)


def main():
    rospy.init_node("space_test", anonymous=True)
    op = rospy.Publisher("/gazebo_truth/odom", Odometry, queue_size=10)
    cp = rospy.Publisher("/super_cloud", PointCloud2, queue_size=5)
    gp = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    last = [None]
    rospy.Subscriber("/azj/path", Path, lambda m: last.__setitem__(0, m), queue_size=10)

    pts = build()
    print("点云 %d 点（只有西侧 x<-0.2 有墙，东侧 x>0.2 全空）" % len(pts))

    r = rospy.Rate(10)

    def feed(n):
        for _ in range(n):
            now = rospy.Time.now()
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = "map"
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = START
            o.pose.pose.orientation.w = 1.0
            op.publish(o)
            cp.publish(pc2.create_cloud_xyz32(
                rospy.Header(frame_id="map", stamp=now), pts))
            r.sleep()

    print("建图 10s ...")
    feed(100)

    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "map"
    g.pose.position.x, g.pose.position.y, g.pose.position.z = GOAL
    g.pose.orientation.w = 1.0
    gp.publish(g)

    t0 = time.time()
    while time.time() - t0 < 12 and last[0] is None:
        feed(5)

    if last[0] is None:
        print("RESULT=NO_PATH")
        return 1

    P = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                  for q in last[0].poses])
    xmean = float(P[:, 0].mean())
    xmin, xmax = float(P[:, 0].min()), float(P[:, 0].max())
    L = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))

    print("\n路径 %d 点  L=%.2fm" % (len(P), L))
    print("  x 范围 [%.2f, %.2f]  均值 %.2f" % (xmin, xmax, xmean))
    print("  西墙在 x=-0.2~-0.5；东侧空旷")
    print("  判据：x 均值 >= 0.6 表示明显走东侧空旷区（正确）")
    print("        x 均值 <  0.3 表示贴西墙点云（错误，就是用户报的现象）")

    if xmean >= 0.6:
        print("\nRESULT=PASS  路径偏向东侧空旷区")
        return 0
    print("\nRESULT=FAIL  路径贴着点云密集处（x 均值 %.2f）" % xmean)
    return 1


if __name__ == "__main__":
    sys.exit(main())
