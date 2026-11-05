#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证层高硬约束：路径不得钻到地板下 / 天花板上。

复现用户报的问题：
  打点回起飞点附近时，路径会从地板下面（有时天花板上面）绕过去。
  根因：地板下方没有点云 → clearance 走 unknown 策略被判为可通行
        → A* 抄近路从下面钻。但物理上下面是地板。

场景（合成点云）：
  · 地板：z=0 平面，x,y ∈ [-3,3]
  · 隔墙：x∈[-0.1,0.1]，y∈[-2,2]，z∈[0,2.5]  ← 挡住直线
  · 起点 (-1.5, 0, 0.8)，目标 (1.5, 0, 0.8)
  正确行为：只能从墙的 y 端绕（不能从地板下、也不能从墙顶上方，因为 ceil_z=2.0）

检查：
  1) 规划成功
  2) 路径所有点 z >= floor_z（不钻地板）
  3) 路径所有点 z <= ceil_z（不越天花板）
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

FLOOR_Z = -0.10
CEIL_Z = 2.00


def build_room():
    """地板 + 隔墙。点云要足够密，否则 rog_map 建不出来。"""
    pts = []
    res = 0.08
    # 地板
    for x in np.arange(-3, 3, res):
        for y in np.arange(-3, 3, res):
            pts.append((x, y, 0.0))
    # 隔墙：x≈0，y∈[-2,2]，z∈[0,2.5]
    # 墙必须**连到地面**（z 从 0 开始）。之前从 0.1 开始，
    # 墙底留了 0.1m 的缝，A* 就从那条缝钻过去了 —— 那是场景缺陷不是代码问题。
    for y in np.arange(-2, 2, res):
        for z in np.arange(0.0, 2.5, res):
            pts.append((0.0, y, z))
            pts.append((0.05, y, z))
    return pts


def main():
    rospy.init_node("limit_test", anonymous=True)
    odom_pub = rospy.Publisher("/gazebo_truth/odom", Odometry, queue_size=10)
    cloud_pub = rospy.Publisher("/super_cloud", PointCloud2, queue_size=5)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    last_path = [None]
    rospy.Subscriber("/azj/path", Path, lambda m: last_path.__setitem__(0, m), queue_size=10)

    pts = build_room()
    print("合成点云 %d 点（地板 + 隔墙）" % len(pts))

    start = (-1.5, 0.0, 0.8)
    goal = (1.5, 0.0, 0.8)
    rate = rospy.Rate(10)

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
            odom_pub.publish(o)
            cloud_pub.publish(pc2.create_cloud_xyz32(
                rospy.Header(frame_id="map", stamp=now), pts))
            rate.sleep()

    # 先喂 6s 让地图建起来
    print("喂点云 6s 建图 ...")
    feed(60)

    g = PoseStamped()
    g.header.stamp = rospy.Time.now()
    g.header.frame_id = "map"
    g.pose.position.x, g.pose.position.y, g.pose.position.z = goal
    g.pose.orientation.w = 1.0
    goal_pub.publish(g)
    print("目标已发: %s" % (goal,))

    # 等路径出现
    t0 = time.time()
    while time.time() - t0 < 12 and last_path[0] is None:
        feed(5)

    if last_path[0] is None:
        print("RESULT: FAIL — 没有规划出路径（可能全被封死）")
        return 1

    zs = [p.pose.position.z for p in last_path[0].poses]
    ys = [p.pose.position.y for p in last_path[0].poses]
    n = len(zs)
    zmin, zmax = min(zs), max(zs)
    print("\n路径: %d 点" % n)
    print("  z 范围 [%.3f, %.3f]   地板约束 %.2f  天花板约束 %.2f"
          % (zmin, zmax, FLOOR_Z, CEIL_Z))
    print("  y 范围 [%.2f, %.2f]   （应当 |y|>2 绕墙端）" % (min(ys), max(ys)))

    fails = []
    # 必须先确认路径真的到达目标，否则"z 在范围内"是假 PASS
    end = np.array([last_path[0].poses[-1].pose.position.x,
                    last_path[0].poses[-1].pose.position.y,
                    last_path[0].poses[-1].pose.position.z])
    d_end = float(np.linalg.norm(end - np.array(goal)))
    print("  终点 = (%.2f, %.2f, %.2f)  距目标 %.3f m" % (end[0], end[1], end[2], d_end))
    if d_end > 0.5:
        fails.append("路径没到达目标（差 %.2f m）—— 下面是假 PASS，路径在墙这侧打转"
                     % d_end)

    if zmin < FLOOR_Z - 1e-3:
        fails.append("路径钻到地板下：zmin=%.3f < floor_z=%.2f" % (zmin, FLOOR_Z))
    if zmax > CEIL_Z + 1e-3:
        fails.append("路径越过天花板：zmax=%.3f > ceil_z=%.2f" % (zmax, CEIL_Z))
    if n < 5:
        fails.append("路径点太少 (%d)" % n)

    if fails:
        print("\nRESULT: FAIL")
        for f in fails:
            print("  - " + f)
        return 1
    print("\nRESULT: PASS — 路径被限制在 [%.2f, %.2f] 之间" % (FLOOR_Z, CEIL_Z))
    return 0


if __name__ == "__main__":
    sys.exit(main())
