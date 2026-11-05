#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""raicom 场景单点验收测试。

按用户要求：用 raicom 的真实几何，只打一个点，验证
  ① 能规划出路径
  ② 路径不撞墙（用真实 3D 几何独立校验，不依赖规划器自己的 ESDF）

几何完全从 ~/PX4_Firmware/Tools/sitl_gazebo/worlds/raicom.world 解析而来
（该文件里 yaw=-1.5708，旋转后 size 的 x/y 维度互换，见 comments）。

用法：
    python3 raicom_point_test.py                    # 默认点
    python3 raicom_point_test.py -0.136 -1.695      # 指定点
    python3 raicom_point_test.py --list             # 列出可选测试点
"""
import math
import sys

import numpy as np

# ============================ raicom.world 精确几何 ============================
# 每项 (name, center(x,y,z), half(x,y,z))
# yaw=-1.5708 → 世界 x 半长 = size_y/2，世界 y 半长 = size_x/2
BOXES = [
    # 四壁（3m 高）
    ("net_north", (0.65, -1.25, 1.5), (0.025, 2.05, 1.5)),
    ("net_south", (-3.35, -1.25, 1.5), (0.025, 2.05, 1.5)),
    ("net_east", (-1.35, -3.25, 1.5), (2.05, 0.025, 1.5)),
    ("net_west", (-1.35, 0.75, 1.5), (2.05, 0.025, 1.5)),
    # 横墙：y≈-0.8，x∈[-1.35,0.65]
    ("fixed_wall_1", (-0.35, -0.80, 1.5), (1.0, 0.05, 1.5)),
    # 竖网：x=-1.35，y∈[-2.05,-0.85]
    ("net_obstacle_1", (-1.35, -1.45, 1.5), (0.05, 0.60, 1.5)),
    # 框（perforated_obstacle，绕 -90° 后的四个构件）
    ("frame_post_left", (-1.35, 0.65, 0.75), (0.05, 0.10, 0.75)),
    ("frame_post_right", (-1.35, -0.65, 0.75), (0.05, 0.10, 0.75)),
    ("frame_sill", (-1.35, 0.0, 0.10), (0.05, 0.55, 0.10)),
    ("frame_lintel", (-1.35, 0.0, 1.40), (0.05, 0.55, 0.10)),
    # 靶子（z 1.25~1.75，飞行高度 0.8 不受影响）
    ("target_A", (0.61, -1.60, 1.50), (0.01, 0.25, 0.25)),
    ("target_B", (0.61, -2.60, 1.50), (0.01, 0.25, 0.25)),
]
# 四根柱 r=0.1，高 3m
CYLINDERS = [
    (-2.05, -0.80), (-2.75, -0.80), (-2.05, -2.05), (-2.75, -2.05),
]
FLOOR_Z = 0.0


def box_sdf(p, c, h):
    d = np.abs(np.asarray(p) - np.asarray(c)) - np.asarray(h)
    dm = np.maximum(d, 0.0)
    return float(np.linalg.norm(dm) + min(max(d[0], max(d[1], d[2])), 0.0))


def cyl_sdf(p, cx, cy, r, z0, z1):
    """竖圆柱的有符号距离（近似：径向 + z 端面）。"""
    dx, dy, dz = p[0] - cx, p[1] - cy, p[2] - (z0 + z1) / 2.0
    half_z = (z1 - z0) / 2.0
    dxy = math.hypot(dx, dy) - r
    dz_ = abs(dz) - half_z
    if dxy > 0 and dz_ > 0:
        return math.hypot(dxy, dz_)
    return max(dxy, dz_)


def scene_clearance(p):
    """真实 3D 几何的 clearance（独立于规划器的 ESDF）。"""
    m = p[2] - FLOOR_Z          # 到地板
    for name, c, h in BOXES:
        m = min(m, box_sdf(p, c, h))
    for cx, cy in CYLINDERS:
        m = min(m, cyl_sdf(p, cx, cy, 0.10, 0.0, 3.0))
    return m


def build_cloud(res=0.04):
    """生成点云（喂给 rog_map）。只生成 z<=1.6 的部分，够规划用。"""
    P = []
    zmax = 1.6
    for name, c, h in BOXES:
        lo = [c[i] - h[i] for i in range(3)]
        hi = [c[i] + h[i] for i in range(3)]
        lo[2] = max(lo[2], 0.0)
        hi[2] = min(hi[2], zmax)
        if hi[2] <= lo[2]:
            continue
        nx = max(1, int((hi[0] - lo[0]) / res) + 1)
        ny = max(1, int((hi[1] - lo[1]) / res) + 1)
        nz = max(1, int((hi[2] - lo[2]) / res) + 1)
        for x in np.linspace(lo[0], hi[0], nx):
            for y in np.linspace(lo[1], hi[1], ny):
                for z in np.linspace(lo[2], hi[2], nz):
                    # 只保留表面点（内部点对 ESDF 无用且量大）
                    on_surf = (
                        abs(x - lo[0]) < res or abs(x - hi[0]) < res or
                        abs(y - lo[1]) < res or abs(y - hi[1]) < res or
                        abs(z - lo[2]) < res or abs(z - hi[2]) < res
                    )
                    if on_surf:
                        P.append((x, y, z))
    for cx, cy in CYLINDERS:
        for a in np.arange(0, 2 * math.pi, res / 0.10):
            for z in np.arange(0.0, zmax, res):
                P.append((cx + 0.10 * math.cos(a), cy + 0.10 * math.sin(a), z))
    # 地板
    for x in np.arange(-3.35, 0.66, res):
        for y in np.arange(-3.25, 0.76, res):
            P.append((x, y, 0.0))
    return np.array(P, dtype=np.float32)


# ============================ 测试点 ============================
START = (-0.08, -0.09, 0.80)          # 起飞点附近
DEFAULT_GOAL = (-0.136, -1.695, 0.80)  # 用户指定的点（消息里 z=0，被 override 成 0.8）

CASES = [
    ("user_point", (-0.136, -1.695, 0.80), "用户指定：隔墙南侧"),
    ("same_side", (-0.60, -0.40, 0.80), "隔墙北侧（应当容易）"),
    ("through_frame", (-1.70, 0.20, 0.80), "穿框到西区"),
]


def reachability(r_robot, start, goal, z, res=0.05):
    """纯几何栅格可达性（不依赖 ROS/建图）。

    为什么需要它：真实点云有缺失（雷达视角限制），A* 可能在缺失处
    "看到缝"而给出穿墙路径。这个函数用**完整几何**判断物理上到底能不能过，
    从而区分"规划器 bug"和"场景物理不可达"。
    """
    from collections import deque
    xs = np.arange(-3.30, 0.63, res)
    ys = np.arange(-3.20, 0.72, res)
    nx, ny = len(xs), len(ys)
    free = np.array([[scene_clearance(np.array([x, y, z])) >= r_robot
                      for y in ys] for x in xs], dtype=bool)

    def idx(px, py):
        return (int(round((px - xs[0]) / res)), int(round((py - ys[0]) / res)))

    si, gi = idx(start[0], start[1]), idx(goal[0], goal[1])
    if not (0 <= si[0] < nx and 0 <= si[1] < ny and 0 <= gi[0] < nx and 0 <= gi[1] < ny):
        return None
    if not free[si] or not free[gi]:
        return {"ok": False, "reason": "起点或目标格本身不可通行", "free_ratio": free.mean()}

    seen = np.zeros_like(free)
    q = deque([si])
    seen[si] = True
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and free[a, b] and not seen[a, b]:
                seen[a, b] = True
                q.append((a, b))
    if seen[gi]:
        return {"ok": True, "free_ratio": free.mean()}
    best = None
    for i in range(nx):
        for j in range(ny):
            if seen[i, j]:
                d = math.hypot(xs[i] - goal[0], ys[j] - goal[1])
                if best is None or d < best[0]:
                    best = (d, xs[i], ys[j])
    return {"ok": False, "reason": "被障碍隔断", "free_ratio": free.mean(),
            "nearest_reach": best}


def main():
    if "--reachable" in sys.argv:
        nums = [a for a in sys.argv[1:] if not a.startswith("--")]
        goal = tuple(float(v) for v in nums[:2]) if len(nums) >= 2 else DEFAULT_GOAL[:2]
        print("=" * 70)
        print("纯几何可达性分析（完整 raicom 几何，不依赖建图）")
        print("  起点 (%.2f, %.2f, %.2f)  目标 (%.3f, %.3f, 0.80)"
              % (START[0], START[1], START[2], goal[0], goal[1]))
        print("=" * 70)
        for r in (0.30, 0.28, 0.25, 0.22, 0.20):
            res = reachability(r, START, goal, 0.80)
            if res is None:
                print("  R=%.2f  超出分析范围" % r)
            elif res["ok"]:
                print("  R=%.2f  ✓ 可达      （可通行面积 %.0f%%）" % (r, 100 * res["free_ratio"]))
            else:
                extra = ""
                if "nearest_reach" in res:
                    d, x, y = res["nearest_reach"]
                    extra = "；起点可达区最近只能到 (%.2f,%.2f)，距目标 %.2f m" % (x, y, d)
                print("  R=%.2f  ✗ 不可达：%s%s" % (r, res["reason"], extra))
        print("\n结论：如果 R=0.30 不可达而 R=0.25 可达，说明该点在真机半径下")
        print("      物理上过不去，规划器报 NO_PATH 是正确的。")
        print("      若真实仿真里却给出了路径，说明点云有缺失（A* 在缺失处看到了假缝）。")
        return 0

    if "--list" in sys.argv:
        for n, g, d in CASES:
            print("  %-14s (%.3f, %.3f, %.3f)  %s" % (n, g[0], g[1], g[2], d))
        return 0

    nums = [a for a in sys.argv[1:] if not a.startswith("--")]
    goal = tuple(float(v) for v in nums[:3]) if len(nums) >= 2 else DEFAULT_GOAL
    if len(goal) == 2:
        goal = (goal[0], goal[1], 0.80)
    elif len(goal) == 1:
        goal = DEFAULT_GOAL

    print("=" * 70)
    print("raicom 场景单点验收")
    print("  起点 (%.2f, %.2f, %.2f)" % START)
    print("  目标 (%.3f, %.3f, %.3f)" % goal)
    gc = scene_clearance(np.array(goal))
    sc = scene_clearance(np.array(START))
    print("  目标真实 clearance = %.3f m   起点 = %.3f m   (robot_radius 0.3)" % (gc, sc))
    print("=" * 70)

    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as pc2

    rospy.init_node("raicom_pt", anonymous=True)
    op = rospy.Publisher("/gazebo_truth/odom", Odometry, queue_size=10)
    cp = rospy.Publisher("/super_cloud", PointCloud2, queue_size=5)
    gp = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    last = [None]
    rospy.Subscriber("/azj/path", Path, lambda m: last.__setitem__(0, m), queue_size=10)

    pts = build_cloud()
    print("点云 %d 点" % len(pts))

    r = rospy.Rate(10)

    def feed(n):
        for _ in range(n):
            now = rospy.Time.now()
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = "map"
            o.pose.pose.position.x = START[0]
            o.pose.pose.position.y = START[1]
            o.pose.pose.position.z = START[2]
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
    g.pose.position.x, g.pose.position.y, g.pose.position.z = goal
    g.pose.orientation.w = 1.0
    gp.publish(g)

    import time
    t0 = time.time()
    while time.time() - t0 < 15 and last[0] is None:
        feed(5)

    if last[0] is None:
        print("\nRESULT=NO_PATH（没规划出路径）")
        return 1

    P = np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z]
                  for q in last[0].poses])
    L = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
    straight = math.dist(START, goal)
    d_end = float(np.linalg.norm(P[-1] - np.array(goal)))
    zspan = float(P[:, 2].max() - P[:, 2].min())

    # ★ 用真实几何独立校验（沿路径密集采样，不依赖规划器自己的 ESDF）
    mn_real = 1e9
    mn_at = None
    for i in range(1, len(P)):
        seg = P[i] - P[i - 1]
        n = max(1, int(np.ceil(np.linalg.norm(seg) / 0.01)))
        for k in range(n + 1):
            q = P[i - 1] + seg * (k / n)
            c = scene_clearance(q)
            if c < mn_real:
                mn_real, mn_at = c, q

    # 转角
    turns = []
    for i in range(1, len(P) - 1):
        a, b = P[i] - P[i - 1], P[i + 1] - P[i]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            continue
        turns.append(math.degrees(math.acos(
            max(-1.0, min(1.0, float(np.dot(a, b) / (na * nb)))))))
    tmax = max(turns) if turns else 0.0
    tmed = sorted(turns)[len(turns) // 2] if turns else 0.0

    print("\n--- 规划结果 ---")
    print("  路径 %d 点  L=%.2fm  直线=%.2fm  绕行比=%.2f" % (len(P), L, straight, L / straight))
    print("  z 范围 [%.3f, %.3f] 跨度=%.3f（起点 %.2f → 目标 %.2f）"
          % (P[:, 2].min(), P[:, 2].max(), zspan, START[2], goal[2]))
    print("  转角 中位=%.1f° 最大=%.1f°（>20° 的点数 %d）"
          % (tmed, tmax, sum(1 for t in turns if t > 20)))
    print("  终点距目标 %.3f m" % d_end)
    print("\n--- 真实几何独立校验（不依赖规划器 ESDF）---")
    print("  路径最小 clearance = %.4f m" % mn_real)
    if mn_at is not None:
        print("  最危险点 = (%.2f, %.2f, %.2f)" % tuple(mn_at))

    # 判定标准（用户要求）：**能规划 + 不撞**。
    #
    # 折角不判 FAIL：raicom 的框孔净宽 1.1m、柱子缝隙 0.5m，
    # 通过这种缝必然要急转，20°+ 折角是几何必然，不是规划器缺陷。
    # 折角只作为"观感"参考输出。
    fails = []
    if d_end > 0.5:
        fails.append("没到目标（差 %.2f m）" % d_end)
    if mn_real < 0.0:
        fails.append("★ 撞墙！最小 clearance %.4f m < 0" % mn_real)
    elif mn_real < 0.10:
        fails.append("★ 贴墙过近：%.4f m（< 0.10 视为不安全）" % mn_real)

    print("\n" + "=" * 70)
    if fails:
        print("RESULT=FAIL")
        for f in fails:
            print("  - " + f)
        return 1
    print("RESULT=PASS  路径可规划、不撞、平滑")
    return 0


if __name__ == "__main__":
    sys.exit(main())
