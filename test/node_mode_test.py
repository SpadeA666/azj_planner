#!/usr/bin/env python3
"""节点级 2D / 3D 模式验证（隔离 master，不需要 Gazebo / 控制器）。

离线测试（plan_mode_test）只能证明算法正确；这里验证**节点里真正接上的那层**：

  1) 启动时读 yaml 的 plan_2d
  2) 运行时 `rosparam set /azj_planner/plan_2d true|false` 真的会切换
  3) 2D 输出严格 z = 目标高度（A* + 曲线优化整链）
  4) 2D 的柱状投影确实抓到了 z=0.8 单层看不见的桌子 / 横梁
  5) 切回 3D 后恢复爬升

场景（z 平面 = 0.8，飞机半径 0.30）：
    矮桌  x=2 顶面 z=0.40  → 单层 ESDF 距离 0.40 > 0.30（看不见）
    横梁  x=4 底面 z=1.25  → 单层 ESDF 距离 0.45 > 0.30（看不见）
  → 2D 必须绕到 |y| > 0.9；3D 在 0.8m 高度可以直飞。

用法（脚本自己起 roscore，用完清掉）：
    python3 test/node_mode_test.py
退出码 0 = 全部通过。
"""
import math
import os
import signal
import subprocess
import sys
import time

# ---- master 隔离：必须在 import rospy 之前 -----------------------------------
MASTER_URI = "http://127.0.0.1:11399"
os.environ["ROS_MASTER_URI"] = MASTER_URI
os.environ.setdefault("ROS_HOME", "/tmp/azj_node_mode_ros")

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
NODE_EXE = os.path.join(WS, "devel", "lib", "azj_planner", "azj_planner_node")
CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config",
                   "azj_planner.yaml")

START = (-1.0, 0.0, 0.5)
GOAL = (5.0, 0.0, 0.8)
Z_PLANE = 0.8


def scene_points():
    """房间 + 矮桌 + 横梁，5cm 网格采样表面。"""
    pts = []

    def grid(xr, yr, z, step=0.05):
        x = xr[0]
        while x <= xr[1] + 1e-9:
            y = yr[0]
            while y <= yr[1] + 1e-9:
                pts.append((x, y, z))
                y += step
            x += step

    def grid_yz(x, yr, zr, step=0.05):
        y = yr[0]
        while y <= yr[1] + 1e-9:
            z = zr[0]
            while z <= zr[1] + 1e-9:
                pts.append((x, y, z))
                z += step
            y += step

    # 地面 / 两侧墙 / 前后墙
    grid((-4.0, 8.0), (-2.5, 2.5), 0.0)
    grid_yz(-4.0, (-2.5, 2.5), (0.0, 1.8))
    grid_yz(8.0, (-2.5, 2.5), (0.0, 1.8))
    for y in (-2.5, 2.5):
        x = -4.0
        while x <= 8.0 + 1e-9:
            z = 0.0
            while z <= 1.8 + 1e-9:
                pts.append((x, y, z))
                z += 0.05
            x += 0.05

    # 矮桌：中心 (2,0)，x/y 半 0.6，顶面 0.40
    grid((1.4, 2.6), (-0.6, 0.6), 0.40)
    # 横梁：x=4，y 半 0.6，底面 1.25、顶面 1.45
    grid((3.7, 4.3), (-0.6, 0.6), 1.25)
    return pts


class Harness:
    def __init__(self):
        self.cloud = scene_points()
        self.path = None
        self.path_rx = 0.0

        self.cloud_pub = rospy.Publisher("/super_cloud", PointCloud2, queue_size=1)
        self.odom_pub = rospy.Publisher("/gazebo_truth/odom", Odometry, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/azj/path", Path, self.path_cb, queue_size=5)

    def path_cb(self, msg):
        self.path = msg
        self.path_rx = rospy.get_time()

    def spin_pub(self, dur):
        """持续发布点云 + 静止 odom（rog_map 需要两者）。"""
        end = rospy.get_time() + dur
        r = rospy.Rate(10)
        while rospy.get_time() < end and not rospy.is_shutdown():
            hdr = Header(frame_id="map", stamp=rospy.Time.now())
            cloud = point_cloud2.create_cloud_xyz32(hdr, self.cloud)
            self.cloud_pub.publish(cloud)

            od = Odometry()
            od.header.frame_id = "map"
            od.header.stamp = rospy.Time.now()
            od.child_frame_id = "base_link"
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = START
            od.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(od)
            r.sleep()

    def send_goal(self, g=GOAL):
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = g
        ps.pose.orientation.w = 1.0
        self.goal_pub.publish(ps)


FAILS = []


def check(ok, what, detail=""):
    print("  [%s] %s%s%s" % ("PASS" if ok else "FAIL", what,
                             "  " if detail else "", detail))
    if not ok:
        FAILS.append(what)


def path_stats(path):
    xs = [p.pose.position.x for p in path.poses]
    ys = [p.pose.position.y for p in path.poses]
    zs = [p.pose.position.z for p in path.poses]
    return xs, ys, zs


def main():
    if not os.path.exists(NODE_EXE):
        print("找不到节点可执行文件: %s" % NODE_EXE)
        return 1
    if not os.path.exists(CFG):
        print("找不到配置: %s" % CFG)
        return 1

    roscore = subprocess.Popen(["roscore", "-p", "11399"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               preexec_fn=os.setsid)

    node = None
    try:
        time.sleep(3.0)
        rospy.init_node("azj_node_mode_test", anonymous=True, disable_signals=True)

        env = dict(os.environ)
        env["ROS_MASTER_URI"] = MASTER_URI
        # ★ 不能 stdout=PIPE 而不读：节点日志很密（参数回显 + "No point cloud" 警告），
        #   64KB 管道缓冲区写满后节点会阻塞在 write()，永远走不到 advertise。
        node_log = open("/tmp/azj_node_mode_test_node.log", "w")
        node = subprocess.Popen(
            [NODE_EXE, "__name:=azj_planner", "_config_file:=" + CFG],
            stdout=node_log, stderr=subprocess.STDOUT, env=env,
            preexec_fn=os.setsid)

        h = Harness()

        # 等节点注册 /azj/path（rog_map 初始化约 6s）
        print("等待节点启动（rog_map ESDF 初始化）...")
        t0 = time.time()
        def path_registered():
            return "/azj/path" in [t for t, _ in rospy.get_published_topics()]

        while time.time() - t0 < 40:
            h.spin_pub(0.2)
            if path_registered():
                break
        if not path_registered():
            print("FAIL: 节点未注册 /azj/path")
            return 1
        # 再喂 3s 点云，让 ESDF 收敛
        h.spin_pub(3.0)
        print("节点已就绪。\n")

        print("[1] 默认 3D：起点 z=0.5 → 目标 z=0.8，路径应有爬升")
        h.send_goal()
        t0 = time.time()
        while time.time() - t0 < 20 and h.path is None:
            h.spin_pub(0.2)
        check(h.path is not None, "收到 /azj/path")
        if h.path is None:
            return 1
        xs, ys, zs = path_stats(h.path)
        check(min(zs) < 0.76 and max(zs) > 0.79, "3D 路径有爬升",
              "z=[%.3f, %.3f]" % (min(zs), max(zs)))
        y3 = max(abs(y) for y in ys)
        print("      3D: %d 点  |y|max=%.3f" % (len(xs), y3))

        print("\n[2] 运行时切到 2D（rosparam set /azj_planner/plan_2d true）")
        subprocess.check_call(["rosparam", "set", "/azj_planner/plan_2d", "true"],
                              env=dict(os.environ, ROS_MASTER_URI=MASTER_URI))
        before = h.path_rx
        t0 = time.time()
        while time.time() - t0 < 20 and (h.path_rx <= before):
            h.spin_pub(0.2)
        check(h.path_rx > before, "切换后收到新路径")
        xs, ys, zs = path_stats(h.path)
        check(max(zs) - min(zs) < 1e-6, "2D 路径 z 严格恒定",
              "z=[%.6f, %.6f]" % (min(zs), max(zs)))
        check(abs(zs[0] - Z_PLANE) < 1e-6, "2D 平面 = 目标高度",
              "z=%.4f" % zs[0])
        y2 = max(abs(y) for y in ys)
        check(y2 > 0.9, "2D 绕开单层看不见的桌子/横梁",
              "|y|max=%.3f (>0.9)" % y2)
        print("      2D: %d 点  |y|max=%.3f" % (len(xs), y2))

        print("\n[3] 切回 3D")
        subprocess.check_call(["rosparam", "set", "/azj_planner/plan_2d", "false"],
                              env=dict(os.environ, ROS_MASTER_URI=MASTER_URI))
        before = h.path_rx
        t0 = time.time()
        while time.time() - t0 < 20 and (h.path_rx <= before):
            h.spin_pub(0.2)
        check(h.path_rx > before, "切回后收到新路径")
        xs, ys, zs = path_stats(h.path)
        check(max(zs) - min(zs) > 1e-3, "3D 恢复爬升（z 不再被锁）",
              "z=[%.3f, %.3f]" % (min(zs), max(zs)))

        print("\n" + "=" * 53)
        if FAILS:
            print("失败 %d 项：" % len(FAILS))
            for f in FAILS:
                print("  - " + f)
            return 1
        print("全部通过。")
        return 0
    finally:
        for p in (node, roscore):
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=8)
                except Exception:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception:
                        pass


if __name__ == "__main__":
    sys.exit(main())
