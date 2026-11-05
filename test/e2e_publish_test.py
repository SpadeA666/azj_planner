#!/usr/bin/env python3
"""azj_planner 端到端闭环验证（隔离 master，不影响正在运行的 11311）。

模拟一个「一阶跟随」的飞机：
  - 订阅 /planning/pos_cmd 拿到参考 (p, v)
  - 用一阶滞后更新仿真位置/速度，作为 odom 与 velocity_local 反馈
这样才验证得了：参考点是否真的推进、速度是否上得去、终点是否 COMPLETED。

用 unknown_as_free=true 绕开建图（本测试只验证控制链路，不验证 ESDF）。
"""
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

TAU = 0.25          # 一阶跟随时间常数 [s]，模拟控制器+机体响应
DT = 0.02           # 20Hz 反馈
REF_TIMEOUT = 0.5   # 参考超时 [s]，对齐 as_controller 的 nav_traj_timeout


class Sim:
    def __init__(self):
        self.p = [0.0, 0.0, 1.0]
        self.v = [0.0, 0.0, 0.0]
        self.ref_p = None
        self.ref_v = [0.0, 0.0, 0.0]
        self.flag = 0
        self.got = 0
        self.last_rx = 0.0

        self.odom_pub = rospy.Publisher("/gazebo_truth/odom", Odometry,
                                        queue_size=10)
        self.vel_pub = rospy.Publisher("/gazebo_truth/odom_vel",
                                       TwistStamped, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cb, queue_size=50)

    def cb(self, msg):
        self.ref_p = [msg.position.x, msg.position.y, msg.position.z]
        self.ref_v = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        self.flag = msg.trajectory_flag
        self.got += 1
        self.last_rx = rospy.get_time()

    def step(self, dt):
        # 参考超时 → 速度指令归零。
        # 这是必需的：as_controller 的 nav_traj_timeout=0.5s，规划器停发参考后
        # 它会自行刹车。仿真不模拟这一点，飞机会带着最后一次参考速度一直飞，
        # 测出"无限前进"的假故障。
        if rospy.get_time() - self.last_rx > REF_TIMEOUT:
            self.ref_v = [0.0, 0.0, 0.0]
            self.ref_p = None

        # 参考速度一阶跟随（模拟控制器把指令变成机体速度）
        for i in range(3):
            self.v[i] += (self.ref_v[i] - self.v[i]) * min(1.0, dt / TAU)
            self.p[i] += self.v[i] * dt
        self.publish()

    def publish(self):
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = "map"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.p
        o.pose.pose.orientation.w = 1.0
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = self.v
        self.odom_pub.publish(o)

        v = TwistStamped()
        v.header.stamp = rospy.Time.now()
        v.twist.linear.x, v.twist.linear.y, v.twist.linear.z = self.v
        self.vel_pub.publish(v)

    def send_goal(self, g):
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = g
        m.pose.orientation.w = 1.0
        self.goal_pub.publish(m)


def main():
    # --expect-fail：期望规划失败，此时必须**一条都不发**，
    # 让控制器的 nav_traj_timeout 自行刹车，而不是发脏参考把飞机带走。
    #
    # 注意：不能再用"没有地图"来制造失败 —— 现在 UNKNOWN 默认按
    # unknown_clearance_cap 给值（>=robot_radius 就可通行），无地图反而能规划。
    # 要制造真正不可达，需传 _azj/unknown_clearance_cap:=0.1（< robot_radius 0.3）。
    expect_fail = "--expect-fail" in sys.argv

    rospy.init_node("azj_e2e_sim", anonymous=True)
    sim = Sim()
    rate = rospy.Rate(int(1.0 / DT))

    start = list(sim.p)
    goal = [8.0, 0.0, 1.0]

    # 先喂 1.5s 空反馈，让节点拿到 odom
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < 1.5 and not rospy.is_shutdown():
        sim.step(DT)
        rate.sleep()

    sim.last_rx = rospy.get_time()
    sim.send_goal(goal)
    print("goal sent:", goal)

    t_start = rospy.get_time()
    traj = []
    reached = False
    timeout = 25.0

    while not rospy.is_shutdown():
        el = rospy.get_time() - t_start
        if el > timeout:
            break
        sim.step(DT)
        traj.append((el, list(sim.p), list(sim.v), sim.flag))

        # 到达判定：接近目标，且轨迹已 COMPLETED 或参考已停（控制器刹车）
        d = math.dist(sim.p, goal)
        ref_gone = (rospy.get_time() - sim.last_rx) > REF_TIMEOUT
        if d < 0.40 and (sim.flag == PositionCommand.TRAJECTORY_STATUS_COMPLETED or ref_gone):
            reached = True
            break
        rate.sleep()

    el = traj[-1][0]
    vmax = max(math.sqrt(sum(x * x for x in r[2])) for r in traj)
    pmax = max(r[1][0] for r in traj)
    final = traj[-1]

    print("\n--- 闭环结果 ---")
    print("耗时 = %.2f s" % el)
    print("收到 pos_cmd = %d 条" % sim.got)
    print("最大速度 = %.3f m/s" % vmax)
    print("最远 x = %.3f m （目标 %.1f）" % (pmax, goal[0]))
    print("终点位置 = (%.3f, %.3f, %.3f)" % tuple(final[1]))
    print("终点 flag = %d （COMPLETED=%d）" % (final[3], PositionCommand.TRAJECTORY_STATUS_COMPLETED))

    # 每 2s 打印一次轨迹
    print("\n  t(s)    x      vx     flag")
    last_dec = -1
    for el_i, p, v, f in traj:
        dec = int(el_i // 2)
        if dec != last_dec:
            print("  %5.2f  %6.3f  %6.3f   %d" % (el_i, p[0], v[0], f))
            last_dec = dec

    fails = []
    if expect_fail:
        if sim.got != 0:
            fails.append("expected NO pos_cmd on unresolvable goal, got %d" % sim.got)
        print("\nRESULT: " + ("FAIL" if fails else "PASS (正确保持静默)"))
        for f in fails:
            print("  - " + f)
        return 1 if fails else 0

    if sim.got < 20:
        fails.append("too few pos_cmd (%d)" % sim.got)
    if vmax < 0.5:
        fails.append("velocity never rose above 0.5 m/s (max %.3f)" % vmax)
    if pmax < goal[0] - 1.0:
        fails.append("did not advance toward goal (max x=%.2f)" % pmax)
    if not reached:
        fails.append("never reached goal within %.0fs" % timeout)

    print("\nRESULT: " + ("FAIL" if fails else "PASS"))
    for f in fails:
        print("  - " + f)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
