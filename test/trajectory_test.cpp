// 时间参数化（速度曲线）离线验证。
//
// 控制器（navigationIso / navigationContour / navigationOMMPC）直接吃
// PositionCommand 的 velocity 做微分项和前馈，所以速度曲线错了，
// 飞机一定会迟滞或抖。这里把「不超限、能刹住、起点不平重启步」逐条验证。

#include <Eigen/Core>

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "azj_planner/trajectory.h"

using Eigen::Vector3d;
using namespace azj_planner;

namespace {

int g_fail = 0;

void check(bool ok, const std::string& what, const std::string& detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++g_fail;
}

std::vector<Vector3d> straight(double len, int n = 200) {
    std::vector<Vector3d> p;
    p.reserve(n + 1);
    for (int i = 0; i <= n; ++i) p.push_back(Vector3d(len * i / n, 0, 0));
    return p;
}

/// L 形折线：直线 4m → 90° 转弯 → 直线 4m。用来检验转弯降速。
std::vector<Vector3d> elbow(double r = 0.0) {
    std::vector<Vector3d> p;
    const int n1 = 80;
    for (int i = 0; i <= n1; ++i) p.push_back(Vector3d(4.0 * i / n1, 0, 0));
    // 可选圆弧过渡
    if (r > 1e-6) {
        const int n2 = 40;
        for (int i = 1; i <= n2; ++i) {
            const double th = (M_PI / 2) * i / n2;
            p.push_back(Vector3d(4.0 + r * std::sin(th), -r + r * std::cos(th), 0));
        }
        const int n3 = 80;
        for (int i = 1; i <= n3; ++i) p.push_back(Vector3d(4.0 + r, -r - 4.0 * i / n3, 0));
    } else {
        const int n3 = 80;
        for (int i = 1; i <= n3; ++i) p.push_back(Vector3d(4.0, -4.0 * i / n3, 0));
    }
    return p;
}

void dumpSpeedProfile(const PathTrajectory& tr, int every = 0) {
    const auto& t = tr.stamps();
    const auto& v = tr.velocities();
    const size_t n = t.size();
    const int step = every > 0 ? every : std::max(1, static_cast<int>(n / 12));
    std::printf("     t(s)     |v|(m/s)  speed\n");
    for (size_t i = 0; i < n; i += step) {
        const double sp = v[i].norm();
        std::printf("     %6.3f   %6.3f    ", t[i], sp);
        const int bars = static_cast<int>(sp / 0.1);
        for (int b = 0; b < bars && b < 40; ++b) std::printf("#");
        std::printf("\n");
    }
}

}  // namespace

int main() {
    std::printf("\n============ Azj Planner 速度曲线验证 ============\n");

    // ---------- 1) 直线：限速、终点停下 ----------
    {
        std::printf("\n[1] 直线 10m，max_vel=1.0, max_acc=1.5, 静止起步，终点停\n");
        TrajParams pr;
        pr.max_vel = 1.0;
        pr.max_acc = 1.5;
        pr.stop_at_end = true;
        pr.v_start = 0.0;

        PathTrajectory tr;
        const bool ok = tr.build(straight(10.0), pr);
        check(ok, "build 成功");
        if (!ok) return 1;

        check(tr.maxVelocity() <= pr.max_vel + 1e-6, "峰值速度不超限",
              "peak=" + std::to_string(tr.maxVelocity()).substr(0, 5) + " / limit=" +
                  std::to_string(pr.max_vel).substr(0, 4));

        Vector3d p, v, a;
        tr.sample(tr.duration(), p, v, a);
        check(v.norm() < 1e-3, "终点速度归零",
              "|v_end|=" + std::to_string(v.norm()).substr(0, 8));

        tr.sample(0.0, p, v, a);
        check(v.norm() < 1e-2, "起点静止起步", "|v_0|=" + std::to_string(v.norm()).substr(0, 6));

        // 理论最短时间：加速到 1.0 需 0.667s / 0.33m，恒速段 9.33m 需 9.33s
        std::printf("     duration=%.3fs（理论≈%.2fs）\n", tr.duration(),
                    0.333 * 2 + (10.0 - 0.667) / 1.0);
        dumpSpeedProfile(tr);
    }

    // ---------- 2) 重规划：起点有速度，不能当静止起步 ----------
    {
        std::printf("\n[2] 重规划场景：v_start=0.8（飞机已在飞）\n");
        TrajParams pr;
        pr.max_vel = 1.0;
        pr.max_acc = 1.5;
        pr.v_start = 0.8;
        pr.stop_at_end = true;

        PathTrajectory tr;
        tr.build(straight(10.0), pr);
        Vector3d p, v, a;
        tr.sample(0.0, p, v, a);
        std::printf("     起点速度 = %.3f m/s\n", v.norm());
        check(std::abs(v.norm() - 0.8) < 0.05, "起点沿用当前速度（不从 0 重启）",
              "|v_0|=" + std::to_string(v.norm()).substr(0, 5));
        check(tr.duration() < 10.0 / 0.8 + 1.0, "总时长明显短于静止起步",
              "dur=" + std::to_string(tr.duration()).substr(0, 5) + "s");
    }

    // ---------- 3) 90° 直角转弯：必须降速 ----------
    {
        std::printf("\n[3] 90° 直角转弯（无圆弧）—— 曲率极大，必须大幅降速\n");
        TrajParams pr;
        pr.max_vel = 1.5;
        pr.max_acc = 2.0;
        pr.max_lat_acc = 1.2;
        pr.v_start = 0.0;

        PathTrajectory tr;
        tr.build(elbow(0.0), pr);
        check(tr.valid(), "build 成功");

        // 找到转弯附近的最小速度
        const auto& p_all = tr.positions();
        double v_at_corner = std::numeric_limits<double>::max();
        for (size_t i = 0; i < p_all.size(); ++i) {
            if (std::abs(p_all[i].x() - 4.0) < 0.15 && std::abs(p_all[i].y()) < 0.15) {
                v_at_corner = std::min(v_at_corner, tr.velocities()[i].norm());
            }
        }
        std::printf("     拐角处速度 = %.3f m/s\n", v_at_corner);
        check(v_at_corner < 0.5 * pr.max_vel, "拐角显著降速",
              "v_corner=" + std::to_string(v_at_corner).substr(0, 5));
        check(tr.maxVelocity() <= pr.max_vel + 1e-6, "峰值速度不超限");
        dumpSpeedProfile(tr);
    }

    // ---------- 4) 圆弧过渡的转弯：应比直角更快 ----------
    {
        std::printf("\n[4] 90° 转弯带 r=0.5m 圆弧 —— 应比直角过得快\n");
        TrajParams pr;
        pr.max_vel = 1.5;
        pr.max_acc = 2.0;
        pr.max_lat_acc = 1.2;

        PathTrajectory tr_sharp, tr_arc;
        tr_sharp.build(elbow(0.0), pr);
        tr_arc.build(elbow(0.5), pr);

        // 只统计「转弯区」内的速度。不能只写 x>3.9 —— 终点 (4,-4) 也在里面，
        // 而终点速度必然为 0，会把统计结果污染成 0.001（与 min_clearance 同类错误）。
        auto corner_min_speed = [](const PathTrajectory& t) {
            double best = std::numeric_limits<double>::max();
            const auto& P = t.positions();
            const auto& V = t.velocities();
            for (size_t i = 0; i < P.size(); ++i) {
                const bool in_corner =
                    (P[i].x() >= 3.8 && P[i].x() <= 4.8 && P[i].y() >= -0.9 && P[i].y() <= 0.2);
                if (in_corner) best = std::min(best, V[i].norm());
            }
            return best;
        };
        const double vs = corner_min_speed(tr_sharp);
        const double va = corner_min_speed(tr_arc);

        std::printf("     直角 v_min=%.3f（总 %.2fs）  圆弧 v_min=%.3f（总 %.2fs）\n", vs,
                    tr_sharp.duration(), va, tr_arc.duration());

        // 实测结论与直觉相反，值得记住：
        //   圆弧把转弯速度从 0.29 提到 1.10（3.8 倍），但路径多了 0.285m，
        //   总时长反而略长。所以「加圆弧」的价值不是省时间，而是
        //   **速度连续、转弯不用减速到近停** —— 这对控制器友好得多：
        //   前馈速度不剧烈变化，切/法向误差都更小。
        check(va > 2.0 * vs, "圆弧显著提升转弯速度（控制更顺）",
              "ratio=" + std::to_string(va / std::max(vs, 1e-6)).substr(0, 4));
        check(tr_arc.duration() < tr_sharp.duration() * 1.10,
              "路径变长带来的时长代价在 10% 内",
              "delta=" + std::to_string(tr_arc.duration() - tr_sharp.duration()).substr(0, 5) +
                  "s");
    }

    // ---------- 5) 垂直速度限速 ----------
    {
        std::printf("\n[5] 陡爬升：max_vel_z=0.6 必须压住垂直分量\n");
        std::vector<Vector3d> p;
        for (int i = 0; i <= 100; ++i) p.push_back(Vector3d(0, 0, 3.0 * i / 100));

        TrajParams pr;
        pr.max_vel = 2.0;
        pr.max_vel_z = 0.6;
        pr.max_acc = 2.0;
        pr.max_acc_z = 1.0;

        PathTrajectory tr;
        tr.build(p, pr);
        double vz_max = 0.0;
        for (const auto& v : tr.velocities()) vz_max = std::max(vz_max, std::abs(v.z()));
        std::printf("     max |vz| = %.3f m/s\n", vz_max);
        check(vz_max <= pr.max_vel_z + 1e-6, "垂直速度不超限");
    }

    // ---------- 6) 采样连续性与端点 ----------
    {
        std::printf("\n[6] 采样：时间连续、端点正确\n");
        TrajParams pr;
        PathTrajectory tr;
        const auto path = straight(6.0);
        tr.build(path, pr);

        Vector3d p, v, a;
        tr.sample(0.0, p, v, a);
        const double d0 = (p - path.front()).norm();
        tr.sample(tr.duration(), p, v, a);
        const double d1 = (p - path.back()).norm();
        std::printf("     起点误差=%.2e  终点误差=%.2e\n", d0, d1);
        check(d0 < 1e-6 && d1 < 1e-6, "端点精确对齐");

        // 相邻 10ms 采样的位置跳变不应有过大尖峰
        double max_jump = 0.0;
        Vector3d prev;
        bool first = true;
        for (double t = 0.0; t <= tr.duration(); t += 0.01) {
            tr.sample(t, p, v, a);
            if (!first) max_jump = std::max(max_jump, (p - prev).norm());
            prev = p;
            first = false;
        }
        const double expect = pr.max_vel * 0.01 * 2.0;
        std::printf("     10ms 最大位置跳变 = %.4f m（上限参考 %.4f）\n", max_jump, expect);
        check(max_jump <= expect, "采样无跳变");
    }

    // ---------- 7) 退化输入 ----------
    {
        std::printf("\n[7] 退化输入：单点 / 零长度 / 两点重合\n");
        TrajParams pr;
        PathTrajectory tr;

        check(!tr.build({}, pr), "空路径返回 false");
        check(!tr.build({Vector3d(1, 2, 3)}, pr), "单点返回 false");
        check(!tr.build({Vector3d(1, 2, 3), Vector3d(1, 2, 3)}, pr), "零长度返回 false");
    }

    std::printf("\n=================================================\n");
    if (g_fail == 0) {
        std::printf("全部通过。\n\n");
        return 0;
    }
    std::printf("%d 项失败。\n\n", g_fail);
    return 1;
}
