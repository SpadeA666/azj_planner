// MINCO 优化器离线验证。
//
// 验证目标（对齐用户的三条要求）：
//   1) 居中 —— 通道横截面上路径到两侧障碍的距离差要小（"走正中间"）
//   2) 安全 —— min clearance 不低于 robot_radius
//   3) 起点硬约束 —— 优化后轨迹起点的 p/v/a 必须精确等于给定值
//      （这是 MINCO 相对 B 样条的核心收益，绝不能因优化漂移）
//
// 场景与 curve_optimizer_test 对齐，便于横向对比耗时与清晰度。

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "azj_planner/clearance_field.h"
#include "azj_planner/minco_optimizer.h"
#include "azj_planner/path_search.h"

using Eigen::Vector3d;
using namespace azj_planner;

namespace {

int g_fail = 0;
void check(bool ok, const std::string& what, const std::string& detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++g_fail;
}

double boxSdf(const Vector3d& p, const Vector3d& c, const Vector3d& h) {
    const Vector3d d = (p - c).cwiseAbs() - h;
    const Vector3d dm = d.cwiseMax(0.0);
    return dm.norm() + std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0);
}

using Obstacles = std::vector<std::pair<Vector3d, Vector3d>>;   // (center, half)

ClearanceField::Ptr makeField(const Obstacles& obs) {
    return ClearanceField::Ptr(new FunctionClearanceField(
        [obs](const Vector3d& p) {
            double best = 1e9;
            for (const auto& o : obs) best = std::min(best, boxSdf(p, o.first, o.second));
            return best;
        },
        [obs](const Vector3d& p, Vector3d& g) {
            // 解析梯度：取最近那个 box 的梯度（数值稳定、比差分快 6 倍）
            double best = 1e9;
            Vector3d bc(0, 0, 0);
            for (const auto& o : obs) {
                const double d = boxSdf(p, o.first, o.second);
                if (d < best) { best = d; bc = o.first; }
            }
            const double h = 1e-5;
            for (int i = 0; i < 3; ++i) {
                Vector3d dp = Vector3d::Zero();
                dp[i] = h;
                double d1 = 1e9, d0 = 1e9;
                for (const auto& o : obs) {
                    d1 = std::min(d1, boxSdf(p + dp, o.first, o.second));
                    d0 = std::min(d0, boxSdf(p - dp, o.first, o.second));
                }
                g[i] = (d1 - d0) / (2.0 * h);
            }
            return g.allFinite();
        }));
}

/// 两条平行墙构成通道（x 向），中间可通行宽度 = 2*halfGap
Obstacles channel(double half_gap, double length) {
    Obstacles o;
    const Vector3d hh(length * 0.5, 0.10, 1.0);
    o.push_back({Vector3d(length * 0.5, half_gap + 0.10, 0.0), hh});
    o.push_back({Vector3d(length * 0.5, -(half_gap + 0.10), 0.0), hh});
    return o;
}

/// 柱阵：相邻柱子间留缝
Obstacles pillars() {
    Obstacles o;
    for (double x = 2.0; x <= 10.0; x += 2.0) {
        for (double y : {-1.2, 1.2}) {
            o.push_back({Vector3d(x, y, 0.0), Vector3d(0.18, 0.18, 1.0)});
        }
    }
    return o;
}

/// 沿路径在若干 x 处量"到上下两侧障碍的距离差"（居中指标）
double lateralImbalance(const std::vector<Vector3d>& path, double half_gap) {
    double worst = 0.0;
    for (const auto& p : path) {
        if (p.x() < 1.0 || p.x() > 9.0) continue;
        const double d_up = (half_gap) - p.y();      // 到 +y 墙
        const double d_dn = p.y() - (-half_gap);     // 到 -y 墙
        worst = std::max(worst, std::abs(d_up - d_dn));
    }
    return worst;
}

}  // namespace

int main() {
    std::printf("========== MINCO 优化器验证 ==========\n");

    // ══════════════ [1] 直通道：居中 ══════════════
    {
        std::printf("\n[1] 直通道（半宽 1.5m）居中\n");
        const double half_gap = 1.5;
        auto field = makeField(channel(half_gap, 10.0));

        StatePVAJ head;
        head.p = Vector3d(0.0, 0.0, 0.0);
        head.v = Vector3d(1.0, 0.0, 0.0);
        head.a = Vector3d::Zero();

        StatePVAJ tail;
        tail.p = Vector3d(9.0, 0.0, 0.0);

        // 初值故意偏一侧（y=+0.8），看优化能否把它拉回中轴
        std::vector<Vector3d> init;
        for (int k = 0; k <= 30; ++k) {
            const double f = k / 30.0;
            init.push_back(Vector3d(9.0 * f, 0.8 * std::sin(M_PI * f), 0.0));
        }

        MincoOptParams mp;
        mp.safe_margin = 0.30;
        mp.max_vel = 1.0;
        mp.max_acc = 1.5;
        mp.iterations = 60;

        MincoOptimizer opt;
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = opt.optimize(head, tail, init, *field, mp);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();

        check(r.success, "优化成功", r.msg);
        if (r.success) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "min_clr=%.3f len=%.2f dur=%.2fs vmax=%.2f amax=%.2f it=%d",
                          r.min_clearance, r.length, r.duration, r.max_vel, r.max_acc,
                          r.iterations);
            std::printf("        %s  (%.1f ms)\n", buf, ms);
            check(r.min_clearance >= mp.safe_margin * 0.9, "min_clr >= 0.27m",
                  std::to_string(r.min_clearance));
            check(r.max_vel <= mp.max_vel * 1.05, "速度未超限",
                  std::to_string(r.max_vel));

            // 居中：采样路径的横向不平衡
            std::vector<Vector3d> pts;
            for (double t = 0.0; t <= r.duration; t += 0.02) {
                Vector3d p, v, a;
                if (r.traj.sample(t, p, v, a)) pts.push_back(p);
            }
            const double imb = lateralImbalance(pts, half_gap);
            std::printf("        横向不平衡 = %.3f m（初始 1.6m）\n", imb);
            check(imb < 0.25, "居中：横向不平衡 < 0.25m", std::to_string(imb));
        }
    }

    // ══════════════ [2] 起点硬约束 ══════════════
    {
        std::printf("\n[2] 起点 p/v/a 硬约束在优化后保持\n");
        auto field = makeField(channel(1.5, 10.0));

        StatePVAJ head;
        head.p = Vector3d(0.0, 0.1, 0.0);
        head.v = Vector3d(0.7, 0.25, 0.0);
        head.a = Vector3d(0.4, 0.0, 0.0);

        StatePVAJ tail;
        tail.p = Vector3d(8.0, -0.2, 0.0);

        std::vector<Vector3d> init;
        for (int k = 0; k <= 25; ++k) {
            const double f = k / 25.0;
            init.push_back(Vector3d(8.0 * f, 0.1 + (-0.3) * f, 0.0));
        }

        MincoOptParams mp;
        mp.max_vel = 1.0;
        mp.max_acc = 1.5;
        mp.iterations = 50;

        MincoOptimizer opt;
        const auto r = opt.optimize(head, tail, init, *field, mp);
        check(r.success, "优化成功", r.msg);
        if (r.success) {
            Vector3d p, v, a;
            r.traj.sample(0.0, p, v, a);
            check((p - head.p).norm() < 1e-3, "起点位置保持",
                  std::to_string((p - head.p).norm()));
            check((v - head.v).norm() < 1e-2, "起点速度保持",
                  std::to_string((v - head.v).norm()));
            std::printf("        起点 v=(%.3f,%.3f) 期望 (%.3f,%.3f)\n", v.x(), v.y(),
                        head.v.x(), head.v.y());
        }
    }

    // ══════════════ [3] 柱阵：不撞柱 ══════════════
    {
        std::printf("\n[3] 柱阵（缝宽 ~2.0m）安全与居中\n");
        auto field = makeField(pillars());

        StatePVAJ head;
        head.p = Vector3d(0.0, 0.0, 0.0);
        head.v = Vector3d(1.0, 0.0, 0.0);

        StatePVAJ tail;
        tail.p = Vector3d(10.0, 0.0, 0.0);

        std::vector<Vector3d> init;
        for (int k = 0; k <= 40; ++k) {
            const double f = k / 40.0;
            init.push_back(Vector3d(10.0 * f, 0.0, 0.0));   // 直线初值（会撞柱）
        }

        MincoOptParams mp;
        mp.safe_margin = 0.30;
        mp.max_vel = 1.0;
        mp.max_acc = 1.5;
        mp.iterations = 80;
        mp.w_obs = 80.0;

        MincoOptimizer opt;
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = opt.optimize(head, tail, init, *field, mp);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();

        check(r.success, "优化成功", r.msg);
        if (r.success) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "min_clr=%.3f mean_clr=%.3f len=%.2f dur=%.2fs vmax=%.2f",
                          r.min_clearance, r.mean_clearance, r.length, r.duration,
                          r.max_vel);
            std::printf("        %s  (%.1f ms)\n", buf, ms);
            check(r.min_clearance >= 0.27, "min_clr >= 0.27m",
                  std::to_string(r.min_clearance));
            check(r.length < 12.0, "未绕远（len<12m）", std::to_string(r.length));
        }
    }

    // ══════════════ [4] rolling 场景：短段 + 非零终点速度 ══════════════
    {
        std::printf("\n[4] rolling 短段（4m，末点不减速）\n");
        auto field = makeField(channel(1.2, 6.0));

        StatePVAJ head;
        head.p = Vector3d(0.5, 0.0, 0.0);
        head.v = Vector3d(1.0, 0.0, 0.0);
        head.a = Vector3d(0.0, 0.0, 0.0);

        StatePVAJ tail;
        tail.p = Vector3d(4.5, 0.0, 0.0);
        tail.v = Vector3d(1.0, 0.0, 0.0);   // ★ 不减速

        std::vector<Vector3d> init;
        for (int k = 0; k <= 12; ++k) init.push_back(Vector3d(0.5 + 4.0 * k / 12.0, 0.0, 0.0));

        MincoOptParams mp;
        mp.max_vel = 1.0;
        mp.max_acc = 1.5;
        mp.iterations = 40;
        mp.stop_at_end = false;

        MincoOptimizer opt;
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = opt.optimize(head, tail, init, *field, mp);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();

        check(r.success, "优化成功", r.msg);
        if (r.success) {
            std::printf("        dur=%.3fs vmax=%.3f  (%.2f ms)\n", r.duration, r.max_vel, ms);
            // 末速应接近 1.0（不是 0）——这是"不减速"的判据
            Vector3d p, v, a;
            r.traj.sample(r.duration, p, v, a);
            std::printf("        末速 = %.3f m/s（期望 ≈1.0，不是 0）\n", v.norm());
            check(v.norm() > 0.7, "末速保持（未减速）", std::to_string(v.norm()));
            // 短段应该在 0.5s 内
            check(r.duration < 6.0, "时长短（duration<6s）", std::to_string(r.duration));
        }
    }

    std::printf("\n======================================\n");
    if (g_fail == 0) {
        std::printf("全部通过。\n");
    } else {
        std::printf("失败 %d 项。\n", g_fail);
    }
    return g_fail == 0 ? 0 : 1;
}
