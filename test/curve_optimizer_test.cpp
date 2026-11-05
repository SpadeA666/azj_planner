// 曲线优化离线验证。
//
// 重点验证三件事：
//   1) 折线 → 曲线（曲率连续、转角消失）
//   2) ★「两侧等距」真的成立 —— 这是用户的核心安全理念
//   3) 安全不被牺牲（min clearance 不比折线差）
//
// 第 2 条用直接测量：在通道的每个横截面上，量路径到左墙和右墙的距离，
// 两者之差应当远小于折线路径。

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "azj_planner/clearance_field.h"
#include "azj_planner/curve_optimizer.h"
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

/// 折线路径的转角统计。
struct TurnStats {
    double median{0}, max{0};
    int over20{0};
};
TurnStats turnStats(const std::vector<Vector3d>& path) {
    TurnStats s;
    std::vector<double> t;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const Vector3d a = path[i] - path[i - 1], b = path[i + 1] - path[i];
        const double na = a.norm(), nb = b.norm();
        if (na < 1e-9 || nb < 1e-9) continue;
        t.push_back(std::acos(std::max(-1.0, std::min(1.0, a.dot(b) / (na * nb)))) * 180.0 / M_PI);
    }
    if (t.empty()) return s;
    std::sort(t.begin(), t.end());
    s.median = t[t.size() / 2];
    s.max = t.back();
    for (double v : t) if (v > 20) ++s.over20;
    return s;
}

// =====================================================================
// 场景：一条笔直的斜向通道，两侧各一堵墙，间距 2W。
// 用来精确测量"两侧是否等距"。
// =====================================================================
struct ChannelScene {
    Vector3d origin;        // 通道中心线上的参考点
    Vector3d axis;          // 通道走向（单位向量）
    Vector3d normal;        // 通道横向（单位向量）
    double half_width{1.0}; // 中心到每侧墙的距离

    /// 到两侧墙的距离（带符号）
    double distLeft(const Vector3d& p) const {
        return half_width - (p - origin).dot(normal);
    }
    double distRight(const Vector3d& p) const {
        return half_width + (p - origin).dot(normal);
    }
    /// 有符号距离场：两条平行墙的最小距离
    double sdf(const Vector3d& p) const {
        return std::min(distLeft(p), distRight(p));
    }
    /// 横向偏移（0 = 正中）
    double lateral(const Vector3d& p) const { return (p - origin).dot(normal); }
};

void reportLateral(const std::string& tag, const std::vector<Vector3d>& path,
                   const ChannelScene& sc) {
    double sum = 0.0, mx = 0.0;
    int n = 0;
    for (const auto& p : path) {
        const double l = sc.lateral(p);
        sum += std::abs(l);
        mx = std::max(mx, std::abs(l));
        ++n;
    }
    std::printf("       %-20s 横向偏移 |均值|=%.3f m  |最大|=%.3f m  样本=%d\n",
                tag.c_str(), n ? sum / n : 0.0, mx, n);
}

}  // namespace

int main() {
    std::printf("\n=========== Azj Planner 曲线优化验证 ===========\n");

    // ================= 场景 1：斜向通道，验证「两侧等距」 =================
    {
        std::printf("\n[1] 45° 斜向通道，半净宽 1.0m —— 核心检验：两侧等距\n");
        std::printf("    起点终点刻意偏在一侧，纯最短路径会贴墙\n\n");

        ChannelScene sc;
        sc.origin = Vector3d(0, 0, 0.8);
        sc.axis = Vector3d(1, -1, 0).normalized();
        sc.normal = Vector3d(-sc.axis.y(), sc.axis.x(), 0).normalized();
        sc.half_width = 1.0;

        FunctionClearanceField field([&](const Vector3d& p) { return sc.sdf(p); });

        // 起终点沿通道两端、各偏一侧 0.7m
        const Vector3d start = sc.origin - sc.axis * 4.0 + sc.normal * 0.7;
        const Vector3d goal = sc.origin + sc.axis * 4.0 - sc.normal * 0.7;

        // --- A* 折线 ---
        SearchParams sp;
        sp.resolution = 0.10;
        sp.robot_radius = 0.30;
        sp.clearance_weight = 0.10;
        sp.clearance_cap = 2.0;
        sp.alpha = 2.0;
        sp.box_margin = Vector3d(2, 2, 1);
        sp.goal_tolerance = 0.25;
        sp.max_expansions = 2000000;

        ClearanceAStar astar;
        const auto sr = astar.search(field, start, goal, sp);
        if (!sr.success) {
            std::printf("     A* 失败: %s\n", sr.msg.c_str());
            ++g_fail;
        } else {
            const auto poly = simplifyPath(sr.path);
            const auto smooth3 = smoothPath(poly, field, 0.30, 3, 0.35);

            const TurnStats ts_poly = turnStats(poly);
            std::printf("     折线(simplify): 顶点=%zu 转角中位=%.1f° 最大=%.1f° >20°拐点=%d\n",
                        poly.size(), ts_poly.median, ts_poly.max, ts_poly.over20);
            reportLateral("折线", poly, sc);

            // --- 曲线优化 ---
            CurveOptParams cp;
            cp.samples = 140;
            cp.w_mid = 1.0;
            cp.w_obs = 50.0;
            cp.w_smooth = 1.0;
            cp.w_len = 0.05;
            cp.safe_margin = 0.30;
            cp.clearance_cap = 2.0;
            cp.alpha = 2.0;
            cp.iterations = 150;
            cp.step_init = 0.08;

            CurveOptimizer opt;
            const auto cr = opt.optimize(poly, field, cp);
            check(cr.success, "曲线优化成功", cr.msg);
            if (cr.success) {
                std::vector<Vector3d> pts, tgs;
                cr.spline.sampleByArcLength(0.05, pts, tgs);
                const TurnStats ts_cur = turnStats(pts);
                std::printf("     曲线(优化后):   采样=%zu 转角中位=%.2f° 最大=%.2f° >20°拐点=%d\n",
                            pts.size(), ts_cur.median, ts_cur.max, ts_cur.over20);
                reportLateral("曲线", pts, sc);

                std::printf("     曲线: len=%.2f min_clr=%.3f mean_clr=%.3f 最大曲率=%.2f\n",
                            cr.length, cr.min_clearance, cr.mean_clearance, cr.max_curvature);

                // ---- 断言 1：曲线明显更居中 ----
                double lat_poly = 0, lat_cur = 0;
                for (const auto& p : poly) lat_poly += std::abs(sc.lateral(p));
                for (const auto& p : pts) lat_cur += std::abs(sc.lateral(p));
                lat_poly /= poly.size();
                lat_cur /= pts.size();
                check(lat_cur < lat_poly * 0.5,
                      "曲线横向偏移比折线小一半以上",
                      "poly=" + std::to_string(lat_poly).substr(0, 5) + " cur=" +
                          std::to_string(lat_cur).substr(0, 5));

                // ---- 断言 2/3：★「两侧等距」—— 分开统计中段与端点过渡段 ----
                //
                // 为什么要分段：起点终点是任务给定的位置（本例刻意偏在一侧 0.7m），
                // 曲线必须从那里出发/到达，因此**端点附近本来就不可能居中**。
                // 真正要检验的是「通道内部是否两侧等距」，所以中段才是有效指标。
                const size_t npts = pts.size();
                const size_t lo_i = npts / 4, hi_i = npts - npts / 4;

                double mid_lat = 0.0, mid_diff = 0.0, mid_max_diff = 0.0;
                int mid_cnt = 0;
                double all_diff = 0.0;
                for (size_t i = 0; i < npts; ++i) {
                    const double d = std::abs(sc.distLeft(pts[i]) - sc.distRight(pts[i]));
                    all_diff += d;
                    if (i >= lo_i && i < hi_i) {
                        mid_lat += std::abs(sc.lateral(pts[i]));
                        mid_diff += d;
                        mid_max_diff = std::max(mid_max_diff, d);
                        ++mid_cnt;
                    }
                }
                mid_lat /= std::max(1, mid_cnt);
                mid_diff /= std::max(1, mid_cnt);
                all_diff /= npts;

                std::printf("     居中度量（横向偏移 0 = 正中）:\n");
                std::printf("       中段(排除首尾25%%): |lat|均值=%.4f m  两侧距离差 均值=%.4f 最大=%.4f\n",
                            mid_lat, mid_diff, mid_max_diff);
                std::printf("       全程:              |lat|均值=%.4f m  两侧距离差 均值=%.4f\n",
                            lat_cur, all_diff);

                check(mid_lat < 0.02, "★通道中段严格居中（|lat| < 0.02m）",
                      "mid=" + std::to_string(mid_lat).substr(0, 6));
                check(mid_diff < 0.05, "★通道中段两侧严格等距（距离差 < 0.05m）",
                      "|dL-dR|=" + std::to_string(mid_diff).substr(0, 6));
                check(lat_cur < 0.12, "全程平均横向偏移 < 0.12m（含端点过渡）",
                      "all=" + std::to_string(lat_cur).substr(0, 6));

                // ---- 断言 4：安全没被牺牲 ----
                double mn_poly = 1e9;
                for (const auto& p : poly) mn_poly = std::min(mn_poly, field.clearance(p));
                check(cr.min_clearance >= mn_poly - 0.05, "曲线最小 clearance 不劣于折线",
                      "poly=" + std::to_string(mn_poly).substr(0, 5) + " cur=" +
                          std::to_string(cr.min_clearance).substr(0, 5));

                // ---- 断言 5：曲率连续（无折角）----
                check(ts_cur.over20 == 0, "曲线无 >20° 的折角",
                      "over20=" + std::to_string(ts_cur.over20));

                // ---- 断言 6：长度没有明显增长 ----
                const double len_poly = pathLength(poly);
                check(cr.length < len_poly * 1.15, "长度增长 < 15%",
                      "poly=" + std::to_string(len_poly).substr(0, 5) + " cur=" +
                          std::to_string(cr.length).substr(0, 5));
            }
        }
    }

    // ================= 场景 2：绕障碍，验证折角消失 =================
    {
        std::printf("\n[2] 绕两块错开障碍 —— 折线会有硬角，曲线应当平滑绕过\n\n");

        std::vector<std::pair<Vector3d, Vector3d>> boxes = {
            {Vector3d(0, 0, 0), Vector3d(0.3, 2.0, 1.0)},
            {Vector3d(2.0, 0, 0), Vector3d(0.3, 2.0, 1.0)},
            {Vector3d(4.0, 0, 0), Vector3d(0.6, 0.6, 1.0)},
        };
        FunctionClearanceField field([&](const Vector3d& p) {
            double m = 1e9;
            for (const auto& b : boxes) m = std::min(m, boxSdf(p, b.first, b.second));
            return m;
        });

        SearchParams sp;
        sp.resolution = 0.10;
        sp.robot_radius = 0.30;
        sp.clearance_weight = 0.10;
        sp.clearance_cap = 2.0;
        sp.alpha = 2.0;
        sp.box_margin = Vector3d(2, 3, 1);
        sp.goal_tolerance = 0.25;
        sp.max_expansions = 2000000;

        ClearanceAStar astar;
        const auto sr = astar.search(field, Vector3d(-1, 0, 0.5), Vector3d(6, 0, 0.5), sp);
        check(sr.success, "A* 成功", sr.msg);
        if (sr.success) {
            const auto poly = simplifyPath(sr.path);
            const TurnStats ts0 = turnStats(poly);
            std::printf("     折线: 顶点=%zu 转角中位=%.1f° 最大=%.1f° >20°拐点=%d\n", poly.size(),
                        ts0.median, ts0.max, ts0.over20);

            CurveOptParams cp;
            cp.samples = 160;
            cp.iterations = 150;
            cp.safe_margin = 0.30;
            CurveOptimizer opt;
            const auto cr = opt.optimize(poly, field, cp);
            check(cr.success, "曲线优化成功", cr.msg);
            if (cr.success) {
                std::vector<Vector3d> pts, tgs;
                cr.spline.sampleByArcLength(0.05, pts, tgs);
                const TurnStats ts1 = turnStats(pts);
                std::printf("     曲线: 采样=%zu 转角中位=%.2f° 最大=%.2f° >20°拐点=%d\n",
                            pts.size(), ts1.median, ts1.max, ts1.over20);
                std::printf("     曲线: len=%.2f min_clr=%.3f 最大曲率=%.2f\n", cr.length,
                            cr.min_clearance, cr.max_curvature);

                check(ts1.max < 15.0, "曲线最大转角 < 15°（折角基本消失）",
                      "max=" + std::to_string(ts1.max).substr(0, 5));
                check(cr.min_clearance >= 0.28, "曲线全程保持安全距离",
                      "min=" + std::to_string(cr.min_clearance).substr(0, 5));
            }
        }
    }

    // ================= 场景 3：极窄通道，验证不会鼓包撞墙 =================
    {
        std::printf("\n[3] 窄通道（半净宽 0.40m，机器人半径 0.30m）—— 不得鼓出\n\n");

        ChannelScene sc;
        sc.origin = Vector3d(0, 0, 0.8);
        sc.axis = Vector3d(1, 0, 0);
        sc.normal = Vector3d(0, 1, 0);
        sc.half_width = 0.40;

        FunctionClearanceField field([&](const Vector3d& p) { return sc.sdf(p); });

        SearchParams sp;
        sp.resolution = 0.05;
        sp.robot_radius = 0.30;
        sp.clearance_weight = 0.10;
        sp.box_margin = Vector3d(1, 1, 1);
        sp.goal_tolerance = 0.20;
        sp.max_expansions = 2000000;

        ClearanceAStar astar;
        const auto sr = astar.search(field, Vector3d(-3, 0.05, 0.8), Vector3d(3, 0.05, 0.8), sp);
        check(sr.success, "窄通道 A* 成功", sr.msg);
        if (sr.success) {
            const auto poly = simplifyPath(sr.path);
            CurveOptParams cp;
            cp.iterations = 120;
            cp.w_obs = 200.0;      // 窄缝里安全权重要更高
            CurveOptimizer opt;
            const auto cr = opt.optimize(poly, field, cp);
            check(cr.success, "窄通道曲线优化成功", cr.msg);
            if (cr.success) {
                std::vector<Vector3d> pts, tgs;
                cr.spline.sampleByArcLength(0.05, pts, tgs);
                double mn = 1e9, mx_lat = 0;
                for (const auto& p : pts) {
                    mn = std::min(mn, field.clearance(p));
                    mx_lat = std::max(mx_lat, std::abs(sc.lateral(p)));
                }
                std::printf("     曲线: min_clr=%.3f 最大横向偏移=%.3f（半宽 %.2f）\n", mn, mx_lat,
                            sc.half_width);
                check(mn > 0.05, "窄通道内仍保持在障碍外", "min=" + std::to_string(mn).substr(0, 5));
                check(mx_lat < sc.half_width, "未鼓出通道",
                      "lat=" + std::to_string(mx_lat).substr(0, 5));
            }
        }
    }

    std::printf("\n==================================================\n");
    if (g_fail == 0) {
        std::printf("全部通过。\n\n");
        return 0;
    }
    std::printf("%d 项失败。\n\n", g_fail);
    return 1;
}
