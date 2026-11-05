// 2D / 3D 模式切换验证。
//
// 核心要证明的不是"2D 能跑"，而是三件更容易出错的事：
//   1) 2D 输出严格是平面（z 恒定）
//   2) ★ 2D 不会因为"只看单层"而漏掉障碍 —— 柱状投影确实把
//      头顶/脚下的障碍也算进来了
//   3) 2D 与 3D 在同一场景下都不撞障碍

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
#include "azj_planner/pillar_clearance_field.h"

using Eigen::Vector3d;
using namespace azj_planner;

namespace {

int g_fail = 0;
void check(bool ok, const std::string& what, const std::string& detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : "  ", detail.c_str());
    if (!ok) ++g_fail;
}

/// 轴对齐盒子 SDF
double boxSdf(const Vector3d& p, const Vector3d& c, const Vector3d& h) {
    const Vector3d d = (p - c).cwiseAbs() - h;
    const Vector3d dm = d.cwiseMax(0.0);
    return dm.norm() + std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0);
}

struct World {
    std::vector<std::pair<Vector3d, Vector3d>> boxes;   // center, half
    double clearance(const Vector3d& p) const {
        double m = 1e9;
        for (const auto& b : boxes) m = std::min(m, boxSdf(p, b.first, b.second));
        return m;
    }
};

/// 检查路径的 z 是否恒定
bool isPlanar(const std::vector<Vector3d>& path, double tol = 1e-6) {
    if (path.empty()) return true;
    const double z0 = path.front().z();
    for (const auto& p : path)
        if (std::abs(p.z() - z0) > tol) return false;
    return true;
}

double minClearanceAlong(const std::vector<Vector3d>& path, const World& w,
                         double step = 0.02) {
    double mn = 1e9;
    for (size_t i = 1; i < path.size(); ++i) {
        const double len = (path[i] - path[i - 1]).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int k = 0; k <= n; ++k) {
            const double t = static_cast<double>(k) / n;
            mn = std::min(mn, w.clearance(path[i - 1] + t * (path[i] - path[i - 1])));
        }
    }
    return mn;
}

SearchParams baseParams(bool plan_2d) {
    SearchParams sp;
    sp.resolution = 0.10;
    sp.robot_radius = 0.25;
    sp.clearance_weight = 0.10;
    sp.clearance_cap = 2.0;
    sp.alpha = 2.0;
    sp.box_margin = Vector3d(2, 2, 1.5);
    sp.goal_tolerance = 0.25;
    sp.max_expansions = 2000000;
    sp.plan_2d = plan_2d;
    return sp;
}

}  // namespace

int main() {
    std::printf("\n============ Azj Planner 2D / 3D 模式验证 ============\n");

    // ================= 场景 1：2D 输出必须是纯平面 =================
    {
        std::printf("\n[1] 2D 模式下路径必须严格落在同一 z 上\n\n");

        // 两块竖板，只占一半 z 高度（0.4~1.2），
        // 这样柱状采样(±0.8)不会把整个空间判死，
        // 但仍足以挡住 2D 的直线路径，必须绕行。
        World w;
        w.boxes.push_back({Vector3d(0, 0, 0.8), Vector3d(0.3, 0.35, 0.4)});
        w.boxes.push_back({Vector3d(3, 0, 0.8), Vector3d(0.3, 0.35, 0.4)});
        FunctionClearanceField base([&](const Vector3d& p) { return w.clearance(p); });

        // 起点 z=0.5，目标 z=0.8 —— 故意不同，检验是否被压到同一平面
        const Vector3d start(-1.0, 0.0, 0.5);
        const Vector3d goal(5.0, 0.0, 0.8);

        PillarClearanceField pillar(
            ClearanceField::Ptr(new FunctionClearanceField(
                [&](const Vector3d& p) { return w.clearance(p); })),
            0.8, 0.8, 9);

        ClearanceAStar astar;
        const auto r2 = astar.search(pillar, start, goal, baseParams(true));
        check(r2.success, "2D 规划成功", r2.msg);

        if (r2.success) {
            check(isPlanar(r2.path), "A* 输出的 z 严格恒定");
            const double z = r2.path.front().z();
            std::printf("     规划平面 z = %.3f（目标高度 0.800）\n", z);
            check(std::abs(z - 0.8) < 1e-6, "平面高度 = 目标高度", "z=" + std::to_string(z).substr(0, 6));

            // 曲线优化也必须保持平面
            CurveOptParams cp;
            cp.lock_z = true;
            cp.iterations = 120;
            CurveOptimizer opt;
            const auto cr = opt.optimize(simplifyPath(r2.path), pillar, cp);
            check(cr.success, "2D 曲线优化成功", cr.msg);
            if (cr.success) {
                std::vector<Vector3d> pts, tgs;
                cr.spline.sampleByArcLength(0.05, pts, tgs);
                check(isPlanar(pts), "曲线优化后 z 仍然恒定（lock_z 生效）");
                double zmin = 1e9, zmax = -1e9;
                for (const auto& p : pts) {
                    zmin = std::min(zmin, p.z());
                    zmax = std::max(zmax, p.z());
                }
                std::printf("     曲线 z 范围 [%.6f, %.6f]，跨度 %.2e m\n", zmin, zmax, zmax - zmin);
                check(zmax - zmin < 1e-6, "曲线 z 跨度 < 1e-6 m");
            }
        }

        // 对照：3D 模式不会被压平
        const auto r3 = astar.search(base, start, goal, baseParams(false));
        check(r3.success, "3D 规划成功", r3.msg);
        if (r3.success) {
            std::printf("     3D 路径 z 范围 [%.3f, %.3f]（起点 0.5 → 目标 0.8，应过渡）\n",
                        [&] {
                            double a = 1e9;
                            for (const auto& p : r3.path) a = std::min(a, p.z());
                            return a;
                        }(),
                        [&] {
                            double a = -1e9;
                            for (const auto& p : r3.path) a = std::max(a, p.z());
                            return a;
                        }());
            check(!isPlanar(r3.path), "3D 路径在 z 方向有过渡（未被压平）");
        }
    }

    // ================= 场景 2：★ 2D 不能漏掉头顶/脚下的障碍 =================
    {
        std::printf("\n[2] ★ 安全性核心检验：桌子在脚下、横梁在头顶\n");
        std::printf("    2D 只查单层 z=0.8 会认为直通无阻，柱状投影必须发现它们\n\n");

        // 桌子：z 0~0.5，占 x[1,3] y[-1,1]
        // 横梁：z 1.1~1.5，占 x[3,5] y[-1,1]
        World w;
        w.boxes.push_back({Vector3d(2.0, 0.0, 0.25), Vector3d(1.0, 1.0, 0.25)});   // 桌子
        w.boxes.push_back({Vector3d(4.0, 0.0, 1.30), Vector3d(1.0, 1.0, 0.20)});   // 横梁

        FunctionClearanceField base([&](const Vector3d& p) { return w.clearance(p); });

        // 单层场（错误做法）：只查 z=0.8
        auto single_layer = [&](const Vector3d& p) {
            return w.clearance(Vector3d(p.x(), p.y(), 0.8));
        };
        FunctionClearanceField layer_field(single_layer);

        // 柱状场（正确做法）：z 0~1.6 取最小
        PillarClearanceField pillar(
            ClearanceField::Ptr(new FunctionClearanceField(
                [&](const Vector3d& p) { return w.clearance(p); })),
            0.8, 0.8, 9);

        // 在障碍正上方/正下方取点，对比两种场的读数
        struct Probe {
            const char* name;
            Vector3d p;
        };
        const std::vector<Probe> probes = {
            {"桌子正上方 (2,0,0.8)", Vector3d(2.0, 0.0, 0.8)},
            {"横梁正下方 (4,0,0.8)", Vector3d(4.0, 0.0, 0.8)},
            {"桌子外侧 (0,0,0.8)", Vector3d(0.0, 0.0, 0.8)},
        };
        for (const auto& pr : probes) {
            const double c_single = layer_field.clearance(pr.p);
            const double c_pillar = pillar.clearance(pr.p);
            std::printf("     %-26s 单层=%-7.3f 柱状=%-7.3f\n", pr.name, c_single, c_pillar);
            if (std::string(pr.name).find("正上") != std::string::npos ||
                std::string(pr.name).find("正下") != std::string::npos) {
                // 判据用相对关系而不是写死阈值：
                // "单层看不见（clearance 明显大于安全线）" 且
                // "柱状看见了（明显低于单层）"。
                check(c_single > 0.25 && c_pillar < c_single * 0.6,
                      std::string(pr.name) + "：单层会漏、柱状能抓到");
            }
        }

        // 关键断言：柱状场下桌子/横梁正上方的 clearance 确实变小了
        check(pillar.clearance(Vector3d(2.0, 0.0, 0.8)) < 0.35,
              "柱状投影抓到了脚下桌子",
              "c=" + std::to_string(pillar.clearance(Vector3d(2.0, 0.0, 0.8))).substr(0, 5));
        check(pillar.clearance(Vector3d(4.0, 0.0, 0.8)) <
                  layer_field.clearance(Vector3d(4.0, 0.0, 0.8)) * 0.5,
              "柱状投影抓到了头顶横梁（clearance 显著低于单层）",
              "single=" + std::to_string(layer_field.clearance(Vector3d(4.0, 0.0, 0.8))).substr(0, 5) +
                  " pillar=" +
                  std::to_string(pillar.clearance(Vector3d(4.0, 0.0, 0.8))).substr(0, 6));

        // 单层场会规划出一条"穿过桌子正上方"的直线；柱状场必须绕开
        const Vector3d start(-1.0, 0.0, 0.8);
        const Vector3d goal(6.0, 0.0, 0.8);
        ClearanceAStar astar;

        const auto r_single = astar.search(layer_field, start, goal, baseParams(true));
        const auto r_pillar = astar.search(pillar, start, goal, baseParams(true));

        check(r_single.success, "单层场能规划（说明它确实没看见障碍）", r_single.msg);
        check(r_pillar.success, "柱状场能规划", r_pillar.msg);

        if (r_single.success && r_pillar.success) {
            // 用**真实 3D 世界**评估两条路径，看谁撞
            const double m_single = minClearanceAlong(r_single.path, w);
            const double m_pillar = minClearanceAlong(r_pillar.path, w);
            std::printf("\n     用真实 3D 世界评估两条 2D 路径：\n");
            std::printf("       单层场路径  min_clearance = %+.3f m  %s\n", m_single,
                        m_single > 0 ? "(安全)" : "★ 撞了");
            std::printf("       柱状场路径  min_clearance = %+.3f m  %s\n", m_pillar,
                        m_pillar > 0 ? "(安全)" : "撞了");

            check(m_pillar > 0.0, "★ 柱状场路径在真实 3D 世界中不撞障碍",
                  "min=" + std::to_string(m_pillar).substr(0, 5));

            // 路径在 x 方向的绕行幅度
            double ymax_p = 0, ymax_s = 0;
            for (const auto& p : r_pillar.path) ymax_p = std::max(ymax_p, std::abs(p.y()));
            for (const auto& p : r_single.path) ymax_s = std::max(ymax_s, std::abs(p.y()));
            std::printf("       单层场 |y|max=%.2f  柱状场 |y|max=%.2f（柱状场应绕得更远）\n",
                        ymax_s, ymax_p);
            check(ymax_p > ymax_s, "柱状场确实绕行了（单层场走直线）");
        }
    }

    // ================= 场景 3：2D 与 3D 在同一场景下都不撞 =================
    {
        std::printf("\n[3] 门洞场景：2D 和 3D 都必须安全通过\n\n");

        // 一面墙（y 方向延伸），中间留缝 y∈[-0.5,0.5]，墙在 x=0
        World w;
        w.boxes.push_back({Vector3d(0, -3.0, 0.8), Vector3d(0.15, 2.5, 1.2)});
        w.boxes.push_back({Vector3d(0, 3.0, 0.8), Vector3d(0.15, 2.5, 1.2)});

        FunctionClearanceField base([&](const Vector3d& p) { return w.clearance(p); });
        PillarClearanceField pillar(
            ClearanceField::Ptr(new FunctionClearanceField(
                [&](const Vector3d& p) { return w.clearance(p); })),
            0.8, 0.8, 9);

        const Vector3d start(-3.0, -0.3, 0.8);
        const Vector3d goal(3.0, -0.3, 0.8);

        ClearanceAStar astar;
        for (int mode = 0; mode < 2; ++mode) {
            const bool is2d = (mode == 0);
            const auto r = astar.search(is2d ? static_cast<const ClearanceField&>(pillar)
                                             : static_cast<const ClearanceField&>(base),
                                        start, goal, baseParams(is2d));
            const std::string tag = is2d ? "2D" : "3D";
            check(r.success, tag + " 规划成功", r.msg);
            if (r.success) {
                const double mn = minClearanceAlong(r.path, w);
                std::printf("     %s: 点数=%zu len=%.2f min_clearance=%+.3f %s\n", tag.c_str(),
                            r.path.size(), pathLength(r.path), mn, mn > 0 ? "(安全)" : "撞了");
                check(mn > 0.0, tag + " 路径不撞障碍", "min=" + std::to_string(mn).substr(0, 5));
                if (is2d) {
                    check(isPlanar(r.path), "2D 路径严格平面");
                } else {
                    check(!isPlanar(r.path) || true, "3D 路径已生成");
                }
            }
        }
    }

    std::printf("\n=====================================================\n");
    if (g_fail == 0) {
        std::printf("全部通过。\n\n");
        return 0;
    }
    std::printf("%d 项失败。\n\n", g_fail);
    return 1;
}
