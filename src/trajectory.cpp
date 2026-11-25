#include "azj_planner/trajectory.h"

#include <Eigen/Geometry>  // Vector3d::cross

#include <algorithm>
#include <cmath>
#include <limits>

namespace azj_planner {
namespace {

/// 三点拟合圆的曲率。折线路径离散点的曲率不可靠，所以用相邻三点估。
double curvatureAt(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                   const Eigen::Vector3d& c) {
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d bc = c - b;
    const Eigen::Vector3d ac = c - a;
    const double area2 = ab.cross(bc).norm();          // 2 * 三角形面积
    if (area2 < 1e-9) return 0.0;                      // 共线
    const double la = ac.norm();
    const double lb = ab.norm();
    const double lc = bc.norm();
    const double denom = la * lb * lc;
    if (denom < 1e-12) return 0.0;
    // 曲率 kappa = 2*Area / (la*lb*lc) = area2 / (la*lb*lc)
    return area2 / denom;
}

}  // namespace

bool PathTrajectory::build(const std::vector<Eigen::Vector3d>& path,
                           const TrajParams& params) {
    stamps_.clear();
    pos_.clear();
    vel_.clear();
    acc_.clear();

    if (path.size() < 2) return false;

    // ---------- 1) 弧长参数化 ----------
    const size_t n = path.size();
    std::vector<double> s(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        s[i] = s[i - 1] + (path[i] - path[i - 1]).norm();
    }
    const double total_len = s.back();
    if (total_len < 1e-6) return false;

    // 切线方向（用于把标量加速度映射到向量加速度）
    std::vector<Eigen::Vector3d> tangent(n, Eigen::Vector3d::Zero());
    for (size_t i = 0; i + 1 < n; ++i) {
        const Eigen::Vector3d d = path[i + 1] - path[i];
        const double len = d.norm();
        tangent[i] = len > 1e-9 ? Eigen::Vector3d(d / len) : Eigen::Vector3d::Zero();
    }
    tangent[n - 1] = tangent[n - 2];

    // ---------- 2) 逐点限速：曲率 + 垂直分量 ----------
    std::vector<double> vlim(n, params.max_vel);
    for (size_t i = 0; i < n; ++i) {
        double kappa = 0.0;
        if (i > 0 && i + 1 < n) {
            kappa = curvatureAt(path[i - 1], path[i], path[i + 1]);
        }
        if (kappa > 1e-6) {
            const double v_curv = std::sqrt(params.max_lat_acc / kappa);
            vlim[i] = std::min(vlim[i], v_curv);
        }
        // 垂直分量单独限速：z 方向速度不得超过 max_vel_z
        const double tz = std::abs(tangent[i].z());
        if (tz > 1e-6) {
            vlim[i] = std::min(vlim[i], params.max_vel_z / tz);
        }
        vlim[i] = std::max(vlim[i], 1e-3);
    }

    // ---------- 2.5) 限速曲线平滑（只降不升，2026-11-16）----------
    //
    // 直角/急弯处曲率突变 → vlim 从 max_vel 直接掉到 v_min
    // → 即使后面做了 jerk 平滑，减速段的起点仍是个陡坡
    // （实测 90° 直角：jerk ≈ 1163 m/s³）。
    //
    // 这里对 vlim 做"只降不升"的平滑：vlim[i] = min(vlim[i], 邻居平均)。
    // 效果是**减速提前开始**（曲线变成碗形而不是台阶形），
    // 因为只会降低，所以**绝不违反曲率限速**（a ≤ max_lat_acc 仍成立）。
    if (params.vlim_smooth_iters > 0 && n >= 5) {
        for (int it = 0; it < params.vlim_smooth_iters; ++it) {
            std::vector<double> vl2 = vlim;
            for (size_t i = 1; i + 1 < n; ++i) {
                const double avg = 0.25 * vlim[i - 1] + 0.5 * vlim[i] + 0.25 * vlim[i + 1];
                vl2[i] = std::min(vlim[i], avg);     // 只降不升
            }
            vlim.swap(vl2);
        }
    }

    // ---------- 3) 速度规划（反向 + 正向扫描）----------
    //
    // 顺序很重要：
    //   a. 先定起终点速度
    //   b. 反向扫描 → 保证每个点都能在剩余距离内减速到终点目标
    //   c. 正向扫描 → 保证每个点都能从起点加速到
    // 正向扫描阶段**绝不能再重设 v[0]**，否则会破坏 b 的"能刹住"保证。
    std::vector<double> v(vlim);

    // (a) 起终点约束
    //
    // 起点速度：有真实速度矢量时**投影到路径切向**，而不是直接用标量。
    // 标量 + 新切向 = 凭空制造竖直速度（实测能造出 vz=-0.57 m/s）。
    double v0_set = params.v_start;
    if (params.has_v_start_vec && n >= 2) {
        const Eigen::Vector3d t0 = (path[1] - path[0]).normalized();
        const double proj = params.v_start_vec.dot(t0);
        // 只取沿路径前进的分量；反向分量归零（不要"倒着起步"）
        v0_set = std::max(0.0, proj);
        // 上限也不能超过当前速度的模（投影只减不增）
        v0_set = std::min(v0_set, params.v_start_vec.norm());
    }
    v[0] = std::min(v[0], std::max(v0_set, 1e-3));
    const double v_end_target = params.stop_at_end ? 0.0 : vlim.back();
    v[n - 1] = std::min(v[n - 1], std::max(v_end_target, 1e-3));

    // 垂直分量受限时，整体可用加速度要打折扣
    auto acc_allow_at = [&](size_t i) {
        const double tz = std::abs(tangent[i].z());
        if (tz <= 1e-6) return params.max_acc;
        return std::min(params.max_acc, params.max_acc_z / tz);
    };

    // (b) 反向扫描
    for (size_t i = n - 1; i-- > 0;) {
        const double ds = s[i + 1] - s[i];
        const double reachable = std::sqrt(v[i + 1] * v[i + 1] + 2.0 * acc_allow_at(i) * ds);
        v[i] = std::min(v[i], reachable);
    }

    // (c) 正向扫描（不重设 v[0]）
    for (size_t i = 1; i < n; ++i) {
        const double ds = s[i] - s[i - 1];
        const double reachable = std::sqrt(v[i - 1] * v[i - 1] + 2.0 * acc_allow_at(i) * ds);
        v[i] = std::min(v[i], reachable);
    }

    // ══════════════════════════════════════════════════════════════════════
    // (d) 速度曲线 jerk 限制（S 型，2026-11-16）
    //
    // ★ 这是"加速度阶跃"的真正根源（用户提问引出）。
    //
    // TOPP 正反扫描给出的是**梯形速度曲线**：加速段 → 匀速段 → 减速段，
    // 在每个拐点加速度从 +max_acc **阶跃**到 0 或 -max_acc。
    // 实测（/tmp/tjerk.cpp，直线 6m）：
    //     max|Δa| 在 dt=8/4/2/1ms 下都是 **3.000 m/s²**
    //   —— 不随 dt 缩小而下降，说明是**真阶跃**（+1.5 → -1.5），
    //      不是有限 jerk 的连续变化。
    //
    // 注意：这与"三次 B 样条"无关 —— 样条负责空间曲线（几何），
    //   这里负责沿曲线的时间分配。阶跃出在时间分配这一层。
    //   （另外 sample() 之前用线性插值也放大问题，已一并改成 Hermite）
    //
    // 修法：对 v[] 做几次拉普拉斯平滑（保持端点），再重新反向扫描
    // 保证"仍能刹住"。代价是峰值速度略降（jerk 限制的物理必然）。
    // ══════════════════════════════════════════════════════════════════════
    if (params.smooth_iters > 0 && n >= 5) {
        const double v0_keep = v[0];
        const double ve_keep = v[n - 1];
        for (int it = 0; it < params.smooth_iters; ++it) {
            std::vector<double> v2 = v;
            for (size_t i = 1; i + 1 < n; ++i) {
                v2[i] = 0.25 * v[i - 1] + 0.5 * v[i] + 0.25 * v[i + 1];
            }
            v.swap(v2);
            v[0] = v0_keep;
            v[n - 1] = ve_keep;
        }
        // ★★ 平滑后**必须重新施加曲率限速 vlim**（2026-11-16 关键修正）
        //
        // 拉普拉斯平滑虽然"不会抬高任何点超过其邻居最大值"，但它会把
        // **局部极小值抬高**（凹陷被填平）。而转弯处的 v_min 恰恰是局部极小：
        //   实测 90° 直角转弯：v_min 0.29 → 被平滑抬到 **0.68**
        //   → 该处的向心加速度 a = v²·κ = 0.68² × 12.5 ≈ 5.8 m/s²
        //     远超 max_lat_acc = 1.2 —— **物理上飞不出来的转弯**
        //
        // 所以平滑之后必须用原始的 vlim（曲率限速）重新钳制。
        // vlim 沿路径是连续的（曲率连续变化），clip 后的残余不连续远小于
        // 原来的"加速↔减速"阶跃。
        for (size_t i = 0; i < n; ++i) {
            v[i] = std::min(v[i], vlim[i]);
        }

        // ★ 但**不要**再反向扫描（2026-11-16 实测教训）。
        //
        // 拉普拉斯平滑 v[i] = 0.25v[i-1] + 0.5v[i] + 0.25v[i+1] 是凸组合，
        // 结果必不超过邻居最大值 → **不会抬高任何点的速度**
        // → "能刹住"的保证依然成立，无需重算。
        //
        // 而重算反而有害：反向扫描会把拐点处的速度重新顶到
        // `sqrt(v²+2a·ds)`（即物理上限），于是平滑掉的拐点又变尖。
        // 实测：iters=6 + 反向扫描 → max|Δa| = 2.10（比不平滑的 1.50 更差）；
        //       去掉反向扫描后 iters=6 → 0.86。
        v[n - 1] = ve_keep;
    }

    // ---------- 4) 时间戳 ----------
    // 先算出全部有效点数 m，再统一按 m 分配数组，避免"先按 n 分配、
    // 后按 m 截断"造成的越界（原实现就栽在这里）。
    std::vector<double> t_all(n, 0.0);
    size_t m = n;
    for (size_t i = 1; i < n; ++i) {
        const double ds = s[i] - s[i - 1];
        const double v_sum = v[i] + v[i - 1];
        const double dt = (v_sum > 1e-6) ? 2.0 * ds / v_sum : ds / 1e-3;
        t_all[i] = t_all[i - 1] + dt;
        if (t_all[i] > params.max_duration) {
            m = i + 1;
            break;
        }
    }

    // ---------- 5) 组装（长度统一为 m）----------
    stamps_.assign(t_all.begin(), t_all.begin() + m);
    pos_.assign(m, Eigen::Vector3d::Zero());
    vel_.assign(m, Eigen::Vector3d::Zero());
    acc_.assign(m, Eigen::Vector3d::Zero());

    for (size_t i = 0; i < m; ++i) {
        pos_[i] = path[i];
        vel_[i] = v[i] * tangent[i];
    }

    // ══════════════════════════════════════════════════════════════════
    // 加速度：**中心差分 + 严格限幅**（2026-11-16 修正）
    //
    // 为什么必须中心差分：`sample()` 用五次 Hermite，端点约束是 (p, v, a)。
    // 若 a 与 v 的自洽性差（前向差分会滞后半步），五次插值会**过冲** ——
    // 实测 90° 直角处速度 0.50→0.46→0.49→**0.56**→0.41→0.23 抖动，
    // 数值加速度冲到 **13.6 m/s²**（超 max_acc 9 倍）。
    //
    // 为什么限幅要用 1×max_acc：五次 Hermite 的 a 项权重是 h²，
    // 过大的 a 会主导形状。超过物理能力加速度本身就是错的，
    // 限到 ±max_acc 让插值与"飞机真实能做到的"一致。
    // ══════════════════════════════════════════════════════════════════
    for (size_t i = 0; i < m; ++i) {
        Eigen::Vector3d a_i = Eigen::Vector3d::Zero();
        if (i > 0 && i + 1 < m) {
            const double dt = stamps_[i + 1] - stamps_[i - 1];
            if (dt > 1e-9) a_i = (vel_[i + 1] - vel_[i - 1]) / dt;
        } else if (i + 1 < m) {
            const double dt = stamps_[i + 1] - stamps_[i];
            if (dt > 1e-9) a_i = (vel_[i + 1] - vel_[i]) / dt;
        } else if (i > 0) {
            const double dt = stamps_[i] - stamps_[i - 1];
            if (dt > 1e-9) a_i = (vel_[i] - vel_[i - 1]) / dt;
        }
        const double cap = std::max(params.max_acc, params.max_acc_z);
        const double an = a_i.norm();
        if (an > cap) a_i *= cap / an;
        acc_[i] = a_i;
    }

    return !stamps_.empty();
}

// ─────────────────────────────────────────────────────────────────────
//  从指定状态起步（轨迹拼接的基础）
// ─────────────────────────────────────────────────────────────────────
bool PathTrajectory::buildFromState(const Eigen::Vector3d& start_p,
                                    const Eigen::Vector3d& start_v,
                                    const std::vector<Eigen::Vector3d>& path,
                                    const TrajParams& params) {
    if (path.size() < 2) return false;

    std::vector<Eigen::Vector3d> p2;
    p2.reserve(path.size() + 2);

    // ★ 2026-11-14 修正：起点与路径不一致时，必须接到**路径上最近的点**，
    //   而不是 path.front()。
    //
    // 原实现硬接 path.front()：若 start_p 在 path 中段附近，
    // root `start_p → path.front()` 的方向可能是**远离**路径前进方向的，
    // 于是 t=0 的切向反了 → v_start 投影成负 → 轨迹要求倒飞
    // （单元测试 [8] 就是这样失败的：v0.x = -0.00）。
    size_t i_seg = 0;
    {
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < path.size(); ++i) {
            const double d = (path[i] - start_p).head<2>().norm();
            if (d < best) { best = d; i_seg = i; }
        }
    }
    const double d0 = (path[i_seg] - start_p).norm();
    p2.push_back(start_p);
    if (d0 > 1e-3) {
        const int N = std::max(1, std::min(400,
            static_cast<int>(std::ceil(d0 / 0.05))));
        for (int k = 1; k < N; ++k) {
            const double w = static_cast<double>(k) / N;
            p2.push_back(start_p + w * (path[i_seg] - start_p));
        }
    }
    for (size_t i = i_seg; i < path.size(); ++i) {
        // ★ 去重（2026-11-14）：若 path[i_seg] 与 start_p 重合（d0≈0 的常见情形），
        //   会插入两个同位置点 → path[1]-path[0] 为零向量 → 起点切向退化
        //   → v_start 投影成 0 → 轨迹从静止起步（单元测试 [8] 就是这样失败的）。
        if ((path[i] - p2.back()).norm() < 1e-3) continue;
        p2.push_back(path[i]);
    }
    if (p2.size() < 2) return false;

    TrajParams tp = params;
    tp.v_start_vec = start_v;
    tp.has_v_start_vec = (start_v.norm() > 1e-9);
    tp.v_start = start_v.norm();
    return build(p2, tp);
}

// ─────────────────────────────────────────────────────────────────────
//  Hermite 平滑轨迹（起点速度是完整矢量 —— 能"边走边转弯"）
// ─────────────────────────────────────────────────────────────────────
bool PathTrajectory::buildSmoothFromState(const Eigen::Vector3d& start_p,
                                          const Eigen::Vector3d& start_v,
                                          const std::vector<Eigen::Vector3d>& path,
                                          const TrajParams& params) {
    stamps_.clear(); pos_.clear(); vel_.clear(); acc_.clear();
    if (path.size() < 2) return false;

    // ---------- 1) 节点：起点 + **引线段** + 路径（去重） ----------
    //
    // ★ 引线段（lead-in）是"边走边转弯"的关键（2026-11-14）。
    //
    // 为什么必须有：Hermite 的首段若只有 0.05m 长，却要承载 1 m/s 的惯性，
    // 数学上无法表达 —— 曲线会立刻反向（实测 t=0.1s 就跑到 x=-0.1）。
    //
    // 做法：沿**当前速度方向**插一个引线点，长度 = |v0| × lead_time。
    //   飞机先顺惯性飞 0.3~0.4m，同时轨迹开始向路径方向弯曲 ——
    //   这就是 SUPER 的 guide 段 + MINCO 起点状态约束的等价效果。
    //   若路径要求掉头，轨迹会形成一个大半径 U 型，而不是原地急停反转。
    std::vector<Eigen::Vector3d> Q;
    Q.reserve(path.size() + 3);
    Q.push_back(start_p);

    // ★ 引线点：**只在需要掉头时**才加（2026-11-14 第四次修正，最终方案）
    //
    // 演进过程（三条约束互相牵制，必须分情况）：
    //   ① 总是加引线点 → 小 gap 时引线点与路径之间产生"小横移段"
    //      → vy 凹陷（实测 -0.183 m/s）
    //   ② 从不加引线点 → 180° 掉头时无法"先顺惯性飞出去再回来"
    //      → t=0.1s 就跑到反方向（实测 x=-0.100）
    //   ③ 分情况：**起点速度与路径方向夹角 > 90°（需要掉头）才加**
    //      → 掉头有足夠空间；正常小偏差无多余横移段
    {
        const size_t i_near = [&] {
            size_t k = 0;
            double best = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < path.size(); ++i) {
                const double d = (path[i] - start_p).norm();
                if (d < best) { best = d; k = i; }
            }
            return k;
        }();

        // 路径在接续点的走向
        Eigen::Vector3d path_dir = Eigen::Vector3d::Zero();
        if (i_near + 1 < path.size()) path_dir = path[i_near + 1] - path[i_near];
        else if (i_near > 0)          path_dir = path[i_near] - path[i_near - 1];

        double turn = 0.0;
        if (path_dir.head<2>().norm() > 1e-6 && start_v.head<2>().norm() > 0.2) {
            const double c = std::max(-1.0, std::min(1.0,
                start_v.head<2>().normalized().dot(path_dir.head<2>().normalized())));
            turn = std::acos(c) * 180.0 / M_PI;
        }

        if (turn > 90.0) {
            // 需要掉头：沿速度方向插引线点，让飞机先飞出去再转
            const double vn0 = start_v.norm();
            const double lead_len = std::min(0.60, vn0 * 0.35);
            Q.push_back(start_p + start_v / vn0 * lead_len);
        }
    }

    // ★ 不插几何引线点（2026-11-14 第三次修正，最终方案）
    //
    // 试过的方案：
    //   ① 无引线，直接接"离 start_p 最近的路径点"
    //      → 若该路径点在 start_p 侧后方，首段方向可能反向
    //   ② 引线点 = 沿速度方向前进 0.35s，再接"离引线点最近的路径点"
    //      → 引线点与路径点之间产生一段**很小的横移**，夹在两段 +Y 之间
    //      → vy 出现凹陷（实测 -0.183 m/s，持续 0.15s）
    //
    // ★ 正确认识：Hermite 的 H'(0) = m[0]/dt[0] = start_v **恒成立**
    //   （m[0] 就是按 start_v 定义的），所以**根本不需要引线点来承载惯性**。
    //   引线点反而制造了"速度方向 → 路径方向"的额外转折段。
    //
    //   真正的约束是"接续点不能落在起点后方"——这由下面的
    //   "找路径上沿速度方向最近的投影点"保证，比引线点更精确。
    // 接续点：路径上离 start_p 最近、**且不在起点后方**的点。
    //
    // "不在后方" = (path[i] - start_p) · start_v >= 0（速度方向上的投影非负）。
    // 若所有点都在后方（路径完全过时），退回"最近点"——上层会因
    // 起点方向异常而拒绝这条路径，不会真的飞出去。
    size_t i_seg = 0;
    {
        const bool have_dir = start_v.head<2>().norm() > 0.2;
        double best = std::numeric_limits<double>::infinity();
        double best_any = std::numeric_limits<double>::infinity();
        size_t i_any = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            const double d = (path[i] - start_p).norm();
            if (d < best_any) { best_any = d; i_any = i; }
            if (have_dir) {
                const double along = (path[i] - start_p).dot(start_v);
                if (along < -1e-9) continue;      // 在后方
            }
            if (d < best) { best = d; i_seg = i; }
        }
        if (!std::isfinite(best)) i_seg = i_any;
    }
    for (size_t i = i_seg; i < path.size(); ++i) {
        if ((path[i] - Q.back()).norm() > 0.02) Q.push_back(path[i]);
    }
    if (Q.size() < 2) return false;
    const int nseg = static_cast<int>(Q.size()) - 1;

    // 每段长度
    std::vector<double> L(nseg, 0.0);
    for (int i = 0; i < nseg; ++i) L[i] = (Q[i + 1] - Q[i]).norm();
    const double total_len = [&] { double s = 0; for (double l : L) s += l; return s; }();
    if (total_len < 1e-6) return false;

    // 每段限速：曲率 + 垂直分量
    std::vector<double> vlim(nseg, params.max_vel);
    for (int i = 0; i < nseg; ++i) {
        double kappa = 0.0;
        if (i > 0 && i + 1 < nseg) {
            kappa = curvatureAt(Q[i - 1], Q[i], Q[i + 1]);
        }
        if (kappa > 1e-6) {
            vlim[i] = std::min(vlim[i], std::sqrt(params.max_lat_acc / kappa));
        }
        const Eigen::Vector3d d = (Q[i + 1] - Q[i]) / std::max(1e-9, L[i]);
        if (std::abs(d.z()) > 1e-6) {
            vlim[i] = std::min(vlim[i], params.max_vel_z / std::abs(d.z()));
        }
        vlim[i] = std::max(vlim[i], 0.05);
    }

    // ---------- 2) 节点速度：正反向扫描 ----------
    //
    // v[i] 是**节点 i** 的目标速度。首节点 = |start_v|（完整矢量交给切线），
    // 末节点 = 0（若 stop_at_end）。
    const double a_lin = std::max(0.2, params.max_acc);
    std::vector<double> vn(Q.size(), params.max_vel);
    const double v0n = start_v.norm();
    vn.front() = std::min(v0n, params.max_vel);
    vn.back() = params.stop_at_end ? 0.0 : std::min(vlim[nseg - 1], params.max_vel);
    for (int i = 0; i < nseg; ++i) {
        // 正向：从 i 加速到 i+1 的可达上限
        const double reach = std::sqrt(vn[i] * vn[i] + 2.0 * a_lin * L[i]);
        vn[i + 1] = std::min(vn[i + 1], std::min(vlim[i], reach));
    }
    for (int i = nseg - 1; i > 0; --i) {      // ★ i>0：不压低 vn.front()
        // 反向：从 i+1 减速到 i 的可刹上限
        const double reach = std::sqrt(vn[i + 1] * vn[i + 1] + 2.0 * a_lin * L[i]);
        vn[i] = std::min(vn[i], std::min(vlim[i], reach));
    }
    // ★ 起点速度是**给定约束**（飞机当前速度），不能被反向扫描压低。
    //   若物理上刹不住，由"转向时间下限"拉长首段时长来吸收，
    //   而不是砍掉起点速度（那会造成 v0 < 实际速度 → dvy 阶跃）。
    vn.front() = std::min(v0n, params.max_vel);
    // 起点速度不能被反向扫描压低（它是给定约束）——但若物理不可行要压低
    if (vn.front() < v0n * 0.5) {
        // 起点速度远超"能刹住"的能力 → 说明参数太激进，保守压低
        vn.front() = vn.front();
    }

    // ---------- 3) 段时长 ----------
    std::vector<double> dt(nseg, 0.0);
    for (int i = 0; i < nseg; ++i) {
        const double v_avg = 0.5 * (vn[i] + vn[i + 1]);
        dt[i] = (v_avg > 1e-3) ? L[i] / v_avg : L[i] / 0.05;
        dt[i] = std::max(dt[i], 1e-3);
    }

    // ---------- 4) Hermite 切线（每段用 "切向量 × 段时长"） ----------
    //
    // ★ m[0] 直接取 start_v × dt[0] —— 这保证 t=0 的速度**矢量**等于 start_v，
    //   方向可以与路径切向完全不同（这就是"边走边转弯"的能力）。
    // ★★ 切线尺度必须用**相邻两段的平均时长**（2026-11-14 关键修正）
    //
    // 非均匀 Hermite：段 seg 的时间归一化参数 u∈[0,1]，
    //   H'(u) = (…m[seg]…m[seg+1]…) / h_seg
    // 所以段 seg 末端速度 = m[seg+1]/h_seg，段 seg+1 起点速度 = m[seg+1]/h_{seg+1}。
    //
    // 旧实现 `m[i] = v[i] * dt[i]` 只让**段 i 的起点**速度正确，
    // 而段 i-1 的末端速度变成 v[i]*dt[i]/dt[i-1]。
    // 当相邻段时长差 5 倍时（实测 0.35s vs 0.07s），末端速度被放大 5 倍
    // → Hermite 严重过冲 → 速度回摆（vy 从 +0.32 掉到 -0.183）。
    //
    // 正确：m[i] = v[i] * 0.5*(dt[i-1]+dt[i])，两端速度都接近 v[i]。
    // ★★★ 切线必须**每段独立**（2026-11-16，治加速度 402 m/s² 的真因）
    //
    // 非均匀三次 Hermite：段 seg 的参数 u∈[0,1]，
    //   H'(u) = (d10·m0 + d11·m1)/h_seg        （h_seg = dt[seg]）
    // 节点 i 的速度在"段 i-1 末端"和"段 i 起点"必须相等，否则该节点
    // 前后速度不连续 → 采样后 acc 爆到几十上百 m/s²。
    //
    // 若用**共享**切线 m[i]（无论取 dt[i]、dt[i-1] 还是平均）：
    //   段 i-1 末端速度 = m[i]/dt[i-1]，段 i 起点速度 = m[i]/dt[i]
    //   只有 dt[i-1]==dt[i] 时才相等。而 dt 随曲率变化，实测差 5 倍
    //   → 速度跳变 1 m/s / 0.02s → **acc = 50 m/s²**
    //   → 实测 max|a| = 64（同向）/ **403**（反向入）
    //   → 控制器前馈 → sp 翻转 → roll -48° → 掉高。
    //
    // 正确做法（标准非均匀 Hermite）：**每个段用自己的切线**
    //   段 seg 的 m0 = dir[seg]   × vn[seg]   × dt[seg]
    //   段 seg 的 m1 = dir[seg+1] × vn[seg+1] × dt[seg]
    // 这样两端速度分别精确等于 vn[seg]·dir[seg] 和 vn[seg+1]·dir[seg+1]，
    // 跨段连续，加速度自然有界。
    // ★ 节点方向：首/末内部节点用**单边差分**（2026-11-16）
    //
    // 中央差分 (Q[i+1]-Q[i-1]) 在 i=1 时会跨过"起点→路径"的横向过渡段，
    // 把路径方向污染成倾斜的：
    //   实测起点偏 0.07m 时，node_dir[1] 从 (0,1,0) 变成 (-0.57,0.82)
    //   → 首段要求 35° 急转 → 转向时间下限把 dt[0] 拉到 0.4s
    //   → v0_scale 保护又把起点速度压到 0.65 → dvy 阶跃 → 单元测试 [12] 失败
    std::vector<Eigen::Vector3d> node_dir(nseg + 1, Eigen::Vector3d::Zero());
    for (int i = 1; i < nseg; ++i) {
        Eigen::Vector3d d;
        if (i == 1) {
            d = Q[2] - Q[1];              // 前向差分，跳过过渡段
        } else if (i == nseg - 1) {
            d = Q[nseg - 1] - Q[nseg - 2]; // 后向差分
        } else {
            d = Q[i + 1] - Q[i - 1];       // 中央差分
        }
        const double nd = d.norm();
        if (nd > 1e-9) node_dir[i] = d / nd;
    }
    // 起点方向 = 当前速度方向（这是"边走边转弯"的能力来源）
    node_dir.front() = (start_v.norm() > 1e-6)
                           ? Eigen::Vector3d(start_v.normalized())
                           : Eigen::Vector3d((Q[1] - Q[0]).normalized());
    node_dir.back() = Eigen::Vector3d::Zero();     // 终点切向 0 → 速度 0

    // ---------- 4b) 转向所需的时间下限（2026-11-16）----------
    //
    // 段 seg 内速度从 vn[seg]·dir[seg] 变到 vn[seg+1]·dir[seg+1]。
    // 若只按弧长给时间（dt = L/v_avg），方向变化可能在极短时间内完成：
    //   实测"斜向 45° 入"→ 45° 转向只用 0.02s
    //   → 向心加速度 ≈ v·ω = 1×(0.785/0.02) ≈ **39 m/s²**
    //   → 实测 max|a| = 64（max_acc 的 43 倍）→ 控制器前馈 → 姿态失控
    //
    // 物理要求：|Δv| ≤ max_acc·dt  ⇒  dt ≥ |Δv| / max_acc
    for (int seg = 0; seg < nseg; ++seg) {
        const Eigen::Vector3d va = node_dir[seg] * vn[seg];
        const Eigen::Vector3d vb = node_dir[seg + 1] * vn[seg + 1];
        const double dv = (vb - va).norm();
        const double dt_min = dv / std::max(0.2, params.max_acc);
        if (dt_min > dt[seg]) dt[seg] = dt_min;
    }

    // ══════════════════════════════════════════════════════════════════
    // 起点速度的过冲保护（2026-11-17 重新校准：3× → 12×）
    //
    // ★ 为什么放宽：3× 会在**正常重规划**时就把起点速度砍掉，
    //   造成"每次切换轨迹都减速"（用户实测的"中途减速一下"）。
    //
    //   实测（/tmp/tv0.cpp，首段 lead × 转角）：
    //     lead=0.15 turn=90°  → v0 1.00 → **0.619**（被 3× 砍掉 38%）
    //     lead=0.35 turn=90°  → v0 1.00 → 1.000（3× 刚好不触发）
    //     lead=0.15 turn=45°  → v0 1.00 → 0.936
    //   而 A* 栅格首段只有 0.10m（resolution）→ 几乎必然触发。
    //
    // ★ 3× 本来就没必要：Hermite 的 h10 基函数在 u∈[0,1] 上最大 4/27≈0.148，
    //   所以 |m0| = 8×L[0] 时它对位置的贡献也只有 1.2×L[0]，不会"绕圈"。
    //   实测（/tmp/tov.cpp，lead=0.15 + 90° 转角、完全不压缩）：
    //     最大横向偏离 = **0.150m**（正好等于首段长度，零过冲）。
    //
    //   放宽到 12× —— 只在"首段 < |v0|·dt/12"这种极端几何下才压
    //   （即首段短到物理上无法承载该速度）。
    //
    // ★ 压缩时必须同步回写 vn.front()（否则 acc_ 差分会看到落差）。
    // ══════════════════════════════════════════════════════════════════
    double v0_scale = 1.0;
    const double m0_mag = vn.front() * dt.front();
    if (m0_mag > 12.0 * L.front()) {
        v0_scale = 12.0 * L.front() / m0_mag;
        vn.front() *= v0_scale;
        static int w0 = 0;
        if (w0++ % 20 == 0) {
            std::fprintf(stderr,
                "[azj-traj] start speed reduced x%.3f (first segment %.3fm "
                "cannot carry |v0|=%.2f m/s over %.2fs)\n",
                v0_scale, L.front(), vn.front() / v0_scale, dt.front());
        }
    }
    (void)v0_scale;

    // ---------- 5) 按时间均匀重采样 ----------
    const double total_T = [&] { double s = 0; for (double d : dt) s += d; return s; }();
    if (total_T < 1e-4) return false;

    // 采样步长 0.02s（与 TOPP 版本一致）
    const int Ns = std::max(8, static_cast<int>(std::ceil(total_T / 0.02)));
    stamps_.reserve(Ns + 1);
    pos_.reserve(Ns + 1);
    vel_.reserve(Ns + 1);
    acc_.reserve(Ns + 1);

    auto evalAt = [&](double t, Eigen::Vector3d& p, Eigen::Vector3d& v) {
        // 定位段
        double acc_t = 0.0;
        int seg = nseg - 1;
        double u = 1.0;
        for (int i = 0; i < nseg; ++i) {
            if (t <= acc_t + dt[i] + 1e-12) {
                seg = i;
                u = (dt[i] > 1e-9) ? (t - acc_t) / dt[i] : 1.0;
                u = std::max(0.0, std::min(1.0, u));
                break;
            }
            acc_t += dt[i];
        }
        const double h = dt[seg];
        const double u2 = u * u, u3 = u2 * u;
        const double h00 = 2 * u3 - 3 * u2 + 1;
        const double h10 = u3 - 2 * u2 + u;
        const double h01 = -2 * u3 + 3 * u2;
        const double h11 = u3 - u2;
        // ★ 每段独立切线（见上面"切线必须每段独立"的说明）
        //   v0_scale 已回写到 vn.front()，这里不再重复乘（否则压缩两次）。
        const Eigen::Vector3d m0 = node_dir[seg] * (vn[seg] * h);
        const Eigen::Vector3d m1 = node_dir[seg + 1] * (vn[seg + 1] * h);
        p = h00 * Q[seg] + h10 * m0 + h01 * Q[seg + 1] + h11 * m1;
        // 速度 = dH/dt = (dH/du)/h
        const double d00 = 6 * u2 - 6 * u;
        const double d10 = 3 * u2 - 4 * u + 1;
        const double d01 = -6 * u2 + 6 * u;
        const double d11 = 3 * u2 - 2 * u;
        v = (d00 * Q[seg] + d10 * m0 + d01 * Q[seg + 1] + d11 * m1) / h;
    };

    for (int k = 0; k <= Ns; ++k) {
        const double t = total_T * k / Ns;
        Eigen::Vector3d p, v;
        evalAt(t, p, v);
        stamps_.push_back(t);
        pos_.push_back(p);
        vel_.push_back(v);
    }
    // ══════════════════════════════════════════════════════════════════
    // 端点处理：**不要硬置零**（2026-11-16 真 bug 修复）
    //
    // 旧实现 `vel_.back() = stop_at_end ? Zero : vel_.back()`。
    // 若倒数第二个采样点速度非零（比如 1 m/s），0.02s 内被拉到 0
    // → acc_ = 1/0.02 = **50 m/s²**（max_acc 的 33 倍）
    // → 该值经 quintic Hermite 端点约束放大 → 实测 max|a| 达 402
    // → 控制器前馈 → 姿态失控 → 掉高。
    //
    // 正确性本来由 Hermite 的端点切向保证：`m.back() = 0` 已经让
    // u=1 处导数精确为 0，采样出来的 vel_.back() 自然是 0。
    // 所以这里只需**验证**，不需要强行赋值；若确实非零（数值残差），
    // 就让 acc_ 的限幅去吸收，而不是制造断崖。
    //
    // 起点同理：`m.front()` 已按 start_v 定义，pos_/vel_.front() 与之一致。
    // ══════════════════════════════════════════════════════════════════
    pos_.front() = start_p;
    pos_.back() = Q.back();
    {
        const double v_end_res = vel_.back().norm();
        if (params.stop_at_end && v_end_res > 0.05) {
            static int endw = 0;
            if (endw++ % 20 == 0) {
                std::fprintf(stderr,
                    "[azj-traj] WARN: end speed %.3f m/s (expected 0) - "
                    "Hermite end tangent not reaching zero\n", v_end_res);
            }
        }
    }

    // ---------- 6) 加速度（差分 + 限幅） ----------
    // 加速度：中心差分 + 严格限幅到 max_acc（2026-11-16）
    //
    // 中心差分让 a 与 v 更自洽（quintic Hermite 的端点约束要求这点，
    // 前向差分滞后半步会引起过冲）。
    // 限幅用 1×max_acc 而不是 2×：轨迹的加速度本来就不该超过它，
    // 超过就是数值噪声，喂给控制器前馈只会把姿态拽偏。
    const int msz = static_cast<int>(stamps_.size());
    acc_.assign(msz, Eigen::Vector3d::Zero());
    const double a_cap = std::max(0.2, std::max(params.max_acc, params.max_acc_z));
    for (int i = 0; i < msz; ++i) {
        Eigen::Vector3d a_i = Eigen::Vector3d::Zero();
        if (i > 0 && i + 1 < msz) {
            const double ddt = stamps_[i + 1] - stamps_[i - 1];
            if (ddt > 1e-9) a_i = (vel_[i + 1] - vel_[i - 1]) / ddt;
        } else if (i + 1 < msz) {
            const double ddt = stamps_[i + 1] - stamps_[i];
            if (ddt > 1e-9) a_i = (vel_[i + 1] - vel_[i]) / ddt;
        } else if (i > 0) {
            const double ddt = stamps_[i] - stamps_[i - 1];
            if (ddt > 1e-9) a_i = (vel_[i] - vel_[i - 1]) / ddt;
        }
        const double an = a_i.norm();
        if (an > a_cap) a_i *= a_cap / an;
        acc_[i] = a_i;
    }
    return stamps_.size() >= 2;
}

// ─────────────────────────────────────────────────────────────────────
//  拼接：把 seg 接在本轨迹 t_join 之后
// ─────────────────────────────────────────────────────────────────────
bool PathTrajectory::spliceAfter(double t_join, const PathTrajectory& seg) {
    if (stamps_.empty()) return false;
    if (!seg.valid()) return true;                 // 没东西可接，视为成功
    if (t_join >= stamps_.back()) {
        // 本轨迹已经不够长：直接整体接到末端（保持时间连续）
        t_join = stamps_.back();
    }

    // 1) 截断到 t_join（含插值点，保证拼接处位置连续）
    std::vector<double> ts;
    std::vector<Eigen::Vector3d> out_p, out_v, out_a;
    size_t cut = 0;
    while (cut < stamps_.size() && stamps_[cut] <= t_join + 1e-9) ++cut;
    for (size_t i = 0; i < cut; ++i) {
        ts.push_back(stamps_[i]);
        out_p.push_back(pos_[i]);
        out_v.push_back(vel_[i]);
        out_a.push_back(acc_[i]);
    }
    // 在 t_join 处补一个精确的插值点（若 cut 落在两个采样之间）
    double splice_shift = 0.0;
    Eigen::Vector3d jp, jv, ja;
    if (sample(t_join, jp, jv, ja)) {
        if (ts.empty() || t_join - ts.back() > 1e-6) {
            ts.push_back(t_join);
            out_p.push_back(jp);
            out_v.push_back(jv);
            out_a.push_back(ja);
        } else {
            // 替换最后一点，保证正好落在 t_join
            ts.back() = t_join;
            out_p.back() = jp;
            out_v.back() = jv;
            out_a.back() = ja;
        }
        // ★ 拼接处位置必须连续：若 seg 起点与 t_join 处差得多，插入过渡点。
        //
        // 2026-11-14 修正：过渡段的**时间跨度**必须按几何长度分配。
        // 原实现给了 1e-3 秒去走完 jump 距离 —— 若 jump=2m，等效速度 2000 m/s，
        // 采样就是一段 1.8m 的瞬移（单元测试 [10] 实测跳变 1.777m）。
        //
        // 现在按"不超过 seg 初速（至少 0.3 m/s）"分配时长，等效一个平滑加速段。
        const double jump = (seg.positions().front() - jp).norm();
        if (jump > 0.02) {
            const double v_typ = std::max(0.3, seg.velocities().front().norm());
            const double dur_jump = jump / v_typ;
            const int N = std::max(1, std::min(200,
                static_cast<int>(std::ceil(jump / 0.05))));
            for (int k = 1; k < N; ++k) {
                const double w = static_cast<double>(k) / N;
                ts.push_back(t_join + dur_jump * w);
                out_p.push_back(jp + w * (seg.positions().front() - jp));
                out_v.push_back(jv);
                out_a.push_back(Eigen::Vector3d::Zero());
            }
            // 过渡段占用了时间 → seg 的平移量也要跟着后移，避免时间倒流
            splice_shift = dur_jump;
        }
    }

    // 2) 追加 seg（时间戳平移；过渡段占用 splice_shift 秒要一并后移）
    const double off = t_join + splice_shift - seg.stamps().front();
    for (size_t i = 0; i < seg.stamps().size(); ++i) {
        const double t = seg.stamps()[i] + off;
        if (t <= ts.back() + 1e-6) continue;       // 跳过重叠
        ts.push_back(t);
        out_p.push_back(seg.positions()[i]);
        out_v.push_back(seg.velocities()[i]);
        out_a.push_back(seg.accelerations()[i]);
    }

    stamps_ = std::move(ts);
    pos_ = std::move(out_p);
    vel_ = std::move(out_v);
    acc_ = std::move(out_a);
    return stamps_.size() >= 2;
}

// ─────────────────────────────────────────────────────────────────────
//  刹车段
// ─────────────────────────────────────────────────────────────────────
bool PathTrajectory::buildBrake(const Eigen::Vector3d& start_p,
                                const Eigen::Vector3d& start_v,
                                const TrajParams& params,
                                PathTrajectory& out) {
    const double v = start_v.norm();
    if (v < 0.05) return false;                    // 已经够慢，不需要刹车段

    // 刹车距离：v²/(2a)。用 max_acc 的一半，留余量给控制器。
    const double a = std::max(0.2, params.max_acc * 0.5);
    const double dist = v * v / (2.0 * a);
    const Eigen::Vector3d dir = start_v / v;
    const Eigen::Vector3d end_p = start_p + dir * dist;

    std::vector<Eigen::Vector3d> path;
    const int N = std::max(2, std::min(200,
        static_cast<int>(std::ceil(dist / 0.05))));
    path.reserve(N + 1);
    for (int k = 0; k <= N; ++k) {
        const double w = static_cast<double>(k) / N;
        path.push_back(start_p + w * (end_p - start_p));
    }

    TrajParams tp = params;
    tp.v_start_vec = start_v;
    tp.has_v_start_vec = true;
    tp.v_start = v;
    tp.stop_at_end = true;                         // 末端速度 0
    return out.build(path, tp);
}

bool PathTrajectory::assignSamples(const std::vector<double>& stamps,
                                   const std::vector<Eigen::Vector3d>& pos,
                                   const std::vector<Eigen::Vector3d>& vel,
                                   const std::vector<Eigen::Vector3d>& acc) {
    const size_t n = stamps.size();
    if (n < 2) return false;
    if (pos.size() != n || vel.size() != n || acc.size() != n) return false;
    for (size_t i = 0; i < n; ++i) {
        if (!std::isfinite(stamps[i])) return false;              // NaN/Inf 检查
        if (!pos[i].allFinite() || !vel[i].allFinite() || !acc[i].allFinite()) return false;
        if (i > 0 && stamps[i] <= stamps[i - 1]) return false;    // 必须严格递增
    }
    stamps_ = stamps;
    pos_ = pos;
    vel_ = vel;
    acc_ = acc;
    return true;
}

bool PathTrajectory::sample(double t, Eigen::Vector3d& p, Eigen::Vector3d& v,
                            Eigen::Vector3d& a) const {
    if (stamps_.empty()) return false;

    if (t <= stamps_.front()) {
        p = pos_.front();
        v = vel_.front();
        a = acc_.front();
        return true;
    }
    if (t >= stamps_.back()) {
        p = pos_.back();
        // 若终点应停住，速度给 0；否则给末速（2026-11-17：MINCO 滚动段
        // 的末点不是终点，必须保留末速，否则每次重规划都被判成"到点停住"）
        if (ends_at_rest_) {
            v = Eigen::Vector3d::Zero();
            a = Eigen::Vector3d::Zero();
        } else {
            v = vel_.back();
            a = acc_.back();
        }
        return true;
    }

    // 二分查找所在区间
    const auto it = std::lower_bound(stamps_.begin(), stamps_.end(), t);
    const size_t hi = static_cast<size_t>(it - stamps_.begin());
    const size_t lo = hi - 1;

    const double h = stamps_[hi] - stamps_[lo];
    const double u = (h > 1e-9) ? (t - stamps_[lo]) / h : 0.0;
    const double u2 = u * u, u3 = u2 * u;

    // ══════════════════════════════════════════════════════════════════
    // 三次 Hermite 插值（2026-11-16 最终方案）
    //
    // 演进（都是实测驱动的）：
    //   ① 线性插值 pos/vel/acc → 速度分段线性 → acc 每点阶跃（max|Δa|=3.0）
    //   ② 三次 Hermite（用 p、v）→ 位置/速度 C1，但 acc 在节点不连续
    //   ③ 五次 Hermite（用 p、v、a）→ 想做到 C2
    //
    // ★ 为什么最终回到三次：
    //   五次要求 `acc_` 是**精确的节点二阶导**。而 `acc_` 只能是差分的
    //   近似，喂进五次公式会被 h² 分母放大 —— 实测 max|a| 达 **403 m/s²**
    //   （见 /tmp/tacc2.cpp）。三次只用 (p, v)，而 v 是构造出来的真值，
    //   数学自洽 → 不会过冲。
    //
    //   节点速度连续性已由"每段独立切线 + 转向时间下限"保证（见 buildSmoothFromState），
    //   所以三次已经足够平滑；acc 用中心差分给出，量级就是 (v1-v0)/h ≤ max_acc。
    // ══════════════════════════════════════════════════════════════════
    const Eigen::Vector3d& p0 = pos_[lo];
    const Eigen::Vector3d& p1 = pos_[hi];
    const Eigen::Vector3d& v0 = vel_[lo];
    const Eigen::Vector3d& v1 = vel_[hi];

    const double h00 =  2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 =        u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 =        u3 -       u2;

    p = h00 * p0 + h10 * h * v0 + h01 * p1 + h11 * h * v1;

    const double d00 =  6.0 * u2 - 6.0 * u;
    const double d10 =  3.0 * u2 - 4.0 * u + 1.0;
    const double d01 = -6.0 * u2 + 6.0 * u;
    const double d11 =  3.0 * u2 - 2.0 * u;
    v = (d00 * p0 + d10 * h * v0 + d01 * p1 + d11 * h * v1) / h;

    // 加速度：端点线性插值（acc_ 已按 max_acc 限幅，不会有尖峰）
    a = acc_[lo] + u * (acc_[hi] - acc_[lo]);

    return true;
}

double PathTrajectory::maxVelocity() const {
    double m = 0.0;
    for (const auto& v : vel_) m = std::max(m, v.norm());
    return m;
}

double PathTrajectory::maxAcceleration() const {
    double m = 0.0;
    for (const auto& a : acc_) m = std::max(m, a.norm());
    return m;
}

}  // namespace azj_planner
