#pragma once

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "azj_planner/clearance_field.h"

namespace azj_planner {

/// 三次 B 样条曲线（C2 连续：位置、切线、曲率都连续）。
///
/// 为什么用三次 B 样条而不是 Catmull-Rom 或五次样条：
///   - 三次是保证**曲率连续**的最低阶次，而曲率不连续会让控制器在
///     曲率跳变处出现加速度突变（这正是折线路径抖动的来源）。
///   - 五次更光滑但控制点影响范围大、局部性差，窄缝里容易整条鼓包。
///   - B 样条天然局部支撑：动一个控制点只影响局部 4 段，缝里调整不外溢。
class BSpline {
public:
    BSpline() = default;

    /// 用控制点构造（阶数固定 3，clamped 节点向量保证过首末控制点）。
    bool init(const Eigen::Matrix<double, 3, Eigen::Dynamic>& ctrl, int order = 3);

    bool valid() const { return ctrl_.cols() >= static_cast<int>(order_ + 1); }
    int order() const { return order_; }
    int numControlPoints() const { return valid() ? static_cast<int>(ctrl_.cols()) : 0; }

    const Eigen::Matrix<double, 3, Eigen::Dynamic>& controlPoints() const { return ctrl_; }

    /// 曲线参数域。
    double uMin() const { return 0.0; }
    double uMax() const { return valid() ? knots_[knots_.size() - 1 - order_] : 0.0; }

    /// 求值。deriv=0/1/2 分别返回位置 / 一阶导 / 二阶导。
    bool evaluate(double u, int deriv, Eigen::Vector3d& out) const;

    /// 曲率 κ = |p' × p''| / |p'|³。返回 0 表示切向退化。
    double curvature(double u) const;

    /// 按弧长把曲线重采样成等距折线（供速度规划与可视化用）。
    /// step_m 是目标点间距；返回的每个点都带切线（单位向量）。
    bool sampleByArcLength(double step_m, std::vector<Eigen::Vector3d>& points,
                          std::vector<Eigen::Vector3d>& tangents) const;

    /// 曲线总长（弧长数值积分）。
    double arcLength(int samples = 400) const;

    /// 第 i 个控制点对冲量的基函数值（用于把目标函数梯度回传到控制点）。
    /// 返回 false 表示 u 落在该控制点支撑域外。
    bool basisValue(int i, double u, double& value) const;

    /// 一次性返回 u 处所有非零基函数（阶数+1 个），比逐个调用快。
    /// out_idx / out_val 长度应 >= order+1。
    int nonzeroBasis(double u, int* out_idx, double* out_val) const;

private:
    void buildKnots();

    Eigen::Matrix<double, 3, Eigen::Dynamic> ctrl_;
    std::vector<double> knots_;
    int order_{3};
};

/// 曲线优化参数。
struct CurveOptParams {
    /// 采样点数（沿曲线）。太少 → 窄缝被漏掉；太多 → 慢。100~200 合适。
    int samples{120};

    /// 控制点数量。控制点越少曲线越硬（难以贴合窄缝），越多越灵活但易抖。
    /// 0 = 按 control_point_spacing 自动定。
    int control_points{0};

    /// 自动定控制点时的间距 [m]。
    ///
    /// ★ 这个值是"密集场景能否成功"的关键。
    ///   原来固定 0.6m：柱子间距 1.2m 时**一个控制点管两段拐弯**，
    ///   B 样条在柱子拐角必然过冲 → 实测 min_clearance=0（曲线穿进柱子）。
    ///   改 0.30m 后，每个拐弯至少 2 个控制点，曲线能贴合"绕柱"的形状。
    double control_point_spacing{0.30};

    /// 平滑权重：∫|p''|²
    double w_smooth{1.0};

    /// 居中权重：mean((ref / clamp(c,·,cap))^alpha)
    ///
    /// ★ 这一项就是用户理念的数学表达。
    ///   通道横截面上 clearance = min(左距, 右距)，而 min 只有在
    ///   **两侧距离相等**处才取到该剖面的最大值。所以最大化本项
    ///   会自动把路径推到两侧等距的中轴 —— 不需要额外的对称项。
    ///   又因为 min 不可微，两侧不等时梯度只反映较近那一侧，
    ///   方向正好把路径往回推，这是一个天然的"回中"反馈。
    /// 居中权重。
    ///
    /// 居中权重。梯度按该处半宽归一化（见 curve_optimizer.cpp），
    /// 所以 1.0 就能同时适配宽通道和窄缝，不需要靠大权重。
    double w_mid{1.0};

    /// 安全硬约束权重：Σ max(0, safe_margin - c)²
    double w_obs{50.0};

    /// 端点钉住权重（起点=飞机当前位、终点=目标）。通常给很大值。
    double w_end{2000.0};

    /// 长度权重：约束"不要为了更宽而绕远"。
    double w_len{0.05};

    /// 居中项的归一化基准与上限 [m]（含义同 SearchParams）。
    double clearance_ref{1.0};
    double clearance_cap{2.0};
    double alpha{2.0};

    /// 安全下界 [m]。低于它就罚（这是硬安全的软实现）。
    double safe_margin{0.30};

    /// 锚定权重：Σ|Q_i - Q_i^0|²。
    /// 没有它，B 样条会在平滑项驱动下整体漂移（实测长度能涨 46%）。
    /// 这是"保形"的代价：让优化在初始路径附近找更平滑/更居中的解，
    /// 而不是自由变形。
    double w_anchor{0.5};

    // ================= 安全硬投影（关键）=================
    //
    // 为什么软惩罚不够：hinge 项权重 w_obs/S 在 S=140 时每点只有 0.36，
    // 压不过 w_anchor(0.5) 的保形力。密集障碍里曲线必然被拽进柱子
    // （实测 min_clearance=0）。
    //
    // 硬投影（PBD 风格）：每步梯度下降后，沿曲线密集采样，
    // 对 clearance < safe_margin 的点，把它对应的控制点沿 ∇c **直接推开**，
    // 推开量 = (safe_margin - c)。这是位置投影，不受权重竞争影响。
    bool enable_projection{true};
    /// 每步后做多少次投影迭代（2~3 次足够收敛）
    int projection_iters{3};
    /// 投影时沿曲线的采样数（比优化采样更密，防止漏掉窄处）
    int projection_samples{240};

    // ================= 每段独立居中 =================
    //
    // "每一段都在障碍中间"：不能只用全局平均的 (ref/c)^α
    // （空旷段 c=2m 时惩罚≈0，会把梯度稀释掉）。
    //
    // 改成：对每个采样点算**局部居中度** —— 用该点的 clearance
    // 相对"这一段能达到的最好 clearance"的差距来罚。
    // local_ref 由扫描垂直方向得到，代表该横截面能提供的最大间隙。
    bool enable_local_centering{true};
    /// 垂直方向扫描的半径 [m]（找该横截面两侧障碍）
    double local_scan_radius{2.0};

    /// 迭代次数与初始步长。
    /// 步长不宜大：曾有 step=0.08×√ncp≈0.33m/步，曲线几十步就冲穿墙。
    int iterations{150};
    double step_init{0.02};

    /// 末端是否钉死。false 时只做弱约束（用于闭环重规划时终点也在动的情况）。
    bool pin_endpoints{true};

    /// 2D 时锁定的平面高度。仅当 lock_z=true 时生效。
    ///
    /// 为什么不能"保持初值 z"：初值来自 A* 路径，而 A* 起点是飞机当前位，
    /// 若飞机当前高度 ≠ 规划平面（很常见：刚起飞、掉高后恢复），
    /// 就会在路径上留一段 z 过渡，"2D" 名不副实。
    /// 这里直接用规划平面高度覆盖，保证输出严格平面。
    /// NaN = 用初值（保持向后兼容）。
    double lock_z_value{std::numeric_limits<double>::quiet_NaN()};

    /// z 方向的硬边界 [m]。enable_z_bounds=false 时不生效。
    ///
    /// 为什么必须有：曲线优化在 A* 之后运行，如果不管层高，
    /// 平滑/居中项会把曲线推出 A* 的可行域 —— 实测下界被顶破
    /// （floor_z=-0.10 时曲线 z 到 -0.103）。
    bool enable_z_bounds{false};
    double z_min{-1e9};
    double z_max{1e9};

    /// true = 2D 规划：把 z 分量完全锁死（梯度清零 + 控制点 z 保持初值）。
    ///
    /// 单靠"所有控制点 z 相同"是不够的：三次 B 样条确实是控制点的凸组合，
    /// 初值 z 相同时曲线 z 恒等；但优化过程中 z 梯度会把它推走，
    /// 于是 2D 规划会悄悄变成 3D。必须显式锁。
    bool lock_z{false};
};

/// 曲线优化结果。
struct CurveOptResult {
    bool success{false};
    std::string msg;
    BSpline spline;

    std::vector<double> clearance_along;   ///< 采样点处的 clearance 曲线
    double min_clearance{0.0};
    double mean_clearance{0.0};
    double length{0.0};
    double max_curvature{0.0};
    int iterations{0};
};

/// ESDF 梯度驱动的曲线优化。
///
/// 输入是 A* 的折线路径，输出是 C2 连续的三次 B 样条。
///
/// 目标函数（全部对控制点求解析梯度）：
///   E = w_smooth·∫|p''|²
///     + w_mid   ·mean((ref/c)^α)          ← 居中（= 两侧等距，见 w_mid 注释）
///     + w_obs   ·Σ hinge(safe_margin - c)
///     + w_len   ·(length - length0)
///     + w_end   ·端点偏差
class CurveOptimizer {
public:
    CurveOptResult optimize(const std::vector<Eigen::Vector3d>& init_path,
                           const ClearanceField& field,
                           const CurveOptParams& params) const;
};

}  // namespace azj_planner
