#pragma once

// ══════════════════════════════════════════════════════════════════════════
//  MINCO 轨迹优化器（2026-11-17）
// ══════════════════════════════════════════════════════════════════════════
//
// 与 curve_optimizer 的分工：
//
//   curve_optimizer（B 样条）  → 只优化**形状**，时间由 trajectory.cpp 的
//                                TOPP 事后参数化。起点只约束 v。
//   minco_optimizer（本文件）  → **形状与时间联合优化**。起点硬约束 p/v/a/j。
//
// 为什么需要后者（实测依据 /tmp/azj_roll_20261117_113332）：
//   rolling 每 0.1~0.16s 重建轨迹，而 B 样条每条都从 v0 靠 max_acc 重新加速
//   → 速度永远停在加速段（|v0| 恒 0.285，max_vel=1.0）。
//   因为 B 样条**没有加速度约束**，无法表达"我正以 a 加速，请接着加速"。
//
// MINCO 的 headPVAJ 把当前 p/v/a/j 全部钉死，且段间 C3 连续，
// 所以"接着飞"是结构性保证，不依赖速度补偿。
//
// 代价函数（全部解析梯度，不解数值差分）：
//   E = w_smooth · ∫|p'''|²dt          ← MINCO 的 jerk 泛函
//     + w_mid    · Σ (ref/clamp(c))^α   ← 居中（= 两侧等距，用户的核心理念）
//     + w_obs    · Σ hinge(margin - c)² ← 安全硬下界的软实现
//     + w_vel    · Σ hinge(|v| - vmax)²
//     + w_acc    · Σ hinge(|a| - amax)²
//     + w_anchor · Σ |q_i - q_i^0|²     ← 保形（防整体漂移）
//     + w_time   · Σ T_i                 ← 时间正则（防无限拉长）
//
// 优化方法：投影梯度下降 + 回溯线搜索 + 安全硬投影。
// 不用 LBFGS（SUPER 用的）：本包其余部分都是自写梯度下降（curve_optimizer），
// 风格统一、行为可预测；MINCO 的收敛性主要靠"好初值 + 解析梯度"，
// 实测 40~80 次迭代足够。
// ══════════════════════════════════════════════════════════════════════════

#include <limits>
#include <string>
#include <vector>

#include "azj_planner/clearance_field.h"
#include "azj_planner/minco.h"
#include "azj_planner/trajectory.h"

namespace azj_planner {

struct MincoOptParams {
    /// 段数。0 = 按 piece_len 自动（clamp 到 [3, 40]）。
    int pieces{0};
    /// 自动定段数时的每段长度 [m]。
    ///
    /// 0.40m 的依据与 curve_optimizer 的 control_point_spacing(0.30) 同源：
    /// 段长必须小于障碍特征尺寸，否则一个多项式跨过整个拐弯会过冲。
    /// 但 MINCO 是 7 次多项式、段间 C3，表达力比三次 B 样条强，
    /// 所以 0.40 就够（更短只会让带状系统变大、耗时上升）。
    double piece_len{0.40};

    // ---- 安全 / 居中（语义与 CurveOptParams 保持一致）----
    double safe_margin{0.30};
    double clearance_ref{1.0};
    double clearance_cap{2.0};
    double alpha{2.0};
    /// 居中权重。**相比 smooth 的 1.0 要大得多**，这是实测定的（不是拍脑袋）。
    ///
    /// 权重归一化后（各项初始贡献 ≈ w_i）实测横扫（直通道半宽 1.5m，
    /// 初值偏 0.8m，60 次迭代）：
    ///
    ///   w_mid   横向不平衡    min_clr    耗时
    ///   1.0     1.240 m       0.880      14.6ms
    ///   3.0     0.844 m       1.078      13.6ms
    ///   10.0    0.281 m       1.360      12.2ms
    ///   30.0    0.010 m       1.495      11.0ms   ← 最优
    ///
    /// 30 的含义：居中项的权重是 jerk 能量的 30 倍。这正对应用户的理念
    /// "不珍惜时间和空间最优，只关心安全"——居中优先于平滑与最短。
    double w_mid{30.0};
    double w_obs{50.0};

    // ---- 平滑 / 时间 / 保形 ----
    double w_smooth{1.0};
    /// 时间正则权重。**实测 0.20 最优，不要为提高它去"压时间"**。
    ///
    /// 横扫（直通道半宽 1.5m、初值偏 0.8m、60 次迭代）：
    ///
    ///   w_time   横向不平衡    dur     说明
    ///   0.20     **0.005**     11.29   ← 居中最好
    ///   0.50     **0.000**     10.49   居中完美，dur 略短
    ///   0.80     0.040         9.88
    ///   1.20     0.274         9.81   居中开始明显退化
    ///   1.50     0.402         9.76
    ///   2.50     0.620         9.69
    ///
    /// 关键发现：**dur 对 w_time 几乎不敏感**（11.29 → 9.69，仅 14%），
    /// 而居中质量退化 100 倍以上。所以"时间被拉长"不是 w_time 的问题
    /// —— 那次 `dur=20s / len=4.28m` 的真因是 lag 速度压制把
    /// `max_vel` 压到 0.25（见 rollingReplan 里 vmax_allow 的下限修正）。
    /// 加大 w_time 只会牺牲居中，治不了那个病。
    double w_time{0.20};
    /// 保形锚定。实测 0.30 / 0.10 / 0.02 对结果几乎无影响
    /// （居中权重足够大时，锚定拉不住它），故取小值只作数值稳定用。
    double w_anchor{0.10};

    // ---- 动力学可行性 ----
    double max_vel{1.0};
    double max_acc{1.5};
    double w_vel{2.0};
    double w_acc{2.0};

    // ---- 优化控制 ----
    int iterations{60};
    double step_init{0.02};
    int line_search_iters{4};
    int stall_iters{10};
    double rel_improve{1e-4};

    // ---- 时间变量 ----
    bool opt_time{true};
    double t_min{0.05};
    double t_max{3.0};

    // ---- 采样 ----
    /// 每段内的积分采样数（越多越准，越慢）。8~16 合适。
    int integral_res{10};
    /// 输出轨迹的采样步长 [s]。≤0.02 保证 Hermite 插值精度。
    double sample_dt{0.01};

    // ---- 安全硬投影 ----
    bool enable_projection{true};
    int projection_iters{2};
    /// 投影采样数（比积分采样更密，防止漏掉窄处）
    int projection_samples{200};

    // ---- z 约束（2D 模式 / 层高）----
    bool lock_z{false};
    double lock_z_value{std::numeric_limits<double>::quiet_NaN()};
    bool enable_z_bounds{false};
    double z_min{-1e9};
    double z_max{1e9};

    /// 终点是否停在原地（true = 末速/末加速度/末 jerk 全 0）。
    /// rolling 的局部目标处应为 false（匀速穿过），全量规划为 true。
    bool stop_at_end{true};
};

struct MincoOptResult {
    bool success{false};
    std::string msg;

    MincoS4NU minco;
    PathTrajectory traj;

    std::vector<double> clearance_along;
    double min_clearance{0.0};
    double mean_clearance{0.0};
    double length{0.0};
    double duration{0.0};
    double max_vel{0.0};
    double max_acc{0.0};
    int iterations{0};
};

/// MINCO 轨迹优化。
///
/// @param head 起点状态（p 通常是飞机当前位置，v/a/j 是**规划**状态，
///             不是 odom 差分值 —— 见 rolling 的注释）
/// @param tail 终点状态（p 为局部/全局目标；stop_at_end 时 v/a/j 置 0）
/// @param init_path 初始折线（A* 路径），用来定内点与初始 T
/// @param field clearance 场
class MincoOptimizer {
public:
    MincoOptResult optimize(const StatePVAJ& head, const StatePVAJ& tail,
                            const std::vector<Eigen::Vector3d>& init_path,
                            const ClearanceField& field,
                            const MincoOptParams& params) const;
};

}  // namespace azj_planner
