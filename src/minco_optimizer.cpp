// MINCO 轨迹优化器实现。见 minco_optimizer.h 的设计说明。

#include "azj_planner/minco_optimizer.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <Eigen/Dense>

namespace azj_planner {
namespace {

/// 平滑 hinge：max(0, x)，在 |x| < eps 处用二次过渡避免梯度跳变。
/// 返回罚值与对 x 的导数。
inline void hinge(double x, double eps, double& pena, double& dpena) {
    if (x <= 0.0) {
        pena = 0.0;
        dpena = 0.0;
    } else if (x < eps) {
        pena = x * x / (2.0 * eps);
        dpena = x / eps;
    } else {
        pena = x - 0.5 * eps;
        dpena = 1.0;
    }
}

/// 三次 B 样条基相关：MINCO 段内的基函数及其一阶/二阶/三阶导。
inline void betaBasis(double s, Eigen::Matrix<double, 8, 1>& b0,
                      Eigen::Matrix<double, 8, 1>& b1,
                      Eigen::Matrix<double, 8, 1>& b2,
                      Eigen::Matrix<double, 8, 1>& b3) {
    const double s2 = s * s, s3 = s2 * s, s4 = s2 * s2, s5 = s4 * s, s6 = s4 * s2;
    b0 << 1.0, s, s2, s3, s4, s5, s6, s4 * s3;
    b1 << 0.0, 1.0, 2.0 * s, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    b2 << 0.0, 0.0, 2.0, 6.0 * s, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    b3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s, 60.0 * s2, 120.0 * s3, 210.0 * s4;
}

}  // namespace

MincoOptResult MincoOptimizer::optimize(const StatePVAJ& head, const StatePVAJ& tail,
                                        const std::vector<Eigen::Vector3d>& init_path,
                                        const ClearanceField& field,
                                        const MincoOptParams& params) const {
    MincoOptResult res;
    if (init_path.size() < 2) {
        res.msg = "init path too short";
        return res;
    }

    // ─────────── 1) 总长与段数 ───────────
    std::vector<double> arc(init_path.size(), 0.0);
    for (size_t i = 1; i < init_path.size(); ++i) {
        arc[i] = arc[i - 1] + (init_path[i] - init_path[i - 1]).norm();
    }
    const double total_len = arc.back();
    if (total_len < 1e-3) {
        res.msg = "init path has zero length";
        return res;
    }

    int N = params.pieces;
    if (N <= 0) {
        const double pl = std::max(0.15, params.piece_len);
        N = static_cast<int>(std::ceil(total_len / pl));
    }
    N = std::min(std::max(N, 2), 40);

    // ─────────── 2) 内点初值：沿弧长均匀取 N-1 个 ───────────
    //
    // 为什么要沿弧长均匀而不是直接取 A* 折线顶点：
    // MINCO 的变量是"内点位置"，其个数必须等于 N-1。均匀弧长采样保证
    // 初值形状与折线一致，且不会出现超短段（超短段会让带状系统病态）。
    Eigen::MatrixXd q(3, std::max(1, N - 1));
    {
        for (int k = 1; k <= N - 1; ++k) {
            const double target = total_len * k / static_cast<double>(N);
            size_t idx = 1;
            while (idx + 1 < arc.size() && arc[idx] < target) ++idx;
            const double d0 = arc[idx - 1], d1 = arc[idx];
            const double w = (d1 - d0 > 1e-9) ? (target - d0) / (d1 - d0) : 0.0;
            q.col(k - 1) = init_path[idx - 1] + w * (init_path[idx] - init_path[idx - 1]);
        }
    }
    const Eigen::MatrixXd q0 = q;   // 保形锚点

    // ─────────── 3) 初始段时长 ───────────
    //
    // 按"平均速度 = max_vel 的 0.8 倍"起步。太短会让加速度惩罚爆掉，
    // 太长会让 w_time 项把它压回来 —— 从中间起步收敛更快。
    Eigen::VectorXd T(N);
    {
        const double v_ref = std::max(0.2, params.max_vel * 0.8);
        for (int i = 0; i < N; ++i) {
            double len_i = total_len / N;
            if (i == 0) {
                len_i = (q.col(0) - head.p).norm();
            } else if (i == N - 1) {
                len_i = (tail.p - q.col(N - 2)).norm();
            } else {
                len_i = (q.col(i) - q.col(i - 1)).norm();
            }
            const double t_floor =
                std::max(params.t_min, len_i / std::max(0.2, params.max_vel * 1.3));
            T(i) = std::min(std::max(len_i / v_ref, t_floor), params.t_max);
        }
    }

    // 起点/终点状态矩阵
    Eigen::Matrix<double, 3, 4> headM = head.toMat();
    StatePVAJ tailEff = tail;
    if (params.stop_at_end) {
        tailEff.v.setZero();
        tailEff.a.setZero();
        tailEff.j.setZero();
    }
    Eigen::Matrix<double, 3, 4> tailM = tailEff.toMat();

    MincoS4NU minco;
    minco.setConditions(headM, tailM, N);

    // ─────────── 4) 优化 ───────────
    //
    // 变量：q（3×(N-1) 内点）与 T（N 段时长）。
    // 梯度来自两块：
    //   · MINCO 的 jerk 泛函（getEnergy 系列，已是解析梯度）
    //   · 采样点的 clearance / vel / acc / anchor 罚（本文件算）
    // 两者都先聚到"系数梯度 gdC + 时间梯度 gdT"，再由 propogateGrad
    // 用带状伴随求解回传到 q 和 T。所以对 q / T 都是解析梯度。

    const int nseg = params.integral_res;
    double best_cost = std::numeric_limits<double>::infinity();
    int stall = 0;
    double step = params.step_init;

    // ── 权重归一化系数（2026-11-17 关键修正）──
    //
    // 事故（minco_optimizer_test [1]）：优化跑了 60 次迭代、cost 从 42059 降到 822，
    // 但横向不平衡 1.597m 完全没变（初始 1.6m）。
    // 调试打印显示 gqmax = 2.98e5 —— jerk 泛函的梯度比居中项大 5 个数量级：
    //   · jerk 能量 E ≈ 10^4（因为 E ∝ Σ c_k² T^k，系数 c_7 天然很大）
    //   · 居中项 w_mid·(ref/c)^α ≈ 0.44 × 采样点数 ≈ 10^2
    // 归一化方向时 y 分量被 x 分量淹没 → 60 步只优化了时间，位置纹丝不动。
    //
    // 修法：优化前先探测各项初始量级，把权重缩放到"初始贡献≈w_i"。
    // 这样 w_smooth=1 / w_mid=1 才真的可比，而不用现场试凑数量级。
    double sc_energy = 1.0, sc_mid = 1.0, sc_obs = 1.0, sc_anchor = 1.0, sc_time = 1.0;
    {
        minco.setParameters(q, T);
        const double e0 = minco.getEnergy();
        if (e0 > 1e-9) sc_energy = 1.0 / e0;
        const double tsum = T.sum();
        if (tsum > 1e-9) sc_time = 1.0 / tsum;

        // 采样一次统计居中/安全/锚定的初始量级
        const auto& C0 = minco.coeffs();
        double m0 = 0.0, o0 = 0.0, a0 = 0.0;
        const double cap0 = std::max(0.05, params.clearance_cap);
        const double ref0 = std::max(1e-3, params.clearance_ref);
        for (int i = 0; i < N; ++i) {
            const double ds = T(i) / nseg;
            const Eigen::Matrix<double, 8, 3> ci = C0.block<8, 3>(8 * i, 0);
            for (int j = 0; j <= nseg; ++j) {
                const double node = (j == 0 || j == nseg) ? 0.5 : 1.0;
                const double wgt = node * ds;
                Eigen::Matrix<double, 8, 1> b0, b1, b2, b3;
                betaBasis(j * ds, b0, b1, b2, b3);
                const Eigen::Vector3d pos = ci.transpose() * b0;
                const double c = field.clearance(pos);
                if (!std::isfinite(c) || c <= 0.0) continue;
                if (c < cap0) m0 += wgt * std::pow(ref0 / c, params.alpha);
                if (c < params.safe_margin) {
                    const double d = params.safe_margin - c;
                    o0 += wgt * d * d;
                }
            }
        }
        for (int k = 0; k < N - 1; ++k) a0 += (q.col(k) - q0.col(k)).squaredNorm();
        if (m0 > 1e-9) sc_mid = 1.0 / m0;
        if (o0 > 1e-9) sc_obs = 1.0 / o0;
        if (a0 > 1e-9) sc_anchor = 1.0 / a0;
    }

    // z 边界的内点收缩量（给 7 次多项式的段内 overshoot 留空间）
    // 实测 overshoot 约 0.003m，取 0.04 留一个数量级余量。
    const double z_ovs = 0.04;

    auto evalCost = [&](const Eigen::MatrixXd& qc, const Eigen::VectorXd& Tc, double& cost,
                        Eigen::MatrixXd& gq, Eigen::VectorXd& gT) -> bool {
        minco.setParameters(qc, Tc);
        if (!minco.valid()) return false;

        Eigen::MatrixX3d gdC = Eigen::MatrixX3d::Zero(8 * N, 3);
        Eigen::VectorXd gdT = Eigen::VectorXd::Zero(N);
        cost = 0.0;

        // ---- 4.1 jerk 泛函 + 时间正则 ----
        cost += params.w_smooth * sc_energy * minco.getEnergy();
        {
            Eigen::MatrixX3d gc;
            Eigen::VectorXd gt;
            minco.getEnergyPartialGradByCoeffs(gc);
            minco.getEnergyPartialGradByTimes(gt);
            gdC += params.w_smooth * sc_energy * gc;
            gdT += params.w_smooth * sc_energy * gt;
        }
        // ★ 时间正则必须**随 sum(T) 线性增长**，不能只用归一化项（2026-11-17）
        //
        // 事故：MINCO 输出 `len=4.28m dur=20.1s` —— 平均速度只有 **0.21 m/s**
        // （max_vel=1.0），参考慢到飞机轻松超过它，用户看到"轨迹不跟无人机"。
        //
        // 成因（能量-时间权衡失控）：
        //   归一化后 cost = w_smooth·(E/E0) + w_time·(ΣT/ΣT0)
        //   而 7 次多项式的能量 E ∝ 1/T³（段长不变时）。
        //   把 T 拉长 k 倍 → cost = 1/k³ + w_time·k
        //   令 w_time=0.2，最优 k = (3/0.2)^(1/4) ≈ **1.97**
        //   实测更夸张（20s / 5s ≈ 4 倍），因为 w_mid/w_obs 在空旷区不罚，
        //   能量项成了唯一阻力 → 优化器一路拉长时间"抹平"轨迹。
        //
        // 修法：把 ΣT 的绝对值也计入（乘 max_vel 归一化到"等效长度"），
        //   使"无谓拉长时间"有恒定代价，而不是只跟初始值比。
        //   同时 w_time 提高到 1.5（见 MincoOptParams 的推导）。
        cost += params.w_time * sc_time * Tc.sum();
        gdT.array() += params.w_time * sc_time;

        // ---- 4.2 采样点代价（clearance / 居中 / 速度 / 加速度 / 锚定）----
        const auto& C = minco.coeffs();
        const double vmax2 = params.max_vel * params.max_vel;
        const double amax2 = params.max_acc * params.max_acc;
        const double cap = std::max(0.05, params.clearance_cap);
        const double ref = std::max(1e-3, params.clearance_ref);
        const double hstep = 1e-3;

        for (int i = 0; i < N; ++i) {
            const double Ti = Tc(i);
            const double ds = Ti / nseg;
            const Eigen::Matrix<double, 8, 3> ci = C.block<8, 3>(8 * i, 0);

            for (int j = 0; j <= nseg; ++j) {
                // 梯形积分权重（段端点算半个，避免相邻段重复计全额）
                const double node = (j == 0 || j == nseg) ? 0.5 : 1.0;
                const double wgt = node * ds;
                const double s = j * ds;

                Eigen::Matrix<double, 8, 1> b0, b1, b2, b3;
                betaBasis(s, b0, b1, b2, b3);
                const Eigen::Vector3d pos = ci.transpose() * b0;
                const Eigen::Vector3d vel = ci.transpose() * b1;
                const Eigen::Vector3d acc = ci.transpose() * b2;

                Eigen::Vector3d gPos = Eigen::Vector3d::Zero();
                Eigen::Vector3d gVel = Eigen::Vector3d::Zero();
                Eigen::Vector3d gAcc = Eigen::Vector3d::Zero();
                double tmp = 0.0;

                // -- 居中 + 安全 --
                //
                // 居中项 (ref/c)^alpha：c 越大越便宜 → 解落在通道中轴。
                // 这是用户"必须走障碍正中间"的直接数学表达（与
                // curve_optimizer 的 w_mid 同源，见其注释）。
                {
                    const double c = field.clearance(pos);
                    if (std::isfinite(c) && c > 0.0) {
                        Eigen::Vector3d gc;
                        if (field.clearanceGradient(pos, gc) && gc.allFinite()) {
                            if (c < cap) {
                                // f = (ref/c)^a,  df/dp = -a * (ref/c)^a / c * dc/dp
                                const double ratio = ref / c;
                                const double f = std::pow(ratio, params.alpha);
                                tmp += params.w_mid * sc_mid * f;
                                gPos += params.w_mid * sc_mid * (-params.alpha * f / c) * gc;
                            }
                            if (c < params.safe_margin) {
                                const double d = params.safe_margin - c;
                                tmp += params.w_obs * sc_obs * d * d;
                                // d(d^2)/dp = -2d * dc/dp
                                gPos += params.w_obs * sc_obs * (-2.0 * d) * gc;
                            }
                        }
                    }
                }

                // -- 速度 / 加速度上限 --
                {
                    double pv = 0.0, dv = 0.0;
                    hinge(vel.squaredNorm() - vmax2, hstep * vmax2 + 1e-9, pv, dv);
                    if (pv > 0.0) {
                        tmp += params.w_vel * pv;
                        gVel += params.w_vel * dv * 2.0 * vel;
                    }
                    double pa = 0.0, da = 0.0;
                    hinge(acc.squaredNorm() - amax2, hstep * amax2 + 1e-9, pa, da);
                    if (pa > 0.0) {
                        tmp += params.w_acc * pa;
                        gAcc += params.w_acc * da * 2.0 * acc;
                    }
                }

                // -- 保形锚定（只对段的两个端点计，避免段内重复计）--
                if (params.w_anchor > 0.0 && (j == 0 || j == nseg)) {
                    int qi = -1;
                    if (j == 0) {
                        qi = i - 1;            // 段首内点（i=0 时是 head，不计）
                    } else {
                        qi = i;                // 段尾内点（i=N-1 时是 tail，不计）
                    }
                    if (qi >= 0 && qi < N - 1) {
                        const Eigen::Vector3d d = pos - q0.col(qi);
                        tmp += params.w_anchor * sc_anchor * d.squaredNorm();
                        gPos += params.w_anchor * sc_anchor * 2.0 * d;
                    }
                }

                cost += wgt * tmp;

                // ---- 回传到系数与时间 ----
                //
                // p(s) = Σ c_k s^k  →  ∂p/∂c_k = s^k = b0
                // 同理 v 对 c 的偏导是 b1，a 用 b2。
                // gradT 用链式：∂/∂T 通过 s = j·T/nseg 的比例关系，
                // 等价于 SUPER 里的 alpha·(gradPos·vel + ...) 形式。
                gdC.block<8, 3>(8 * i, 0) +=
                    (b0 * gPos.transpose() + b1 * gVel.transpose() + b2 * gAcc.transpose()) * wgt;
                {
                    const double alpha_s = (nseg > 0) ? (static_cast<double>(j) / nseg) : 0.0;
                    gdT(i) += (gPos.dot(vel) + gVel.dot(acc)) * alpha_s * wgt;
                    gdT(i) += node * ds * tmp / std::max(1e-6, Ti) * 0.0;  // 保守：不重复计
                }
            }
        }

        // ---- 4.3 回传到 q / T ----
        Eigen::Matrix3Xd gqp;
        Eigen::VectorXd gTp;
        minco.propogateGrad(gdC, gdT, gqp, gTp, false);
        gq = gqp;
        gT = gTp;

        // ---- 4.4 z 约束（2D 锁平面 / 层高硬边界）----
        //
        // ★★★ 必须留 overshoot 余量（2026-11-17 严重 bug）
        //
        // 事故（bench 复现，~/azj_flights/azj_flight_20261117_215742 同型）：
        //   MINCO_REJECT reason=clearance min_clr=-1m
        //   z=[0.804, **1.00168**]  ceil_z=1
        //   worst=(5.888, -0.030, **1.00018**) c=-1
        //
        //   7 次多项式在**内点之间会 overshoot**（Runge 型摆动）。
        //   内点被夹在 ceil_z 以内，段内却冲出去 0.0002~0.003m。
        //   而 clearanceAt 的层高约束是硬边界（越界返回 -1 不可通行）
        //   → 整条轨迹被判不安全 → MINCO 被拒 → keep_old
        //   → /azj/path 长时间不更新（用户看到"轨迹静止、起点不跟"）。
        //
        //   用户说"完整建图后没有此情况"：因为完整建图时真实 clearance
        //   本来就远大于 robot_radius，少数点飘到 z>ceil 也被 z 边界夹住，
        //   而未知区边界处 clearance 只剩 0.8 左右，一有 overshoot 就直接判死。
        //
        // 修法：内点约束收缩 z_overshoot_margin，给多项式摆动留空间。
        // （z_ovs 定义在函数作用域，见下方）
        //
        // ★ 必须写在 propogateGrad **之后**（2026-11-17 bug）：
        //   原来写在它之前，而 `gq = gqp` 是覆盖赋值，
        //   于是 z 约束的梯度被整个丢掉 —— 2D 模式会悄悄漂出平面。
        //
        // 只在**内点**上罚，因为首尾是硬约束（改不了）。
        if (params.lock_z || params.enable_z_bounds) {
            for (int k = 0; k < N - 1; ++k) {
                const double z = qc(2, k);
                if (params.lock_z && std::isfinite(params.lock_z_value)) {
                    const double d = z - params.lock_z_value;
                    cost += 50.0 * d * d;
                    gq(2, k) += 100.0 * d;
                } else if (params.enable_z_bounds) {
                    const double zlo = params.z_min + z_ovs;
                    const double zhi = params.z_max - z_ovs;
                    if (z < zlo) {
                        const double d = zlo - z;
                        cost += 50.0 * d * d;
                        gq(2, k) += -100.0 * d;
                    } else if (z > zhi) {
                        const double d = z - zhi;
                        cost += 50.0 * d * d;
                        gq(2, k) += 100.0 * d;
                    }
                }
            }
        }

        return cost == cost;   // NaN 检查
    };

    const bool dbg = (std::getenv("AZJ_MINCO_DEBUG") != nullptr);
    Eigen::MatrixXd gq(3, std::max(1, N - 1));
    Eigen::VectorXd gT(N);
    double cost = 0.0;
    if (!evalCost(q, T, cost, gq, gT)) {
        res.msg = "initial evaluation failed";
        return res;
    }
    best_cost = cost;
    res.iterations = 0;

    for (int it = 0; it < params.iterations; ++it) {
        res.iterations = it + 1;

        // 投影梯度方向
        Eigen::MatrixXd dirq = -gq;
        Eigen::VectorXd dirT = params.opt_time ? Eigen::VectorXd(-gT) : Eigen::VectorXd::Zero(N);

        // ★ 步长归一化必须**分别**做（2026-11-17 真 bug）
        //
        // 事故（minco_optimizer_test [1]）：横向不平衡 1.597m 完全没变，
        // 而 dur 从 25.9s 收到 11.8s —— 说明 T 在动、q 完全没动。
        //
        // 原因：原来用一个统一的 gmax（q 与 T 的梯度取最大值）同时归一化两者。
        // T 的梯度量级（w_time=0.2 直接进 gdT）远大于 q 的梯度
        // （要经过 A^{-T} 传播，且 w_mid 项只有 ~0.6），
        // 于是 dirq 被除到 ~1e-3，相当于每一步只动毫米级 → 60 次迭代等于没动。
        //
        // 分别归一化后两者都以"最大分量 = 1"进入线搜索，步长语义一致。
        {
            const double gq_max = dirq.cwiseAbs().maxCoeff();
            if (gq_max > 1e-12) dirq /= gq_max;
            if (params.opt_time) {
                const double gT_max = dirT.cwiseAbs().maxCoeff();
                if (gT_max > 1e-12) dirT /= gT_max;
            }
        }

        // 回溯线搜索
        bool accepted = false;
        double trial_step = step;
        for (int ls = 0; ls < std::max(1, params.line_search_iters); ++ls) {
            Eigen::MatrixXd qn = q + trial_step * dirq;
            Eigen::VectorXd Tn = T + trial_step * dirT;
            for (int i = 0; i < N; ++i) {
                // ★ 段时长的下界必须按**该段几何长度**算，不能用常数 t_min
                //   （2026-11-17 速度爆炸根因）
                //
                // 事故（/tmp/azj_flight_20261117_181457）：
                //   MINCO ok: len=6.14m dur=1.38s min_clr=0 vmax=10.27
                //   v(t)= 0.00:0.63 0.20:1.86 0.40:2.57 0.60:9.22 0.80:0.60 1.00:10.13
                //
                //   原来 t_min=0.05：优化为了"最短时间"把每段 T 压到 0.05s，
                //   而段长仍有 1~2m → 需求速度 20~40 m/s → 7 次多项式系数
                //   爆到 1e+ 量级 → 速度曲线在段内剧烈振荡（9.22 与 0.60 交替）
                //   → vmax 7~10 m/s，飞机完全追不上（用户报"控制器追不上"）。
                //
                // 修法：T_i >= len_i / (max_vel * 1.3)，即该段至少要用
                //   1.3 倍的 max_vel 才可能走完的时间 —— 物理下界。
                //   系数 1.3 留余量给加速段（实际平均速度低于峰值速度）。
                const double len_i = (i == 0)
                                         ? (qn.col(0) - head.p).norm()
                                         : (i == N - 1 ? (tail.p - qn.col(N - 2)).norm()
                                                       : (qn.col(i) - qn.col(i - 1)).norm());
                const double t_floor =
                    std::max(params.t_min, len_i / std::max(0.2, params.max_vel * 1.3));
                Tn(i) = std::min(std::max(Tn(i), t_floor), params.t_max);
            }
            if (params.lock_z && std::isfinite(params.lock_z_value)) {
                qn.row(2).setConstant(params.lock_z_value);
            }
            if (params.enable_z_bounds) {
                for (int k = 0; k < N - 1; ++k) {
                    qn(2, k) = std::min(std::max(qn(2, k), params.z_min + z_ovs),
                                        params.z_max - z_ovs);
                }
            }

            Eigen::MatrixXd gqn(3, N - 1);
            Eigen::VectorXd gTn(N);
            double cn = 0.0;
            if (evalCost(qn, Tn, cn, gqn, gTn) && cn < cost) {
                q = qn;
                T = Tn;
                gq = gqn;
                gT = gTn;
                cost = cn;
                accepted = true;
                break;
            }
            trial_step *= 0.5;
        }

        if (accepted) {
            step = std::min(step * 1.3, params.step_init * 8.0);
            if (best_cost - cost > std::abs(best_cost) * params.rel_improve) {
                best_cost = cost;
                stall = 0;
            } else {
                ++stall;
            }
        } else {
            step *= 0.5;
            if (step < 1e-6) break;
            ++stall;
        }
        if (dbg) {
            std::printf("[minco-dbg] it=%d cost=%.6f step=%.4f gqmax=%.4g gTmax=%.4g "
                        "accepted=%d stall=%d qy_mid=%.3f\n",
                        it, cost, step, gq.cwiseAbs().maxCoeff(), gT.cwiseAbs().maxCoeff(),
                        (int)accepted, stall, (N > 1) ? q(1, (N - 1) / 2) : 0.0);
        }
        if (params.stall_iters > 0 && stall >= params.stall_iters) break;
    }

    // ─────────── 5) 安全硬投影（PBD 风格）───────────
    //
    // 软 hinge 权重 w_obs 在密集场景会被 w_mid/w_smooth 压过去，
    // 所以再沿 ∇c 把越界点对应的**内点**直接推开。
    // 与 curve_optimizer 的 projectSafe 同源思路，只是作用对象是内点。
    if (params.enable_projection && params.projection_iters > 0) {
        for (int pit = 0; pit < params.projection_iters; ++pit) {
            minco.setParameters(q, T);
            if (!minco.valid()) break;
            const auto& C = minco.coeffs();
            int nfixed = 0;
            for (int i = 0; i < N; ++i) {
                const double Ti = T(i);
                const int ns = std::max(2, params.projection_samples / std::max(1, N));
                const double ds = Ti / ns;
                const Eigen::Matrix<double, 8, 3> ci = C.block<8, 3>(8 * i, 0);
                for (int j = 1; j < ns; ++j) {
                    const double s = j * ds;
                    Eigen::Matrix<double, 8, 1> b0, b1, b2, b3;
                    betaBasis(s, b0, b1, b2, b3);
                    const Eigen::Vector3d pos = ci.transpose() * b0;
                    const double c = field.clearance(pos);
                    if (!std::isfinite(c) || c >= params.safe_margin) continue;
                    Eigen::Vector3d gc;
                    if (!field.clearanceGradient(pos, gc) || !gc.allFinite()) continue;
                    if (gc.norm() < 1e-6) continue;
                    // 归属内点：段内 s < 0.5 归段首内点，否则归段尾内点
                    const int qi = (s / Ti < 0.5) ? (i - 1) : i;
                    if (qi < 0 || qi >= N - 1) continue;
                    const double push = (params.safe_margin - c) * 0.6;
                    q.col(qi) += gc.normalized() * push;
                    ++nfixed;
                }
            }
            if (params.lock_z && std::isfinite(params.lock_z_value)) {
                q.row(2).setConstant(params.lock_z_value);
            }
            if (nfixed == 0) break;
        }
    }

    // ─────────── 6) 输出采样 ───────────
    minco.setParameters(q, T);
    if (!minco.valid()) {
        res.msg = "final setParameters failed";
        return res;
    }
    const double dur = minco.duration();
    if (!(dur > 1e-3)) {
        res.msg = "zero duration";
        return res;
    }

    const double dt = std::max(1e-3, params.sample_dt);
    const int ns = std::max(1, static_cast<int>(std::ceil(dur / dt)));
    std::vector<double> ts;
    std::vector<Eigen::Vector3d> ps, vs, as;
    ts.reserve(ns + 1);
    ps.reserve(ns + 1);
    vs.reserve(ns + 1);
    as.reserve(ns + 1);
    for (int k = 0; k <= ns; ++k) {
        const double t = std::min(dur, k * dur / ns);
        Eigen::Vector3d p, v, a, j;
        if (!minco.evaluatePVAJ(t, p, v, a, j)) continue;
        // 输出双保险：多项式段内 overshoot 若仍超出硬边界，直接钳回
        if (params.enable_z_bounds) {
            p.z() = std::min(std::max(p.z(), params.z_min + 0.01),
                             params.z_max - 0.01);
        }
        ts.push_back(t);
        ps.push_back(p);
        vs.push_back(v);
        as.push_back(a);
    }
    if (ts.size() < 2) {
        res.msg = "sampling produced too few points";
        return res;
    }
    if (!res.traj.assignSamples(ts, ps, vs, as)) {
        res.msg = "assignSamples failed";
        return res;
    }
    res.traj.setEndsAtRest(params.stop_at_end);

    // ─────────── 7) 统计 ───────────
    res.minco = std::move(minco);
    res.duration = dur;
    {
        double min_c = std::numeric_limits<double>::max();
        double sum_c = 0.0;
        int n = 0;
        double len = 0.0;
        double vmax = 0.0, amax = 0.0;
        for (size_t k = 0; k < ps.size(); ++k) {
            const double c = field.clearance(ps[k]);
            if (std::isfinite(c)) {
                min_c = std::min(min_c, c);
                sum_c += c;
                ++n;
            }
            if (k > 0) len += (ps[k] - ps[k - 1]).norm();
            vmax = std::max(vmax, vs[k].norm());
            amax = std::max(amax, as[k].norm());
        }
        res.min_clearance = (n > 0) ? min_c : 0.0;
        res.mean_clearance = (n > 0) ? sum_c / n : 0.0;
        res.length = len;
        res.max_vel = vmax;
        res.max_acc = amax;
    }
    res.success = true;
    res.msg = "ok";
    return res;
}

}  // namespace azj_planner
