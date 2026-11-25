#pragma once

// ══════════════════════════════════════════════════════════════════════════
//  MINCO（Minimum Control-effort polynomial trajectory）参数化
// ══════════════════════════════════════════════════════════════════════════
//
// 为什么需要它（2026-11-17 引入）：
//
//   之前的轨迹由「B 样条形状优化 + TOPP 时间参数化」两层拼成，起点只约束
//   **速度 v**，不约束加速度 a / 加加速度 j。实测后果
//   （/tmp/azj_roll_20261117_113332）：
//     · 每条新轨迹都从 v0 靠 max_acc **重新加速**
//     · 而 rolling 每 0.1~0.16s 重建一次
//     · → 速度永远停在加速段，|v0| 恒 0.285 m/s（max_vel=1.0）
//   当时用"取前方 0.4s 窗口峰值速度"绕过去了，但那是补偿，不是根治。
//
//   MINCO 的 setConditions(headPVAJ) 把起点 **p/v/a/j 全部硬约束**，
//   且段间是 C3 连续（位置/速度/加速度/加加速度都不跳）。
//   于是"从当前状态继续"是结构性保证，不依赖速度补偿。
//
// 数学（s=4，每段 7 次多项式，最小化 jerk 泛函）：
//   每段 p(t) = Σ_{i=0..7} c_i t^i
//   约束：段首/段尾的 p,v,a,j 连续；整条轨迹的首尾由 headPVAJ/tailPVAJ 给定
//   变量：内点位置 q_1..q_{N-1} + 每段时长 T_i
//   系数 c 通过一个**带状线性方程组** A·c = b 解出（A 只依赖 T）
//   → 求解复杂度 O(N)，与段数线性（不是 O(N³) 的通用 LU）
//
// 依赖 A 的伴随求解（A^T x = y）把代价梯度回传到 q 和 T，
// 所以对 q / T 都能给**解析梯度**，不需要数值差分。
//
// ── 来源与许可 ──────────────────────────────────────────────────────────
// 算法与线性系统构造移植自 SUPER（HKU MaRS Lab）的
//   include/utils/optimization/minco.h  /  src/utils/minco.cpp
// 其 MINCO 实现又源自 Zhepei Wang 的 MIT 许可版本（ego-planner 同源）。
// 本文件保留原始许可与署名，仅做了：
//   · 去掉 ROS / SUPER 数据类型依赖（不引入 Trajectory / Piece / StatePVAJ）
//   · 改名为 azj_planner 命名空间
//   · 增加直接求值接口 evaluate()（p/v/a/j），省掉外部 Piece 类
// MIT License / Copyright (c) 2021 Zhepei Wang
// ══════════════════════════════════════════════════════════════════════════

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace azj_planner {

namespace minco_impl {

/// 带状矩阵（按 "Matrix Computation" 的存储法）。
///
/// 为什么不用 Eigen 的通用 LU：MINCO 的系数矩阵 A 是 8N×8N 的带状矩阵
/// （下带宽 8、上带宽 8），通用 LU 是 O(N³)，带状 LU 是 O(N·bw²)。
/// N=20 段时差距约两个数量级，而这是**每次重规划都要跑**的路径。
///
/// 注意：不做主元选取（no pivot）。A 在正常 T 下是良态的；
/// 若某段 T→0 会退化，所以上层必须给 T 加下限。
class BandedSystem {
public:
    BandedSystem() = default;
    ~BandedSystem() { destroy(); }

    // 持有裸指针（A 的唯一所有者）→ 禁拷贝，但支持移动。
    BandedSystem(const BandedSystem&) = delete;
    BandedSystem& operator=(const BandedSystem&) = delete;

    BandedSystem(BandedSystem&& o) noexcept
        : N_(o.N_), lowerBw_(o.lowerBw_), upperBw_(o.upperBw_), data_(o.data_) {
        o.data_ = nullptr;
        o.N_ = 0;
    }
    BandedSystem& operator=(BandedSystem&& o) noexcept {
        if (this != &o) {
            destroy();
            N_ = o.N_;
            lowerBw_ = o.lowerBw_;
            upperBw_ = o.upperBw_;
            data_ = o.data_;
            o.data_ = nullptr;
            o.N_ = 0;
        }
        return *this;
    }

    void create(int n, int p, int q) {
        destroy();
        N_ = n;
        lowerBw_ = p;
        upperBw_ = q;
        const int sz = N_ * (lowerBw_ + upperBw_ + 1);
        data_ = new double[sz];
        std::fill_n(data_, sz, 0.0);
    }

    void destroy() {
        if (data_ != nullptr) {
            delete[] data_;
            data_ = nullptr;
        }
    }

    void reset() { std::fill_n(data_, N_ * (lowerBw_ + upperBw_ + 1), 0.0); }

    const double& operator()(int i, int j) const {
        return data_[(i - j + upperBw_) * N_ + j];
    }
    double& operator()(int i, int j) { return data_[(i - j + upperBw_) * N_ + j]; }

    /// 原地 LU 分解（无主元）。
    void factorizeLU() {
        for (int k = 0; k <= N_ - 2; ++k) {
            const int iM = std::min(k + lowerBw_, N_ - 1);
            const double cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; ++i) {
                if (operator()(i, k) != 0.0) operator()(i, k) /= cVl;
            }
            const int jM = std::min(k + upperBw_, N_ - 1);
            for (int j = k + 1; j <= jM; ++j) {
                const double cv = operator()(k, j);
                if (cv != 0.0) {
                    for (int i = k + 1; i <= iM; ++i) {
                        if (operator()(i, k) != 0.0) {
                            operator()(i, j) -= operator()(i, k) * cv;
                        }
                    }
                }
            }
        }
    }

    /// 解 A x = b（b 会被就地覆盖为 x）。b 为 N×m。
    template <typename EIGENMAT>
    void solve(EIGENMAT& b) const {
        for (int j = 0; j <= N_ - 1; ++j) {
            const int iM = std::min(j + lowerBw_, N_ - 1);
            for (int i = j + 1; i <= iM; ++i) {
                if (operator()(i, j) != 0.0) b.row(i) -= operator()(i, j) * b.row(j);
            }
        }
        for (int j = N_ - 1; j >= 0; --j) {
            b.row(j) /= operator()(j, j);
            const int iM = std::max(0, j - upperBw_);
            for (int i = iM; i <= j - 1; ++i) {
                if (operator()(i, j) != 0.0) b.row(i) -= operator()(i, j) * b.row(j);
            }
        }
    }

    /// 解 A^T x = b（伴随求解，用于把梯度回传）。
    template <typename EIGENMAT>
    void solveAdj(EIGENMAT& b) const {
        for (int j = 0; j <= N_ - 1; ++j) {
            b.row(j) /= operator()(j, j);
            const int iM = std::min(j + upperBw_, N_ - 1);
            for (int i = j + 1; i <= iM; ++i) {
                if (operator()(j, i) != 0.0) b.row(i) -= operator()(j, i) * b.row(j);
            }
        }
        for (int j = N_ - 1; j >= 0; --j) {
            const int iM = std::max(0, j - lowerBw_);
            for (int i = iM; i <= j - 1; ++i) {
                if (operator()(j, i) != 0.0) b.row(i) -= operator()(j, i) * b.row(j);
            }
        }
    }

private:
    int N_{0};
    int lowerBw_{0};
    int upperBw_{0};
    double* data_{nullptr};
};

}  // namespace minco_impl

/// 起点/终点的 (位置, 速度, 加速度, 加加速度) 约束。
struct StatePVAJ {
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Vector3d j{Eigen::Vector3d::Zero()};

    /// 转成 MINCO 需要的 3×4 矩阵（列序 p, v, a, j）。
    Eigen::Matrix<double, 3, 4> toMat() const {
        Eigen::Matrix<double, 3, 4> m;
        m.col(0) = p;
        m.col(1) = v;
        m.col(2) = a;
        m.col(3) = j;
        return m;
    }
};

/// MINCO s=4（jerk 泛函最小 / 每段 7 次多项式 / 非均匀时间）。
class MincoS4NU {
public:
    MincoS4NU() = default;
    // 成员含 BandedSystem（裸指针）→ 禁拷贝、允许移动（默认成员移动即可）
    MincoS4NU(const MincoS4NU&) = delete;
    MincoS4NU& operator=(const MincoS4NU&) = delete;
    MincoS4NU(MincoS4NU&&) noexcept = default;
    MincoS4NU& operator=(MincoS4NU&&) noexcept = default;

    /// 设定首尾状态与段数。必须在 setParameters 之前调用。
    void setConditions(const Eigen::Matrix<double, 3, 4>& headState,
                       const Eigen::Matrix<double, 3, 4>& tailState, int pieceNum);

    /// 只改终点位置（保持其余终点状态），用于滚动窗口逐次前推。
    void setEndPosition(const Eigen::Vector3d& end_p) { tailPVAJ_.col(0) = end_p; }

    /// 求解多项式系数。
    /// @param inPs 3×(N-1) 的内点位置（**不含**首尾）
    /// @param ts   长度 N 的段时长
    void setParameters(const Eigen::MatrixXd& inPs, const Eigen::VectorXd& ts);

    int pieceNum() const { return N_; }
    bool valid() const { return N_ > 0 && b_.rows() == 8 * N_; }

    const Eigen::MatrixX3d& coeffs() const { return b_; }

    /// 求值。t 为**整条轨迹**上的时间（非归一化），会被钳到 [0, duration]。
    /// deriv: 0=位置 1=速度 2=加速度 3=加加速度
    bool evaluate(double t, int deriv, Eigen::Vector3d& out) const;

    /// 一次求全 p/v/a/j，并返回所在段索引（调试/可视化用）。
    bool evaluatePVAJ(double t, Eigen::Vector3d& p, Eigen::Vector3d& v,
                      Eigen::Vector3d& a, Eigen::Vector3d& j, int* piece = nullptr) const;

    double duration() const { return T1_.size() ? T1_.sum() : 0.0; }
    double pieceDuration(int i) const { return (i >= 0 && i < T1_.size()) ? T1_(i) : 0.0; }

    /// 段内最小/最大速度与加速度（用于可行性校验与降速）。
    void sampleMaxVA(double dt, double& max_v, double& max_a) const;

    /// jerk 泛函 ∫|p'''|² dt（平滑度代价，越小越"顺"）。
    double getEnergy() const;
    void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d& gdC) const;
    void getEnergyPartialGradByTimes(Eigen::VectorXd& gdT) const;

    /// 把系数梯度 + 时间梯度回传成「内点位置梯度 + 段时长梯度」。
    /// @param free_end true 时 gradByPoints 含终点（3×N），否则 3×(N-1)
    void propogateGrad(const Eigen::MatrixX3d& partialGradByCoeffs,
                       const Eigen::VectorXd& partialGradByTimes,
                       Eigen::Matrix3Xd& gradByPoints, Eigen::VectorXd& gradByTimes,
                       bool free_end = false);

private:
    int N_{0};
    Eigen::Matrix<double, 3, 4> headPVAJ_;
    Eigen::Matrix<double, 3, 4> tailPVAJ_;
    minco_impl::BandedSystem A_;
    Eigen::MatrixX3d b_;   ///< (8N)×3，index 8i+k 是第 i 段 t^k 的系数
    Eigen::VectorXd T1_, T2_, T3_, T4_, T5_, T6_, T7_;
};

}  // namespace azj_planner
