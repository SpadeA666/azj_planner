// MINCO s=4 实现。算法与线性系统构造移植自 SUPER / Zhepei Wang（MIT），
// 详见 minco.h 顶部的许可说明。

#include "azj_planner/minco.h"

#include <cmath>

namespace azj_planner {

void MincoS4NU::setConditions(const Eigen::Matrix<double, 3, 4>& headState,
                              const Eigen::Matrix<double, 3, 4>& tailState, int pieceNum) {
    N_ = std::max(1, pieceNum);
    headPVAJ_ = headState;
    tailPVAJ_ = tailState;
    // 每段 8 个未知系数 → 8N 阶；下/上带宽均为 8
    A_.create(8 * N_, 8, 8);
    b_.resize(8 * N_, 3);
    b_.setZero();
    T1_.resize(N_);
    T2_.resize(N_);
    T3_.resize(N_);
    T4_.resize(N_);
    T5_.resize(N_);
    T6_.resize(N_);
    T7_.resize(N_);
}

void MincoS4NU::setParameters(const Eigen::MatrixXd& inPs, const Eigen::VectorXd& ts) {
    if (N_ <= 0) return;
    if (ts.size() != N_) return;
    if (N_ > 1 && (inPs.rows() != 3 || inPs.cols() != N_ - 1)) return;

    T1_ = ts;
    T2_ = T1_.cwiseProduct(T1_);
    T3_ = T2_.cwiseProduct(T1_);
    T4_ = T2_.cwiseProduct(T2_);
    T5_ = T4_.cwiseProduct(T1_);
    T6_ = T4_.cwiseProduct(T2_);
    T7_ = T4_.cwiseProduct(T3_);

    A_.reset();
    b_.setZero();

    // ── 段首约束：p, v, a, j（左端固定，右端 = 该段的 "内点"）──
    //
    // 未知量排列：第 i 段占 8i..8i+7，b 的 8i+7 行存放该段终点位置。
    A_(0, 0) = 1.0;
    A_(1, 1) = 1.0;
    A_(2, 2) = 2.0;
    A_(3, 3) = 6.0;
    b_.row(0) = headPVAJ_.col(0).transpose();
    b_.row(1) = headPVAJ_.col(1).transpose();
    b_.row(2) = headPVAJ_.col(2).transpose();
    b_.row(3) = headPVAJ_.col(3).transpose();

    for (int i = 0; i < N_ - 1; ++i) {
        // 段间连续性：第 i 段末端与第 i+1 段首端的 jerk/acc/vel/pos 相等
        A_(8 * i + 4, 8 * i + 4) = 24.0;
        A_(8 * i + 4, 8 * i + 5) = 120.0 * T1_(i);
        A_(8 * i + 4, 8 * i + 6) = 360.0 * T2_(i);
        A_(8 * i + 4, 8 * i + 7) = 840.0 * T3_(i);
        A_(8 * i + 4, 8 * i + 12) = -24.0;

        A_(8 * i + 5, 8 * i + 5) = 120.0;
        A_(8 * i + 5, 8 * i + 6) = 720.0 * T1_(i);
        A_(8 * i + 5, 8 * i + 7) = 2520.0 * T2_(i);
        A_(8 * i + 5, 8 * i + 13) = -120.0;

        A_(8 * i + 6, 8 * i + 6) = 720.0;
        A_(8 * i + 6, 8 * i + 7) = 5040.0 * T1_(i);
        A_(8 * i + 6, 8 * i + 14) = -720.0;

        A_(8 * i + 7, 8 * i) = 1.0;
        A_(8 * i + 7, 8 * i + 1) = T1_(i);
        A_(8 * i + 7, 8 * i + 2) = T2_(i);
        A_(8 * i + 7, 8 * i + 3) = T3_(i);
        A_(8 * i + 7, 8 * i + 4) = T4_(i);
        A_(8 * i + 7, 8 * i + 5) = T5_(i);
        A_(8 * i + 7, 8 * i + 6) = T6_(i);
        A_(8 * i + 7, 8 * i + 7) = T7_(i);
        A_(8 * i + 8, 8 * i) = 1.0;
        A_(8 * i + 8, 8 * i + 1) = T1_(i);
        A_(8 * i + 8, 8 * i + 2) = T2_(i);
        A_(8 * i + 8, 8 * i + 3) = T3_(i);
        A_(8 * i + 8, 8 * i + 4) = T4_(i);
        A_(8 * i + 8, 8 * i + 5) = T5_(i);
        A_(8 * i + 8, 8 * i + 6) = T6_(i);
        A_(8 * i + 8, 8 * i + 7) = T7_(i);
        A_(8 * i + 8, 8 * i + 8) = -1.0;
        A_(8 * i + 9, 8 * i + 1) = 1.0;
        A_(8 * i + 9, 8 * i + 2) = 2.0 * T1_(i);
        A_(8 * i + 9, 8 * i + 3) = 3.0 * T2_(i);
        A_(8 * i + 9, 8 * i + 4) = 4.0 * T3_(i);
        A_(8 * i + 9, 8 * i + 5) = 5.0 * T4_(i);
        A_(8 * i + 9, 8 * i + 6) = 6.0 * T5_(i);
        A_(8 * i + 9, 8 * i + 7) = 7.0 * T6_(i);
        A_(8 * i + 9, 8 * i + 9) = -1.0;
        A_(8 * i + 10, 8 * i + 2) = 2.0;
        A_(8 * i + 10, 8 * i + 3) = 6.0 * T1_(i);
        A_(8 * i + 10, 8 * i + 4) = 12.0 * T2_(i);
        A_(8 * i + 10, 8 * i + 5) = 20.0 * T3_(i);
        A_(8 * i + 10, 8 * i + 6) = 30.0 * T4_(i);
        A_(8 * i + 10, 8 * i + 7) = 42.0 * T5_(i);
        A_(8 * i + 10, 8 * i + 10) = -2.0;
        A_(8 * i + 11, 8 * i + 3) = 6.0;
        A_(8 * i + 11, 8 * i + 4) = 24.0 * T1_(i);
        A_(8 * i + 11, 8 * i + 5) = 60.0 * T2_(i);
        A_(8 * i + 11, 8 * i + 6) = 120.0 * T3_(i);
        A_(8 * i + 11, 8 * i + 7) = 210.0 * T4_(i);
        A_(8 * i + 11, 8 * i + 11) = -6.0;

        b_.row(8 * i + 7) = inPs.col(i).transpose();
    }

    // ── 段尾约束：最后一段末端的 p, v, a, j = tailPVAJ ──
    A_(8 * N_ - 4, 8 * N_ - 8) = 1.0;
    A_(8 * N_ - 4, 8 * N_ - 7) = T1_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 6) = T2_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 5) = T3_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 4) = T4_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 3) = T5_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 2) = T6_(N_ - 1);
    A_(8 * N_ - 4, 8 * N_ - 1) = T7_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 7) = 1.0;
    A_(8 * N_ - 3, 8 * N_ - 6) = 2.0 * T1_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 5) = 3.0 * T2_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 4) = 4.0 * T3_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 3) = 5.0 * T4_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 2) = 6.0 * T5_(N_ - 1);
    A_(8 * N_ - 3, 8 * N_ - 1) = 7.0 * T6_(N_ - 1);
    A_(8 * N_ - 2, 8 * N_ - 6) = 2.0;
    A_(8 * N_ - 2, 8 * N_ - 5) = 6.0 * T1_(N_ - 1);
    A_(8 * N_ - 2, 8 * N_ - 4) = 12.0 * T2_(N_ - 1);
    A_(8 * N_ - 2, 8 * N_ - 3) = 20.0 * T3_(N_ - 1);
    A_(8 * N_ - 2, 8 * N_ - 2) = 30.0 * T4_(N_ - 1);
    A_(8 * N_ - 2, 8 * N_ - 1) = 42.0 * T5_(N_ - 1);
    A_(8 * N_ - 1, 8 * N_ - 5) = 6.0;
    A_(8 * N_ - 1, 8 * N_ - 4) = 24.0 * T1_(N_ - 1);
    A_(8 * N_ - 1, 8 * N_ - 3) = 60.0 * T2_(N_ - 1);
    A_(8 * N_ - 1, 8 * N_ - 2) = 120.0 * T3_(N_ - 1);
    A_(8 * N_ - 1, 8 * N_ - 1) = 210.0 * T4_(N_ - 1);

    b_.row(8 * N_ - 4) = tailPVAJ_.col(0).transpose();
    b_.row(8 * N_ - 3) = tailPVAJ_.col(1).transpose();
    b_.row(8 * N_ - 2) = tailPVAJ_.col(2).transpose();
    b_.row(8 * N_ - 1) = tailPVAJ_.col(3).transpose();

    A_.factorizeLU();
    A_.solve(b_);
}

bool MincoS4NU::evaluate(double t, int deriv, Eigen::Vector3d& out) const {
    Eigen::Vector3d p, v, a, j;
    if (!evaluatePVAJ(t, p, v, a, j)) return false;
    switch (deriv) {
        case 0: out = p; break;
        case 1: out = v; break;
        case 2: out = a; break;
        case 3: out = j; break;
        default: return false;
    }
    return true;
}

bool MincoS4NU::evaluatePVAJ(double t, Eigen::Vector3d& p, Eigen::Vector3d& v,
                             Eigen::Vector3d& a, Eigen::Vector3d& j, int* piece) const {
    if (!valid()) return false;
    const double dur = duration();
    if (dur <= 0.0) return false;
    t = std::min(std::max(t, 0.0), dur);

    // 定位段：前缀和比较
    int i = 0;
    double t0 = 0.0;
    while (i < N_ - 1 && t > t0 + T1_(i)) {
        t0 += T1_(i);
        ++i;
    }
    const double tau = std::min(std::max(t - t0, 0.0), T1_(i));
    if (piece) *piece = i;

    // 第 i 段的系数：b_.row(8i + k) 是 t^k 的系数（k=0..7）
    const int base = 8 * i;
    const double t1 = tau;
    const double t2 = t1 * t1;
    const double t3 = t2 * t1;
    const double t4 = t2 * t2;
    const double t5 = t4 * t1;
    const double t6 = t4 * t2;
    const double t7 = t4 * t3;

    const auto& c0 = b_.row(base + 0);
    const auto& c1 = b_.row(base + 1);
    const auto& c2 = b_.row(base + 2);
    const auto& c3 = b_.row(base + 3);
    const auto& c4 = b_.row(base + 4);
    const auto& c5 = b_.row(base + 5);
    const auto& c6 = b_.row(base + 6);
    const auto& c7 = b_.row(base + 7);

    p = c0 + t1 * c1 + t2 * c2 + t3 * c3 + t4 * c4 + t5 * c5 + t6 * c6 + t7 * c7;
    v = c1 + 2.0 * t1 * c2 + 3.0 * t2 * c3 + 4.0 * t3 * c4 + 5.0 * t4 * c5 +
        6.0 * t5 * c6 + 7.0 * t6 * c7;
    a = 2.0 * c2 + 6.0 * t1 * c3 + 12.0 * t2 * c4 + 20.0 * t3 * c5 + 30.0 * t4 * c6 +
        42.0 * t5 * c7;
    j = 6.0 * c3 + 24.0 * t1 * c4 + 60.0 * t2 * c5 + 120.0 * t3 * c6 + 210.0 * t4 * c7;

    return p.allFinite() && v.allFinite() && a.allFinite() && j.allFinite();
}

void MincoS4NU::sampleMaxVA(double dt, double& max_v, double& max_a) const {
    max_v = 0.0;
    max_a = 0.0;
    if (!valid()) return;
    const double dur = duration();
    if (dur <= 0.0) return;
    dt = std::max(1e-3, dt);
    for (double t = 0.0; t <= dur + 1e-9; t += dt) {
        Eigen::Vector3d p, v, a, j;
        if (!evaluatePVAJ(t, p, v, a, j)) continue;
        max_v = std::max(max_v, v.norm());
        max_a = std::max(max_a, a.norm());
    }
}

double MincoS4NU::getEnergy() const {
    double e = 0.0;
    if (!valid()) return e;
    for (int i = 0; i < N_; ++i) {
        e += 576.0 * b_.row(8 * i + 4).squaredNorm() * T1_(i) +
             2880.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 5)) * T2_(i) +
             4800.0 * b_.row(8 * i + 5).squaredNorm() * T3_(i) +
             5760.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 6)) * T3_(i) +
             21600.0 * b_.row(8 * i + 5).dot(b_.row(8 * i + 6)) * T4_(i) +
             10080.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 7)) * T4_(i) +
             25920.0 * b_.row(8 * i + 6).squaredNorm() * T5_(i) +
             40320.0 * b_.row(8 * i + 5).dot(b_.row(8 * i + 7)) * T5_(i) +
             100800.0 * b_.row(8 * i + 6).dot(b_.row(8 * i + 7)) * T6_(i) +
             100800.0 * b_.row(8 * i + 7).squaredNorm() * T7_(i);
    }
    return e;
}

void MincoS4NU::getEnergyPartialGradByCoeffs(Eigen::MatrixX3d& gdC) const {
    gdC.resize(8 * N_, 3);
    gdC.setZero();
    for (int i = 0; i < N_; ++i) {
        gdC.row(8 * i + 7) = 10080.0 * b_.row(8 * i + 4) * T4_(i) +
                             40320.0 * b_.row(8 * i + 5) * T5_(i) +
                             100800.0 * b_.row(8 * i + 6) * T6_(i) +
                             201600.0 * b_.row(8 * i + 7) * T7_(i);
        gdC.row(8 * i + 6) = 5760.0 * b_.row(8 * i + 4) * T3_(i) +
                             21600.0 * b_.row(8 * i + 5) * T4_(i) +
                             51840.0 * b_.row(8 * i + 6) * T5_(i) +
                             100800.0 * b_.row(8 * i + 7) * T6_(i);
        gdC.row(8 * i + 5) = 2880.0 * b_.row(8 * i + 4) * T2_(i) +
                             9600.0 * b_.row(8 * i + 5) * T3_(i) +
                             21600.0 * b_.row(8 * i + 6) * T4_(i) +
                             40320.0 * b_.row(8 * i + 7) * T5_(i);
        gdC.row(8 * i + 4) = 1152.0 * b_.row(8 * i + 4) * T1_(i) +
                             2880.0 * b_.row(8 * i + 5) * T2_(i) +
                             5760.0 * b_.row(8 * i + 6) * T3_(i) +
                             10080.0 * b_.row(8 * i + 7) * T4_(i);
        // 前 4 行（常数/一次/二次/三次）不含在 jerk 泛函里
        gdC.block<4, 3>(8 * i, 0).setZero();
    }
}

void MincoS4NU::getEnergyPartialGradByTimes(Eigen::VectorXd& gdT) const {
    gdT.resize(N_);
    for (int i = 0; i < N_; ++i) {
        gdT(i) = 576.0 * b_.row(8 * i + 4).squaredNorm() +
                 5760.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 5)) * T1_(i) +
                 14400.0 * b_.row(8 * i + 5).squaredNorm() * T2_(i) +
                 17280.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 6)) * T2_(i) +
                 86400.0 * b_.row(8 * i + 5).dot(b_.row(8 * i + 6)) * T3_(i) +
                 40320.0 * b_.row(8 * i + 4).dot(b_.row(8 * i + 7)) * T3_(i) +
                 129600.0 * b_.row(8 * i + 6).squaredNorm() * T4_(i) +
                 201600.0 * b_.row(8 * i + 5).dot(b_.row(8 * i + 7)) * T4_(i) +
                 604800.0 * b_.row(8 * i + 6).dot(b_.row(8 * i + 7)) * T5_(i) +
                 705600.0 * b_.row(8 * i + 7).squaredNorm() * T6_(i);
    }
}

void MincoS4NU::propogateGrad(const Eigen::MatrixX3d& partialGradByCoeffs,
                              const Eigen::VectorXd& partialGradByTimes,
                              Eigen::Matrix3Xd& gradByPoints, Eigen::VectorXd& gradByTimes,
                              bool free_end) {
    if (!valid()) return;
    gradByTimes.resize(N_);
    gradByTimes.setZero();
    Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
    A_.solveAdj(adjGrad);

    const int npts = free_end ? N_ : N_ - 1;
    gradByPoints.resize(3, npts);

    for (int i = 0; i < N_ - 1; ++i) {
        gradByPoints.col(i) = adjGrad.row(8 * i + 7).transpose();
    }
    if (free_end) {
        gradByPoints.col(N_ - 1) = adjGrad.row(8 * N_ - 4).transpose();
    }

    Eigen::Matrix<double, 8, 3> B1;
    Eigen::Matrix<double, 4, 3> B2;
    for (int i = 0; i < N_ - 1; ++i) {
        // 段末各阶导（负号 = 对 T 求导的链式符号）
        B1.row(3) = -(b_.row(i * 8 + 1) + 2.0 * T1_(i) * b_.row(i * 8 + 2) +
                      3.0 * T2_(i) * b_.row(i * 8 + 3) + 4.0 * T3_(i) * b_.row(i * 8 + 4) +
                      5.0 * T4_(i) * b_.row(i * 8 + 5) + 6.0 * T5_(i) * b_.row(i * 8 + 6) +
                      7.0 * T6_(i) * b_.row(i * 8 + 7));
        B1.row(4) = B1.row(3);
        B1.row(5) = -(2.0 * b_.row(i * 8 + 2) + 6.0 * T1_(i) * b_.row(i * 8 + 3) +
                      12.0 * T2_(i) * b_.row(i * 8 + 4) + 20.0 * T3_(i) * b_.row(i * 8 + 5) +
                      30.0 * T4_(i) * b_.row(i * 8 + 6) + 42.0 * T5_(i) * b_.row(i * 8 + 7));
        B1.row(6) = -(6.0 * b_.row(i * 8 + 3) + 24.0 * T1_(i) * b_.row(i * 8 + 4) +
                      60.0 * T2_(i) * b_.row(i * 8 + 5) + 120.0 * T3_(i) * b_.row(i * 8 + 6) +
                      210.0 * T4_(i) * b_.row(i * 8 + 7));
        B1.row(7) = -(24.0 * b_.row(i * 8 + 4) + 120.0 * T1_(i) * b_.row(i * 8 + 5) +
                      360.0 * T2_(i) * b_.row(i * 8 + 6) +
                      840.0 * T3_(i) * b_.row(i * 8 + 7));
        B1.row(0) = -(120.0 * b_.row(i * 8 + 5) + 720.0 * T1_(i) * b_.row(i * 8 + 6) +
                      2520.0 * T2_(i) * b_.row(i * 8 + 7));
        B1.row(1) = -(720.0 * b_.row(i * 8 + 6) + 5040.0 * T1_(i) * b_.row(i * 8 + 7));
        B1.row(2) = -5040.0 * b_.row(i * 8 + 7);

        gradByTimes(i) = B1.cwiseProduct(adjGrad.block<8, 3>(8 * i + 4, 0)).sum();
    }

    B2.row(0) = -(b_.row(8 * N_ - 7) + 2.0 * T1_(N_ - 1) * b_.row(8 * N_ - 6) +
                  3.0 * T2_(N_ - 1) * b_.row(8 * N_ - 5) +
                  4.0 * T3_(N_ - 1) * b_.row(8 * N_ - 4) +
                  5.0 * T4_(N_ - 1) * b_.row(8 * N_ - 3) +
                  6.0 * T5_(N_ - 1) * b_.row(8 * N_ - 2) +
                  7.0 * T6_(N_ - 1) * b_.row(8 * N_ - 1));
    B2.row(1) = -(2.0 * b_.row(8 * N_ - 6) + 6.0 * T1_(N_ - 1) * b_.row(8 * N_ - 5) +
                  12.0 * T2_(N_ - 1) * b_.row(8 * N_ - 4) +
                  20.0 * T3_(N_ - 1) * b_.row(8 * N_ - 3) +
                  30.0 * T4_(N_ - 1) * b_.row(8 * N_ - 2) +
                  42.0 * T5_(N_ - 1) * b_.row(8 * N_ - 1));
    B2.row(2) = -(6.0 * b_.row(8 * N_ - 5) + 24.0 * T1_(N_ - 1) * b_.row(8 * N_ - 4) +
                  60.0 * T2_(N_ - 1) * b_.row(8 * N_ - 3) +
                  120.0 * T3_(N_ - 1) * b_.row(8 * N_ - 2) +
                  210.0 * T4_(N_ - 1) * b_.row(8 * N_ - 1));
    B2.row(3) = -(24.0 * b_.row(8 * N_ - 4) + 120.0 * T1_(N_ - 1) * b_.row(8 * N_ - 3) +
                  360.0 * T2_(N_ - 1) * b_.row(8 * N_ - 2) +
                  840.0 * T3_(N_ - 1) * b_.row(8 * N_ - 1));

    gradByTimes(N_ - 1) = B2.cwiseProduct(adjGrad.block<4, 3>(8 * N_ - 4, 0)).sum();
    gradByTimes += partialGradByTimes;
}

}  // namespace azj_planner
