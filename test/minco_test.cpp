// MINCO 核心数学验证。
//
// 移植第三方数值代码最容易出的错不是编译失败，而是"看起来能跑、数字不对"。
// 所以这里不测"轨迹好不好看"，只测**数学是否严格成立**：
//   1. 起点 p/v/a/j 精确等于给定约束（硬约束是 MINCO 的全部价值所在）
//   2. 终点 p/v/a/j 精确等于给定约束
//   3. 段间位置/速度/加速度/jerk 连续（C3）
//   4. jerk 泛函对系数的梯度 == 数值差分
//   5. jerk 泛函对段时长的梯度 == 数值差分
//   6. propogateGrad 回传的内点梯度 == 数值差分（链式法则没写错）
//   7. 采样接口（Hermite）与多项式解析值一致

#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/Dense>

#include "azj_planner/minco.h"
#include "azj_planner/trajectory.h"

using namespace azj_planner;

static int g_pass = 0;
static int g_fail = 0;

static void check(bool ok, const char* name, double val = 0.0, double tol = 0.0) {
    if (ok) {
        ++g_pass;
        std::printf("  [PASS] %s\n", name);
    } else {
        ++g_fail;
        if (tol > 0.0) {
            std::printf("  [FAIL] %s  (val=%.6g, tol=%.6g)\n", name, val, tol);
        } else {
            std::printf("  [FAIL] %s\n", name);
        }
    }
}

// 构造一个典型的 4 段轨迹：起点在原点带速度/加速度，终点在 (4,1,0.5)
static MincoS4NU makeMinco(int N = 4) {
    StatePVAJ head;
    head.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    head.v = Eigen::Vector3d(0.6, 0.1, 0.0);
    head.a = Eigen::Vector3d(0.3, 0.05, 0.0);
    head.j = Eigen::Vector3d(0.0, 0.0, 0.0);

    StatePVAJ tail;
    tail.p = Eigen::Vector3d(4.0, 1.0, 0.5);
    tail.v = Eigen::Vector3d::Zero();
    tail.a = Eigen::Vector3d::Zero();
    tail.j = Eigen::Vector3d::Zero();

    MincoS4NU m;
    m.setConditions(head.toMat(), tail.toMat(), N);

    Eigen::MatrixXd q(3, N - 1);
    for (int k = 1; k <= N - 1; ++k) {
        const double f = static_cast<double>(k) / N;
        q.col(k - 1) = head.p + f * (tail.p - head.p);
        q(1, k - 1) += 0.15 * std::sin(3.14159 * f);   // 稍微弯曲
    }
    Eigen::VectorXd T(N);
    for (int i = 0; i < N; ++i) T(i) = 1.2;
    m.setParameters(q, T);
    return m;
}

int main() {
    std::printf("========== MINCO 核心数学验证 ==========\n");

    // ---------- [1] 起点硬约束 ----------
    {
        std::printf("\n[1] 起点 p/v/a/j 硬约束\n");
        const MincoS4NU m = makeMinco();
        Eigen::Vector3d p, v, a, j;
        check(m.evaluatePVAJ(0.0, p, v, a, j), "evaluate at t=0 成功");

        const Eigen::Vector3d ep(0.0, 0.0, 0.0), ev(0.6, 0.1, 0.0), ea(0.3, 0.05, 0.0);
        check((p - ep).norm() < 1e-9, "起点位置 == 约束", (p - ep).norm(), 1e-9);
        check((v - ev).norm() < 1e-9, "起点速度 == 约束", (v - ev).norm(), 1e-9);
        check((a - ea).norm() < 1e-9, "起点加速度 == 约束", (a - ea).norm(), 1e-9);
        check(j.norm() < 1e-9, "起点 jerk == 约束(0)", j.norm(), 1e-9);
    }

    // ---------- [2] 终点硬约束 ----------
    {
        std::printf("\n[2] 终点 p/v/a/j 硬约束\n");
        const MincoS4NU m = makeMinco();
        Eigen::Vector3d p, v, a, j;
        check(m.evaluatePVAJ(m.duration(), p, v, a, j), "evaluate at t=T 成功");
        const Eigen::Vector3d ep(4.0, 1.0, 0.5);
        check((p - ep).norm() < 1e-9, "终点位置 == 约束", (p - ep).norm(), 1e-9);
        check(v.norm() < 1e-9, "终点速度 == 0", v.norm(), 1e-9);
        check(a.norm() < 1e-9, "终点加速度 == 0", a.norm(), 1e-9);
        check(j.norm() < 1e-9, "终点 jerk == 0", j.norm(), 1e-9);
    }

    // ---------- [3] 段间 C3 连续 ----------
    {
        std::printf("\n[3] 段间连续性（C3）\n");
        const MincoS4NU m = makeMinco(4);
        double t = 0.0;
        double max_dp = 0, max_dv = 0, max_da = 0, max_dj = 0;
        for (int i = 0; i < 3; ++i) {
            t += m.pieceDuration(i);
            Eigen::Vector3d p1, v1, a1, j1, p2, v2, a2, j2;
            m.evaluatePVAJ(t - 1e-7, p1, v1, a1, j1);
            m.evaluatePVAJ(t + 1e-7, p2, v2, a2, j2);
            max_dp = std::max(max_dp, (p1 - p2).norm());
            max_dv = std::max(max_dv, (v1 - v2).norm());
            max_da = std::max(max_da, (a1 - a2).norm());
            max_dj = std::max(max_dj, (j1 - j2).norm());
        }
        // 用 ±1e-7 的差分，容许变化率量级 * 2e-7
        check(max_dp < 1e-5, "位置连续", max_dp, 1e-5);
        check(max_dv < 1e-4, "速度连续", max_dv, 1e-4);
        check(max_da < 1e-3, "加速度连续", max_da, 1e-3);
        check(max_dj < 1e-2, "jerk 连续", max_dj, 1e-2);
    }

    // ---------- [4] 能量对系数的梯度 ----------
    {
        std::printf("\n[4] jerk 能量对系数的梯度（vs 数值差分）\n");
        MincoS4NU m = makeMinco(3);
        const double e0 = m.getEnergy();

        Eigen::MatrixX3d gdC;
        m.getEnergyPartialGradByCoeffs(gdC);

        // 扰动第 i 段的第 5 个系数（t^5 项，能量里含它）
        const int seg = 1, k = 5;
        const double h = 1e-6;
        const Eigen::MatrixX3d C0 = m.coeffs();
        Eigen::MatrixX3d Cp = C0;
        Cp(8 * seg + k, 0) += h;

        // 用相同的 T、直接以新系数重建能量：需要绕开 setParameters（那会重解系数）
        // 这里改用"扰动内点"的方式验证 propogateGrad 更实际（见 [6]）。
        // 对纯能量项，用解析式独立复算：
        auto energyOf = [](const Eigen::MatrixX3d& C, int N, const Eigen::VectorXd& T) {
            Eigen::VectorXd T1 = T, T2 = T.cwiseProduct(T), T3 = T2.cwiseProduct(T);
            Eigen::VectorXd T4 = T2.cwiseProduct(T2), T5 = T4.cwiseProduct(T);
            Eigen::VectorXd T6 = T4.cwiseProduct(T2), T7 = T4.cwiseProduct(T3);
            double e = 0.0;
            for (int i = 0; i < N; ++i) {
                e += 576.0 * C.row(8 * i + 4).squaredNorm() * T1(i) +
                     2880.0 * C.row(8 * i + 4).dot(C.row(8 * i + 5)) * T2(i) +
                     4800.0 * C.row(8 * i + 5).squaredNorm() * T3(i) +
                     5760.0 * C.row(8 * i + 4).dot(C.row(8 * i + 6)) * T3(i) +
                     21600.0 * C.row(8 * i + 5).dot(C.row(8 * i + 6)) * T4(i) +
                     10080.0 * C.row(8 * i + 4).dot(C.row(8 * i + 7)) * T4(i) +
                     25920.0 * C.row(8 * i + 6).squaredNorm() * T5(i) +
                     40320.0 * C.row(8 * i + 5).dot(C.row(8 * i + 7)) * T5(i) +
                     100800.0 * C.row(8 * i + 6).dot(C.row(8 * i + 7)) * T6(i) +
                     100800.0 * C.row(8 * i + 7).squaredNorm() * T7(i);
            }
            return e;
        };
        Eigen::VectorXd T(3);
        for (int i = 0; i < 3; ++i) T(i) = m.pieceDuration(i);

        const double ep = energyOf(Cp, 3, T);
        const double num = (ep - energyOf(C0, 3, T)) / h;
        const double ana = gdC(8 * seg + k, 0);
        check(std::abs(num - ana) < 1e-3 * std::max(1.0, std::abs(num)),
              "dE/dc 解析 == 数值", std::abs(num - ana), 1e-3);
        (void)e0;
    }

    // ---------- [5] 能量对时间的梯度 ----------
    {
        std::printf("\n[5] jerk 能量对段时长的梯度（vs 数值差分）\n");
        MincoS4NU m = makeMinco(3);
        Eigen::VectorXd gdT;
        m.getEnergyPartialGradByTimes(gdT);

        auto energyWithT = [](int N, const Eigen::MatrixX3d& C, const Eigen::VectorXd& T) {
            Eigen::VectorXd T1 = T, T2 = T.cwiseProduct(T), T3 = T2.cwiseProduct(T);
            Eigen::VectorXd T4 = T2.cwiseProduct(T2), T5 = T4.cwiseProduct(T);
            Eigen::VectorXd T6 = T4.cwiseProduct(T2), T7 = T4.cwiseProduct(T3);
            double e = 0.0;
            for (int i = 0; i < N; ++i) {
                e += 576.0 * C.row(8 * i + 4).squaredNorm() * T1(i) +
                     2880.0 * C.row(8 * i + 4).dot(C.row(8 * i + 5)) * T2(i) +
                     4800.0 * C.row(8 * i + 5).squaredNorm() * T3(i) +
                     5760.0 * C.row(8 * i + 4).dot(C.row(8 * i + 6)) * T3(i) +
                     21600.0 * C.row(8 * i + 5).dot(C.row(8 * i + 6)) * T4(i) +
                     10080.0 * C.row(8 * i + 4).dot(C.row(8 * i + 7)) * T4(i) +
                     25920.0 * C.row(8 * i + 6).squaredNorm() * T5(i) +
                     40320.0 * C.row(8 * i + 5).dot(C.row(8 * i + 7)) * T5(i) +
                     100800.0 * C.row(8 * i + 6).dot(C.row(8 * i + 7)) * T6(i) +
                     100800.0 * C.row(8 * i + 7).squaredNorm() * T7(i);
            }
            return e;
        };
        // 注意：严格来说 T 变了系数也会变（要重解 A）。但 getEnergyPartialGradByTimes
        // 给的是"固定系数下对 T 的偏导"，这也是 SUPER 的用法（配合 propogateGrad
        // 里的伴随项才构成完整导数）。所以这里只验证这个偏导本身。
        Eigen::VectorXd T(3);
        for (int i = 0; i < 3; ++i) T(i) = m.pieceDuration(i);
        const double h = 1e-6;
        Eigen::VectorXd Tp = T;
        Tp(1) += h;
        const double num = (energyWithT(3, m.coeffs(), Tp) - energyWithT(3, m.coeffs(), T)) / h;
        const double ana = gdT(1);
        check(std::abs(num - ana) < 1e-3 * std::max(1.0, std::abs(num)),
              "dE/dT 解析 == 数值", std::abs(num - ana), 1e-3);
    }

    // ---------- [6] propogateGrad：内点梯度 vs 数值差分 ----------
    {
        std::printf("\n[6] propogateGrad 内点梯度（vs 数值差分）\n");
        StatePVAJ head, tail;
        head.p = Eigen::Vector3d(0, 0, 0);
        head.v = Eigen::Vector3d(0.6, 0, 0);
        tail.p = Eigen::Vector3d(3.0, 0.8, 0.2);
        const int N = 3;

        Eigen::MatrixXd q(3, N - 1);
        q.col(0) = Eigen::Vector3d(1.0, 0.5, 0.05);
        q.col(1) = Eigen::Vector3d(2.0, 0.7, 0.12);
        Eigen::VectorXd T(N);
        for (int i = 0; i < N; ++i) T(i) = 1.0;

        auto build = [&](const Eigen::MatrixXd& qq, const Eigen::VectorXd& TT) {
            MincoS4NU mm;
            mm.setConditions(head.toMat(), tail.toMat(), N);
            mm.setParameters(qq, TT);
            return mm;
        };
        MincoS4NU m0 = build(q, T);
        const double e0 = m0.getEnergy();

        Eigen::MatrixX3d gdC;
        Eigen::VectorXd gdT0;
        m0.getEnergyPartialGradByCoeffs(gdC);
        m0.getEnergyPartialGradByTimes(gdT0);

        Eigen::Matrix3Xd gq;
        Eigen::VectorXd gT;
        m0.propogateGrad(gdC, gdT0, gq, gT, false);

        const double h = 1e-6;
        double worst = 0.0;
        for (int k = 0; k < N - 1; ++k) {
            for (int d = 0; d < 3; ++d) {
                Eigen::MatrixXd qp = q;
                qp(d, k) += h;
                const double ep = build(qp, T).getEnergy();
                const double num = (ep - e0) / h;
                const double ana = gq(d, k);
                worst = std::max(worst, std::abs(num - ana) / std::max(1.0, std::abs(num)));
            }
        }
        check(worst < 2e-3, "dE/dq 解析 == 数值（全内点）", worst, 2e-3);
    }

    // ---------- [7] 采样接口与解析值一致 ----------
    {
        std::printf("\n[7] assignSamples + sample() 与解析值一致\n");
        MincoS4NU m = makeMinco(3);
        const double dur = m.duration();
        const double dt = 0.01;
        const int ns = static_cast<int>(std::ceil(dur / dt));

        std::vector<double> ts;
        std::vector<Eigen::Vector3d> ps, vs, as;
        for (int k = 0; k <= ns; ++k) {
            const double t = std::min(dur, k * dur / ns);
            Eigen::Vector3d p, v, a, j;
            if (!m.evaluatePVAJ(t, p, v, a, j)) continue;
            ts.push_back(t);
            ps.push_back(p);
            vs.push_back(v);
            as.push_back(a);
        }
        PathTrajectory tr;
        check(tr.assignSamples(ts, ps, vs, as), "assignSamples 成功");

        double max_ep = 0.0, max_ev = 0.0;
        for (int k = 0; k <= 200; ++k) {
            const double t = dur * k / 200.0;
            Eigen::Vector3d p, v, a;
            if (!tr.sample(t, p, v, a)) continue;
            Eigen::Vector3d pa, va, aa, ja;
            m.evaluatePVAJ(t, pa, va, aa, ja);
            max_ep = std::max(max_ep, (p - pa).norm());
            max_ev = std::max(max_ev, (v - va).norm());
        }
        check(max_ep < 1e-3, "插值位置误差 < 1mm", max_ep, 1e-3);
        check(max_ev < 1e-2, "插值速度误差 < 1cm/s", max_ev, 1e-2);
    }

    // ---------- [8] 单段退化情形 ----------
    {
        std::printf("\n[8] 单段（N=1）不崩\n");
        StatePVAJ head, tail;
        head.p = Eigen::Vector3d(0, 0, 0);
        head.v = Eigen::Vector3d(1, 0, 0);
        tail.p = Eigen::Vector3d(2, 0, 0);
        MincoS4NU m;
        m.setConditions(head.toMat(), tail.toMat(), 1);
        Eigen::MatrixXd q(3, 0);
        Eigen::VectorXd T(1);
        T(0) = 2.0;
        m.setParameters(q, T);
        check(m.valid(), "N=1 valid()");
        Eigen::Vector3d p, v, a, j;
        check(m.evaluatePVAJ(1.0, p, v, a, j), "N=1 evaluate 成功");
        check(p.allFinite() && v.allFinite(), "N=1 数值有限");
    }

    std::printf("\n=========================================\n");
    std::printf("通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
