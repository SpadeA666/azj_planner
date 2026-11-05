#include "azj_planner/curve_optimizer.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <string>
#include <cmath>
#include <limits>

namespace azj_planner {
namespace {

inline constexpr double kEps = 1e-9;

}  // namespace


// ============================ BSpline ============================

namespace {

/// 找到 u 所在的 span k，使 knots[k] <= u < knots[k+1]。
/// u == uMax 时返回最后一个有效 span（n-1），这是 clamped 节点向量的标准处理。
int findSpan(int n, int p, double u, const std::vector<double>& knots) {
    if (u >= knots[n]) return n - 1;
    if (u <= knots[p]) return p;
    int low = p, high = n, mid = (low + high) / 2;
    while (u < knots[mid] || u >= knots[mid + 1]) {
        if (u < knots[mid]) high = mid; else low = mid;
        mid = (low + high) / 2;
        if (mid <= p) return p;
        if (mid >= n - 1) return n - 1;
    }
    return mid;
}

/// 在 span k 上计算 p+1 个非零基函数 N_{k-p..k, p}(u)。
///
/// 这是 NURBS 标准的 span-based 实现（de Boor 的 Cox-de Boor 局部形式）。
/// 早期版本用全局递归，末点处理写错（把 uMax 归给了一个不存在的控制点索引），
/// 导致整条曲线求值全错 —— 直线的控制点算出来的端点还是原点。
void basisFuns(int span, double u, int p, const std::vector<double>& knots, double* N) {
    double left[16], right[16];
    N[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double den = right[r + 1] + left[j - r];
            const double temp = (std::abs(den) > kEps) ? N[r] / den : 0.0;
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

}  // namespace

bool BSpline::init(const Eigen::Matrix<double, 3, Eigen::Dynamic>& ctrl, int order) {
    order_ = order;
    ctrl_ = ctrl;
    if (!valid()) {
        knots_.clear();
        return false;
    }
    buildKnots();
    return true;
}

void BSpline::buildKnots() {
    const int n = static_cast<int>(ctrl_.cols());
    const int m = n + order_ + 1;      // 节点数
    knots_.assign(m, 0.0);

    // clamped 节点向量：两端各重复 order+1 次，保证曲线过首末控制点
    const int inner = n - order_;      // 内部区间个数
    for (int i = 0; i < m; ++i) {
        if (i <= order_) {
            knots_[i] = 0.0;
        } else if (i >= n) {
            knots_[i] = static_cast<double>(inner);
        } else {
            knots_[i] = static_cast<double>(i - order_);
        }
    }
}

bool BSpline::evaluate(double u, int deriv, Eigen::Vector3d& out) const {
    if (!valid()) return false;
    const double lo = uMin(), hi = uMax();
    u = std::min(std::max(u, lo), hi);

    if (deriv == 0) {
        out.setZero();
        const int n = numControlPoints();
        const int span = findSpan(n, order_, u, knots_);
        double N[16];
        basisFuns(span, u, order_, knots_, N);
        for (int j = 0; j <= order_; ++j) {
            const int i = span - order_ + j;
            if (i >= 0 && i < n) out += N[j] * ctrl_.col(i);
        }
        return true;
    }

    // 导数用降阶控制点：Q'_i = p/(knots[i+p+1]-knots[i+1]) * (Q_{i+1}-Q_i)
    const int n = numControlPoints();
    if (n - 1 <= order_) return false;

    Eigen::Matrix<double, 3, Eigen::Dynamic> d1(3, n - 1);
    for (int i = 0; i + 1 < n; ++i) {
        const double den = knots_[i + order_ + 1] - knots_[i + 1];
        // Eigen 表达式模板不能和 ?: 混用（两侧类型不同），必须显式分支
        if (den > 1e-12) {
            d1.col(i) = (order_ / den) * (ctrl_.col(i + 1) - ctrl_.col(i));
        } else {
            d1.col(i).setZero();
        }
    }
    BSpline d1s;
    d1s.init(d1, order_ - 1);
    if (deriv == 1) return d1s.evaluate(u, 0, out);

    // 二阶导：对一阶导曲线再求一次导
    if (n - 1 - 1 <= order_ - 1) return false;
    Eigen::Matrix<double, 3, Eigen::Dynamic> d2(3, n - 1 - 1);
    const int nd = static_cast<int>(d1.cols());
    for (int i = 0; i + 1 < nd; ++i) {
        const double den = d1s.knots_[i + (order_ - 1) + 1] - d1s.knots_[i + 1];
        if (den > 1e-12) {
            d2.col(i) = ((order_ - 1) / den) * (d1.col(i + 1) - d1.col(i));
        } else {
            d2.col(i).setZero();
        }
    }
    BSpline d2s;
    d2s.init(d2, order_ - 2);
    return d2s.evaluate(u, 0, out);
}

double BSpline::curvature(double u) const {
    Eigen::Vector3d p1, p2;
    if (!evaluate(u, 1, p1) || !evaluate(u, 2, p2)) return 0.0;
    const double n1 = p1.norm();
    if (n1 < 1e-9) return 0.0;
    return p1.cross(p2).norm() / (n1 * n1 * n1);
}

int BSpline::nonzeroBasis(double u, int* out_idx, double* out_val) const {
    if (!valid()) return 0;
    u = std::min(std::max(u, uMin()), uMax());
    const int n = numControlPoints();
    const int span = findSpan(n, order_, u, knots_);
    double N[16];
    basisFuns(span, u, order_, knots_, N);
    int cnt = 0;
    for (int j = 0; j <= order_; ++j) {
        const int i = span - order_ + j;
        if (i >= 0 && i < n && N[j] != 0.0) {
            out_idx[cnt] = i;
            out_val[cnt] = N[j];
            ++cnt;
        }
    }
    return cnt;
}

bool BSpline::basisValue(int i, double u, double& value) const {
    if (!valid() || i < 0 || i >= numControlPoints()) return false;
    int idx[16];
    double val[16];
    const int c = nonzeroBasis(u, idx, val);
    for (int t = 0; t < c; ++t) {
        if (idx[t] == i) {
            value = val[t];
            return true;
        }
    }
    value = 0.0;
    return false;
}

double BSpline::arcLength(int samples) const {
    if (!valid() || samples < 2) return 0.0;
    const double lo = uMin(), hi = uMax();
    double len = 0.0;
    Eigen::Vector3d prev, cur;
    for (int i = 0; i <= samples; ++i) {
        const double u = lo + (hi - lo) * i / samples;
        if (!evaluate(u, 0, cur)) continue;
        if (i > 0) len += (cur - prev).norm();
        prev = cur;
    }
    return len;
}

bool BSpline::sampleByArcLength(double step_m, std::vector<Eigen::Vector3d>& points,
                                std::vector<Eigen::Vector3d>& tangents) const {
    points.clear();
    tangents.clear();
    if (!valid()) return false;
    if (step_m <= 1e-4) step_m = 0.05;

    // 先用密集参数采样，再按弧长抽稀
    const int dense = 2000;
    const double lo = uMin(), hi = uMax();
    std::vector<Eigen::Vector3d> dp(dense + 1);

    // 按参数均匀采样后长度不均匀（B 样条参数非弧长），
    // 这里用"二分找弧长"的方式反查，避免点数在拐角处扎堆。
    std::vector<double> arc(dense + 1, 0.0);
    Eigen::Vector3d p;
    for (int i = 0; i <= dense; ++i) {
        evaluate(lo + (hi - lo) * i / dense, 0, p);
        dp[i] = p;
        if (i > 0) arc[i] = arc[i - 1] + (dp[i] - dp[i - 1]).norm();
    }
    const double total = arc.back();
    if (total < 1e-6) return false;

    const int n = std::max(2, static_cast<int>(std::ceil(total / step_m)) + 1);
    size_t j = 0;
    for (int k = 0; k < n; ++k) {
        const double target = total * k / (n - 1);
        while (j + 1 < arc.size() && arc[j + 1] < target) ++j;
        const double u = lo + (hi - lo) * j / dense;
        Eigen::Vector3d pt, tg;
        evaluate(u, 0, pt);
        if (!evaluate(u, 1, tg) || tg.norm() < 1e-9) {
            tg = (k > 0) ? (pt - points.back()) : Eigen::Vector3d(1, 0, 0);
        }
        tangents.push_back(tg.normalized());
        points.push_back(pt);
    }

    // 首末点强制对齐到曲线端点。
    // 弧长反查是离散的（j 只在 dense 采样点之间跳），末点会差到 res/2 量级
    // （实测 3.0000 的直线重采样末点只有 2.9968，差 3mm）。
    // 这个误差会直接变成"路径没到达目标"，所以必须钳住。
    Eigen::Vector3d p_first, p_last, t_first, t_last;
    if (evaluate(uMin(), 0, p_first) && evaluate(uMax(), 0, p_last)) {
        points.front() = p_first;
        points.back() = p_last;
        if (evaluate(uMin(), 1, t_first) && t_first.norm() > 1e-9)
            tangents.front() = t_first.normalized();
        if (evaluate(uMax(), 1, t_last) && t_last.norm() > 1e-9)
            tangents.back() = t_last.normalized();
    }
    return points.size() >= 2;
}

// ========================= CurveOptimizer =========================

CurveOptResult CurveOptimizer::optimize(const std::vector<Eigen::Vector3d>& init_path,
                                        const ClearanceField& field,
                                        const CurveOptParams& params) const {
    CurveOptResult res;
    if (init_path.size() < 2) {
        res.msg = "initial path too short";
        return res;
    }

    const double total_len = [&] {
        double l = 0;
        for (size_t i = 1; i < init_path.size(); ++i) l += (init_path[i] - init_path[i - 1]).norm();
        return l;
    }();
    if (total_len < 1e-6) {
        res.msg = "initial path has zero length";
        return res;
    }

    // ---------- 1) 控制点数：按间距自适应 ----------
    //
    // ★ 间距必须小于"障碍特征尺寸"。柱阵里柱子间距 1.2m，
    //   0.6m 间距会让一个控制点跨过整个转弯 → B 样条过冲穿进柱子。
    //   0.30m 间距保证每个拐弯至少 2 个控制点。
    int ncp = params.control_points;
    if (ncp <= 0) {
        const double sp = std::max(0.10, params.control_point_spacing);
        ncp = std::max(6, static_cast<int>(std::ceil(total_len / sp)) + 3);
    }
    ncp = std::max(ncp, 4);
    // 上限保护：控制点过多会让优化变慢且易抖
    ncp = std::min(ncp, 400);

    // ---------- 2) 用折线路径的弧长均匀取初值控制点 ----------
    Eigen::Matrix<double, 3, Eigen::Dynamic> Q(3, ncp);
    {
        std::vector<double> arc(init_path.size(), 0.0);
        for (size_t i = 1; i < init_path.size(); ++i)
            arc[i] = arc[i - 1] + (init_path[i] - init_path[i - 1]).norm();
        for (int k = 0; k < ncp; ++k) {
            const double target = total_len * k / (ncp - 1);
            size_t j = 0;
            while (j + 2 < init_path.size() && arc[j + 1] < target) ++j;
            const double seg = arc[j + 1] - arc[j];
            const double w = (seg > 1e-9) ? (target - arc[j]) / seg : 0.0;
            Q.col(k) = init_path[j] + w * (init_path[j + 1] - init_path[j]);
        }
    }

    // 2D：把整列控制点的 z 覆盖成规划平面高度。
    // 必须在取 start_fixed/goal_fixed 之前做，否则端点钉住会把
    // 初始的 z 过渡重新拉回来（线搜索每步都在钉端点）。
    if (params.lock_z && std::isfinite(params.lock_z_value)) {
        Q.row(2).setConstant(params.lock_z_value);
    }

    const Eigen::Vector3d start_fixed = Q.col(0);
    const Eigen::Vector3d goal_fixed = Q.col(ncp - 1);

    BSpline spline;
    if (!spline.init(Q, 3)) {
        res.msg = "spline init failed";
        return res;
    }

    const double lo = spline.uMin(), hi = spline.uMax();

    // ★ 采样数必须随路径长度自适应（根源性修正）。
    //
    // 固定 120 采样在 15m 路径上是 12.5cm 一格 —— 看不见“被平滑切进去的
    // 柱子边”。实测（15m 目标）：曲线优化后 min_clearance=0.0085m，
    // 但优化过程一直以为安全（采样点都在柱外），于是安全校验拒掉 → 回落折线。
    //
    // 提采样分辨率比提迭代数有效得多：每个采样点都过 ESDF，梯度就知道
    // “这个拐角切进去了”，不需要多跑几百轮去猜。
    //
    // 目标间距 10cm（实测 6cm 太密：15m 路径 S=250，加上线搜索每次
    // 全量重采样，单次规划飙升到 7s）。10cm 仍小于最小柱径（0.30m），
    // 能看见每根柱子；上限 1500 防极端长路径卡住。
    const int S = std::min(
        1500, std::max(params.samples,
                       static_cast<int>(std::ceil(total_len / 0.10))));
    std::vector<double> us(S);
    for (int k = 0; k < S; ++k) us[k] = lo + (hi - lo) * k / (S - 1);

    // 基函数缓存：每个采样点的 4 个非零基函数，避免迭代里重复算递归
    std::vector<std::array<int, 8>> idx(S);
    std::vector<std::array<double, 8>> bval(S);
    std::vector<int> nz(S);
    auto rebuild_cache = [&] {
        for (int k = 0; k < S; ++k) {
            int id[8];
            double v[8];
            const int c = spline.nonzeroBasis(us[k], id, v);
            nz[k] = c;
            for (int t = 0; t < c; ++t) {
                idx[k][t] = id[t];
                bval[k][t] = v[t];
            }
        }
    };
    rebuild_cache();

    const double cr = std::max(1e-3, params.clearance_ref);
    const double cap = std::max(cr, params.clearance_cap);
    const double alpha = params.alpha;

    // 锚定基准：初始控制点
    const Eigen::Matrix<double, 3, Eigen::Dynamic> Q0 = Q;

    // ---- 总代价（用于线搜索的下降判据）----
    auto totalCost = [&](const Eigen::Matrix<double, 3, Eigen::Dynamic>& Qt) -> double {
        BSpline sp;
        if (!sp.init(Qt, 3)) return std::numeric_limits<double>::infinity();
        const int N = std::max(40, S);
        double cost = 0.0;

        // 平滑（控制点二阶差分）
        const double ws = params.w_smooth / std::max(1, ncp);
        for (int i = 1; i + 1 < ncp; ++i) {
            const Eigen::Vector3d d = Qt.col(i - 1) - 2.0 * Qt.col(i) + Qt.col(i + 1);
            cost += ws * d.squaredNorm();
        }

        // 居中 + 安全
        //
        // 注：曾经试过 OpenMP 并行（采样点逐点独立、ESDF 只读），
        // 实测**反而变慢** 1.7s → 2.6s：N 只有 150 左右，
        // 每次迭代 fork/join 的线程开销超过并行收益。别再试了。
        for (int k = 0; k < N; ++k) {
            const double u = lo + (hi - lo) * k / std::max(1, N - 1);
            Eigen::Vector3d p;
            if (!sp.evaluate(u, 0, p)) continue;
            const double c = field.clearance(p);
            if (c <= 0.0) {
                cost += params.w_obs * 10.0 / N;      // 穿障碍：重罚
                continue;
            }
            const double cc = std::min(std::max(c, 0.02), cap);
            cost += (params.w_mid / N) * std::pow(cr / cc, alpha);
            if (c < params.safe_margin) {
                const double gap = params.safe_margin - c;
                cost += (params.w_obs / N) * gap * gap;
            }
        }

        // 锚定
        cost += (params.w_anchor / std::max(1, ncp)) * (Qt - Q0).squaredNorm();

        // 端点
        if (params.pin_endpoints) {
            cost += params.w_end * ((Qt.col(0) - start_fixed).squaredNorm() +
                                    (Qt.col(ncp - 1) - goal_fixed).squaredNorm());
        }
        return cost;
    };

    // ================= 安全硬投影（PBD 风格）=================
    //
    // 软惩罚（hinge）在密集障碍里压不过保形项，曲线必然被拽进柱子。
    // 这里用**位置投影**：找到曲线上 clearance < safe_margin 的点，
    // 把影响它的控制点沿 ∇c 直接推开 (safe_margin - c)。
    //
    // 为什么用位置投影而不是加大权重：
    //   权重竞争是零和的（w_obs↑ 就压死 w_smooth，曲线变折）。
    //   投影是"先满足安全，再谈其他"，安全不再是可交易的代价项。
    auto projectSafe = [&](Eigen::Matrix<double, 3, Eigen::Dynamic>& Qt) -> int {
        if (!params.enable_projection) return 0;

        BSpline sp;
        if (!sp.init(Qt, 3)) return 0;

        const int M = std::min(
            1500, std::max(60, std::max(params.projection_samples,
                                       static_cast<int>(std::ceil(total_len / 0.10)))));
        int violations = 0;

        for (int rep = 0; rep < std::max(1, params.projection_iters); ++rep) {
            violations = 0;
            for (int m = 0; m < M; ++m) {
                const double u = lo + (hi - lo) * m / std::max(1, M - 1);
                Eigen::Vector3d p, gp;
                if (!sp.evaluate(u, 0, p)) continue;
                const double c = field.clearance(p);
                if (c >= params.safe_margin) continue;
                if (!field.clearanceGradient(p, gp) || gp.norm() < 1e-9) continue;

                ++violations;
                // 需要推出的距离（清障场梯度指向 clearance 增大方向）
                const double push = (params.safe_margin - c);
                const Eigen::Vector3d dir = gp.normalized();

                // 把影响该曲线点的控制点按基函数权重推开。
                // 用 1.0 倍（不放大），多次迭代收敛；放大容易震荡。
                int idxs[8];
                double bv[8];
                const int nzb = sp.nonzeroBasis(u, idxs, bv);
                for (int t = 0; t < nzb; ++t) {
                    const int ci = idxs[t];
                    if (ci == 0 || ci == ncp - 1) continue;   // 端点不动
                    Qt.col(ci) += dir * (push * bv[t]);
                }
            }
            if (violations == 0) break;
            sp.init(Qt, 3);
            if (params.lock_z) Qt.row(2) = Q0.row(2);
            if (params.enable_z_bounds) {
                Qt.row(2) = Qt.row(2).cwiseMax(params.z_min).cwiseMin(params.z_max);
            }
        }
        return violations;
    };

    double step = params.step_init;
    double best_cost = totalCost(Q);
    int it = 0;

    for (; it < params.iterations; ++it) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> grad = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, ncp);

        // ---------- (a) 平滑项：Σ|Q_{i-1}-2Q_i+Q_{i+1}|² ----------
        // 对控制点二阶差分，等价于惩罚曲线的弯曲程度
        const double ws = params.w_smooth / std::max(1, ncp);
        for (int i = 1; i + 1 < ncp; ++i) {
            const Eigen::Vector3d d = Q.col(i - 1) - 2.0 * Q.col(i) + Q.col(i + 1);
            grad.col(i - 1) += ws * 2.0 * d;
            grad.col(i) += ws * -4.0 * d;
            grad.col(i + 1) += ws * 2.0 * d;
        }

        // ---------- (b) 居中项 + (c) 安全项 ----------
        // 两者共用同一次 clearance / 梯度查询
        double mid_sum = 0.0;
        int mid_cnt = 0;
        for (int k = 0; k < S; ++k) {
            Eigen::Vector3d p, gp;
            if (!spline.evaluate(us[k], 0, p)) continue;
            const double c = field.clearance(p);
            if (c <= 0.0) {
                // 已经穿进障碍：给一个强力的"推出"梯度（用数值梯度兜底，
                // 因为障碍内部 ESDF 梯度可能为 0 或非法）
                if (field.clearanceGradient(p, gp) && gp.norm() > 1e-9) {
                    for (int t = 0; t < nz[k]; ++t)
                        grad.col(idx[k][t]) += -params.w_obs * 4.0 * gp.normalized() * bval[k][t];
                }
                mid_cnt++;
                continue;
            }
            if (!field.clearanceGradient(p, gp)) continue;

            // ================= 居中：左右两侧距离差 =================
            //
            // ★ 核心修正（v1.2）：之前用 ∇c 方向扫描找"最宽处"，
            //   但 ∇c 是指向**远离最近障碍**的方向 ——
            //   那是"垂直于墙面"的，不是"沿通道横向"的。
            //   结果 local_ref 算错，居中梯度失效（实测居中度仅 0.40）。
            //
            //   正确做法：沿**路径法向**（切向的左正交）扫描两侧，
            //   分别得到到左障碍 dL、到右障碍 dR。
            //   居中 ⇔ dL == dR。
            //   偏差 e = (dL - dR) / 2，梯度沿法向把路径往中间推。
            //
            //   这个定义直接对应"从障碍正中心走，不贴任何一侧"。
            double mid_e = 0.0;          // 横向偏差（>0 表示偏右，需往左推）
            double mid_hw = 1.0;         // 该处横截面半宽，用于归一化
            bool mid_valid = false;
            if (params.enable_local_centering) {
                // 路径切向 → 横向（水平面内左正交）
                Eigen::Vector3d tan_u;
                if (spline.evaluate(us[k], 1, tan_u) && tan_u.norm() > 1e-6) {
                    Eigen::Vector3d t = tan_u.normalized();
                    Eigen::Vector3d n(-t.y(), t.x(), 0.0);      // 水平法向
                    if (n.norm() > 1e-6) {
                        n.normalize();
                        // 沿 ±n 找第一次进入障碍的距离
                        double dL = 0.0, dR = 0.0;
                        bool hitL = false, hitR = false;
                        const double step_scan = 0.05;
                        for (double d = step_scan; d <= params.local_scan_radius; d += step_scan) {
                            if (!hitL && field.clearance(p + d * n) <= 0.0) { dL = d; hitL = true; }
                            if (!hitR && field.clearance(p - d * n) <= 0.0) { dR = d; hitR = true; }
                            if (hitL && hitR) break;
                        }
                        // 两侧都找到障碍才算有效（单侧开放时无法定义"中间"）
                        if (hitL && hitR && dL > 1e-3 && dR > 1e-3) {
                            // 精确化：用 clearance 修正（障碍表面离采样点的距离）
                            dL += field.clearance(p + (dL - step_scan) * n);
                            dR += field.clearance(p - (dR - step_scan) * n);
                            mid_e = 0.5 * (dR - dL);   // >0 偏左(距右更远) → 往右推
                            // 该处半宽，用于把偏差归一化成"相对偏离"
                            mid_hw = std::max(0.15, 0.5 * (dL + dR));
                            mid_valid = true;
                        } else if (hitL || hitR) {
                            // 单侧有障碍：用该侧 clearance 做"远离"（退化为原行为）
                            mid_e = 0.0;
                            mid_valid = false;
                        }
                    }
                }
            }

            if (mid_valid && std::abs(mid_e) > 1e-3) {
                // 居中梯度：沿法向把点往中间推，力度与偏差成正比
                Eigen::Vector3d n(-0.0, 0.0, 0.0);
                Eigen::Vector3d tan_u;
                if (spline.evaluate(us[k], 1, tan_u) && tan_u.norm() > 1e-6) {
                    const Eigen::Vector3d t = tan_u.normalized();
                    n = Eigen::Vector3d(-t.y(), t.x(), 0.0);
                    if (n.norm() > 1e-6) n.normalize();
                }
                // mid_e > 0 → 偏左 → 沿 -n 推（往右）
                //
                // ★ 权重归一化：锚定项是 per-control-point（2*w_anchor/ncp），
                //   而居中项是 per-sample（w_mid/S）。S(140) >> ncp(30)，
                //   导致居中的**有效梯度只有锚定的 1/4.7**（实测），
                //   居中完全被压制 —— 这就是"怎么调 w_mid 都不居中"的原因。
                //   这里乘 S/ncp 让两者量级可比。
                // ★ 按半宽归一化：偏差相对于该处可用半宽的比例。
                //
                // 为什么必须归一化：固定 w_mid 无法同时适配宽通道和窄缝。
                //   宽通道(hw=1.5) 偏 0.3m 只算 0.2 的相对偏离，不该用力推；
                //   窄缝(hw=0.4)  偏 0.3m 是 0.75，必须用力推。
                // 用 (mid_e/hw) 归一化后，梯度自动按场景缩放。
                // 实测：固定 w_mid=5 会让 45° 宽通道的居中度从 0.0002 退化到 0.034；
                //       归一化后两个场景都能收敛。
                const double rel = mid_e / (mid_hw * mid_hw);
                const double mid_scale = params.w_mid / S * (double(S) / std::max(1, ncp));
                const Eigen::Vector3d dE = (-rel * n) * mid_scale;
                for (int tt = 0; tt < nz[k]; ++tt)
                    grad.col(idx[k][tt]) += dE * bval[k][tt];

                // ★ 叠加"clearance 最大化"项（与左右偏差互补）。
                //
                // 为什么两个都要：
                //   · 左右偏差项：显式把点推向 dL==dR（真正的"居中"），
                //     在柱阵这类散点障碍里有效。
                //   · (ref/c)^α 项：推高 clearance 本身。
                //     在**平行墙通道**里 clearance 的唯一局部最大就是中轴，
                //     这一项能精确收敛到 0 偏差（实测 0.0002m），
                //     而左右偏差项在宽通道里因 hw 归一化反而变弱。
                //
                // 两者同向时加速收敛，冲突时由 w_mid 权衡。
                const double ccg = std::min(std::max(c, 0.02), cap);
                const double dterm_dc = -alpha * std::pow(std::min(cr, cap), alpha) /
                                        std::pow(ccg, alpha + 1.0);
                const Eigen::Vector3d dE_c =
                    (dterm_dc * gp) * (params.w_mid * 0.5 / S * (double(S) / std::max(1, ncp)));
                for (int tt = 0; tt < nz[k]; ++tt)
                    grad.col(idx[k][tt]) += dE_c * bval[k][tt];
                mid_sum += std::abs(mid_e);
                mid_cnt++;
            } else if (!params.enable_local_centering) {
                // 退回全局居中（(ref/c)^α）：只在单侧障碍时有一点作用
                const double cc0 = std::min(std::max(c, 0.02), cap);
                const double term = std::pow(cr / cc0, alpha);
                mid_sum += term;
                mid_cnt++;
                const double dterm_dc =
                    -alpha * std::pow(cr, alpha) / std::pow(cc0, alpha + 1.0);
                const Eigen::Vector3d dE_dp = (dterm_dc * gp) * (params.w_mid / S);
                for (int tt = 0; tt < nz[k]; ++tt)
                    grad.col(idx[k][tt]) += dE_dp * bval[k][tt];
            } else {
                // 单侧无障碍：不能定义"中间"，只做"远离"（弱权重）
                const double cc0 = std::min(std::max(c, 0.02), cap);
                const double dterm_dc =
                    -alpha * std::pow(std::max(cr, 0.1), alpha) / std::pow(cc0, alpha + 1.0);
                const double mid_scale = 0.1 * params.w_mid / S * (double(S) / std::max(1, ncp));
                const Eigen::Vector3d dE_dp = (dterm_dc * gp) * mid_scale;
                for (int tt = 0; tt < nz[k]; ++tt)
                    grad.col(idx[k][tt]) += dE_dp * bval[k][tt];
            }

            // --- 安全：hinge(safe_margin - c)² ---
            if (c < params.safe_margin) {
                const double gap = params.safe_margin - c;
                // dE/dp = -2·gap·∇c  → 下降方向 +∇c（远离障碍）
                const Eigen::Vector3d dE = (-2.0 * gap * gp) * (params.w_obs / S);
                for (int t = 0; t < nz[k]; ++t)
                    grad.col(idx[k][t]) += dE * bval[k][t];
            }
        }
        (void)mid_sum;
        (void)mid_cnt;

        // ---------- (d) 长度项 ----------
        // 只在明显变长时生效，避免为了宽度绕远路
        {
            double len = 0.0;
            Eigen::Vector3d a, b;
            for (int k = 0; k + 1 < S; ++k) {
                if (!spline.evaluate(us[k], 0, a) || !spline.evaluate(us[k + 1], 0, b)) continue;
                len += (b - a).norm();
            }
            if (len > total_len * 1.02) {
                const double scale = params.w_len / std::max(1, S);
                for (int k = 0; k + 1 < S; ++k) {
                    if (!spline.evaluate(us[k], 0, a) || !spline.evaluate(us[k + 1], 0, b)) continue;
                    const Eigen::Vector3d d = b - a;
                    const double n = d.norm();
                    if (n < 1e-9) continue;
                    const Eigen::Vector3d u = d / n;
                    for (int t = 0; t < nz[k]; ++t) grad.col(idx[k][t]) += (-u) * bval[k][t] * scale;
                    for (int t = 0; t < nz[k + 1]; ++t)
                        grad.col(idx[k + 1][t]) += u * bval[k + 1][t] * scale;
                }
            }
        }

        // ---------- (e) 锚定：防止曲线整体漂移 ----------
        {
            const double wa = params.w_anchor / std::max(1, ncp);
            grad += 2.0 * wa * (Q - Q0);
        }

        // ---------- (f) 端点 ----------
        if (params.pin_endpoints) {
            grad.col(0) += params.w_end * 2.0 * (Q.col(0) - start_fixed);
            grad.col(ncp - 1) += params.w_end * 2.0 * (Q.col(ncp - 1) - goal_fixed);
        }

        // ---------- 2D：锁死 z ----------
        // 必须清零梯度而不是只靠"控制点 z 相同"：z 梯度会把曲线推离平面。
        if (params.lock_z) {
            grad.row(2).setZero();
        }

        // ---------- 更新：梯度下降 + 回溯线搜索 ----------
        //
        // 曾经用「全局归一化梯度 + 大步长」，两个问题：
        //   1) 归一化把各目标项的**绝对权重**全毁了 —— 安全项再也压不过
        //      平滑项，实测曲线穿进墙里 0.3m。
        //   2) step=0.08×√ncp≈0.33m/步，几十步就冲出可行域，长度涨 46%。
        // 现在：不归一化（保留权重语义），并用回溯线搜索保证代价单调下降。
        // 回溯只做 4 次：每次 totalCost 都要全量重采样（S 次 ESDF 查询），
        // 12 次的尾段步长已小到 1/4096，收益接近零却占掉大半时间。
        const double gn = grad.norm();
        if (!std::isfinite(gn) || gn < 1e-7) break;   // 收敛

        bool accepted = false;
        double trial = step;
        for (int bt = 0; bt < 4; ++bt) {
            Eigen::Matrix<double, 3, Eigen::Dynamic> Qt = Q - trial * grad;
            if (params.pin_endpoints) {
                Qt.col(0) = start_fixed;
                Qt.col(ncp - 1) = goal_fixed;
            }
            const double cost = totalCost(Qt);
            if (cost < best_cost) {          // Armijo 式：只要下降就接受
                Q = Qt;
                best_cost = cost;
                accepted = true;
                break;
            }
            trial *= 0.5;
        }

        if (!accepted) break;                // 步长降到很小仍不下降 → 收敛

        // ★ 安全硬投影：梯度步之后，把仍越界的控制点推出去。
        //
        // ⚠ 这里曾经有一个致命逻辑错误（已修，2026-11-05）：
        //   旧代码是 `const int viol = projectSafe(Qproj); if (viol > 0) {…}`。
        //   但 projectSafe 返回的是**最后一轮 rep 的**越界数：
        //     · 第一轮投影就修好了 → 后续 rep 没有越界 → 返回 0
        //     · `viol > 0` 为假 → **投影成果被整个丢弃**
        //   即“投影越成功，越容易不被采纳”。实测后果：15m 路径优化后
        //   min_clearance=0.0085m，而 A* 折线都有 0.153m —— 优化反而更糟。
        //
        // 正确判据是“投影是否真的动了控制点”，并用**改动量**而不是
        // 代价判断。代价变差必须接受：本规划器的理念是
        // 安全不是可交易的代价项（用户明确：安全优先于最优）。
        {
            Eigen::Matrix<double, 3, Eigen::Dynamic> Qproj = Q;
            projectSafe(Qproj);
            if ((Qproj - Q).squaredNorm() > 1e-18) {
                Q = Qproj;
                // 基准代价必须跟着更新，否则下一轮线搜索的比较基准是错的
                best_cost = totalCost(Q);
            }
        }

        // 2D：硬锁 z 到初值平面。
        // 线搜索内部已经把 z 梯度清零了，这里再兜一层，
        // 防止线搜索里端点钉住等操作引入 z 漂移。
        if (params.lock_z) Q.row(2) = Q0.row(2);

        // z 硬边界：控制点钳制。
        // B 样条是控制点的凸组合，控制点都在 [z_min,z_max] 内时曲线必然也在，
        // 所以钳控制点就够，不用逐点检查曲线。
        if (params.enable_z_bounds) {
            Q.row(2) = Q.row(2).cwiseMax(params.z_min).cwiseMin(params.z_max);
        }

        spline.init(Q, 3);
        rebuild_cache();
        step = std::min(trial * 1.2, params.step_init);   // 成功则略微放大
    }
    res.iterations = it;

    // ---------- 结果统计 ----------
    res.spline = spline;
    const int S2 = std::max(60, params.samples);
    double mn = std::numeric_limits<double>::max(), sum = 0.0;
    double kmax = 0.0;
    for (int k = 0; k < S2; ++k) {
        const double u = lo + (hi - lo) * k / (S2 - 1);
        Eigen::Vector3d p;
        if (!spline.evaluate(u, 0, p)) continue;
        const double c = field.clearance(p);
        res.clearance_along.push_back(c);
        mn = std::min(mn, c);
        sum += c;
        kmax = std::max(kmax, spline.curvature(u));
    }
    res.min_clearance = res.clearance_along.empty() ? 0.0 : mn;
    res.mean_clearance = res.clearance_along.empty() ? 0.0 : sum / res.clearance_along.size();
    res.length = spline.arcLength();
    res.max_curvature = kmax;
    res.success = true;
    res.msg = "ok";
    return res;
}

}  // namespace azj_planner
