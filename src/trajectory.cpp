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

    // 加速度：用相邻速度差分，并限幅防止数值尖峰喂给控制器
    for (size_t i = 0; i < m; ++i) {
        Eigen::Vector3d a_i = Eigen::Vector3d::Zero();
        if (i + 1 < m) {
            const double dt = stamps_[i + 1] - stamps_[i];
            if (dt > 1e-9) a_i = (vel_[i + 1] - vel_[i]) / dt;
        }
        const double an = a_i.norm();
        const double cap = std::max(params.max_acc, params.max_acc_z) * 2.0;
        if (an > cap) a_i *= cap / an;
        acc_[i] = a_i;
    }

    return !stamps_.empty();
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
        // 若终点应停住，速度给 0；否则给末速
        v = Eigen::Vector3d::Zero();
        a = Eigen::Vector3d::Zero();
        return true;
    }

    // 二分查找所在区间
    const auto it = std::lower_bound(stamps_.begin(), stamps_.end(), t);
    const size_t hi = static_cast<size_t>(it - stamps_.begin());
    const size_t lo = hi - 1;

    const double dt = stamps_[hi] - stamps_[lo];
    const double w = (dt > 1e-9) ? (t - stamps_[lo]) / dt : 0.0;
    p = pos_[lo] + w * (pos_[hi] - pos_[lo]);
    v = vel_[lo] + w * (vel_[hi] - vel_[lo]);
    a = acc_[lo] + w * (acc_[hi] - acc_[lo]);
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
