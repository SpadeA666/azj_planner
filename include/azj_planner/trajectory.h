#pragma once

#include <Eigen/Core>
#include <vector>

namespace azj_planner {

/// 轨迹生成约束。
struct TrajParams {
    double max_vel{1.0};        ///< 最大速度 [m/s]
    double max_vel_z{0.6};      ///< 最大垂直速度 [m/s]
    double max_acc{1.5};        ///< 最大加速度 [m/s^2]
    double max_acc_z{1.0};      ///< 最大垂直加速度 [m/s^2]
    double max_lat_acc{1.2};    ///< 最大横向（向心）加速度 [m/s^2]，决定转弯限速

    /// 起点速度**标量** [m/s]。若 v_start_vec 未设置则用它乘路径切向。
    ///
    /// ⚠ 重规划时**不要只用标量**：新路径的切向方向可能和当前实际速度方向
    /// 差很多（尤其竖直分量）。实测：飞机水平飞 0.8m/s、新路径起点切线朝下
    /// 45° 时，会凭空生成 vz=-0.57 m/s 的参考 —— 控制器照做 → 瞬间掉高。
    /// 有真实速度矢量时请用 v_start_vec。
    double v_start{0.0};

    /// 起点速度**矢量**（世界系）[m/s]。设置后优先于 v_start。
    ///
    /// 语义：把当前真实速度**投影到新路径的切向**，
    /// 只保留沿路径的推进分量，不凭空制造与路径不符的竖直/横向速度。
    Eigen::Vector3d v_start_vec{Eigen::Vector3d::Zero()};
    bool has_v_start_vec{false};

    /// 是否在终点停住。局部规划通常 true（到点悬停）。
    bool stop_at_end{true};

    /// 轨迹总时长上限 [s]，防止病态路径生成超长轨迹。
    double max_duration{30.0};
};

/// 时间参数化后的轨迹：位置 / 速度 / 加速度 对时间的映射。
///
/// 为什么必须有这一层：
///   `navigationIso/Contour` 用 `planner_cmd_.velocity` 同时做微分项与前馈，
///   `navigationOMMPC` 的 Reference 需要 (p, v, a, yaw)。只给路径点的话，
///   控制器退化成纯位置跟踪，必然滞后、抖动。
///
/// 算法：沿弧长的速度规划（TOPP 的简化版）
///   1. 按曲率限速  v <= sqrt(a_lat / kappa)
///   2. 反向扫描保证终点能刹住
///   3. 正向扫描保证起点能加速（受 v_start 约束）
class PathTrajectory {
public:
    bool build(const std::vector<Eigen::Vector3d>& path, const TrajParams& params);

    /// 按时间取参考点。t 会被截断到 [0, duration]。
    bool sample(double t, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a) const;

    bool valid() const { return !stamps_.empty(); }
    double duration() const { return stamps_.empty() ? 0.0 : stamps_.back(); }

    const std::vector<double>& stamps() const { return stamps_; }
    const std::vector<Eigen::Vector3d>& positions() const { return pos_; }
    const std::vector<Eigen::Vector3d>& velocities() const { return vel_; }
    const std::vector<Eigen::Vector3d>& accelerations() const { return acc_; }

    /// 供调试/验证：每个采样点的速度，用来确认是否超限。
    double maxVelocity() const;
    double maxAcceleration() const;

private:
    std::vector<double> stamps_;
    std::vector<Eigen::Vector3d> pos_;
    std::vector<Eigen::Vector3d> vel_;
    std::vector<Eigen::Vector3d> acc_;
};

}  // namespace azj_planner
