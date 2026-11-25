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

    /// 最大加加速度 [m/s³]（七段式 S 曲线的核心约束，2026-11-16）。
    ///
    /// 速度规划直接约束 |da/dt| ≤ max_jerk，产出 S 型速度曲线
    /// （不再需要事后拉普拉斯平滑，因此也不会填平转弯处的 v_min）。
    ///
    /// 取值依据：PX4 电机响应时间常数约 0.0125s
    /// （`MPC_ATT_RATE_FF` / 从 0 到满推力约 40ms），
    /// 对应可跟踪的 jerk 量级 100~300 m/s³。
    /// 取 8.0 偏保守（柔和），实测直线场景 max|Δa| 可压到 0.1 量级。
    double max_jerk{8.0};

    /// 速度曲线 jerk 限制的平滑迭代次数（2026-11-16）。
    ///
    /// TOPP 正反扫描给的是梯形速度曲线 → 加速度在拐点**阶跃**
    /// （实测 +1.5 → -1.5，max|Δa|=3.0 m/s²，与采样率无关）。
    /// 这里对速度序列做拉普拉斯平滑把它变成 S 型。
    /// 0 = 关闭（退回梯形，加速度阶跃）。
    /// 6 次对应 jerk 约 5~8 m/s³（与 PX4 电机响应 timeConstant 0.0125s 匹配）。
    int smooth_iters{15};

    /// 限速曲线（曲率限速）的"只降不升"平滑迭代次数（2026-11-16）。
    ///
    /// 直角/急弯处曲率突变 → vlim 台阶式下降 → 减速段起点仍是陡坡。
    /// 平滑让减速**提前开始**（碗形而非台阶形），且只降不升
    /// → 绝不违反 a ≤ max_lat_acc。
    /// 0 = 关闭。
    int vlim_smooth_iters{20};
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

    /// 从**指定状态**（位置 + 速度矢量）起步构建轨迹。
    ///
    /// 与 build() 的区别：起点不再是 path[0]，而是 start_p；起点速度直接
    /// 用 start_v（会投影到路径切向）。
    ///
    /// ★ 这是"轨迹拼接"的基础（2026-11-14，学 SUPER 的 replan_forward_dt）：
    ///   新轨迹应当从**旧轨迹在 t_now 的状态**起步，而不是从飞机实际位置起步。
    ///   这样参考点连续 → 控制器看不到速度阶跃 → 不会猛打姿态。
    ///   飞机与参考的偏差交给控制器的位置/速度环去消化（那是它的本职）。
    bool buildFromState(const Eigen::Vector3d& start_p,
                        const Eigen::Vector3d& start_v,
                        const std::vector<Eigen::Vector3d>& path,
                        const TrajParams& params);

    /// ★ 从指定状态起步，用**三次 Hermite 样条**（2026-11-14，对齐 SUPER 的 MINCO 语义）。
    ///
    /// 与 buildFromState 的根本区别：
    ///   buildFromState  → TOPP：速度**标量**沿弧长分布，方向完全由路径切向决定。
    ///                     起点速度被投影到首段切向 → 若路径要掉头，投影接近 0
    ///                     → 轨迹从静止起步 → 飞机有动量 → 控制器猛打 → 掉高/卡顿。
    ///   buildSmoothFromState → Hermite：起点速度是**完整矢量**约束，
    ///                     方向可以与路径切向不同 → 轨迹能"边走边转弯"。
    ///
    /// 这正是 SUPER 用 MINCO 多项式做到的事（`pos_init_state` 含完整速度矢量）。
    /// 实测痛点（/tmp/azj_flight_20261114_153350）：
    ///   未知建图区重规划时路径常需掉头 129~135°，
    ///   TOPP 无法表达，只能"先刹停再转" → 7 次 TURN 刹车 → 用户看到卡顿。
    ///
    /// 实现：
    ///   1) 节点 = start_p + path（去重），节点按弧长/曲率分配时间（含起终点速度约束）
    ///   2) 切线：m[i] = 路径切向 × 速度；**m[0] = start_v（直接是速度矢量）**
    ///   3) 每段三次 Hermite 插值，按时间均匀重采样
    ///   4) 速度/加速度超限时整体拉长时间（迭代修正，最多 4 轮）
    bool buildSmoothFromState(const Eigen::Vector3d& start_p,
                              const Eigen::Vector3d& start_v,
                              const std::vector<Eigen::Vector3d>& path,
                              const TrajParams& params);

    /// 直接注入采样点（2026-11-17，供 MINCO 后端使用）。
    ///
    /// MINCO 输出的是**多项式解析式**而不是折线。要复用本类现成的
    /// sample()/duration()/spliceAfter()，只需按固定步长采样后填进来。
    /// 采样间隔按 dt 取（建议 ≤0.02s）：sample() 内部用三次 Hermite 插值，
    /// 节点足够密时与多项式解析值差异可忽略（见 minco_test 的验证）。
    ///
    /// 要求：四个数组等长、时间严格递增、至少 2 点。
    bool assignSamples(const std::vector<double>& stamps,
                       const std::vector<Eigen::Vector3d>& pos,
                       const std::vector<Eigen::Vector3d>& vel,
                       const std::vector<Eigen::Vector3d>& acc);

    /// 末速是否为零（assignSamples 后由调用方声明语义）。
    void setEndsAtRest(bool v) { ends_at_rest_ = v; }
    bool endsAtRest() const { return ends_at_rest_; }

    /// 在 t_join 之后拼接 seg（seg 的时间戳整体平移 +t_join）。
    ///
    /// 保留本轨迹 [0, t_join] 的部分。用于把 backup 刹车段接在 exp 之后，
    /// 使"任何时刻都有一条已验证安全的参考在发"（SUPER 的 cmd_traj 拼接）。
    bool spliceAfter(double t_join, const PathTrajectory& seg);

    /// 生成**刹车段**：从 start_p / start_v 沿速度方向减速到静止。
    ///
    /// 返回 false 表示无需刹车（速度已经足够小）。
    static bool buildBrake(const Eigen::Vector3d& start_p,
                           const Eigen::Vector3d& start_v,
                           const TrajParams& params,
                           PathTrajectory& out);

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
    bool ends_at_rest_{true};
    std::vector<double> stamps_;
    std::vector<Eigen::Vector3d> pos_;
    std::vector<Eigen::Vector3d> vel_;
    std::vector<Eigen::Vector3d> acc_;
};

}  // namespace azj_planner
