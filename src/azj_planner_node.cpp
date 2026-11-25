// Azj Planner —— 安全优先的 3D 局部规划器。
//
// 与 EGO / SUPER 的根本区别：
//   它们把 clearance 当**阈值**（dist > dist0 时代价恒为 0，解由最短路径决定 → 贴边）；
//   这里把 clearance 当**连续收益**（代价 = 长度 × (1 + w · (ref/c)^alpha)），
//   在必须通过的窄处自然走到中轴，同时长度仍是主目标，不会为宽度绕远。
//
// 建图完全复用 rog_map（ROG-Map），本文件不修改其任何源码。
//
// 与控制器（as_controller）的接口：
//   发布 quadrotor_msgs/PositionCommand 到 /planning/pos_cmd —— 与 SUPER/EGO 同一约定，
//   `navigationIso` / `navigationContour` / `navigationOMMPC` 可直接跟踪，控制器侧无需改代码。
//
//   ⚠ 该话题与 SUPER/EGO 共用，**一次只能跑一个规划器**。

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/ParamGet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <rog_map_ros/rog_map_ros1.hpp>
#include <yaml-cpp/yaml.h>

#include <cstring>

#include "azj_planner/curve_optimizer.h"
#include "azj_planner/minco_optimizer.h"
#include "azj_planner/pillar_clearance_field.h"
#include "azj_planner/path_search.h"
#include "azj_planner/trajectory.h"

namespace azj_planner {

class AzjPlannerNode : public rog_map::ROGMapROS {
public:
    /// 沿轨迹预先规划的 yaw 曲线（2026-11-05，照 SUPER 的做法）。
    ///
    /// ★ 为什么不能逐帧跟瞬时速度方向（这是 yaw 掉高的根因）：
    ///   旧写法 `target = atan2(v.y(), v.x())` 每帧取瞬时速度方向。
    ///   虽然单帧只变 0.012 rad，但参考**一直跟着速度抖**，PX4 就一直追；
    ///   实测实际 yaw 稳态落后指令 26~30°。
    ///   持续的姿态跟踪误差 → 姿态环持续输出偏航力矩 → 电机差速占用行程
    ///   → 推力下降 → 掉高（实测 z 从 0.94 掉到 0.00）。
    ///
    /// ★ SUPER 的做法（super_planner/src/traj_opt/yaw_traj_opt.cpp）：
    ///   waypoint 的 yaw = `pos(t+0.5s) - pos(t)` 的方向 —— **前视 0.5 秒
    ///   的平均方向**，不是瞬时方向；再用 minimumJerk 多项式插值成连续曲线。
    ///   参考平滑缓慢 → PX4 跟得上 → 无持续力矩 → 不掉高。
    ///
    /// 这里用简化版：同样前视 0.5s，unwrapped 成连续序列，
    /// 0.1s 间隔 + smoothstep 插值（端点导数为 0，段间无折角）。
    struct YawProfile {
        std::vector<double> t;      // 时间戳 [s]
        std::vector<double> yaw;    // 连续（unwrapped）yaw [rad]
        bool valid() const { return t.size() >= 2; }

        double at(double tq) const {
            if (!valid()) return 0.0;
            if (tq <= t.front()) return yaw.front();
            if (tq >= t.back()) return yaw.back();
            size_t lo = 0, hi = t.size() - 1;
            while (hi - lo > 1) {
                const size_t mid = (lo + hi) / 2;
                if (t[mid] <= tq) lo = mid; else hi = mid;
            }
            const double dt = t[hi] - t[lo];
            if (dt < 1e-9) return yaw[lo];
            const double u = (tq - t[lo]) / dt;
            const double w = u * u * (3.0 - 2.0 * u);   // smoothstep
            return yaw[lo] + w * (yaw[hi] - yaw[lo]);
        }
    };

    struct TrajSnapshot {
        PathTrajectory traj;
        ros::Time start;
        uint32_t id{0};
        Eigen::Vector3d v0{Eigen::Vector3d::Zero()};   // 生成时的实际速度（淡入用）
        YawProfile yaw_prof;                           // 预规划 yaw 曲线

        /// backup（刹车）段在轨迹时间轴上的起点 [s]（2026-11-14，学 SUPER）。
        /// >= 1e6 表示无 backup 段；否则 [backup_start, duration] 是刹车段。
        /// 发布时用它把 trajectory_flag 标成 2（与 SUPER 的 on_backup_traj 对齐）。
        double backup_start{1e9};
        bool onBackup(double tt) const { return tt >= backup_start; }
    };

    AzjPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                   const std::string& cfg_path)
        : rog_map::ROGMapROS(nh, cfg_path), nh_(nh), pnh_(pnh) {
        if (!cfg_.esdf_en || !esdf_map_) {
            throw std::runtime_error(
                "rog_map ESDF disabled: set rog_map/esdf/enable true in the yaml");
        }
        loadParams(cfg_path);
        clampYawRateMax();   // 防止 yaw_rate_max 超过 PX4 能力（实测会掉高）
        checkHoverThrust();  // 推力基准自检（实测这是掉高的第二大原因）
        // 运行时切换用的初值 = 启动参数（yaml / launch 命令行）
        yaw_mode_rt_.store(np_.yaw_mode);

        goal_sub_ = nh_.subscribe(np_.goal_topic, 1, &AzjPlannerNode::goalCallback, this);

        // rog_map 的 odom 回调只填 p/q/yaw，**不填速度**。
        // 重规划需要真实速度做轨迹起点速度，否则每次都当静止起步。
        if (!np_.vel_topic.empty()) {
            vel_sub_ = nh_.subscribe(np_.vel_topic, 10, &AzjPlannerNode::velCallback, this);
            ROS_INFO("[azj] velocity from %s", np_.vel_topic.c_str());
        } else {
            // 从建图用的同一个 odom 话题取 twist —— 位姿与速度同源
            odom_vel_sub_ = nh_.subscribe(cfg_.odom_topic, 10,
                                          &AzjPlannerNode::odomTwistCallback, this);
            ROS_INFO("[azj] velocity from odom twist (%s)", cfg_.odom_topic.c_str());
        }

        // latch=true：rviz 晚于规划器启动时也能立刻看到最后一次的路径/目标，
        // 否则订阅建立之前发布的消息会丢，表现为"路径没有可视化"。
        path_pub_ = nh_.advertise<nav_msgs::Path>(np_.path_topic, 1, /*latch=*/true);
        marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>(np_.marker_topic, 1, /*latch=*/true);
        goal_marker_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/azj/goal_marker", 1, /*latch=*/true);
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(np_.cmd_topic, 10);

        // 双定时器：
        //   replan  — 低频，重新搜索路径并做时间参数化
        //   publish — 高频，按轨迹推进发布参考点
        // 为什么必须高频发：控制器的 nav_traj_timeout 默认 0.5s，
        // 超过就认为轨迹失效并刹车。不能"规划一次发一次"。
        replan_timer_ = nh_.createTimer(ros::Duration(1.0 / np_.replan_rate),
                                        &AzjPlannerNode::replanCallback, this);
        // 发布**不走 timer**，走独立线程（见成员区注释）。
        // timer 挂在 spinner 上，规划阻塞时它也就停了 —— 那正是断流的根因。
        startPublishThread();

        ROS_INFO("[azj] node ready.");
        ROS_INFO("[azj] robot_radius=%.2fm  clearance_weight=%.3f (0=pure shortest)  cap=%.2fm",
                 sp_.robot_radius, sp_.clearance_weight, sp_.clearance_cap);
        ROS_INFO("[azj] max_vel=%.2f max_acc=%.2f max_lat_acc=%.2f  publish_rate=%.0fHz",
                 tp_.max_vel, tp_.max_acc, tp_.max_lat_acc, np_.publish_rate);
        if (std::isnan(np_.goal_z_override)) {
            ROS_INFO("[azj] goal_z_override = disabled (use message z)");
        } else {
            ROS_INFO("[azj] goal_z_override = %.2f m (rviz 2D Nav Goal gives z=0)",
                     np_.goal_z_override);
        }
        // ================= 启动自检：z 边界是否可用 =================
        //
        // rog_map 会把 virtual_ground/ceil_height 各侵蚀 inflation_step*resolution
        // （config.hpp:407 起）。配置值和实际生效值可能差很多，这里用
        // cfg_ 里的**重算后值**做检查 —— 这是唯一可信的来源。
        //
        // 两个必须成立的条件：
        //   ① ceil_eff  > 飞行高度    否则 "Odom above virtual ceil"，
        //                             整帧点云被丢弃、地图永远为空
        //   ② ground_eff <= 0         否则飞机在地面时就报
        //                             "Odom below virtual ground"
        {
            const double zc = cfg_.virtual_ceil_height;
            const double zg = cfg_.virtual_ground_height;
            ROS_INFO("[azj] map z bounds (after inflation erosion): [%.2f, %.2f]",
                     zg, zc);

            double expect_z = std::isnan(np_.goal_z_override) ? 0.8 : np_.goal_z_override;
            if (zc <= expect_z + 0.05) {
                ROS_FATAL("[azj] virtual_ceil_height=%.2f <= flight height %.2f !",
                          zc, expect_z);
                ROS_FATAL("[azj]   -> point cloud will be DISCARDED as 'above virtual ceil'");
                ROS_FATAL("[azj]   -> raise rog_map/virtual_ceil_height in the yaml");
            } else if (zc - expect_z < 0.30) {
                ROS_WARN("[azj] virtual_ceil_height=%.2f is only %.2f m above flight "
                         "height %.2f -- tight",
                         zc, zc - expect_z, expect_z);
            }
            if (zg > -0.02) {
                ROS_FATAL("[azj] virtual_ground_height=%.2f > 0 !", zg);
                ROS_FATAL("[azj]   -> on the ground (z=0) rog_map will warn "
                          "'Odom below virtual ground' and skip the frame");
                ROS_FATAL("[azj]   -> lower rog_map/virtual_ground_height, or set "
                          "inflation_step: 1");
            }
            if (np_.enforce_height_limits && np_.ceil_z > zc) {
                ROS_WARN("[azj] ceil_z=%.2f > map ceil=%.2f: planner would allow "
                         "flight where there is no map",
                         np_.ceil_z, zc);
            }
            if (np_.enforce_height_limits && np_.floor_z < zg) {
                ROS_WARN("[azj] floor_z=%.2f < map ground=%.2f: planner would allow "
                         "flight below the map",
                         np_.floor_z, zg);
            }
            if (np_.enforce_height_limits) {
                ROS_INFO("[azj] plan height limits: [%.2f, %.2f] (floor/ceil_z)",
                         np_.floor_z, np_.ceil_z);
                ROS_INFO("[azj] height hold: soft band=+-%.2fm w=%.1f(w=%.0f beyond), "
                         "HARD z_band=+-%.2fm",
                         np_.height_band, np_.w_height, np_.w_height_band, np_.z_band);
            }
        }

        ROS_INFO("[azj] plan_mode=%s (runtime switch: rosparam set /azj_planner/plan_2d true|false)",
                 np_.plan_2d ? "2D (XY only)" : "3D");
        if (np_.plan_2d) {
            if (np_.plan_2d_slab_half <= 1e-6) {
                ROS_INFO("[azj]   2D: z locked at goal height, single-slice ESDF "
                         "(no projection - box holes stay passable)");
            } else {
                ROS_INFO("[azj]   2D: z locked at goal height, slab +-%.2f (projected)",
                         np_.plan_2d_slab_half);
            }
        }
        ROS_INFO("[azj] curve=%s  w_mid=%.2f w_obs=%.1f w_anchor=%.2f iters=%d",
                 np_.curve_enable ? "ON (B-spline, C2)" : "OFF (polyline)", cp_.w_mid, cp_.w_obs,
                 cp_.w_anchor, cp_.iterations);
        ROS_INFO("[azj] curve v1.2: cp_spacing=%.2fm projection=%s(%dx%d) local_center=%s",
                 cp_.control_point_spacing, cp_.enable_projection ? "on" : "off",
                 cp_.projection_iters, cp_.projection_samples,
                 cp_.enable_local_centering ? "on" : "off");
        ROS_INFO("[azj] output: |vz|<=%.2f m/s, vel blend %.2fs after replan",
                 np_.max_vel_z_out, np_.vel_blend_time);
        ROS_INFO("[azj] yaw: mode=%d follow_velocity=%d rate_max=%.2f rate_flight=%.2f "
                 "deadband=%.2f alt_guard=%.2fm (0=off)",
                 np_.yaw_mode, static_cast<int>(np_.yaw_follow_velocity),
                 np_.yaw_rate_max, np_.yaw_rate_flight, np_.yaw_deadband,
                 np_.yaw_alt_guard_drop);
        ROS_INFO("[azj] goal z: override=%s min_goal_z=%s",
                 std::isnan(np_.goal_z_override) ? "disabled" : "set",
                 std::isnan(np_.min_goal_z) ? "disabled" : "set");
        ROS_INFO("[azj] goal=%s  cmd_out=%s", np_.goal_topic.c_str(), np_.cmd_topic.c_str());
        ROS_WARN("[azj] cmd topic %s is shared with SUPER/EGO - run only one planner at a time",
                 np_.cmd_topic.c_str());
    }

private:
    struct NodeParams {
        double replan_rate{5.0};
        double publish_rate{50.0};
        std::string goal_topic{"/move_base_simple/goal"};
        std::string path_topic{"/azj/path"};
        std::string marker_topic{"/azj/path_marker"};
        std::string cmd_topic{"/planning/pos_cmd"};
        // 空 = 直接从 rog_map 订阅的那个 odom 话题的 twist 取速度。
        // 这样仿真(/gazebo_truth/odom)和真机(mavros odom)都不用额外配置前缀，
        // 且与建图位姿严格同源（不会出现"位置用 A、速度用 B"的错配）。
        std::string vel_topic{""};
        std::string frame_id{"map"};
        bool unknown_as_free{false};
        double free_clearance_if_unknown{0.5};
        // UNKNOWN 区域的 clearance 上限。unknown 不代表有障碍，
        // 只是没探测过；给个上限即可（不硬拒，也不鼓励）。
        double unknown_clearance_cap{0.5};

        // ---- 层高硬约束（★ 解决"路径从地板下/天花板上绕"）----
        //
        // 为什么必须有：地板下面、天花板上面没有点云（雷达看不到），
        // 于是 clearance 走 unknown 策略（cap 0.5 > robot_radius 0.3）
        // → 判为"可通行"。A* 于是抄近路从下面钻过去 —— 但下面是地板。
        //
        // 注意这两个参数和 rog_map 的 virtual_ground/ceil_height **不是一回事**：
        //   · rog_map 那两个是"点云裁剪边界"，而且会被 inflation_step 侵蚀
        //     （config.hpp:407 起，每侧各吃 inflation_step*resolution）
        //   · 这里是"规划可通行空间"，直接参与 clearance 判定
        //
        // floor_z 取略低于 0（给点云噪声和落地留余量），并建议 >= robot_radius。
        bool enforce_height_limits{true};
        double floor_z{-0.10};
        double ceil_z{2.00};

        // 高度保持软约束权重。0 = 关闭。
        // 不加这个的话，绕行时 z 方向完全自由，会无谓地爬升/下潜；
        // 权重不宜大，否则真的需要翻越障碍时会被压住。
        double w_height{0.0};
        // 高度参考：取起终点平均高度 + 这个偏移
        double height_ref_offset{0.0};
        // dz 归一化尺度 [m]
        double height_scale{1.0};
        // 高度豁免带 [m]：|dz| <= 本值不额外罚
        double height_band{0.30};
        // 超出豁免带的强罚权重（压住"往 clearance 大的地方钻"）
        double w_height_band{20.0};
        // ★ 高度硬约束带 [m]：|z-参考| 超过它直接不可通行。0=关闭。
        // 实测软惩罚压不住（贴墙代价 5.44 vs 钻下方 1.84），必须硬约束。
        double z_band{0.40};

        // z 方向速度输出限幅 [m/s]。
        // 控制器的 vz 会被当作速度前馈直接下发（type_mask 未设 IGNORE_VZ），
        // 所以规划器输出的 velocity.z 要保守，避免重规划瞬间把飞机拽下去/顶上去。
        double max_vel_z_out{0.30};
        // 重规划后参考速度的淡入时间 [s]。
        // 新轨迹刚生成时，参考速度从"当前实际速度"平滑过渡到轨迹速度，
        // 避免 vz 阶跃（实测重规划瞬间 vz 跳变是掉高的直接诱因）。
        double vel_blend_time{0.35};

        // 曲线最大允许曲率 [1/m]。超过就退回折线。
        // 曲率半径 = 1/kmax。≤12 对应 R≥0.083m。
        // 实测无保护时会出现 kmax=418（R=2.4mm）的打结曲线，控制器根本跟不了。
        // 考虑到 raicom 的框孔 1.1m、柱缝 0.5m，通过时必然急转。
        // R=0.1m（kmax=10）是几何下限。速度规划会按 max_lat_acc 自动降速
        // （v = sqrt(1.2/10) = 0.35 m/s），所以高曲率不等于不安全。
        // R = 1/kmax。20 对应 R=0.05m。
        // 速度规划会按 max_lat_acc 自动降速（v=sqrt(1.2/20)=0.24 m/s），
        // 所以高曲率本身不等于不安全。实测 raicom 紧缝里曲线 kmax≈12.5，
        // 卡在 12 会频繁退回 5 点折线（用户报"没有曲线优化"）。
        double max_curvature{20.0};
        double goal_yaw{NAN};          // 固定目标 yaw；NaN = 按 yaw_mode 决定

        // ── yaw 控制模式（2026-11-05 新增）──
        // 为什么要有模式而不是一个 bool：机头朝向的语义分好几种，
        // 不同任务要的不一样，写死任何一个都会在下个场景里变成问题。
        //
        //   0 = HOLD          保持起飞时的机头，全程不转（旧默认）
        //   1 = VELOCITY      朝速度方向（EGO/SUPER 的行为，穿越时最有用）
        //   2 = GOAL          朝目标点（悬停对准、拍照/投放前定点朝向）
        //   3 = FROM_MSG      用打点消息里的 orientation（rviz "2D Nav Goal"
        //                     按住拖方向，或地面站指定机头）
        //
        // 默认 1：用户明确要 yaw 控制，而穿越障碍时朝前是唯一有意义的默认。
        // 想退回"不转"就设 0。
        int yaw_mode{1};

        // 速度门限 [m/s]：低于它不跟随（悬停/微调时速度方向是噪声，
        // 跟着转会左右抖）。仅 mode=1 生效。
        double yaw_vel_thresh{0.15};

        // 角度死区 [rad]：目标与当前夹角小于它就不动。
        // 抑制"路径微摆 → 机头跟着摆"。0.05 rad ≈ 2.9°。
        double yaw_deadband{0.05};

        // 兼容旧参数：yaw_follow_velocity=true 且 yaw_mode==0 时升到 VELOCITY。
        // 老 yaml 里这个字段还在，不能因为新增 yaw_mode 就让它静默失效。
        bool yaw_follow_velocity{false};

        // yaw 参考的最大变化率 [rad/s]。
        //
        // ⚠ 1.5 会导致掉高！2026-11-05 实测事故：
        //   偏航角速度持续 1.45~1.48 rad/s 转约 1 秒后，飞机从 z=0.83 掉到地面。
        //   同一次飞行里 PX4 一直在发 +0.70 m/s 的爬升指令，实际 vz=-1.62
        //   （超过 MPC_Z_VEL_MAX_DN=1.0，说明已不受速度环控制）。
        //   姿态正常（roll≤11.7°、pitch≤17.8°）、未撞柱、PX4 高度估计准确（误差<0.15m）。
        //
        //   机理：四旋翼靠对角电机差速产生偏航力矩。持续大角速度偏航占用推力预算，
        //   转向需求与爬升需求同时拉满时总升力不够。
        //   对照：yaw 不动的 18 分钟飞行（vfix.bag）里 |yaw_rate|>0.3 仅 23 帧，
        //   且那些时刻高度仍在爬升（+0.028m/0.5s）—— 慢速/瞬时偏航是安全的。
        //
        // 现在默认 0.5 rad/s（≈29°/s，转 180° 需 6.3s）。慢，但推力不塌。
        // 想快一点可试 0.7~0.8，务必先确认高度不掉。
        double yaw_rate_max{0.6};

        /// 飞行中的偏航角速度上限 [rad/s]（2026-11-14 新增）。
        ///
        /// 与 yaw_rate_max 的区别：
        ///   yaw_rate_max       = **到点/悬停**时的上限（速度环没有竞争）
        ///   yaw_rate_flight    = **飞行中**的上限（速度环正在抢电机行程）
        ///
        /// 为什么必须分档（yaw1_before_rate_fix.bag 实测）：
        ///   t=19.25~21.0，cmd yaw 以 1.0 rad/s 转 80°，PX4 实际落后 43°；
        ///   同段 z 从 0.85 掉到 0.01（truth vz=-1.8 m/s，而 sp_vz 已顶在
        ///   +0.70 限幅）。roll/pitch 只有 ±11° —— 不是姿态失控，
        ///   是偏航力矩吃掉了推力预算。
        ///
        /// 0.5 rad/s 依据：yaml 历史实测里 0.5 不掉、1.5 掉。
        /// 0 = 关闭分档（飞行中也用 yaw_rate_max）。
        double yaw_rate_flight{0.5};

        /// 掉高保护阈值 [m]（2026-11-14 新增）。
        ///
        /// 参考 z 比实际 z 高出这个值时，**暂停偏航转向**，全力保高。
        /// 安全优先理念的兑底：不管 yaw 参考多想转，飞机正在掉就先不转。
        /// 0 = 关闭。
        double yaw_alt_guard_drop{0.35};

        /// 预规划 yaw 曲线的 waypoint 间隔 [s]，与 SUPER 的
        /// `interp_dt = π/yaw_dot_max`（yaw_dot_max=1.5 → 2.09s）对齐。
        ///
        /// 与 yaw_rate_max **解耦**：后者是每帧转角限幅（受 PX4 能力约束），
        /// 这个只影响参考曲线的稀疏度。见 buildYawProfile 里的说明。
        double yaw_profile_dt{2.1};

        /// PX4 的 MPC_YAWRAUTO_MAX [deg/s]，用于钳制 yaw_rate_max。
        ///
        /// 0 = 自动从 mavros 读（读不到时按 45 兜底）。
        /// 显式指定更可靠 —— PX4 参数改过之后，这里也要跟着改。
        /// 2026-11-09：PX4 已调到 120 deg/s，所以这里默认写 120。
        double px4_yaw_auto_max_deg{120.0};
        // 目标高度覆盖。NaN = 用消息里的 z（真机地面站会带正确高度）。
        // 仿真 rviz 的 2D Nav Goal 在地面平面上点击，z≈0，直接用会掉到地面，
        // 所以仿真里必须设成 fly_height。
        double goal_z_override{NAN};

        /// 目标高度安全下限 [m]（2026-11-14 新增）。
        ///
        /// 消息里的 goal.z 低于它时抬到这个值。
        /// 防的是“rviz 2D Nav Goal 的 z≈0”和“点云把目标吸附到地面附近”：
        /// 一旦 goal.z≈0，A*/轨迹会合法地规划到地面 —— 看起来就是“掉高”。
        /// 建议取 floor_z + 0.3 左右；NaN = 关闭。
        double min_goal_z{0.30};
        double arrive_tol{0.25};       // 判定到达的距离
        // 判定到达的速度上限 [m/s]。只看距离会导致“带着速度判到位”，
        // 规划器停止输出轨迹后 PX4 位置环刹不住 → 冲过目标 0.15~0.25m。
        // 实测轨迹末端的自然末速很小，0.20 不会拖延到达。
        double arrive_vel_tol{0.20};
        // ---- 2D / 3D 模式 ----
        // true  = 只在 XY 平面规划（高度锁在目标高度）
        // false = 完整 3D
        // 支持运行时用 `rosparam set /azj_planner/plan_2d true` 切换
        // （节点每个重规划周期读一次，切换后自动重建轨迹）。
        bool plan_2d{false};
        // 2D 模式下柱状 clearance 的 z 采样半范围 [m] 与采样点数。
        // 必须覆盖"当前高度↔目标高度"的过渡段，否则过渡途中会撞到
        // 只在该高度区间内存在的障碍。
        double plan_2d_z_half{0.8};      // 已废弃（保留兼容，不再使用）
        int plan_2d_z_samples{9};
        // 2D 单层规划：默认 0 = 只用该层的真实 3D ESDF（推荐）。
        //
        // 为什么不投影：投影会把垂直方向的空隙压掉。穿框时框的孔在
        // 规划层是空的，投影后变实心，本来能过变成不能过。
        //
        // 想留一点垂直余量（机体厚度/控制误差）就设 0.05~0.15。
        // **不要超过 robot_radius**，一旦覆盖到地板/天花板，
        // 每列 clearance 会被压到 ~0，整个平面判死。
        double plan_2d_slab_half{0.0};
        // 已废弃
        double plan_2d_z_extra{0.10};

        bool curve_enable{true};       // 曲线优化开关（false 退回折线）
        double curve_sample_step{0.05};// 曲线按弧长重采样的点间距 [m]
        double replan_lead{0.5};       // 轨迹剩余时间小于它就重规划 [s]
        double replan_deviation{0.5};  // 偏离参考点超过它就重规划 [m]
        double safety_lookahead{0.5};  // 前方安全监控前瞻时长 [s]

        // ═══ BLOCKED 判据的分层阈值（2026-11-14，学 SUPER 的语义）═══
        //
        // ★ 为什么必须改（实测震荡根因）
        //
        // 旧判据是 `clearance < robot_radius`（0.30）。而实测日志里：
        //     len=4.88918m  min_clearance=0.3m (safe 0.3m)
        // 路径 clearance **恰好等于** robot_radius —— 曲线优化的目标本来就是
        // "贴着 safe_margin=robot_radius 走"（w_obs hinge 在 c<safe_margin 时才罚），
        // 所以最优解自然收敛到 c≈robot_radius。于是判据在边界上来回抖：
        //   · ESDF 插值/曲线重采样的微小误差让某点变成 0.2999 → BLOCKED
        //   · 下次规划后又是 0.3001 → 不 BLOCKED
        // 再叠加 `unknown_clearance_cap=0.32` 只比 robot_radius 高 0.02 ——
        // 前方未建图区按 0.32 过线，ESDF 一更新就跌破 0.30。
        //
        // 实测后果（/tmp/azj_planner.log 13:23）：
        //   37.026 BLOCKED → 折线 ; 37.950 曲线 ; 40.150 BLOCKED → 折线 ; 41.249 曲线
        // 每 3 秒一次，每次都是"折线→平滑→曲线→再折线"的循环。
        //
        // ★ SUPER 怎么做的（super_planner.cpp:454 起 generateExpTraj）
        //   `map_ptr_->getInfGridType(temp_pt) == OCCUPIED || OUT_OF_MAP`
        //   —— 判的是**离散膨胀格占用**，不是连续 clearance 阈值。
        //   路径只要在膨胀格外就绝不触发，天然有 1 格（0.1m）的量化余量。
        //   而且它的前瞻只到 `replan_forward_dt = 0.1s`（不是 1.5s）。
        //
        // ⚠⚠ 阈值不能放松太多（2026-11-14 事故教训）
        //
        // 上一版设 near=0.85(0.255m) / far=0.55(0.165m)，理由是"阈值已由
        // 规划层保证"。**这个理由是错的**：路径可能因为地图更新而变得不安全，
        // 监控层必须独立把关，不能假设规划层的保证永远成立。
        //
        // 实测后果（/tmp/azj_flight_20261114_150005）：
        //   clearance 掉到 0.151m 才第一次报 BLOCKED（比 robot_radius 低一半），
        //   在此之前飞机已经贴着障碍飞了很久 → 撞上。
        //
        // 现在只比 robot_radius 低 5%：
        //   近段 0.95 → 0.285m（刚好避开"曲线优化收敛到 0.300"的浮点抖动）
        //   远段 0.90 → 0.270m（只放松 10%，仍能在变得危险前发现）
        //
        // 与 SUPER 对照：它判 `getInfGridType==OCCUPIED`（离散膨胀格，
        // 膨胀半径 = robot_r），等价于 `clearance < robot_radius` 但无浮点问题。
        // 我们用 0.95 的容差来等价地获得那点余量。
        double safety_near_frac{0.95};
        double safety_far_frac{0.90};
        double safety_near_span{0.5};   // 近段时长 [s]

        // ═══ 热启动（hot init / receding horizon）2026-11-14 ═══
        //
        // ★ 这是学 SUPER 最关键的一处（super_planner.cpp:379 generateExpTraj）
        //
        // SUPER 的做法：
        //   · 把**上一条已提交轨迹**当作引导路径（guide_path）
        //   · 沿它采样做碰撞检查（getInfGridType != OCCUPIED 即畅通）
        //   · 畅通时 `split_dis = max`（super_planner.cpp:478~482）
        //     → **整条旧轨迹都保留**，A* 只规划末端到目标那一小段
        //   · 用 guide_path + guide_stamp 作**优化初值**（热启动）
        //
        // 为什么这决定了"优化快不快"：
        //   初值本身已经在一条**平滑可行**的曲线上（旧轨迹就是优化出来的），
        //   梯度下降几步就收敛。而冷启动的初值是 A* 折线 ——
        //   45°/90° 硬角，必须迭代几十次才能压平（实测 60 次 850ms）。
        //
        // azj 的对等实现：
        //   1. A* 之前，沿旧轨迹取一段**当前畅通**的引导段
        //   2. A* 只规划「引导段末端 → 目标」（搜索空间从 6m 降到 2m 级）
        //   3. 合并「引导段 + A* 段」作为曲线优化初值
        //
        // 开关与范围：
        double hot_init_reuse{1.5};      // 复用旧轨迹的时长 [s]；0=关闭热启动
        double hot_init_min_len{0.5};    // 引导段最小长度 [m]，太短没意义
        double hot_init_max_len{3.0};    // 引导段最大长度 [m]，防止搜索空间过小
        double hot_init_clear_frac{0.95};// 引导段要求的 clearance（× robot_radius）

        // ══════════════════ 滚动窗口重规划（2026-11-17，学 SUPER）══════════════════
        //
        // 现状问题：每次重规划都做「anchor → goal 全量 A* + 全量曲线优化」，
        //   单次 10~70ms，15Hz 下等于每秒把整条轨迹重算 15 遍；
        //   且终点始终是 goal，末段形状由「远处的目标」决定，
        //   ESDF 新扫出的障碍只能用整条重优化去响应 —— 慢且不居中。
        //
        // SUPER 的做法（exp_traj_optimizer_s4.cpp:589 的 setConditions）：
        //   起点 = 旧轨迹在未来 replan_forward_dt 的状态（硬约束 p,v,a），
        //   末点 = 沿全局路径向前滚动的一段（receding horizon），
        //   只优化这一段，前一秒的轨迹不动 → 速度天然连续。
        //
        // 本实现（rolling）把同一套语义搬到 azj：
        //   1. 起点：旧轨迹在 t_now + rolling_forward_dt 的 (p,v)，不是 odom
        //   2. 末点：沿缓存的全局 A* 路径前探 rolling_lookahead 米
        //   3. 末端居中引导：末点处沿 clearance 梯度升到通道中轴
        //      → ESDF 一变，下一次重规划立刻在**末端**体现，不需要整条重优化
        //   4. 只优化 [起点, 末点] 段；stop_at_end=false → 局部目标处不减速
        //   5. 全局 A* 只在 goal 变了 / 缓存过期 / 剩余段被挡时重跑
        bool rolling_enable{true};

        /// 起点前视时长 [s]。**默认 0，不要调大**（2026-11-17 定论）。
        ///
        /// ★ 这里曾经设 0.1（模仿 SUPER 的 replan_forward_dt），结果造成
        ///   "路径跑得比飞机快、控制器一直追不上"。实测铁证
        ///   （/tmp/tc.bag，forward_dt=0.1）：
        ///
        ///     参考点推进速率 p50=0.44 max=40.6，其中 **82 次跳变**
        ///     （间隔 0.21s，每次前跳 0.06~0.10m → 瞬时 3~5 m/s）
        ///
        ///   为什么：`p0` 同时是"新轨迹 t=0 位置"和"发布的第一帧参考位置"。
        ///   取旧轨迹 t_now+dt 处，就等于**每重建一次参考点就瞬移 v*dt**。
        ///   重建周期 0.21s、dt=0.1 时，等效额外推进 0.08/0.21 ≈ 0.38 m/s，
        ///   叠加在真实播放速度上 → 参考永远比飞机快 → lag 稳定在 0.3~0.6 不收敛。
        ///
        ///   SUPER 能用 replan_forward_dt 是因为它**同时把轨迹的 start_WT
        ///   也延后**（新轨迹在前 dt 秒内还没生效，继续发旧轨迹），
        ///   时间轴连续。我们没有那个机制，所以必须取 0。
        ///
        /// 实测对比（同一 bench，PI 控制器）：
        ///
        ///   forward_dt  跳变次数  最大速率   lag(稳态)   飞机位移
        ///   0.1         82        40.6 m/s   0.30~0.60   14.05m
        ///   0.0         **1**     **8.7**    **0.15**    14.33m
        ///
        /// 为什么 0 也不会"参考贴着飞机、飞机不动"：
        ///   p0 取旧轨迹**当前**位置，而旧轨迹本身领先飞机 lag 米
        ///   → 参考仍然领先飞机，有前进目标；再加上速度前馈在拉。
        double rolling_forward_dt{0.0};

        /// 末点前探距离 [m]（receding horizon 长度）。
        ///
        /// 这是「滚动窗口」的核心：末点不是 goal，而是沿全局路径前方
        /// lookahead 米处的点。取值权衡：
        ///   · 太小（<1.5m）→ 优化段太短，A* 每次都得往前重搜，且转弯
        ///     来不及提前量，曲线贴着拐角 → 靠近一侧
        ///   · 太大（>8m）→ 退化成全量规划，失去"只改末端"的收益
        /// 4m 在 1 m/s 下约 4s 前视，与 SUPER 的 planning_horizon 同量级。
        double rolling_lookahead{4.0};

        /// 前探下限 [m]。剩余距离小于它就直奔 goal（末段不再滚）。
        double rolling_lookahead_min{1.5};

        /// 全局路径缓存有效期 [s]。超过就重跑一次 A*（防缓存过旧）。
        double rolling_global_ttl{1.5};

        /// 目标移动超过该值才重建全局路径 [m]。
        double rolling_goal_refresh_dist{0.50};

        /// 末端居中引导：梯度上升迭代次数。
        ///
        /// 作用对象是**末点**：沿 ∇clearance 把末点推到该横截面 clearance
        /// 最大处（= 两侧等距）。这就是「末端引导居中避障」。
        /// 0 = 关闭（末点取全局路径原样）。
        int rolling_center_iters{8};

        /// 末端居中引导的位移步长 [m/次]。
        double rolling_center_step{0.08};

        /// 末端居中引导的最大位移 [m]。防止末点被推到通道另一端。
        double rolling_center_max_shift{0.60};

        // ══════════════════ MINCO 后端（2026-11-17）══════════════════
        //
        // 0 = B 样条 + TOPP（旧路径，curve_optimizer + trajectory.cpp）
        // 1 = MINCO s=4（起点 p/v/a/j 硬约束，形状与时间联合优化）
        //
        // 为什么需要 MINCO：B 样条只约束起点速度 v，不约束加速度。
        // rolling 每 0.1s 重建时，每条轨迹都从 v0 靠 max_acc 重新加速，
        // 速度永远停在加速段（实测 |v0| 恒 0.285，max_vel=1.0）。
        int traj_backend{1};

        /// MINCO 居中权重。30 是横扫实测的最优值（见 minco_optimizer.h 的表格）：
        /// 直通道半宽 1.5m、初值偏 0.8m 时，60 次迭代可做到横向不平衡 0.05m。
        double minco_w_mid{30.0};
        double minco_w_obs{50.0};
        double minco_w_smooth{1.0};
        /// MINCO 迭代次数。60 实测约 8ms（4m 段只要 1.6ms），够用。
        int minco_iterations{60};

        /// rolling 段**可跟踪加速度** [m/s²]。
        ///
        /// ★ 为什么单独一个参数，而不是直接调小 max_acc：
        ///   max_acc(1.5) 是"规划器认为的物理上限"，用于 A*/B 样条的可行性判断，
        ///   调小会让全量规划也变保守（实机 PX4 能力比仿真强，没必要）。
        ///   而仿真 PX4 实测加速度只有约 0.4 m/s²
        ///   （/tmp/azj_flight_20261117_132140：0.9s 走 0.377m）。
        ///   所以 rolling 段的轨迹与参考速度都改用这个更保守的值。
        ///
        /// 实测（弱控制器 bench，PX4 能力 0.4 m/s²）：
        ///   max_acc=1.5 (只用 track=1.5) → lag 0.88~1.03 平台，追不上
        ///   track=0.6, gain=1.0 → lag 0.79
        ///   track=0.8, gain=1.0 → lag **0.13**   ← 取这个
        ///   track=0.8, gain=2.0 → lag 0.56（增益过大反而抖）
        double rolling_track_acc{0.80};

        /// 允许的"参考速度领先飞机"量 [m/s]（2026-11-17）。
        ///
        /// 实测标定（三次实飞对照）：
        ///   0.05（v3.2.2 误打误撞的值）→ 横向偏离 0.120、跳变 1 次  ← 最好
        ///   0.63（按 sqrt(2aL) 理论值）→ 横向偏离 0.264、跳变 6 次、max 148m/s
        ///
        /// 为什么理论值反而差：sqrt(2aL) 假设飞机能持续以 a 加速去追，
        /// 但真实 PX4 在跟踪曲线时可用加速度远小于 max_acc（转弯要分配横向加速度）。
        /// 参考领先太多 → 飞机横向上忙于追赶 → 偏离反而变大。
        ///
        /// 所以这个值必须按"真实伺服能跟上"标定，不能按理论推。
        double rolling_v_lead{0.05};

        /// A* 的 clearance 余量倍数（2026-11-25）。
        ///
        /// A* 按 `robot_radius × 本值` 作为硬约束搜路，给 MINCO 的平滑
        /// 切角留空间。1.0 = 关闭（旧行为）。
        ///
        /// 实测依据：A* 按 1.0 规划时路径 clearance 恰好 0.30，MINCO 段内
        /// 切角压到 0.0039 → 连续被拒 → 折线。取 1.3 让折线离障碍 0.39m，
        /// 切角后仍有 ~0.3 余量。
        ///
        /// 找不到路会自动退回 1.0，所以窄缝场景不会因此瘫痪。
        double rolling_astar_margin{1.30};

        /// MINCO 输入/输出完整 dump（2026-11-25，为定位"为什么被拒"）。
        ///
        /// 空字符串 = 关闭。设成目录（如 `/tmp/azj_minco_dump`）后，每次
        /// `buildTrajectoryMinco` 调用会写一个文件，包含：
        ///   · head / tail 状态（p,v,a,j）
        ///   · 输入路径逐点 + 其 clearance（这是 A\* 给的东西）
        ///   · MINCO 输出轨迹逐点 + clearance（这是优化结果）
        ///   · 三道校验的判定值与阈值
        ///   · 优化参数（w_mid / w_obs / iterations / lock_z ...）
        ///
        /// 为什么需要：如果只看"被拒"的结果，无法区分"输入就贴障碍"
        /// 与"优化把好路径弄坏了"。这两种情况的修法完全不同。
        std::string minco_dump_dir{""};

        /// 从旧轨迹**继承**的引导段长度 [m]（2026-11-17，学 SUPER 的 receding_dis）。
        ///
        /// 这是重规划连续性的核心：新轨迹的前 receding_dis 米直接沿用旧轨迹
        /// 的几何形状，A\* 只补剩余部分 → 新旧轨迹在切换时刻空间重合
        /// → 不可能出现横向突变。
        ///
        /// 0 = 关闭（退回"每次从 A\* 全量结果取段"，会有横向跳变）。
        ///
        /// 为什么不是越长越好：继承段越长，A\* 能规划的剩余越短，
        /// 遇到新障碍时可调整的余地越小。2.0m 在 1 m/s 下约 2s 前视，
        /// 足够覆盖一个滚动周期（0.1s）的连续性需求，又留足重规划空间。
        double rolling_receding_dis{2.0};

        /// 轨迹过期（跑完但没到目标）时的保底速度 [m/s]（2026-11-17）。
        ///
        /// 为什么需要：控制器 `type_mask=3043` → XY 是**速度控制**，
        /// 参考速度为 0 就等于"原地停住"，位置误差完全不起作用。
        /// 一旦轨迹时间轴跑完而飞机还没到目标，不发保底速度就会停在半空。
        ///
        /// 取值：0.30（约 max_vel 的 1/3）。这只是防止停摆的兜底，
        /// 正常情况下永远不该触发 —— 触发时日志会报 EXPIRED，那本身就是要修的病。
        double traj_expired_min_vel{0.30};

        /// MINCO 连续被拒多少次后放弃 keep_old、改走 B 样条（2026-11-17）。
        ///
        /// keep_old 的目的是"避免路径形状突变"，但连续被拒说明当前场景
        /// （典型：未知建图区边界）MINCO 就是不适用 —— 此时一直保留旧轨迹
        /// 会让 /azj/path 长时间不更新，用户看到"轨迹静止"。
        ///
        /// 3 次 ≈ 0.3s（rolling_period=0.1）。既能跨过单次偶发拒绝，
        /// 又不会让 viz 停太久。
        int minco_max_reject_streak{3};

        /// lag 反馈的死区 [m]：落后小于它不扣速度（正常跟踪误差，不必惩罚）。
        ///
        /// 实测控制器稳态跟踪误差约 0.3~0.4m，死区取 0.35 正好放过它。
        double rolling_lag_free{0.35};
        /// lag 反馈增益 [1/s]：v_cap -= (lag - lag_free) * gain。
        ///
        /// 1.0 时，落后 1.35m 就要扣 1.0 m/s —— 足以让参考主动降到飞机速度以下。
        double rolling_lag_gain{1.0};
        /// 轨迹整体速度上限的 lag 衰减增益 [m/s per m]。
        ///
        /// v_allow = max_vel - (lag - lag_free) * gain
        /// lag=0.9 时 0.5 → v_allow=0.72，飞机（实测稳态 1.28）有 0.5 余量追上去。
        /// 0 = 关闭（只限制起点速度，不限制轨迹中段）。
        double rolling_lag_vel_gain{0.50};

        /// v_cap 的下限 [m/s]。不能归零，否则又变成"参考贴着飞机"的自举失败。
        double rolling_v_floor{0.25};

        /// 滚动周期 [s]：needReplan 报 NONE 时，每隔该时长也推进一次滚动。
        ///
        /// ★ 2026-11-17 修正：默认 0.1s（10Hz），真正的**连续滚动**。
        ///
        /// 曾经设成 0（只在 TRAJ_END 触发）—— 那就是用户说的
        /// “轨迹更新是无人机到达末点才更新，做不到一直向梯度最低衍生”。
        ///
        /// 中间还踩过一个坑：当时 0.5s 周期实测 45s 只前进 4.27m。
        /// 但那**不是**周期的错，是起点速度取了飞机实际速度（速度自举失败）。
        /// 根因已在 rollingReplan 里修掉（v0 改用旧轨迹的规划速度），
        /// 所以现在可以放心高频滚动。
        ///
        /// 0.1s 的依据：单次 rolling 实测 1~4ms，10Hz 只占 1~4% CPU；
        /// 比 SUPER 的 15Hz 略保守，但参考点更新粒度已经是 0.1m 级。
        double rolling_period{0.10};

        /// 末端居中引导只在与前进方向垂直的平面内移动（保留沿路径进度）。
        bool rolling_center_lateral_only{true};

        /// 滚动段的 A* 补搜索上限 [m]。全局路径剩余不足时用它补一段。
        double rolling_patch_len{6.0};

        /// 末端居中后是否要把末点贴回全局路径（false=允许偏离）。
        bool rolling_snap_end{true};

        // BLOCKED 滞回（2026-11-14）。
        // 一旦触发 BLOCKED，必须等 clearance 恢复到 robot_radius × 该系数以上，
        // 才允许再次因 clearance 触发 —— 防止"刚好卡在阈值上"的反复触发。
        // 1.0 = 关闭滞回（旧行为）。
        double safety_hysteresis{1.05};
        // 重规划最小间隔 [s]。机制性的防活锁兑底：
        // 实测一次完整规划 1.5~1.7s，所以这个值不会拖慢正常重规划；
        // 它只防止“规划→被拒→重规划”循环把主线程跑满。
        double min_replan_period{0.30};
        // 安全触发（BLOCKED）时的退避上限 [s]。只用于防同一帧重复进入，
        // **不是安全延迟**：新障碍侵入轨迹时必须尽快重规划。
        // 实测一次规划 0.33~1.31s，所以这 0.1s 在总延迟里占比很小，
        // 但原来 0.3s 的退避会让总延迟再叠加 0.3s。
        double blocked_replan_period{0.10};
        // 安全触发的重规划是否走“快通道”（跳过曲线优化）。
        // 曲线优化占单次规划耗时 95%+，紧急时用它换平滑度不划算。
        bool blocked_fast_path{true};
        /// 快通道折线后多久发起一次**主动**平滑重规划 [s]（2026-11-14）。
        /// 见 replanCallback 里的说明。0 = 关闭（退回"等 TRAJ_END"的旧行为）。
        double fast_smooth_delay{0.6};

        /// 快通道后"快速平滑"用的迭代数（2026-11-14）。
        ///
        /// 常规 `curve_iterations`（60）实测 0.9~1.1s；快速版压到 ~0.4s，
        /// 让曲线尽快替代折线，后续常规重规划再精修。
        /// 0 = 关闭（快速平滑也用完整迭代）。
        int curve_iterations_quick{35};

        /// 优先用 Hermite 平滑轨迹（2026-11-14，默认 true）。
        ///
        /// true  → 起点速度作为完整矢量约束，能"边走边转弯"，不停顿（对齐 SUPER）
        /// false → 退回旧行为（急转时先刹停再转，会卡顿 0.34~1.0s）
        bool hermite_preferred{true};

        /// 急转前先刹车的角度阈值 [deg]（2026-11-14）。
        ///
        /// 路径首段与当前速度夹角超过它 → 先插一段刹车（减速到静止）再转向。
        /// 实测 70.8° 的急转在 1 m/s 下会让 z 掉 0.27m。
        /// 设 180 = 关闭（永不刹车）。
        double brake_angle_deg{55.0};
        /// 速度低于此值不做急转判断 [m/s]
        double brake_angle_min_speed{0.3};

        /// 规划失败时生成可控刹车轨迹（2026-11-14，学 SUPER backup）。
        ///
        /// 关闭 = 退回"停发参考让控制器超时急刹"的旧行为（会抖）。
        bool backup_enable{true};
        /// 低于这个速度不生成刹车段 [m/s]（省得原地打转）
        double backup_min_speed{0.15};

        /// 贴地起点的最大抬升量 [m]（2026-11-14）。
        ///
        /// 飞机趴在地面时，起点 ESDF 恒为 0（地面点云 + 机体自反射），
        /// 规划器的 `start_clr > 0` 判据会一直失败 → 规划器瘫痪。
        /// 逐步抬升起点直到 clearance > 0，最多抬这么多。
        /// 抬到上限还没间隙 → 说明真的被障碍包住 → 拒绝规划（正确）。
        double ground_lift_max{0.40};

        /// anchor 过期补偿上限 [m]（2026-11-14）。
        ///
        /// 规划耗时超出预期时，沿原飞行方向前移 anchor，避免
        /// "anchor 落后飞机 0.8m → C1 过渡段形状急变 → 曲线 kmax 爆炸被拒
        /// → 退回不居中的折线"。0 = 关闭。
        double plan_anchor_catchup_max{0.5};

        /// 折线回退时的居中修正（2026-11-14）。
        ///
        /// 曲线被拒 / fast path 时退回折线，而折线原本**完全没有居中**。
        /// 开这个开关后，折线也会做 ESDF 梯度投影，保证不贴边。
        bool fallback_center_enable{true};
        /// 已经这么宽就不做居中（避免空旷处乱晃）[m]
        double fallback_center_skip_clear{1.2};

        /// 加速度前馈的钳制上限 [m/s²]（2026-11-16）。
        ///
        /// 控制器把 `planner_cmd_.acceleration` 当前馈直接加到速度指令上。
        /// 实测轨迹切换瞬间该字段可达 13.4 ~ 402 m/s²（max_acc 的 9~268 倍），
        /// 会让 sp 剧烈翻转 → 姿态被拽偏 → 掉高。
        /// 取 1.5 = max_acc，即"最多用到飞机的全部加速度能力"。
        /// 0 = 关闭。
        double acc_feedforward_limit{1.5};

        /// 参考速度的变化率上限 [m/s²]（2026-11-16）。
        ///
        /// 发布前对 `cmd.velocity` 做速率限制，防止轨迹切换瞬间
        /// 参考速度方向突变（实测达 14 m/s²，是 max_acc 的 9 倍）。
        /// 0 = 关闭。
        /// 取 max_acc × 1.5 = 2.25，给正常加速段留余量。
        double ref_vel_rate_limit{2.25};

        /// realign 复用旧路径的门槛 [m]（2026-11-16）。
        ///
        /// 旧路径离飞机超过这个距离就不 realign，改走完整 A* 规划。
        /// 否则 buildTrajectory 会插一个这么大的 C1 过渡段，
        /// 在飞机低速时形状退化 → 掉高。
        /// 0.25m ≈ 2 个栅格，足够容纳正常的跟踪误差。
        double realign_max_gap{0.25};

        /// ★ 是否用"旧轨迹未来时刻"作为 anchor（2026-11-16）。
        ///
        /// false（默认）：从**飞机当前位置**重规划 —— 位置+速度都取自飞机，
        ///   按规划耗时外推。轨迹生效时参考点就在飞机身上，无断层。
        ///   实测：照搬 SUPER 时断层 0.91~1.51m，飞机追断层 → 抖动。
        ///
        /// true：SUPER 式（旧轨迹未来 pos+vel 整组）。**只在飞机紧跟旧轨迹时
        ///   才安全**（dev < anchor_max_dev）。适合规划耗时 <100ms 的场景。
        bool anchor_from_trajectory{false};

        /// anchor_from_trajectory=true 时，允许的"旧轨迹前视点 vs 飞机"距离 [m]。
        /// 超过就用飞机自己的状态（说明飞机没在跟踪旧轨迹）。
        double anchor_max_dev{0.30};

        /// 新轨迹起点允许偏离"已提交参考"的最大距离 [m]（2026-11-14）。
        ///
        /// 新轨迹优先从**旧轨迹在 t_now 的状态**起步（保证参考速度连续）。
        /// 若参考离飞机太远，说明飞机已严重跟不上，此时改用实际位置。
        double ref_start_max_dev{1.5};

        /// 曲率超限的容差倍数（2026-11-14）。
        ///
        /// kmax 在 (max_curvature, max_curvature×tol] 之间时**保留曲线**，
        /// 让速度规划降速；超过才退回折线。见 ② 曲率校验的说明。
        /// 1.0 = 退回旧行为（严格按 max_curvature 硬拒绝）。
        double curvature_tolerance{3.0};
        // 到点后是否发 COMPLETED。默认 false = 只发 READY（对齐 SUPER）。
        // 见 publishCompleted() 里的详细说明 —— COMPLETED 会让控制器切到
        // 位置控制分支，与 yaw 转动争夺电机行程，实测导致掉高。
        bool publish_completed_flag{false};

        // ── 目标点吸附（2026-11-09，学 SUPER 的做法）──
        // 打的点落在膨胀层里时，自动吸附到膨胀层外的最近安全点，
        // 而不是硬冲进障碍。对应 SUPER 的
        // `getNearestInfCellNot(OCCUPIED, click_point, goal, 3.0)`。
        bool snap_goal_out_of_inflation{true};
        // 吸附的搜索半径 [m]。SUPER 用 3.0；我们默认 4.0 略大，
        // 因为柱阵里柱子净距约 1.2m，用户点偏一点可能要跨过一根柱子。
        double goal_snap_search_radius{4.0};
        // 吸附后要求的**最小安全余量** [m]。这是 SUPER 没有的额外约束：
        // SUPER 只保证出了膨胀层（clearance >= robot_radius），
        // 本规划器理念是安全优先，终点也应当留余量。
        // 0 = 只要出膨胀层即可（与 SUPER 完全一致）。
        double min_goal_clearance{0.40};
    };

    /// yaw 角速度的安全钳制。
    ///
    /// ★ 2026-11-09 实测事故：yaml 里 `yaw_rate_max: 3.0`，
    ///   而 PX4 的 `MPC_YAWRAUTO_MAX = 45 deg/s = 0.785 rad/s`。
    ///   指令角速度是 PX4 能力的 **3.8 倍** → 飞机永远追不上 →
    ///   实测稳态误差 **103°**（cmd=-2.66, act=+1.83）→
    ///   PX4 持续饱和输出偏航力矩 → 电机行程被吃光 → 推力不足 → 掉高。
    ///
    /// 用机制保证而不是靠人记住：这里读 PX4 的上限做钳制。
    /// 拿不到参数时退回保守值 0.6（0.785 的 76%，留控制余量）。
    void clampYawRateMax() {
        // ★ 必须用**实际** PX4 参数做钳制，不能硬编码 45 deg/s。
        //
        // 2026-11-09 踩过的坑：第一版硬编码 `45 deg/s`（PX4 出厂默认），
        // 后来把 PX4 的 MPC_YAWRAUTO_MAX 调到 120 deg/s 想让 yaw 快点，
        // 结果这个钳制又把 azj 的 1.5 rad/s 钳回 0.596 —— 改 PX4 完全无效。
        //
        // 现在三级来源，优先级从高到低：
        //   ① 参数 `azj/px4_yaw_auto_max_deg` —— 显式指定（最可靠，推荐）
        //   ② 从 mavros 读实际值 —— 自动，但依赖服务可用
        //   ③ 兜底 45 deg/s —— 出厂默认
        // ★ 优先级：**mavros 实际值 > 配置值 > 兜底**（2026-11-09 修正）
        //
        // 之前的顺序反了（配置优先），导致一个隐蔽的失效链：
        //   PX4 SITL 重启 → MPC_YAWRAUTO_MAX 恢复出厂 45 deg/s
        //   但配置里写的是 180 → 钳制按 180 算 → 不触发
        //   → azj 仍按 yaw_rate_max=1.0 发指令
        //   → 实际 1.0/0.785 = 127% 饱和 → 掉高
        //   → 表现成"参数都调了还是掉高"，极难排查。
        //
        // 现在以**实际读到的值**为准（那是唯一真相），配置只作兜底。
        // ★★ 从 PX4 参数文件读，**不再调 mavros**（2026-11-14）
        //
        // 为什么：mavros 的 param/get 只查本地缓存。PX4 参数异步上传，
        // 启动时缓存还没同步完 → 调用会返回 success: False，而
        // mavros 自己会打一条：
        //     ERROR /iris_0/mavros ... PR: Unknown parameter to get: MPC_YAWRAUTO_MAX
        // 这条 ERROR 是 mavros 内部打的（PR = param request），
        // 我们无论怎么重试都消不掉 —— 只要调一次就可能出现。
        //
        // 而真正生效的位置是 PX4 的 px4-rc.params（rcS:228 source，启动即生效）。
        // 直接读文件：零噪音、不依赖 mavros、读到的是"事实"。
        double px4_deg = 0.0;
        {
            const char* kPath =
                "/home/spadea/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/"
                "px4-rc.params";
            std::ifstream f(kPath);
            if (f) {
                std::string line;
                while (std::getline(f, line)) {
                    if (line.find("param set MPC_YAWRAUTO_MAX") == std::string::npos)
                        continue;
                    std::istringstream iss(line);
                    std::string tok;
                    bool seen = false;
                    while (iss >> tok) {
                        if (tok == "MPC_YAWRAUTO_MAX") { seen = true; continue; }
                        if (!seen) continue;
                        try { px4_deg = std::stod(tok); } catch (...) {}
                        break;
                    }
                }
            }
        }
        if (!(px4_deg > 1.0)) {
            // 读不到 → 用配置值；配置也没给 → 出厂默认 45
            if (np_.px4_yaw_auto_max_deg > 1.0) {
                px4_deg = np_.px4_yaw_auto_max_deg;
                ROS_WARN("[azj] could not read PX4 MPC_YAWRAUTO_MAX from mavros; "
                         "using configured %.0f deg/s. If PX4 was reset to factory 45, "
                         "yaw will saturate -> altitude drop.", px4_deg);
            } else {
                px4_deg = 45.0;
                ROS_WARN("[azj] could not read PX4 MPC_YAWRAUTO_MAX; assuming factory 45 deg/s");
            }
        } else if (np_.px4_yaw_auto_max_deg > 1.0 &&
                   std::abs(px4_deg - np_.px4_yaw_auto_max_deg) > 5.0) {
            // 实际值和配置不符 —— 这是最容易踩的坑，必须醒目
            ROS_WARN("[azj] MISMATCH: PX4 actually reports MPC_YAWRAUTO_MAX=%.0f deg/s "
                     "but config says %.0f. Using the ACTUAL value %.0f (mavros is ground truth). "
                     "Fix: run sh/set_px4_yaw.sh %.0f",
                     px4_deg, np_.px4_yaw_auto_max_deg, px4_deg, np_.px4_yaw_auto_max_deg);
        }

        const double px4_limit = px4_deg * M_PI / 180.0;

        // 只用掉 PX4 能力的 76%：给姿态环留控制余量。
        // 用满 100% 时，任何跟踪误差都会打出饱和力矩 → 电机行程被吃光 → 掉高。
        const double safe = 0.76 * px4_limit;

        if (np_.yaw_rate_max > safe) {
            ROS_WARN("[azj] yaw_rate_max=%.2f rad/s exceeds safe fraction of PX4 capability "
                     "(%.0f deg/s * 0.76 = %.2f rad/s) - clamped. Asking for more makes the "
                     "aircraft chase forever -> steady-state yaw error -> thrust loss -> altitude drop.",
                     np_.yaw_rate_max, px4_deg, safe);
            np_.yaw_rate_max = safe;
        }
        ROS_INFO("[azj] yaw_rate_max = %.2f rad/s (PX4 limit %.0f deg/s = %.2f rad/s)",
                 np_.yaw_rate_max, px4_deg, px4_limit);

        // 飞行中要转 yaw，但 PX4 能力不够 → 必然饱和 → 掉高。
        // 这不是"参数没调好"，是会掉机的配置，必须 rror 级别提示。
        if (np_.yaw_mode == YAW_PATH_TANGENT) {
            const double headroom = px4_limit > 1e-6 ? np_.yaw_rate_max / px4_limit : 9.9;
            if (headroom > 0.85) {
                ROS_ERROR("[azj] yaw_mode=%d enables in-flight yaw rotation, but "
                          "yaw_rate_max=%.2f is %.0f%% of PX4 capability (%.2f rad/s). "
                          "PX4 will saturate -> continuous yaw torque -> thrust loss -> "
                          "ALTITUDE DROP. Either run sh/set_px4_yaw.sh to raise the PX4 "
                          "limit, or set yaw_mode=1 (hold heading, never rotate).",
                          np_.yaw_mode, np_.yaw_rate_max, 100.0 * headroom, px4_limit);
            }
        }
    }

    /// 悬停推力基准自检（2026-11-14）。
    ///
    /// ★ 为什么必须在**规划器**里检查，而不是只靠 sh/set_px4_thrust.sh：
    ///
    /// 实测 2026-11-14 多次飞行：用户手动 `roslaunch px4 indoor3.launch` 起 gazebo，
    /// 没走 raicom_azj.sh，于是 `set_px4_thrust.sh` 从没执行过 ——
    /// MPC_THR_HOVER 保持出厂 0.5，而本机型实际需要 **0.871**。
    ///
    /// 后果（/tmp/azj_flight_20261114_154400，rel_t<8 就掉高）：
    ///   PX4 输出推力比"悬停"还低 → 飞机下坠 → 姿态乱 → 规划器跟着乱。
    ///   用户会误以为是"规划器切换轨迹掉高"，实际是推力基准没设。
    ///
    /// 这个检查把"靠人记得跑脚本"变成"节点自己报警"（机制保证）。
    void checkHoverThrust() {
        // 检查的是**真正的生效源**：PX4 SITL 的 px4-rc.params（2026-11-14）。
        //
        // 为什么不用 mavros 的 param/get：
        //   ① 它只查本地缓存，PX4 参数异步上传，启动瞬间必然返回
        //      `PR: Unknown parameter to get: MPC_THR_HOVER` —— 刷屏噪音，
        //      而且导致 sh/set_px4_thrust.sh "一次都没成功过"。
        //   ② 真正决定起飞推力的就是 PX4 启动时加载的那个值，
        //      直接读文件才是检查"事实"而不是检查"能不能查到"。
        //
        // 完全不碰 ROS 服务 → 零噪音、零阻塞、不依赖 mavros 就绪。
        const char* kPath =
            "/home/spadea/PX4_Firmware/build/px4_sitl_default/etc/init.d-posix/"
            "px4-rc.params";
        std::ifstream f(kPath);
        double thr = -1.0;
        if (f) {
            std::string line;
            while (std::getline(f, line)) {
                // 用 token 解析而不是固定偏移：`param set MPC_THR_HOVER 0.871`
                std::istringstream iss(line);
                std::string tok;
                bool seen_key = false;
                while (iss >> tok) {
                    if (tok == "MPC_THR_HOVER") { seen_key = true; continue; }
                    if (!seen_key) continue;
                    try { thr = std::stod(tok); } catch (...) {}
                    break;
                }
            }
        }
        if (thr < 0.0) {
            ROS_WARN("[azj] MPC_THR_HOVER not found in px4-rc.params (%s). "
                     "PY4 default is 0.5 but this model needs ~0.871 -> "
                     "altitude drop is likely. Add: param set MPC_THR_HOVER 0.871",
                     kPath);
            return;
        }
        const double needed = 0.871;
        if (thr < needed - 0.10) {
            ROS_ERROR("[azj] * MPC_THR_HOVER = %.3f in px4-rc.params but this model "
                      "needs ~%.3f * PX4 under-thrusts -> ALTITUDE DROP (trajectory "
                      "switching will look like the culprit). Fix px4-rc.params.",
                      thr, needed);
        } else {
            ROS_INFO("[azj] MPC_THR_HOVER = %.3f (from px4-rc.params, needs ~%.3f) - ok",
                     thr, needed);
        }
    }

    /// 参数来源三级：yaml 的 azj 段打底 → 参数服务器覆盖 → 代码默认值。
    ///
    /// 为什么必须自己读 yaml：`rosrun` 不会执行 launch 里的 rosparam load，
    /// 只靠参数服务器的话 azj 段会静默失效（用默认值跑，且无任何提示）。
    void loadParams(const std::string& cfg_path) {
        try {
            const YAML::Node root = YAML::LoadFile(cfg_path);
            const YAML::Node a = root["azj"];
            if (a) {
                auto dbl = [&](const char* k, double& v) {
                    if (a[k]) v = a[k].as<double>();
                };
                auto i32 = [&](const char* k, int& v) {
                    if (a[k]) v = a[k].as<int>();
                };
                auto str = [&](const char* k, std::string& v) {
                    if (a[k]) v = a[k].as<std::string>();
                };
                auto bln = [&](const char* k, bool& v) {
                    if (a[k]) v = a[k].as<bool>();
                };

                dbl("replan_rate", np_.replan_rate);
                dbl("publish_rate", np_.publish_rate);
                str("goal_topic", np_.goal_topic);
                str("path_topic", np_.path_topic);
                str("marker_topic", np_.marker_topic);
                str("cmd_topic", np_.cmd_topic);
                str("vel_topic", np_.vel_topic);
                str("frame_id", np_.frame_id);
                bln("unknown_as_free", np_.unknown_as_free);
                dbl("free_clearance_if_unknown", np_.free_clearance_if_unknown);
                dbl("unknown_clearance_cap", np_.unknown_clearance_cap);
                bln("enforce_height_limits", np_.enforce_height_limits);
                dbl("floor_z", np_.floor_z);
                dbl("ceil_z", np_.ceil_z);
                dbl("w_height", np_.w_height);
                dbl("height_ref_offset", np_.height_ref_offset);
                dbl("height_scale", np_.height_scale);
                dbl("height_band", np_.height_band);
                dbl("w_height_band", np_.w_height_band);
                dbl("z_band", np_.z_band);
                dbl("max_curvature", np_.max_curvature);
                dbl("max_vel_z_out", np_.max_vel_z_out);
                dbl("vel_blend_time", np_.vel_blend_time);
                dbl("goal_yaw", np_.goal_yaw);
                i32("yaw_mode", np_.yaw_mode);
                bln("yaw_follow_velocity", np_.yaw_follow_velocity);
                dbl("yaw_vel_thresh", np_.yaw_vel_thresh);
                dbl("yaw_deadband", np_.yaw_deadband);
                dbl("yaw_rate_max", np_.yaw_rate_max);
                dbl("yaw_rate_flight", np_.yaw_rate_flight);
                dbl("yaw_alt_guard_drop", np_.yaw_alt_guard_drop);
                dbl("yaw_profile_dt", np_.yaw_profile_dt);
                dbl("px4_yaw_auto_max_deg", np_.px4_yaw_auto_max_deg);
                dbl("goal_z_override", np_.goal_z_override);
                dbl("min_goal_z", np_.min_goal_z);
                dbl("arrive_tol", np_.arrive_tol);
                dbl("arrive_vel_tol", np_.arrive_vel_tol);
                dbl("replan_lead", np_.replan_lead);
                dbl("replan_deviation", np_.replan_deviation);
                dbl("safety_lookahead", np_.safety_lookahead);
                dbl("safety_near_frac", np_.safety_near_frac);
                dbl("safety_far_frac", np_.safety_far_frac);
                dbl("safety_near_span", np_.safety_near_span);
                dbl("safety_hysteresis", np_.safety_hysteresis);
                dbl("hot_init_reuse", np_.hot_init_reuse);
                dbl("hot_init_min_len", np_.hot_init_min_len);
                dbl("hot_init_max_len", np_.hot_init_max_len);
                dbl("hot_init_clear_frac", np_.hot_init_clear_frac);
                i32("traj_backend", np_.traj_backend);
                dbl("minco_w_mid", np_.minco_w_mid);
                dbl("minco_w_obs", np_.minco_w_obs);
                dbl("minco_w_smooth", np_.minco_w_smooth);
                i32("minco_iterations", np_.minco_iterations);
                bln("rolling_enable", np_.rolling_enable);
                dbl("rolling_forward_dt", np_.rolling_forward_dt);
                dbl("rolling_lookahead", np_.rolling_lookahead);
                dbl("rolling_lookahead_min", np_.rolling_lookahead_min);
                dbl("rolling_global_ttl", np_.rolling_global_ttl);
                dbl("rolling_goal_refresh_dist", np_.rolling_goal_refresh_dist);
                dbl("rolling_center_max_shift", np_.rolling_center_max_shift);
                dbl("rolling_period", np_.rolling_period);
                dbl("rolling_track_acc", np_.rolling_track_acc);
                dbl("rolling_v_lead", np_.rolling_v_lead);
                dbl("rolling_astar_margin", np_.rolling_astar_margin);
                str("minco_dump_dir", np_.minco_dump_dir);
                dbl("rolling_receding_dis", np_.rolling_receding_dis);
                dbl("traj_expired_min_vel", np_.traj_expired_min_vel);
                i32("minco_max_reject_streak", np_.minco_max_reject_streak);
                dbl("rolling_lag_free", np_.rolling_lag_free);
                dbl("rolling_lag_gain", np_.rolling_lag_gain);
                dbl("rolling_lag_vel_gain", np_.rolling_lag_vel_gain);
                dbl("rolling_v_floor", np_.rolling_v_floor);
                dbl("min_replan_period", np_.min_replan_period);
                dbl("blocked_replan_period", np_.blocked_replan_period);
                bln("blocked_fast_path", np_.blocked_fast_path);
                dbl("fast_smooth_delay", np_.fast_smooth_delay);
                dbl("curvature_tolerance", np_.curvature_tolerance);
                dbl("ref_start_max_dev", np_.ref_start_max_dev);
                dbl("ground_lift_max", np_.ground_lift_max);
                dbl("plan_anchor_catchup_max", np_.plan_anchor_catchup_max);
                dbl("acc_feedforward_limit", np_.acc_feedforward_limit);
                dbl("ref_vel_rate_limit", np_.ref_vel_rate_limit);
                dbl("realign_max_gap", np_.realign_max_gap);
                bln("anchor_from_trajectory", np_.anchor_from_trajectory);
                dbl("anchor_max_dev", np_.anchor_max_dev);
                bln("fallback_center_enable", np_.fallback_center_enable);
                dbl("fallback_center_skip_clear", np_.fallback_center_skip_clear);
                bln("backup_enable", np_.backup_enable);
                dbl("backup_min_speed", np_.backup_min_speed);
                bln("hermite_preferred", np_.hermite_preferred);
                dbl("brake_angle_deg", np_.brake_angle_deg);
                dbl("brake_angle_min_speed", np_.brake_angle_min_speed);
                i32("curve_iterations_quick", np_.curve_iterations_quick);
                bln("publish_completed_flag", np_.publish_completed_flag);
                bln("snap_goal_out_of_inflation", np_.snap_goal_out_of_inflation);
                dbl("goal_snap_search_radius", np_.goal_snap_search_radius);
                dbl("min_goal_clearance", np_.min_goal_clearance);

                dbl("resolution", sp_.resolution);
                dbl("robot_radius", sp_.robot_radius);
                dbl("escape_radius", sp_.escape_radius);
                dbl("clearance_weight", sp_.clearance_weight);
                dbl("clearance_ref", sp_.clearance_ref);
                dbl("clearance_cap", sp_.clearance_cap);
                dbl("alpha", sp_.alpha);
                dbl("box_margin_x", sp_.box_margin.x());
                dbl("box_margin_y", sp_.box_margin.y());
                dbl("box_margin_z", sp_.box_margin.z());
                dbl("goal_tolerance", sp_.goal_tolerance);
                i32("max_expansions", sp_.max_expansions);

                // 曲线优化
                dbl("curve_sample_step", np_.curve_sample_step);
                i32("curve_samples", cp_.samples);
                i32("curve_iterations", cp_.iterations);
                i32("curve_stall_iters", cp_.stall_iters);
                dbl("curve_w_mid", cp_.w_mid);
                dbl("curve_w_obs", cp_.w_obs);
                dbl("curve_w_smooth", cp_.w_smooth);
                dbl("curve_w_anchor", cp_.w_anchor);
                dbl("curve_w_len", cp_.w_len);
                dbl("curve_control_point_spacing", cp_.control_point_spacing);
                bln("curve_enable_projection", cp_.enable_projection);
                i32("curve_projection_iters", cp_.projection_iters);
                i32("curve_projection_samples", cp_.projection_samples);
                bln("curve_enable_local_centering", cp_.enable_local_centering);
                dbl("curve_local_scan_radius", cp_.local_scan_radius);
                bln("curve_enable", np_.curve_enable);
                bln("plan_2d", np_.plan_2d);
                dbl("plan_2d_z_half", np_.plan_2d_z_half);
                i32("plan_2d_z_samples", np_.plan_2d_z_samples);
                dbl("plan_2d_z_extra", np_.plan_2d_z_extra);
                dbl("plan_2d_slab_half", np_.plan_2d_slab_half);

                dbl("max_vel", tp_.max_vel);
                dbl("max_vel_z", tp_.max_vel_z);
                dbl("max_acc", tp_.max_acc);
                dbl("max_acc_z", tp_.max_acc_z);
                dbl("max_lat_acc", tp_.max_lat_acc);
            } else {
                ROS_WARN("[azj] yaml has no 'azj' section, using code defaults");
            }
        } catch (const std::exception& e) {
            ROS_WARN("[azj] failed to read yaml (%s), using code defaults", e.what());
        }

        // 参数服务器覆盖（支持 rosrun _azj_robot_radius:=0.3 这类临时改参）
        pnh_.param("azj/replan_rate", np_.replan_rate, np_.replan_rate);
        pnh_.param("azj/publish_rate", np_.publish_rate, np_.publish_rate);
        pnh_.param("azj/goal_topic", np_.goal_topic, np_.goal_topic);
        pnh_.param("azj/path_topic", np_.path_topic, np_.path_topic);
        pnh_.param("azj/marker_topic", np_.marker_topic, np_.marker_topic);
        pnh_.param("azj/cmd_topic", np_.cmd_topic, np_.cmd_topic);
        pnh_.param("azj/vel_topic", np_.vel_topic, np_.vel_topic);
        pnh_.param("azj/frame_id", np_.frame_id, np_.frame_id);
        pnh_.param("azj/unknown_as_free", np_.unknown_as_free, np_.unknown_as_free);
        pnh_.param("azj/free_clearance_if_unknown", np_.free_clearance_if_unknown,
                   np_.free_clearance_if_unknown);
        pnh_.param("azj/unknown_clearance_cap", np_.unknown_clearance_cap,
                   np_.unknown_clearance_cap);
        pnh_.param("azj/enforce_height_limits", np_.enforce_height_limits,
                   np_.enforce_height_limits);
        pnh_.param("azj/floor_z", np_.floor_z, np_.floor_z);
        pnh_.param("azj/ceil_z", np_.ceil_z, np_.ceil_z);
        pnh_.param("azj/w_height", np_.w_height, np_.w_height);
        pnh_.param("azj/height_ref_offset", np_.height_ref_offset,
                   np_.height_ref_offset);
        pnh_.param("azj/height_scale", np_.height_scale, np_.height_scale);
        pnh_.param("azj/height_band", np_.height_band, np_.height_band);
        pnh_.param("azj/w_height_band", np_.w_height_band, np_.w_height_band);
        pnh_.param("azj/z_band", np_.z_band, np_.z_band);
        pnh_.param("azj/max_curvature", np_.max_curvature, np_.max_curvature);
        pnh_.param("azj/max_vel_z_out", np_.max_vel_z_out, np_.max_vel_z_out);
        pnh_.param("azj/vel_blend_time", np_.vel_blend_time, np_.vel_blend_time);
        pnh_.param("azj/goal_yaw", np_.goal_yaw, np_.goal_yaw);
        pnh_.param("azj/yaw_mode", np_.yaw_mode, np_.yaw_mode);
        pnh_.param("azj/yaw_vel_thresh", np_.yaw_vel_thresh, np_.yaw_vel_thresh);
        pnh_.param("azj/yaw_deadband", np_.yaw_deadband, np_.yaw_deadband);
        pnh_.param("azj/yaw_follow_velocity", np_.yaw_follow_velocity,
                   np_.yaw_follow_velocity);
        pnh_.param("azj/yaw_rate_max", np_.yaw_rate_max, np_.yaw_rate_max);
        pnh_.param("azj/yaw_rate_flight", np_.yaw_rate_flight, np_.yaw_rate_flight);
        pnh_.param("azj/yaw_alt_guard_drop", np_.yaw_alt_guard_drop,
                   np_.yaw_alt_guard_drop);
        pnh_.param("azj/yaw_profile_dt", np_.yaw_profile_dt, np_.yaw_profile_dt);
        pnh_.param("azj/px4_yaw_auto_max_deg", np_.px4_yaw_auto_max_deg,
                   np_.px4_yaw_auto_max_deg);
        pnh_.param("azj/goal_z_override", np_.goal_z_override, np_.goal_z_override);
        pnh_.param("azj/min_goal_z", np_.min_goal_z, np_.min_goal_z);
        pnh_.param("azj/arrive_tol", np_.arrive_tol, np_.arrive_tol);
        pnh_.param("azj/arrive_vel_tol", np_.arrive_vel_tol,
                   np_.arrive_vel_tol);
        pnh_.param("azj/replan_lead", np_.replan_lead, np_.replan_lead);
        pnh_.param("azj/replan_deviation", np_.replan_deviation, np_.replan_deviation);
        pnh_.param("azj/safety_lookahead", np_.safety_lookahead, np_.safety_lookahead);
        pnh_.param("azj/safety_near_frac", np_.safety_near_frac, np_.safety_near_frac);
        pnh_.param("azj/safety_far_frac", np_.safety_far_frac, np_.safety_far_frac);
        pnh_.param("azj/safety_near_span", np_.safety_near_span, np_.safety_near_span);
        pnh_.param("azj/safety_hysteresis", np_.safety_hysteresis,
                   np_.safety_hysteresis);
        pnh_.param("azj/hot_init_reuse", np_.hot_init_reuse, np_.hot_init_reuse);
        pnh_.param("azj/hot_init_min_len", np_.hot_init_min_len, np_.hot_init_min_len);
        pnh_.param("azj/hot_init_max_len", np_.hot_init_max_len, np_.hot_init_max_len);
        pnh_.param("azj/hot_init_clear_frac", np_.hot_init_clear_frac,
                   np_.hot_init_clear_frac);
        pnh_.param("azj/traj_backend", np_.traj_backend, np_.traj_backend);
        pnh_.param("azj/minco_w_mid", np_.minco_w_mid, np_.minco_w_mid);
        pnh_.param("azj/minco_w_obs", np_.minco_w_obs, np_.minco_w_obs);
        pnh_.param("azj/minco_w_smooth", np_.minco_w_smooth, np_.minco_w_smooth);
        pnh_.param("azj/minco_iterations", np_.minco_iterations, np_.minco_iterations);
        pnh_.param("azj/rolling_enable", np_.rolling_enable, np_.rolling_enable);
        pnh_.param("azj/rolling_forward_dt", np_.rolling_forward_dt,
                   np_.rolling_forward_dt);
        pnh_.param("azj/rolling_lookahead", np_.rolling_lookahead,
                   np_.rolling_lookahead);
        pnh_.param("azj/rolling_lookahead_min", np_.rolling_lookahead_min,
                   np_.rolling_lookahead_min);
        pnh_.param("azj/rolling_global_ttl", np_.rolling_global_ttl,
                   np_.rolling_global_ttl);
        pnh_.param("azj/rolling_goal_refresh_dist", np_.rolling_goal_refresh_dist,
                   np_.rolling_goal_refresh_dist);
        pnh_.param("azj/rolling_center_iters", np_.rolling_center_iters,
                   np_.rolling_center_iters);
        pnh_.param("azj/rolling_center_step", np_.rolling_center_step,
                   np_.rolling_center_step);
        pnh_.param("azj/rolling_center_max_shift", np_.rolling_center_max_shift,
                   np_.rolling_center_max_shift);
        pnh_.param("azj/rolling_period", np_.rolling_period, np_.rolling_period);
        pnh_.param("azj/rolling_track_acc", np_.rolling_track_acc, np_.rolling_track_acc);
        pnh_.param("azj/rolling_v_lead", np_.rolling_v_lead, np_.rolling_v_lead);
        pnh_.param("azj/rolling_astar_margin", np_.rolling_astar_margin,
                   np_.rolling_astar_margin);
        pnh_.param("azj/minco_dump_dir", np_.minco_dump_dir, np_.minco_dump_dir);
        pnh_.param("azj/rolling_receding_dis", np_.rolling_receding_dis,
                   np_.rolling_receding_dis);
        pnh_.param("azj/traj_expired_min_vel", np_.traj_expired_min_vel,
                   np_.traj_expired_min_vel);
        pnh_.param("azj/minco_max_reject_streak", np_.minco_max_reject_streak,
                   np_.minco_max_reject_streak);
        pnh_.param("azj/rolling_lag_free", np_.rolling_lag_free, np_.rolling_lag_free);
        pnh_.param("azj/rolling_lag_gain", np_.rolling_lag_gain, np_.rolling_lag_gain);
        pnh_.param("azj/rolling_lag_vel_gain", np_.rolling_lag_vel_gain,
                   np_.rolling_lag_vel_gain);
        pnh_.param("azj/rolling_v_floor", np_.rolling_v_floor, np_.rolling_v_floor);
        pnh_.param("azj/rolling_center_lateral_only", np_.rolling_center_lateral_only,
                   np_.rolling_center_lateral_only);
        pnh_.param("azj/rolling_patch_len", np_.rolling_patch_len, np_.rolling_patch_len);
        pnh_.param("azj/rolling_snap_end", np_.rolling_snap_end, np_.rolling_snap_end);
        pnh_.param("azj/min_replan_period", np_.min_replan_period,
                   np_.min_replan_period);
        pnh_.param("azj/blocked_replan_period", np_.blocked_replan_period,
                   np_.blocked_replan_period);
        pnh_.param("azj/blocked_fast_path", np_.blocked_fast_path,
                   np_.blocked_fast_path);
        pnh_.param("azj/fast_smooth_delay", np_.fast_smooth_delay,
                   np_.fast_smooth_delay);
        pnh_.param("azj/curvature_tolerance", np_.curvature_tolerance,
                   np_.curvature_tolerance);
        pnh_.param("azj/ref_start_max_dev", np_.ref_start_max_dev,
                   np_.ref_start_max_dev);
        pnh_.param("azj/ground_lift_max", np_.ground_lift_max, np_.ground_lift_max);
        pnh_.param("azj/plan_anchor_catchup_max", np_.plan_anchor_catchup_max,
                   np_.plan_anchor_catchup_max);
        pnh_.param("azj/acc_feedforward_limit", np_.acc_feedforward_limit,
                   np_.acc_feedforward_limit);
        pnh_.param("azj/ref_vel_rate_limit", np_.ref_vel_rate_limit,
                   np_.ref_vel_rate_limit);
        pnh_.param("azj/realign_max_gap", np_.realign_max_gap, np_.realign_max_gap);
        pnh_.param("azj/anchor_from_trajectory", np_.anchor_from_trajectory,
                   np_.anchor_from_trajectory);
        pnh_.param("azj/anchor_max_dev", np_.anchor_max_dev, np_.anchor_max_dev);
        pnh_.param("azj/fallback_center_enable", np_.fallback_center_enable,
                   np_.fallback_center_enable);
        pnh_.param("azj/fallback_center_skip_clear", np_.fallback_center_skip_clear,
                   np_.fallback_center_skip_clear);
        pnh_.param("azj/backup_enable", np_.backup_enable, np_.backup_enable);
        pnh_.param("azj/backup_min_speed", np_.backup_min_speed, np_.backup_min_speed);
        pnh_.param("azj/hermite_preferred", np_.hermite_preferred, np_.hermite_preferred);
        pnh_.param("azj/brake_angle_deg", np_.brake_angle_deg, np_.brake_angle_deg);
        pnh_.param("azj/brake_angle_min_speed", np_.brake_angle_min_speed,
                   np_.brake_angle_min_speed);
        pnh_.param("azj/curve_iterations_quick", np_.curve_iterations_quick,
                   np_.curve_iterations_quick);
        pnh_.param("azj/publish_completed_flag", np_.publish_completed_flag,
                   np_.publish_completed_flag);
        pnh_.param("azj/snap_goal_out_of_inflation", np_.snap_goal_out_of_inflation,
                   np_.snap_goal_out_of_inflation);
        pnh_.param("azj/goal_snap_search_radius", np_.goal_snap_search_radius,
                   np_.goal_snap_search_radius);
        pnh_.param("azj/min_goal_clearance", np_.min_goal_clearance,
                   np_.min_goal_clearance);

        pnh_.param("azj/resolution", sp_.resolution, sp_.resolution);
        pnh_.param("azj/robot_radius", sp_.robot_radius, sp_.robot_radius);
        pnh_.param("azj/escape_radius", sp_.escape_radius, sp_.escape_radius);
        pnh_.param("azj/clearance_weight", sp_.clearance_weight, sp_.clearance_weight);
        pnh_.param("azj/clearance_ref", sp_.clearance_ref, sp_.clearance_ref);
        pnh_.param("azj/clearance_cap", sp_.clearance_cap, sp_.clearance_cap);
        pnh_.param("azj/alpha", sp_.alpha, sp_.alpha);
        pnh_.param("azj/box_margin_x", sp_.box_margin.x(), sp_.box_margin.x());
        pnh_.param("azj/box_margin_y", sp_.box_margin.y(), sp_.box_margin.y());
        pnh_.param("azj/box_margin_z", sp_.box_margin.z(), sp_.box_margin.z());
        pnh_.param("azj/goal_tolerance", sp_.goal_tolerance, sp_.goal_tolerance);
        pnh_.param("azj/max_expansions", sp_.max_expansions, sp_.max_expansions);

        pnh_.param("azj/plan_2d", np_.plan_2d, np_.plan_2d);
        pnh_.param("azj/plan_2d_z_half", np_.plan_2d_z_half, np_.plan_2d_z_half);
        pnh_.param("azj/plan_2d_z_samples", np_.plan_2d_z_samples, np_.plan_2d_z_samples);
        pnh_.param("azj/plan_2d_z_extra", np_.plan_2d_z_extra, np_.plan_2d_z_extra);
        pnh_.param("azj/plan_2d_slab_half", np_.plan_2d_slab_half, np_.plan_2d_slab_half);

        pnh_.param("azj/curve_enable", np_.curve_enable, np_.curve_enable);
        pnh_.param("azj/curve_sample_step", np_.curve_sample_step, np_.curve_sample_step);
        pnh_.param("azj/curve_samples", cp_.samples, cp_.samples);
        pnh_.param("azj/curve_iterations", cp_.iterations, cp_.iterations);
        pnh_.param("azj/curve_stall_iters", cp_.stall_iters, cp_.stall_iters);
        pnh_.param("azj/curve_w_mid", cp_.w_mid, cp_.w_mid);
        pnh_.param("azj/curve_w_obs", cp_.w_obs, cp_.w_obs);
        pnh_.param("azj/curve_w_smooth", cp_.w_smooth, cp_.w_smooth);
        pnh_.param("azj/curve_w_anchor", cp_.w_anchor, cp_.w_anchor);
        pnh_.param("azj/curve_w_len", cp_.w_len, cp_.w_len);
        pnh_.param("azj/curve_control_point_spacing", cp_.control_point_spacing,
                   cp_.control_point_spacing);
        pnh_.param("azj/curve_enable_projection", cp_.enable_projection,
                   cp_.enable_projection);
        pnh_.param("azj/curve_projection_iters", cp_.projection_iters,
                   cp_.projection_iters);
        pnh_.param("azj/curve_projection_samples", cp_.projection_samples,
                   cp_.projection_samples);
        pnh_.param("azj/curve_enable_local_centering", cp_.enable_local_centering,
                   cp_.enable_local_centering);
        pnh_.param("azj/curve_local_scan_radius", cp_.local_scan_radius,
                   cp_.local_scan_radius);

        pnh_.param("azj/max_vel", tp_.max_vel, tp_.max_vel);
        pnh_.param("azj/max_vel_z", tp_.max_vel_z, tp_.max_vel_z);
        pnh_.param("azj/max_acc", tp_.max_acc, tp_.max_acc);
        pnh_.param("azj/max_acc_z", tp_.max_acc_z, tp_.max_acc_z);
        pnh_.param("azj/max_lat_acc", tp_.max_lat_acc, tp_.max_lat_acc);

        // 把生效值发布到参数服务器。
        // 为什么必须做：参数是 yaml-cpp 直接读文件的（rosrun 不会 rosparam load），
        // 所以参数服务器上原本**没有**这些键。结果：
        //   ① rosparam set 之后 rqt_reconfigure/用户看不到当前值
        //   ② 运行时切换时 getParam 读不到，只能一直用 yaml 里的值
        // 这里把最终生效值写回去，让参数服务器成为"可观测 + 可覆盖"的入口。
        if (np_.enforce_height_limits) {
            clampHeightLimitsToMap();
        }
        publishParamsToServer();

        if (np_.replan_rate <= 0.0) np_.replan_rate = 5.0;
        if (np_.publish_rate <= 0.0) np_.publish_rate = 50.0;
        if (sp_.resolution <= 0.0) sp_.resolution = 0.10;
    }

    /// 把 rog_map 的 ESDF 适配到 azj 的 ClearanceField。
    ///
    /// 边界保护是必需的：`evaluateEDT` 会读取查询点周围 2×2×2 个格点，
    /// 而底层 `getDistance` 用哈希索引，出界不会报错，只会读到错误内存。
    double clearanceAt(const Eigen::Vector3d& p) {
        // 注意：rog_map::Vec3f 虽然名字带 f，但 decimal_t = double，
        // 它实际是 Eigen::Vector3d。不要按名字误用 float。
        const rog_map::Vec3f q = p;

        // ★ 层高硬约束：地板以下 / 天花板以上视为障碍（不可通行）。
        //
        // 为什么这一条至关重要：地板下面没有点云（雷达看不到），
        // clearance 会走 unknown 策略被判为"可通行"，A* 于是从下面钻过去 ——
        // 但物理上下面是地板。实测就是这个原因导致"打点后路径从地底绕"。
        //
        // 顺序必须在 isUnknown / ESDF 之前：这是**绝对约束**，
        // 不因为"没探测过"就放行。
        if (np_.enforce_height_limits) {
            if (p.z() < np_.floor_z || p.z() > np_.ceil_z) {
                return ClearanceField::kUnknownClearance;   // -1 → 不可通行
            }
        }

        if (!esdf_map_->insideLocalMap(q)) return ClearanceField::kUnknownClearance;

        // ★ ESDF 有效范围检查（根源性修正）。
        //
        // `ESDFMap::updateESDF3D` 每帧只重算以 odom 为中心、半径
        // `esdf/local_update_box/2` 的那个盒子；盒外的 `distance_buffer`
        // 从未被写过，而 `std::vector<double>::resize` 填的是 **0**。
        //
        // 后果（实测）：目标点 10m 外时 `evaluateEDT` 三线性插值读到一片 0，
        // clearance 变成 0，于是：
        //   · 日志报 `goal clearance=0m`，看着像“目标贴障碍”，其实不贴；
        //   · A* 认为远处圆盘全是障碍 → `plan failed: fully blocked`，
        //     或被迫在近处绕，路径变形。
        // 这是**未初始化内存**，不是真实测距，绝不能当 0 用。
        //
        // 盒外的正确处理：**不可通行**，不是乐观值。
        //
        // 曾经在这里返回 `unknown_clearance_cap`（0.32，略大于 robot_radius）：
        // 本意是“未知但大概率空旷，允许探索”。但实测发现这是个陷阱：
        //   · 15m 目标超出半径 12m 的 ESDF 盒，盒外一片 0.32；
        //   · A* / 曲线优化看到“到处都有 0.32 的缝”，就把路径往盒外带；
        //   · 而盒外根本没建图 —— 真实柱子就在那里（日志里的
        //     `worst vertex at 12.11 ... c=0.32` 正是盒边界外的假值），
        //     结果曲线穿柱，又因为用同一个假值做安全校验而“看不见”。
        //
        // 安全第一：没建图的区域不能假装安全。超出 ESDF 有效范围的规划
        // 本身就不可信，宁可报不可达，也不发一条未验证的轨迹。
        {
            rog_map::Vec3f box_min, box_max;
            esdf_map_->getUpdatedBbox(box_min, box_max);
            const double eps = cfg_.esdf_resolution;   // 留一格余量，盖住边界插值
            const bool outside = (q.array() < (box_min.array() - eps)).any() ||
                                 (q.array() > (box_max.array() + eps)).any();
            if (outside) {
                return ClearanceField::kUnknownClearance;   // -1 → 不可通行
            }
        }

        const rog_map::Vec3f m = rog_map::Vec3f::Constant(cfg_.esdf_resolution);
        // Eigen 表达式模板必须先物化：直接传 q+m 会让 insideLocalMap 的两个
        // 重载（Vec3f / Vec3i）都参与匹配，产生歧义。
        const rog_map::Vec3f q_plus = q + m;
        const rog_map::Vec3f q_minus = q - m;
        if (!esdf_map_->insideLocalMap(q_plus) || !esdf_map_->insideLocalMap(q_minus)) {
            return ClearanceField::kUnknownClearance;
        }

        // ⚠ 关键：UNKNOWN ≠ 障碍。
        //   雷达装在机体上，看不到自己所在的那个体素，那里必然是 UNKNOWN。
        //   早期版本在这里直接 return -1（当作不可通行），结果飞机永远
        //   "start is inside an obstacle"，规划器一次都没成功过。
        //   而 rog_map 的 ESDF 只对 isOccupied 的格子算距离，unknown 位置
        //   evaluateEDT 会正常返回"到最近真障碍的距离"—— 这个值本来就可用。
        // 先取真实 ESDF。**UNKNOWN 格子的 ESDF 也是可信的**：
        //   evaluateEDT 算的是"到最近**已知**障碍格的距离"，
        //   格子本身是否被 raycast 更新过不影响这个距离的可靠性。
        double d = 0.0;
        esdf_map_->evaluateEDT(p, d);

        if (isUnknown(q)) {
            if (np_.unknown_as_free) return np_.free_clearance_if_unknown;

            // ★ 用真实 ESDF，而不是一刀切返回 cap —— 这是关键修正。
            //
            // 曾经的错误：UNKNOWN 一律返回 unknown_clearance_cap(0.32)。
            // 后果（实测 start diag: esdf=0.547 unknown=1 clearance_used=0.320）：
            //   · 空旷但未探测的区域（真实 ESDF 可能 1.5m）被压成 0.32
            //   · 点云密集处（墙边，真实 ESDF 0.5~2.0）保留真实值
            //   → A* 认为墙边比空旷区更宽敞 → **专门往点云密集处走**
            //     （用户报的"路径从点云密集处规划"）
            //   → 且 cap(0.32) > robot_radius(0.30)，所有 UNKNOWN 一律可通行，
            //     robot_radius 和膨胀层"看不出效果"。
            //
            // ★ 不封顶：真实 ESDF 是多少就用多少。
            //
            // 封顶（min(d, cap)）等于没修 —— 空旷区 ESDF=1.5 仍被压成 0.32，
            // 和墙边已知点云区（真实 0.5~1.0）比还是"更窄"，A* 照样往墙边走。
            //
            // 为什么这样可以不封顶：UNKNOWN 只表示"这格没被 raycast 更新"，
            // 而 ESDF 算的是到**最近已知障碍**的距离，这个距离是真实的。
            // "可能藏着未探测障碍"的风险，由 robot_radius 硬约束 +
            // 规划器在未知区不加速（轨迹层）来兜，不该靠伪造小 clearance。
            //
            // 副作用检查：地面附近若有点云（被标 occupied），ESDF 自然小，
            // A* 不会往下钻；若地面真的没有点云，由 z_band 硬约束管。
            // 所以不再需要靠 cap 压着。
            if (d > 1e-6) {
                return d;
            }
            // 一点数据都没有（空地图启动初期 / 出界）：给乐观值保证能起步
            return np_.unknown_clearance_cap;
        }
        return d;
    }

    /// 从 odom 的 twist 取世界系速度（ENU，不要再转机体系）。
    void odomTwistCallback(const nav_msgs::OdometryConstPtr& msg) {
        current_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                       msg->twist.twist.linear.z);
        vel_received_ = true;
    }

    /// 把生效参数写回参数服务器（用 ros::param::set，写在全局 /azj_planner/ 下）。
    /// 注意顺序：必须在 loadParams 末尾调用，写的是"yaml + 参数服务器覆盖"之后的最终值。
    void publishParamsToServer() {
        ros::param::set("/azj_planner/plan_2d", np_.plan_2d);
        ros::param::set("/azj_planner/plan_2d_z_half", np_.plan_2d_z_half);
        ros::param::set("/azj_planner/plan_2d_z_samples", np_.plan_2d_z_samples);
        ros::param::set("/azj_planner/curve_enable", np_.curve_enable);
        ros::param::set("/azj_planner/max_curvature", np_.max_curvature);
        ros::param::set("/azj_planner/blocked_fast_path", np_.blocked_fast_path);
        ros::param::set("/azj_planner/fast_smooth_delay", np_.fast_smooth_delay);
        ros::param::set("/azj_planner/curvature_tolerance", np_.curvature_tolerance);
        ros::param::set("/azj_planner/ref_start_max_dev", np_.ref_start_max_dev);
        ros::param::set("/azj_planner/ground_lift_max", np_.ground_lift_max);
        ros::param::set("/azj_planner/plan_anchor_catchup_max",
                        np_.plan_anchor_catchup_max);
        ros::param::set("/azj_planner/acc_feedforward_limit",
                        np_.acc_feedforward_limit);
        ros::param::set("/azj_planner/ref_vel_rate_limit", np_.ref_vel_rate_limit);
        ros::param::set("/azj_planner/realign_max_gap", np_.realign_max_gap);
        ros::param::set("/azj_planner/anchor_from_trajectory",
                        np_.anchor_from_trajectory);
        ros::param::set("/azj_planner/anchor_max_dev", np_.anchor_max_dev);
        ros::param::set("/azj_planner/fallback_center_enable",
                        np_.fallback_center_enable);
        ros::param::set("/azj_planner/fallback_center_skip_clear",
                        np_.fallback_center_skip_clear);
        ros::param::set("/azj_planner/backup_enable", np_.backup_enable);
        ros::param::set("/azj_planner/backup_min_speed", np_.backup_min_speed);
        ros::param::set("/azj_planner/hermite_preferred", np_.hermite_preferred);
        ros::param::set("/azj_planner/brake_angle_deg", np_.brake_angle_deg);
        ros::param::set("/azj_planner/brake_angle_min_speed", np_.brake_angle_min_speed);
        // BLOCKED 判据 + 热启动（2026-11-14）
        ros::param::set("/azj_planner/safety_lookahead", np_.safety_lookahead);
        ros::param::set("/azj_planner/safety_near_frac", np_.safety_near_frac);
        ros::param::set("/azj_planner/safety_far_frac", np_.safety_far_frac);
        ros::param::set("/azj_planner/safety_near_span", np_.safety_near_span);
        ros::param::set("/azj_planner/safety_hysteresis", np_.safety_hysteresis);
        ros::param::set("/azj_planner/hot_init_reuse", np_.hot_init_reuse);
        ros::param::set("/azj_planner/hot_init_min_len", np_.hot_init_min_len);
        ros::param::set("/azj_planner/hot_init_max_len", np_.hot_init_max_len);
        ros::param::set("/azj_planner/hot_init_clear_frac", np_.hot_init_clear_frac);
        ros::param::set("/azj_planner/curve_iterations_quick",
                        np_.curve_iterations_quick);
        ros::param::set("/azj_planner/curve_w_mid", cp_.w_mid);
        ros::param::set("/azj_planner/curve_w_obs", cp_.w_obs);
        ros::param::set("/azj_planner/curve_w_anchor", cp_.w_anchor);
        ros::param::set("/azj_planner/robot_radius", sp_.robot_radius);
        ros::param::set("/azj_planner/clearance_weight", sp_.clearance_weight);
        ros::param::set("/azj_planner/goal_z_override", np_.goal_z_override);
        ros::param::set("/azj_planner/min_goal_z", np_.min_goal_z);
        ros::param::set("/azj_planner/yaw_mode", np_.yaw_mode);
        ros::param::set("/azj_planner/yaw_vel_thresh", np_.yaw_vel_thresh);
        ros::param::set("/azj_planner/yaw_deadband", np_.yaw_deadband);
        ros::param::set("/azj_planner/yaw_follow_velocity", np_.yaw_follow_velocity);
        ros::param::set("/azj_planner/yaw_rate_max", np_.yaw_rate_max);
        ros::param::set("/azj_planner/yaw_rate_flight", np_.yaw_rate_flight);
        ros::param::set("/azj_planner/yaw_alt_guard_drop", np_.yaw_alt_guard_drop);
        ros::param::set("/azj_planner/yaw_profile_dt", np_.yaw_profile_dt);
        ros::param::set("/azj_planner/px4_yaw_auto_max_deg",
                        np_.px4_yaw_auto_max_deg);
        ros::param::set("/azj_planner/curve_iterations", cp_.iterations);
        ros::param::set("/azj_planner/curve_stall_iters", cp_.stall_iters);
        ros::param::set("/azj_planner/curve_samples", cp_.samples);
        // v1.2 根源性改进参数（便于运行时调参与诊断）
        ros::param::set("/azj_planner/curve_control_point_spacing",
                        cp_.control_point_spacing);
        ros::param::set("/azj_planner/curve_enable_projection", cp_.enable_projection);
        ros::param::set("/azj_planner/curve_projection_iters", cp_.projection_iters);
        ros::param::set("/azj_planner/curve_projection_samples", cp_.projection_samples);
        ros::param::set("/azj_planner/curve_enable_local_centering",
                        cp_.enable_local_centering);
        ros::param::set("/azj_planner/curve_local_scan_radius", cp_.local_scan_radius);

        // 建图概率参数：发布**实际生效值**，供诊断脚本直接判读。
        //
        // 为什么重要：p_miss 若用了 rog_map 的默认 0.7，
        // l_miss = log(0.7/0.3) = +0.847（正数），raycast 的"空闲"格子
        // 反而在增加占据概率 → 整幅地图被填实 → 规划全线失败。
        // 以前只能去日志里 grep，现在参数服务器上直接能看。
        ros::param::set("/azj_planner/raycast_p_miss", cfg_.p_miss);
        ros::param::set("/azj_planner/raycast_l_miss", cfg_.l_miss);
        ros::param::set("/azj_planner/raycast_l_hit", cfg_.l_hit);
        ros::param::set("/azj_planner/inflation_step", cfg_.inflation_step);
        // 地图 z 裁剪边界（rog_map 重算后的**实际生效值**，不是 yaml 里写的）
        ros::param::set("/azj_planner/map_z_min", cfg_.virtual_ground_height);
        ros::param::set("/azj_planner/map_z_max", cfg_.virtual_ceil_height);
        // 规划层高边界
        ros::param::set("/azj_planner/enforce_height_limits", np_.enforce_height_limits);
        ros::param::set("/azj_planner/floor_z", np_.floor_z);
        ros::param::set("/azj_planner/ceil_z", np_.ceil_z);
        ros::param::set("/azj_planner/w_height", np_.w_height);
    }

    /// 曲线被拒（安全/曲率）时统一退回 A* 折线。
    ///
    /// ⚠ 必须更新 last_path_ / last_min_clearance_ / last_path_length_，
    ///   否则 publishPathViz() 用的是上一轮的旧数据（甚至空），
    ///   表现是"规划器在跑但 rviz 没有路径 / 测试端收不到 path"。
    /// @param override_path 非空时用它代替 res.path（热启动时是"引导段+A*段"）
    void applyPolylineFallback(const SearchResult& res, const ClearanceField& field,
                              const std::vector<Eigen::Vector3d>* override_path = nullptr) {
        // 折线兜底也要"能看"：直接用 A* 的原始栅格路径（每 0.1m 一点）
        // 是锯齿状折线，视觉上是乱的。
        // 这里先 simplify 删共线点，再做**保形**轻平滑（平滑后必须复核 clearance
        // 不下降，否则退回未平滑版本）。
        std::vector<Eigen::Vector3d> path =
            simplifyPath(override_path && override_path->size() >= 2 ? *override_path
                                                                    : res.path);
        const double mc_before = minClearanceAlongPath(path, field, 0.02);
        {
            const auto sm = smoothPath(path, field, sp_.robot_radius * 0.9, 2, 0.25);
            if (sm.size() >= 2) {
                const double mc_after = minClearanceAlongPath(sm, field, 0.02);
                if (mc_after >= mc_before - 1e-6) path = sm;
            }
        }

        // ══════════════════════════════════════════════════════════════════
        // ★★ 折线回退也必须"居中"（2026-11-14，核心理念兜底）
        //
        // 问题：曲线优化被拒（kmax 超限）或走 fast path 时，路径退回折线。
        //   而折线只做 simplify + 几何平滑（移动平均）——**完全没有居中**。
        //   smoothPath 的移动平均甚至会把路径往障碍方向拉（顶点的邻居偏移）。
        //
        // 实测（/tmp/azj_flight_20261114_221940）：
        //   t=51.894  fallback polyline: min_clr=0.125852  ← 只有 robot_radius 的 42%
        //   t=52.007  fast path 折线:    min_clr=0.121266
        //   而同一场景曲线成功时 min_clr=0.451。
        //   用户看到的"重规划后靠近一侧障碍"就是这两条折线。
        //
        // 修法：沿路径做**ESDF 梯度居中投影**（与曲线优化的 projectSafe 同源思路）：
        //   1) 稠密采样路径（0.05m）
        //   2) 对每个采样点，若 clearance < target，沿 ∇c 推开
        //   3) 投影后复核：任何一段的 clearance 不能下降（保形约束）
        //   4) 端点固定（起点=飞机，终点=目标，都是硬约束）
        //
        // 这不是"优化"，只是**安全修正**：不追求曲线平滑，只保证不贴边。
        // 目标是 max(robot_radius, 原路径 clearance)——不强行拉到 ref，
        // 避免在真窄的缝里把路径推出可行域。
        // ══════════════════════════════════════════════════════════════════
        if (np_.fallback_center_enable && path.size() >= 2) {
            // ★★ 折线居中：先稠密重采样，再逐点爬山到剖面最宽处
            //
            // 两个前版失败的原因（2026-11-16 修正）：
            //   ① 只在 simplify 后的**稀疏顶点**上爬山 —— 拐弯之间是直线，
            //      没有点可动，居中没有实效；
            //   ② 判据用**整条路径的 min clearance**，而起点/终点是硬约束
            //      （起点贴着飞机、终点是用户目标），它们的低 clearance
            //      拉低了 min，导致"中间点其实改好了"也被判定为无改进。
            //      实测（/tmp/azj_flight_20261116_210708）：
            //        start_clr=0.1189  worst=0.0943 → 居中从不生效（0 次日志）
            //
            // 现在：
            //   · 按 0.05m 等弧长重采样（点数足够密，每点都能爬山）
            //   · 判据只看**中间段**（排除首尾各 8%），端点固定不动
            //   · 仍要求新点 clearance 确实更大才接受（保形）
            std::vector<Eigen::Vector3d> rs;
            {
                double total = 0.0;
                for (size_t i = 1; i < path.size(); ++i)
                    total += (path[i] - path[i - 1]).norm();
                const int N = std::max(8, std::min(600,
                    static_cast<int>(std::ceil(total / 0.05)) + 1));
                rs.reserve(N);
                rs.push_back(path.front());
                double acc_t = 0.0, target = total / (N - 1);
                size_t seg = 0;
                double seg_acc = 0.0;
                for (int k = 1; k < N - 1; ++k) {
                    acc_t += target;
                    while (seg + 1 < path.size()) {
                        const double L = (path[seg + 1] - path[seg]).norm();
                        if (seg_acc + L >= acc_t) break;
                        seg_acc += L;
                        ++seg;
                    }
                    if (seg + 1 >= path.size()) break;
                    const double L = (path[seg + 1] - path[seg]).norm();
                    const double w = (L > 1e-9) ? (acc_t - seg_acc) / L : 0.0;
                    rs.push_back(path[seg] + w * (path[seg + 1] - path[seg]));
                }
                rs.push_back(path.back());
            }

            auto midMinClear = [&](const std::vector<Eigen::Vector3d>& v) {
                if (v.size() < 5) return -1.0;
                const size_t a = v.size() / 12;          // 排除首 8%
                const size_t b = v.size() - 1 - a;       // 排除尾 8%
                double mn = std::numeric_limits<double>::max();
                for (size_t i = a; i <= b; ++i) mn = std::min(mn, field.clearance(v[i]));
                return mn;
            };

            const double mid_before = midMinClear(rs);
            std::vector<Eigen::Vector3d> centered = rs;
            int moved_cnt = 0;
            // 步长 0.08m × 20 步 = 1.6m —— 足够从通道一侧走到另一侧
            //（实测通道半净宽 1.0m 时，12 步只能到 d≈-0.03，20 步能到正中）
            const double step_len = 0.08;
            const int max_steps = 20;
            for (size_t i = 1; i + 1 < centered.size(); ++i) {
                double c_best = field.clearance(centered[i]);
                if (c_best <= 0.0) continue;
                if (c_best >= np_.fallback_center_skip_clear) continue;
                Eigen::Vector3d q = centered[i];
                for (int k = 0; k < max_steps; ++k) {
                    Eigen::Vector3d g;
                    if (!field.clearanceGradient(q, g)) break;
                    const double gn = g.norm();
                    if (gn < 1e-6) break;
                    Eigen::Vector3d cand = q + (g / gn) * step_len;
                    if (plan_2d_active_) cand.z() = plan_2d_z_active_;
                    const double c_new = field.clearance(cand);
                    if (c_new <= c_best + 1e-3) break;
                    q = cand;
                    c_best = c_new;
                }
                if (c_best > field.clearance(centered[i]) + 1e-3) {
                    centered[i] = q;
                    ++moved_cnt;
                }
            }
            const double mid_after = midMinClear(centered);
            if (moved_cnt > 0 && mid_after > mid_before + 1e-3) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] fallback polyline centered: mid-section min_clr "
                             << mid_before << " -> " << mid_after << " ("
                             << moved_cnt << "/" << centered.size()
                             << " pts moved toward the widest line)");
                path = centered;
            }
        }

        if (plan_2d_active_) {
            for (auto& q : path) q.z() = plan_2d_z_active_;
        } else if (np_.enforce_height_limits) {
            for (auto& q : path) {
                q.z() = std::min(std::max(q.z(), np_.floor_z), np_.ceil_z);
            }
        }
        if (path.size() < 2) {
            has_trajectory_ = false;
            ROS_WARN_THROTTLE(1.0, "[azj] fallback: A* path too short (%zu)", path.size());
            return;
        }

        // 走到这里说明已产出可用路径 → 之后的重规划可以正常热启动复用
        goal_is_new_ = false;

        last_path_ = path;
        last_path_length_ = pathLength(path);
        last_min_clearance_ = minClearanceAlongPath(path, field, 0.02);
        last_max_curvature_ = 0.0;

        has_trajectory_ = buildTrajectory(path);
        if (has_trajectory_) {
            publishPathViz();
            // 定位最危险点：min_clr < robot_radius 时要能立刻看出在哪
            if (last_min_clearance_ < sp_.robot_radius - 1e-6) {
                double mn = 1e9;
                Eigen::Vector3d at = Eigen::Vector3d::Zero();
                for (const auto& q : last_path_) {
                    const double c = field.clearance(q);
                    if (c < mn) { mn = c; at = q; }
                }
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] fallback polyline: len=" << last_path_length_ << "m min_clr="
                                                        << last_min_clearance_
                                                        << " (worst vertex at " << at.transpose()
                                                        << " c=" << mn << ")"
                                                        << " start_clr="
                                                        << field.clearance(last_path_.front()));
            } else {
                ROS_INFO_STREAM_THROTTLE(1.0, "[azj] fallback polyline: len="
                                                  << last_path_length_ << "m min_clr="
                                                  << last_min_clearance_);
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "[azj] fallback: trajectory build failed");
        }
    }

    /// 沿路径的最小 clearance，但**排除终点前 keep_dist 米**。
    ///
    /// 为什么排除：目标点是用户指定的，它可能本身就贴着障碍
    /// （实测 raicom 场景 target 距墙 0.2m → clearance 0.15 < robot_radius 0.3）。
    /// A* 对目标格有例外放行，这是对的；曲线安全校验也不该因此判整条路径不安全。
    double minClearanceAlongPathExcludingEnd(const std::vector<Eigen::Vector3d>& path,
                                             const ClearanceField& field,
                                             double keep_dist, double step) const {
        if (path.empty()) return 0.0;
        if (step <= 1e-6) step = 0.02;

        // ★ 起点豁免（2026-11-05 修正，治“规划器卡死”活锁）。
        //
        // 这里是本规划器曾经最严重的一个死循环：
        //   飞机已经贴着柱子悬停（实测 clearance=0.108m < robot_radius=0.30m），
        //   任何从它当前位置出发的路径，**起点**clearance 必然是 0.108m。
        //   而本函数只排除了【终点】附近，没排除起点 →
        //     mc = 0.108 < robot_radius*0.9 = 0.27 → 永远 REJECTED
        //   → 回落折线（同样 0.108）→ needReplan 发现“前方受阻”
        //   → 立即重规划 → 又 REJECTED ……死循环，主线程 100% CPU，
        //   规划器再也不发 /planning/pos_cmd（用户看到“卡住了”）。
        //
        // 物理上：起点是**既成事实**，飞机就在那里。规划器的职责是把它
        // 安全地带出来，而不是因为它“所处位置不安全”就拒绝规划。
        // A* 那一层已经有同样的豁免（path_search.cpp 的 escape_radius）。
        // 只在**起点本身已不安全**时豁免，起点安全时仍严格校验全路径 ——
        // 否则会让“刚出发就穿墙”的曲线蒙混过关。
        double head = 0.0;   // 从起点算起跳过多少弧长
        const double start_clr = field.clearance(path.front());
        if (start_clr < sp_.robot_radius) {
            head = std::min(keep_dist, 0.5 * pathLength(path));
        }

        // 从终点往回累加弧长，找到 keep_dist 处的截断下标 → [0, cut) 为参与校验的部分
        size_t cut = path.size();          // 默认不截断
        {
            double acc = 0.0;
            for (size_t i = path.size(); i-- > 1;) {
                acc += (path[i] - path[i - 1]).norm();
                if (acc >= keep_dist) {
                    cut = i;                   // [0, cut) 为参与校验的部分
                    break;
                }
            }
        }
        if (cut < 2) return std::numeric_limits<double>::max();   // 整条都在终点附近

        // 从起点往前累加弧长，找到 head 处的起始下标 → [first, cut) 参与校验
        size_t first = 0;
        {
            double acc = 0.0;
            for (size_t i = 1; i < cut; ++i) {
                acc += (path[i] - path[i - 1]).norm();
                if (acc >= head) {
                    first = i;
                    break;
                }
            }
        }
        if (first >= cut) return std::numeric_limits<double>::max();

        double mn = field.clearance(path[first]);
        for (size_t i = first + 1; i < cut; ++i) {
            const Eigen::Vector3d& a = path[i - 1];
            const Eigen::Vector3d& b = path[i];
            const double len = (b - a).norm();
            const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
            for (int k = 0; k <= n; ++k) {
                const double t = static_cast<double>(k) / n;
                mn = std::min(mn, field.clearance(a + t * (b - a)));
            }
        }
        return mn;
    }

    /// 把规划层高边界钳进地图的实际 z 范围。
    ///
    /// 为什么必须钳：ceil_z 若超过地图 ceil，那片区域是 UNKNOWN，
    /// 而 unknown_clearance_cap(0.5) > robot_radius(0.3) 会被判为"可通行"
    /// → 路径往上飞进没有地图的地方（用户报的"从很上面规划"）。
    ///
    /// ⚠ 注意顺序：启动时必须先钳、再 publishParamsToServer()，
    /// 否则参数服务器上发布的是未钳制的值，诊断脚本会误判。
    void clampHeightLimitsToMap() {
        const double map_lo = cfg_.virtual_ground_height;
        const double map_hi = cfg_.virtual_ceil_height;
        if (np_.ceil_z > map_hi) {
            ROS_WARN("[azj] ceil_z=%.2f exceeds map ceil=%.2f; clamped "
                     "(beyond map is UNKNOWN -> would look free)",
                     np_.ceil_z, map_hi);
            np_.ceil_z = map_hi;
        }
        if (np_.floor_z < map_lo) {
            ROS_WARN("[azj] floor_z=%.2f below map ground=%.2f; clamped", np_.floor_z, map_lo);
            np_.floor_z = map_lo;
        }
        if (np_.ceil_z <= np_.floor_z + 0.1) {
            ROS_FATAL("[azj] height limits collapsed: floor=%.2f ceil=%.2f", np_.floor_z,
                      np_.ceil_z);
            np_.ceil_z = np_.floor_z + 0.3;
        }
    }

    /// 构造当前模式下的 clearance 场。
    ///
    /// 统一入口：3D 直接用 ESDF；2D 套一层柱状投影。
    /// ★ 为什么 2D 必须柱状投影：只看单层 z 的 ESDF 会漏掉上方天花板、
    ///   下方桌子等障碍，2D 规划就会直直穿过去 —— 安全性直接丢掉。
    ///   柱状范围取"当前高度到目标高度跨度 + 配置余量"，
    ///   保证爬升/下降的过渡段也被覆盖。
    ///
    /// z_plane：2D 的规划平面高度（= 目标高度）
    /// cur_z  ：飞机当前高度（用来决定柱状范围）
    ClearanceField::Ptr makeField(double /*z_plane*/, double /*cur_z*/) {
        ClearanceField::Ptr base(new FunctionClearanceField(
            [this](const Eigen::Vector3d& p) { return this->clearanceAt(p); },
            [this](const Eigen::Vector3d& p, Eigen::Vector3d& g) {
                return this->gradientAt(p, g);
            }));

        // ★ 2D 模式默认用**单层真实 3D ESDF**，不做任何投影。
        //
        // 为什么不做投影（用户指出的关键设计问题）：
        //   柱状投影把 z 方向压扁，会把**垂直方向上的空隙一起压掉**。
        //   典型例子：穿框 —— 框的孔在 z=0.8 那一层是空的，
        //   但投影把框的上下沿压到同一平面，孔就变成实心的，本来能过变成不能过。
        //
        // 而"漏掉桌子/横梁"的担忧其实不成立：飞机严格在 z_plane 飞行，
        // 桌子（z 0~0.5）和横梁（z 1.1~1.5）根本撞不到，
        // 它们本来就不该参与 z=0.8 这一层的判定。
        //
        // 如果确实想要一点垂直余量（比如考虑机体厚度/控制误差），
        // 用 plan_2d_slab_half 开一个小 slab（±half 取 min）。
        // 不要开太大：一旦覆盖到地板/天花板，每一列的 clearance 都会被它们压到 ~0，
        // 整个平面判死，A* 只能贴着残缝绕远（实测路径 40m、曲率 66、间隙 7mm）。
        if (!plan_2d_active_) return base;

        const double slab = np_.plan_2d_slab_half;
        if (slab <= 1e-6) {
            ROS_INFO_STREAM_THROTTLE(
                10.0, "[azj] 2D: single-slice ESDF at z=" << z_plane_ << " (no projection)");
            return base;
        }

        // 可选的薄 slab：只覆盖机体厚度，且严格钳在机身范围内
        const double half = std::min(slab, std::max(0.02, sp_.robot_radius));
        ROS_INFO_STREAM_THROTTLE(10.0, "[azj] 2D: slab ESDF z=" << z_plane_ << " +-"
                                                                 << half);
        return ClearanceField::Ptr(
            new PillarClearanceField(base, z_plane_, half, np_.plan_2d_z_samples));
    }

    /// clearance 的解析梯度（∇c 指向远离障碍）。出界 / 未知返回 false。
    bool gradientAt(const Eigen::Vector3d& p, Eigen::Vector3d& g) {
        const rog_map::Vec3f q = p;
        // 层外没有有效梯度：让曲线优化跳过这些采样点，
        // 而不是给它一个把曲线往外推的假梯度。
        if (np_.enforce_height_limits &&
            (p.z() < np_.floor_z || p.z() > np_.ceil_z)) {
            return false;
        }
        if (!esdf_map_->insideLocalMap(q)) return false;

        // ESDF 盒外没有有效梯度（distance_buffer 未初始化，读到的是 0）。
        // 与 clearanceAt 同一判定：交给优化器跳过该采样点。
        {
            rog_map::Vec3f box_min, box_max;
            esdf_map_->getUpdatedBbox(box_min, box_max);
            const double eps = cfg_.esdf_resolution;
            if ((q.array() < (box_min.array() - eps)).any() ||
                (q.array() > (box_max.array() + eps)).any()) {
                return false;
            }
        }

        const rog_map::Vec3f m = rog_map::Vec3f::Constant(cfg_.esdf_resolution);
        const rog_map::Vec3f q_plus = q + m;
        const rog_map::Vec3f q_minus = q - m;
        if (!esdf_map_->insideLocalMap(q_plus) || !esdf_map_->insideLocalMap(q_minus))
            return false;
        // UNKNOWN 区域没有真实 ESDF 梯度，交给优化器跳过该采样点
        if (isUnknown(q)) return false;

        esdf_map_->evaluateFirstGrad(p, g);
        return g.allFinite();
    }

    void velCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
        // mavros 的 velocity_local 已是 ENU 世界系，不要再转机体系
        current_vel_ = Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y,
                                       msg->twist.linear.z);
        vel_received_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        double gz = msg->pose.position.z;
        if (!std::isnan(np_.goal_z_override)) {
            // 说明来源：rviz 2D Nav Goal 在地面点击会给 z≈0。
            // 不覆盖就会让飞机降到地面。
            ROS_INFO("[azj] goal z overridden: %.2f -> %.2f", gz, np_.goal_z_override);
            gz = np_.goal_z_override;
        }
        // ★ 目标高度硬下限（2026-11-14）
        //
        // 没传 goal_z_override 时，rviz 的 2D Nav Goal 会把 z=0 直接带进来；
        // 规划器会“合法地”规划到地面，表现成掉高。这里给一个机制性兑底：
        // 低于 min_goal_z 就抬上去。NaN = 关闭（保持旧行为）。
        if (!std::isnan(np_.min_goal_z) && gz < np_.min_goal_z) {
            ROS_WARN("[azj] goal z=%.3f below min_goal_z=%.3f -> raised to %.3f. "
                     "(rviz 2D Nav Goal 给出 z≈0；若确实要飞这么低，把 azj/min_goal_z 设 nan)",
                     gz, np_.min_goal_z, np_.min_goal_z);
            gz = np_.min_goal_z;
        }
        goal_ = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, gz);

        // ══════════════════════════════════════════════════════════════════
        // ★ 目标点在膨胀层内 → 吸附到膨胀层外的最近安全点（2026-11-09）
        //
        // 需求：打的点即使落在膨胀层里，也要能规划出路径，但**终点必须停在
        //       膨胀层外并留安全距离**，而不是硬冲进障碍。
        //
        // ★ 这个功能 SUPER 已有实现，直接学它的做法（避免踩坑）：
        //     super_planner/src/super_core/fsm.cpp:191
        //       getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 3.0)
        //     rog_map/src/rog_map.cpp:105 findNearestInfCellThat
        //
        //   SUPER 的机制：预生成一张 `spherical_neighbor` 偏移表
        //   （rog_map_core/config.hpp:271，半径 5m、**按到原点距离升序排序**），
        //   然后从近到远逐个检查 `getInfGridType(q) != OCCUPIED`，
        //   第一个命中的就是**最近的膨胀层外格子**。
        //   找不到 → 报 "Goal is deeply occupied, skip this goal" 并丢弃目标。
        //
        // ★ 我们额外加了一层 SUPER 没有的约束：**安全距离余量**。
        //   SUPER 只保证"出了膨胀层"（clearance >= robot_radius），
        //   而我们的理念是安全优先，终点应当留出余量。
        //   所以吸附后再用 ESDF 检查，不足 min_goal_clearance 就继续往外找。
        // ══════════════════════════════════════════════════════════════════
        if (np_.snap_goal_out_of_inflation) {
            rog_map::Vec3f snapped = goal_;
            const bool found = getNearestInfCellNot(rog_map::GridType::OCCUPIED, goal_, snapped,
                                                    np_.goal_snap_search_radius);
            if (!found) {
                ROS_ERROR("[azj] goal (%.2f, %.2f, %.2f) is deeply occupied (no free cell within %.1fm) "
                          "- rejecting this goal",
                          goal_.x(), goal_.y(), goal_.z(), np_.goal_snap_search_radius);
                // 明确拒绝，不进入规划。这比"规划到障碍里"安全得多。
                publishGoalMarker();   // 仍然显示用户点击的位置，便于理解为什么没动
                return;
            }

            // 安全距离校验：用 ESDF 确认吸附点确实够宽敞。
            // 不足时沿"原目标 → 吸附点"方向继续外推，最多试 N 步。
            double clr = clearanceAt(snapped);
            if (clr >= 0.0 && clr < np_.min_goal_clearance) {
                const Eigen::Vector3d dir =
                    (snapped - goal_).template cast<double>().normalized();
                bool ok = false;
                for (int k = 1; k <= 10; ++k) {
                    const rog_map::Vec3f cand =
                        snapped + rog_map::Vec3f(dir * (0.1 * k));
                    const double c2 = clearanceAt(cand);
                    if (c2 >= np_.min_goal_clearance) {
                        snapped = cand;
                        clr = c2;
                        ok = true;
                        break;
                    }
                }
                if (!ok) {
                    ROS_WARN("[azj] snapped goal only has clearance %.2fm < %.2fm "
                             "(min_goal_clearance); using it anyway - the path will end there",
                             clr, np_.min_goal_clearance);
                }
            }

            if ((snapped - goal_).norm() > 1e-3) {
                ROS_WARN("[azj] goal moved out of inflation: (%.2f, %.2f, %.2f) -> "
                         "(%.2f, %.2f, %.2f)  moved %.2fm  clearance=%.2fm",
                         goal_.x(), goal_.y(), goal_.z(),
                         snapped.x(), snapped.y(), snapped.z(),
                         (snapped - goal_).norm(), clr);
                goal_ = snapped;
            } else {
                ROS_INFO("[azj] goal already clear of inflation (clearance=%.2fm)", clr);
            }
        }

        has_goal_ = true;
        arrived_ = false;
        has_trajectory_ = false;
        // ★ 新目标 → 旧轨迹不能当热启动引导段（2026-11-14）。
        //
        // 旧轨迹是朝**旧目标**飞的，拿它做引导会把飞机往过时方向带。
        // SUPER 的做法等价：generateExpTraj 里所有 NO_NEED 早退都带
        // `!gi_.new_goal` 条件（super_planner.cpp:441,470,483）。
        //
        // 注意 active_ 故意**不清**：publishTick 还要继续发旧轨迹，
        // 避免"新目标瞬间断流 → 控制器超时刹车"的空窗。
        goal_is_new_ = true;
        // 新目标 → 旧轨迹作废，anchor 也失效（下次 replanCallback 重新计算）
        plan_anchor_valid_ = false;

        // ── 读取打点自带的朝向（yaw_mode=FROM_MSG 用）──
        //
        // 之前这里**完全丢弃** orientation，于是 rviz 的 "2D Nav Goal"
        // 按住拖方向指定机头这个功能在 azj 上是失效的（SUPER 那边是支持的）。
        //
        // 有效性判断：四元数全零（没填）时 w=0，说明发送方没指定朝向，
        // 不能把它当 0 rad 用 —— 那会让机头强行转到 +X。
        const auto& q = msg->pose.orientation;
        const double qn = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (qn > 1e-6) {
            // 只取 yaw 分量（2D 打点不关心横滚/俯仰）
            const double siny = 2.0 * (q.w * q.z + q.x * q.y);
            const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            msg_yaw_ = std::atan2(siny, cosy);
            msg_yaw_valid_ = true;
        } else {
            msg_yaw_valid_ = false;
        }

        publishGoalMarker();
        ROS_INFO("[azj] goal received (%.2f, %.2f, %.2f)%s", goal_.x(), goal_.y(), goal_.z(),
                 msg_yaw_valid_ ? (" yaw=" + std::to_string(msg_yaw_)).c_str() : "");
    }

    /// 把路径转成时间参数化轨迹。
    /// 把路径对齐到飞机**当前**位置，并截掉已经飞过的部分。
    ///
    /// ★ 为什么必须做（2026-11-14 实测，活锁根因）
    ///
    /// 曲线优化单次要 **0.9~1.1 秒**（ncp=60）。这段时间飞机以 1 m/s 前进了
    /// 约 0.9 m，而路径起点是"规划开始时"的位置。
    /// 旧实现只在 `gap <= 0.5` 时对齐起点，**gap 更大就原样使用** → 新轨迹
    /// t=0 的参考点落在飞机身后 0.9 m → `publishTick` 立刻判
    /// `deviated from reference by 0.556m` → 又重规划 → 又 1 秒 → 又过期。
    ///
    /// 日志证据（/tmp/azj_planner.log，2D 模式）：
    ///   38.297  smoothing replan (fast-path polyline is 0.73s old)
    ///   39.125  curve=1089.1ms  kmax=6.27   ← 曲线成功
    ///   39.346  deviated 0.556m → replanning ← 曲线只活了 0.2 秒
    ///   43.984  曲线成功（耗时 1.038s）
    ///   44.102  deviated 0.948m → replanning ← 同样模式
    ///
    /// 这就是"折线很久才优化出曲线、且优化出来马上又没了"的机制：
    /// 不是优化频率低，是**每次优化完就被自己的起点过期打断**。
    ///
    /// 修法：沿路径找离当前位置最近的点（用 XY —— 掉高会让 z 差很大，
    /// 但 2D/3D 下都希望保留原路径的高度意图），从那里开始用，
    /// 并把起点接到当前位置。这样无论 gap 多大，t=0 参考都等于"现在"。
    /// 计算新轨迹的起点状态（学 SUPER 的 replan_forward_dt）。
    ///
    /// ★★★ 这是"切换轨迹掉高"的**唯一正确解法**（2026-11-14）
    ///
    /// SUPER（super_planner.cpp:379~500）：
    ///   replan_state_TT = replan_process_start_TT + replan_forward_dt
    ///   guide_pos_traj.getState(replan_state_TT, pos_init_state)   // (p,v,a) 整组
    ///
    /// 关键：**位置和速度必须取自同一条旧轨迹的同一时刻**。
    /// 混用（ref_p + V_act）会让轨迹先朝 V_act 走、再弯回路径 → S 形：
    ///   /tmp/azj_flight_20261114_215707  t=53.741
    ///     anchor=ref_p, anchor_v=V_act → 同一 tid 内 refV 从
    ///     (-0.28,-0.39) 翻到 (0.66,0.89) → roll -23.8° → z 0.833→-0.05
    ///
    /// 前视时长：SUPER 用固定 0.1s（它规划 0.3ms）。我们用**实测规划耗时**
    /// （plan_duration_ema_），否则新起点仍落在飞机已经飞过的地方。
    /// 计算新轨迹的起点状态（**从当前 odom 出发**）。
    ///
    /// @param look 前视时长 [s]：DEVIATED/realign 传 0（就是当前位置）；
    ///             完整规划传 plan_duration_ema_（预测"发布那一刻"的位置）。
    ///
    /// ★ 为什么必须从当前 odom 出发（2026-11-16 定论）
    ///
    /// 用户的观察完全正确："路径应该以当前 odom 为起点"。
    ///
    /// 试过的三条错路：
    ///   ① 照搬 SUPER 的"旧轨迹未来时刻"（v1.7.0）
    ///      → SUPER 靠 0.3ms 的规划速度让 0.1s 前视 ≈ 当前位置；
    ///        我们规划慢，前视就变成"前方 1 米"，实测断层 0.91~1.51m。
    ///   ② realign 用缓存的 plan_anchor_（v1.8.1）
    ///      → realign 在 A* 分支之前 return，plan_anchor_ 是**上一次**的值
    ///      → 实测 4 次 realign 偏差 0.50→0.73→0.58→0.58，完全不收敛。
    ///   ③ 用 plan_start_pos_（规划开始位置）+ 速度外推
    ///      → 单线程 spinner 下 robot_state_ 在规划期间冻结，
    ///        但 plan_start_pos_ 就是冻结时的值 —— 外推误差取决于转弯程度。
    ///
    /// 正解：在本回调**开头**（spinner 刚处理完上一批回调，状态最新）取
    /// `robot_state_.p` 和 `current_vel_`，按 look 外推。
    ///   · look=0 → 精确等于当前 odom（realign 用）
    ///   · look=ema → 预测发布时刻位置（完整规划用，规划 270ms → 前视 0.27m）
    ///
    /// 位置与速度**同源**（同一时刻的飞机状态）→ 不会产生 S 弯。
    void computeAnchor(double look, Eigen::Vector3d& anchor, Eigen::Vector3d& anchor_v) {
        const Eigen::Vector3d p_now = robot_state_.rcv ? robot_state_.p : Eigen::Vector3d::Zero();
        const Eigen::Vector3d v_now = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();

        // 外推上限 0.8s（防异常耗时把 anchor 推太远）
        const double look_c = std::min(0.8, std::max(0.0, look));

        anchor_v = v_now;
        anchor = p_now + v_now * look_c;

        if (look_c > 1e-3) {
            ROS_INFO_STREAM_THROTTLE(
                2.0, "[azj] anchor = current odom " << p_now.transpose()
                         << " + " << look_c << "s of flight = " << anchor.transpose()
                         << " (|v|=" << v_now.norm() << ")");
        }
        (void)plan_start_pos_;
        (void)plan_start_vel_;
    }

    bool buildTrajectory(const std::vector<Eigen::Vector3d>& path) {
        if (path.size() < 2) return false;
        if (!(vel_received_ || robot_state_.rcv)) {
            TrajParams tp = tp_;
            return buildTrajectoryImpl(path, tp);
        }

        // ★★ 用"规划开始状态 + 耗时"外推，而不是冻结的 robot_state_（2026-11-14）
        //
        // 详见 plan_start_wall_ 的注释。/tmp/azj_flight_20261114_150752 实测：
        // 规划耗时 0.7~1.1s，外推后位置误差从 ~1.0m 降到 ~0.1m 量级。
        //
        // 限幅 1.5s：规划若异常慢（>1.5s），外推会失真，此时宁可保守。
        const double dt_plan = plan_start_wall_.isZero()
                                   ? 0.0
                                   : std::min(1.5, (ros::WallTime::now() - plan_start_wall_).toSec());
        const bool extrapolated = (dt_plan > 0.05);
        Eigen::Vector3d cur = extrapolated
                                  ? plan_start_pos_ + plan_start_vel_ * dt_plan
                                  : robot_state_.p;
        Eigen::Vector3d v_cur_extrap = plan_start_vel_;
        if (extrapolated) {
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] state extrapolated over " << dt_plan
                     << "s of planning: pos " << plan_start_pos_.transpose() << " -> "
                     << cur.transpose() << " (moved "
                     << (cur - plan_start_pos_).norm() << "m)");
        }

        // ══════════════════════════════════════════════════════════════════
        // ★★★ 起点状态：复用 replanCallback 算好的 anchor（2026-11-14）
        //
        // 不在这里重新 computeAnchor —— 那是 A*+曲线之后了，旧轨迹的
        // t_old 已经推进了 0.5~1.0s，会算出**不同的** anchor，
        // 导致 A* 路径起点与轨迹起点错位（正是折返的来源）。
        // 详见 computeAnchor() 的注释。
        Eigen::Vector3d anchor = plan_anchor_valid_ ? plan_anchor_
                                                    : Eigen::Vector3d(robot_state_.p);
        Eigen::Vector3d anchor_v = plan_anchor_valid_ ? plan_anchor_v_
                                                      : Eigen::Vector3d::Zero();

        // ══════════════════════════════════════════════════════════════════
        // ★★ anchor 过期补偿（2026-11-14，治"曲线被拒→折线不居中"）
        //
        // `plan_anchor_` 是 replanCallback **开头**算的（规划开始），
        // 而本函数在 A*+曲线优化**之后**才执行（规划结束）。
        //
        // 实测（/tmp/azj_flight_20261114_221940）：
        //   t=50.107  保存 anchor（飞机位置）
        //   t=51.891  curve rejected: kmax=395.656  ← 规划耗时 1.78s！
        //   t=51.892  state extrapolated over 1.5s（clamp）
        //   t=51.894  fallback polyline: min_clr=0.125852
        //   → C1 transition 0.80m（anchor 比飞机落后 0.8m）
        //
        // 这 0.8m 的错位要被 C1 过渡段硬补回来 → 过渡段形状急变
        // → 曲线 kmax 从正常的 3 涨到 **395** → 超过容差 60 → 曲线被拒
        // → 退回**折线**（折线没有居中优化）→ 用户看到"靠近一侧障碍"。
        //
        // 修法：按"实际耗时 − 预期耗时"沿**原速度方向**外推 anchor。
        //   · 方向用 plan_start_vel_（与 anchor_v 同向）→ 不会引入 S 弯
        //   · 只补偿超出预期的部分，正常情况不动
        //   · 上限 plan_anchor_catchup_max，防止异常耗时把 anchor 推太远
        // ══════════════════════════════════════════════════════════════════
        if (np_.plan_anchor_catchup_max > 1e-6 && plan_anchor_valid_ &&
            !plan_start_wall_.isZero()) {
            const double elapsed = (ros::WallTime::now() - plan_start_wall_).toSec();
            const double expected = std::max(0.05, plan_duration_ema_);
            const double excess = elapsed - expected;
            if (excess > 0.05 && plan_start_vel_.head<2>().norm() > 0.05) {
                const double adv = std::min(np_.plan_anchor_catchup_max, excess) *
                                   plan_start_vel_.norm();
                const Eigen::Vector3d dir = plan_start_vel_.normalized();
                anchor += dir * adv;
                ROS_INFO_STREAM_THROTTLE(
                    1.0, "[azj] anchor catch-up: planning took " << elapsed
                         << "s (expected " << expected << "s) -> advanced anchor "
                         << adv << "m along the flight direction (was too stale, "
                         << "which inflated the transition segment and got the "
                         << "curve rejected)");
            }
        }

        // ══════════════════════════════════════════════════════════════════
        // ★★ 起点对齐必须遵守**运动方向连续性**（2026-11-14 重大修正）
        //
        // 事故（/tmp/azj_flight_20261114_134950/flight.bag，2D）：
        //   t=36.26 轨迹切换 tid 1→2，新轨迹首帧速度 **(-0.24, -0.98)**，
        //   而旧轨迹是 (+0.75, -0.66)、实际是 (+0.45, -0.59)
        //   —— x 分量**符号翻转**，要求飞机立刻反向。
        //   控制器只能猛打姿态：tilt 3.8° → 19°，spVx +0.92 → -1.45。
        //   推重比 1.32（悬停油门 0.871，只剩 12.9% 行程）撑不住
        //   → z 从 0.808 掉到 0.005（1.4 秒内）。
        //
        // 根因：旧实现无脑连一条 `cur → 最近路径点` 的直线过渡段。
        //   一旦最近点在飞机的**侧后方**（飞机被挤离轨迹 0.63m 时很常见），
        //   过渡段方向与运动方向相反 → 轨迹首帧速度反向。
        //
        // SUPER 的做法（super_planner.cpp:487~500）：
        //   引导段起点是**旧轨迹在未来 replan_state_TT 的状态**，
        //   不是飞机当前位置；飞机与轨迹的偏差由轨迹优化的初始状态
        //   （pos_init_state 含速度）吸收，**绝不插几何直线过渡**。
        //
        // 本实现（不引入几何折返）：
        //   ① 在路径上找"起点投影"时，**排除掉切向与当前速度相反的段** ——
        //      这些段已在飞机身后
        //   ② 过渡段仍要插（否则 t=0 参考会跳），但只在
        //      `cur→路径点` 与速度同向时才允许大 gap；
        //      反向时把 gap 压到很小，让轨迹自己的 v_start 去消化
        //   ③ 全部路径点都在身后 → 这条路径已完全过时，返回 false
        //      让上层重新规划（不能用一条朝后的路径起飞）
        // ══════════════════════════════════════════════════════════════════
        const Eigen::Vector3d v_cur = anchor_v;
        const bool have_dir = v_cur.head<2>().norm() > 0.20;

        auto seg_dir_ok = [&](size_t i) -> bool {
            if (!have_dir) return true;              // 速度太小，方向不可靠
            if (i + 1 >= path.size()) return true;
            const Eigen::Vector3d d = path[i + 1] - path[i];
            const double nd = d.head<2>().norm();
            if (nd < 1e-6) return true;
            // 切向与速度夹角 < 90° 才算"前方"
            return d.x() * v_cur.x() + d.y() * v_cur.y() > 0.0;
        };

        // ---- 1) 找起点投影：最近 且 切向朝前 ----
        int i_best = -1;
        double d_best = std::numeric_limits<double>::infinity();
        int i_fallback = 0;                          // 兜底（忽略方向）
        double d_fallback = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < path.size(); ++i) {
            const double d = (path[i].head<2>() - anchor.head<2>()).norm();
            if (d < d_fallback) { d_fallback = d; i_fallback = static_cast<int>(i); }
            if (!seg_dir_ok(i)) continue;
            if (d < d_best) { d_best = d; i_best = static_cast<int>(i); }
        }
        if (i_best < 0) {
            // 没有任何朝前的点 → 路径已完全过时
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[azj] path fully behind the aircraft (motion "
                         << v_cur.head<2>().transpose()
                         << ", nearest pt " << d_fallback << "m) - rejecting it, will replan");
            return false;
        }
        // 留一个点的余量：若最近点就是末端，至少要保留最后一段
        if (i_best > static_cast<int>(path.size()) - 2) {
            i_best = std::max(0, static_cast<int>(path.size()) - 2);
        }

        const Eigen::Vector3d gap_vec = path[i_best] - anchor;
        const double gap3d = gap_vec.norm();

        // ★★ 起点严重错位 → 拒绝这条路径（2026-11-16）
        //
        // 实测 179.4° 反向过渡的来源：anchor 外推过头 → seg_dir_ok 把
        // 路径首段判为"反向" → i_best 被推到后面 → anchor 到 i_best
        // 的过渡段与路径方向相反 → 路径形状是"去而复返"。
        // 这种路径要求飞机 180° 掉头，是掉高/抖动的直接原因。
        //
        // 判据：跳过的点 > 2 个 且 gap > 1.0m → 说明 anchor 与路径严重错位
        // （路径基本是"从别处开始"的）→ 拒绝，让上层保留旧轨迹。
        if (i_best > 2 && gap3d > 1.0) {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[azj] path start badly misaligned (gap " << gap3d
                         << "m after skipping " << i_best << "/" << path.size()
                         << " pts) - rejecting to avoid a U-turn transition");
            return false;
        }

        // ★★ 方向判据只能用**路径切向**，不能用位置偏差向量（2026-11-14 修正）
        //
        // 上一版写的是 `dot(gap_vec, v_cur) >= 0`，这是个几何误判：
        //   gap_vec = path[i_best] - cur 是**位置偏差**，方向任意都正常。
        //   飞机以 (+0.98,-0.38) 飞、路径点在左上 0.26m 时，
        //   gap_vec=(-0.10,+0.26) → dot=-0.197<0 → 被判"反向" → 拒绝整条路径。
        //   实测（15:00 日志）：gap 只有 0.26~0.40m 就拒了 3 次，
        //   而路径切向其实完全朝前。
        //
        // 正确的量是**路径切向**（path[i+1]-path[i]）—— 它才决定新轨迹
        // t=0 的速度方向。这个判据在找 i_best 时已经用过了（seg_dir_ok），
        // 所以只要 i_best 是"朝前的点"，方向就必然一致。
        //
        // 既然方向由 seg_dir_ok 保证，过渡段就**不需要、也不应该**再按方向拒绝。
        // 剩下的只是"怎么平滑地把位置偏差补上" → 用 C1 连续的 Hermite 曲线
        // （起点切向 = 当前速度），t=0 的速度天然等于当前速度，绝不反向。
        const Eigen::Vector3d gap_vec2 = path[i_best] - cur;
        (void)gap_vec2;

        if (d_best > 0.60) {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[azj] trajectory start realigned: nearest FORWARD path point is "
                         << d_best << "m away (XY) at index " << i_best << "/"
                         << path.size() << " - aircraft is off the path");
        } else if (i_best > 0) {
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] trajectory trimmed: skipped " << i_best
                     << " already-passed points (aircraft advanced while planning)");
        }

        // ---- 2) 起点接续：**绝不制造反向速度** ----
        //
        // 关键约束：新轨迹 t=0 的速度方向必须与当前速度夹角 < 90°。
        // 做法：过渡段的几何长度受"方向一致性"限制：
        //   同向  → 允许完整过渡（最多 20 段 × 0.10m）
        //   反向  → 不允许几何折返，直接把起点放在路径点上，
        //           偏差交给 v_start 在时间里消化（轨迹会平滑加速）
        std::vector<Eigen::Vector3d> fixed;
        fixed.reserve(path.size() - i_best + 8);

        // 三次 Hermite 过渡段：起点切向 = 当前速度，终点切向 = 路径切向。
        //
        //   H(t) = (2t³-3t²+1)P0 + (t³-2t²+t)m0 + (-2t³+3t²)P1 + (t³-t²)m1
        //   H'(0) = m0  →  t=0 的速度方向 = m0 的方向 = **当前速度方向**
        //
        // 这就是 SUPER/MINCO 的做法：用 (位置, 速度) 作为轨迹起点约束，
        // 而不是"几何直线连接 + 事后限制方向"。直线连接的问题是
        // t=0 切向 = gap_vec 方向，与飞机真实速度无关 —— 那才是掉高根因。
        Eigen::Vector3d cur_fixed = anchor;
        if (plan_2d_active_) cur_fixed.z() = plan_2d_z_active_;
        fixed.push_back(cur_fixed);
        if (gap3d > 0.02) {
            const Eigen::Vector3d goal_pt = path[i_best];
            // 终点切向：路径在该点的走向
            Eigen::Vector3d t_end = Eigen::Vector3d::Zero();
            if (i_best + 1 < static_cast<int>(path.size())) {
                t_end = (path[i_best + 1] - path[i_best]).normalized();
            } else if (i_best > 0) {
                t_end = (path[i_best] - path[i_best - 1]).normalized();
            }
            // 起点切向：**优先**用当前速度（保证轨迹 t=0 速度连续），
            // 但若速度方向与路径方向夹角过大，就用路径切向。
            //
            // ★ 为什么必须加夹角保护（2026-11-14 实测）：
            //   外推能补偿位置，但补偿不了"转弯中方向已变"这一事实。
            //   实测 t=34.30：速度 (0.64,-0.64)、路径切向 (-0.36,-0.92)，
            //   夹角 70.8°。若照用速度方向做起点切向，C1 过渡段从
            //   旧速度方向弯到路径方向，轨迹首帧就是"侧向猛打"。
            //
            //   夹角 >60° 时说明两种可能：①状态严重过期 ②飞机真的在急转。
            //   两种情况下**沿路径方向起步**都更安全 —— 飞机只需慢慢把
            //   横向速度消化掉，而不是被要求瞬间掉头。
            Eigen::Vector3d t_beg = goal_pt - anchor;
            {
                Eigen::Vector3d t_path = goal_pt - anchor;
                if (t_end.norm() > 1e-6) t_path = t_end;   // 用路径切向做基准
                if (have_dir && v_cur.head<2>().norm() > 0.20 && t_path.norm() > 1e-6) {
                    const Eigen::Vector3d vd = v_cur.normalized();
                    const double cosang = std::max(-1.0, std::min(1.0,
                                               vd.dot(t_path.normalized())));
                    const double ang_deg = std::acos(cosang) * 180.0 / M_PI;
                    if (ang_deg < 60.0) {
                        t_beg = v_cur;      // 方向一致 → 用速度（平滑衔接）
                    } else {
                        t_beg = t_path;     // 方向冲突 → 用路径（不逼飞机掉头）
                        ROS_WARN_STREAM_THROTTLE(
                            1.0, "[azj] velocity/path mismatch " << ang_deg
                                 << "deg (>60) - transition follows the PATH direction "
                                 << "instead of the (stale) velocity");
                    }
                }
            }
            if (t_beg.norm() < 1e-6) t_beg = goal_pt - anchor;
            t_beg.normalize();
            const double m0 = gap3d * 0.8;     // 切向量长度：0.8×间距，不过冲
            const double m1 = gap3d * 0.8;
            const int N = std::max(2, std::min(24,
                static_cast<int>(std::ceil(gap3d / 0.10))));
            for (int k = 1; k < N; ++k) {
                const double t = static_cast<double>(k) / N;
                const double t2 = t * t, t3 = t2 * t;
                const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
                const double h10 = t3 - 2.0 * t2 + t;
                const double h01 = -2.0 * t3 + 3.0 * t2;
                const double h11 = t3 - t2;
                Eigen::Vector3d q = h00 * cur_fixed + h10 * (m0 * (Eigen::Vector3d)t_beg)
                                    + h01 * goal_pt + h11 * (m1 * t_end);
                // 2D 模式：过渡段也必须锁在规划平面。
                // 否则 cur.z 与 path.z 的差会被 Hermite 插值成 z 变化，
                // 违反 "2D 只做 XY" 的约定（且路径 z 会偏离锁定平面）。
                if (plan_2d_active_) q.z() = plan_2d_z_active_;
                fixed.push_back(q);
            }
            if (gap3d > 0.15) {
                ROS_INFO_STREAM_THROTTLE(
                    1.0, "[azj] C1 transition: " << gap3d << "m, start tangent "
                         << t_beg.head<2>().transpose() << " (= current velocity dir)");
            }
        }
        for (size_t i = static_cast<size_t>(i_best); i < path.size(); ++i) {
            fixed.push_back(path[i]);
        }

        last_replan_gap_ = gap3d;

        // ══════════════════════════════════════════════════════════════════
        // ★★ 急转前先刹车（2026-11-14，方向 B 的闭环）
        //
        // 场景：未知建图区里新扫出障碍，路径需要在起点处急转（甚至掉头）。
        // 物理约束：飞机此刻有 1 m/s 动量，推重比 1.32 意味着
        //   "一边高速前进一边急转"会吃掉全部姿态余量 → 掉高。
        //
        // 实测（/tmp/azj_flight_20261114_152514）：
        //   t=44.39 切换轨迹，cmd 速度从 (0.71,-0.70) 变成 (-0.23,-0.95)
        //   → 方向差 70.8°，z 从 0.782 掉到 0.515
        //
        // 修法（等价 SUPER 的 "guide 段 + backup"）：
        //   若路径首段方向与当前速度夹角 > brake_angle_deg，
        //   先拼一段**沿当前速度的刹车段**（减速到静止），再接路径。
        //   飞机先平滑减速 → 速度低了再转向 → 转弯不吃推力 → 不掉高。
        //
        // 这正是 spliceAfter 的用途：刹车段在前，路径段在后，时间/位置连续。
        // ══════════════════════════════════════════════════════════════════
        TrajParams tp = tp_;
        double turn_deg = 0.0;
        if (anchor_v.head<2>().norm() > np_.brake_angle_min_speed && fixed.size() >= 2) {
            const Eigen::Vector2d d0 = (fixed[1] - fixed[0]).head<2>();
            if (d0.norm() > 1e-6) {
                const double c = std::max(-1.0, std::min(1.0,
                    anchor_v.head<2>().normalized().dot(d0.normalized())));
                turn_deg = std::acos(c) * 180.0 / M_PI;
            }
        }

        // ★ Hermite 轨迹已能"边走边转弯"，不再需要"先刹停再转"（2026-11-14）。
        //
        // 刹车（TURN）会带来 0.34~1.0s 的停顿 —— 这正是用户说的"切换时卡顿震荡"。
        // 实测 /tmp/azj_flight_20261114_153350：TURN 触发 7 次，每次停 0.34~1.0s。
        //
        // 现在轨迹用 buildSmoothFromState（起点速度是完整矢量），
        // 129° 的掉头也能靠"引线段 + U 型曲线"平滑完成，不停顿。
        // brake_angle_deg 保留为兜底：若 Hermite 构建失败才用。
        if (np_.backup_enable && !np_.hermite_preferred && turn_deg > np_.brake_angle_deg) {
            // ① 刹车段：从 anchor 沿速度方向减速到静止
            PathTrajectory brake;
            if (PathTrajectory::buildBrake(anchor, anchor_v, tp, brake)) {
                // ② 主轨迹：从刹车段末端沿路径走（起步速度 0）
                Eigen::Vector3d bp = brake.positions().back();
                // 把路径起点接到刹车末端
                std::vector<Eigen::Vector3d> fixed2;
                fixed2.push_back(bp);
                for (const auto& q : fixed) {
                    if ((q - fixed2.back()).norm() > 0.02) fixed2.push_back(q);
                }
                auto snap = std::make_shared<TrajSnapshot>();
                if (snap->traj.build(fixed2, tp)) {
                    // ③ 拼接：刹车段 [0, T] + 主轨迹
                    const double t_join = brake.duration();
                    brake.spliceAfter(t_join, snap->traj);

                    auto out = std::make_shared<TrajSnapshot>();
                    out->traj = brake;
                    out->start = ros::Time::now();
                    out->id = ++trajectory_id_;
                    out->v0 = anchor_v;
                    out->backup_start = 0.0;      // 前段是刹车（backup 语义）
                    out->yaw_prof = buildYawProfile(
                        out->traj, 0.5, -1.0,
                        robot_state_.rcv ? robot_state_.yaw
                                         : std::numeric_limits<double>::quiet_NaN());
                    {
                        std::lock_guard<std::mutex> lk(state_mtx_);
                        active_ = std::move(out);
                        has_trajectory_ = true;
                    }
                    ROS_WARN_STREAM(
                        "[azj] TURN " << turn_deg << "deg (> " << np_.brake_angle_deg
                        << "): braking " << t_join << "s first, then turning at low speed "
                        << "(avoids losing altitude while turning at 1 m/s)");
                    return true;
                }
            }
        }

        // 交给实现层：从 anchor 状态起步（参考连续）
        return buildTrajectoryImpl(fixed, tp, &anchor, &anchor_v);
    }

    /// @param start_p/start_v 非空时用它们作为轨迹起点状态（参考连续），
    ///                       为空则退回"当前位置 + 当前速度"。
    /// 把当前生效轨迹按空间间隔抽稀成折线（供 rviz 显示）。
    ///
    /// 为什么不能直接把轨迹全部采样点当路径发：
    ///   MINCO 按 0.01s 采样，5m 轨迹就是 500+ 点，rviz 的 LINE_STRIP
    ///   每帧画一遍会明显掉帧。按 0.10m 抽稀后约 50 点，形状不失真。
    std::vector<Eigen::Vector3d> sampleTrajAsPath(double step) const {
        std::vector<Eigen::Vector3d> out;
        const auto snap = snapTraj();
        if (!snap || !snap->traj.valid()) return out;
        step = std::max(0.01, step);
        const double dur = snap->traj.duration();
        Eigen::Vector3d last_p;
        bool have_last = false;
        for (double t = 0.0; t <= dur + 1e-9; t += 0.02) {
            Eigen::Vector3d p, v, a;
            if (!snap->traj.sample(std::min(t, dur), p, v, a)) continue;
            if (!have_last || (p - last_p).norm() >= step) {
                out.push_back(p);
                last_p = p;
                have_last = true;
            }
        }
        // 末尾点一定要有（否则曲线看起来"没走到目标"）
        Eigen::Vector3d pe, ve, ae;
        if (snap->traj.sample(dur, pe, ve, ae)) {
            if (out.empty() || (pe - out.back()).norm() > 1e-3) out.push_back(pe);
        }
        return out;
    }

    /// 限制相邻点最大间距：长直线段中间插点。
    ///
    /// ★★★ 为什么必须做（2026-11-25 实飞 dump 定位）
    ///
    /// `simplifyPath` 只保留拐角、删掉共线点。于是 A* 输出里会出现
    /// **1.2m 的长直线段只有两个端点**。而 MINCO 的 `piece_len=0.40`
    /// 决定内点数：1.2m 段只分到 3 个内点，配上 36° 的转折，
    /// 7 次多项式必然过冲。
    ///
    /// 实测（flight_20261125_195750, minco_00045）：
    ///   in_path:  [11](-2.56,-0.90) → [12](-1.62,-1.60)  **1.175m**
    ///             [12] → [13](-0.42,-1.60)               **1.200m**
    ///   输出:     vmax=**4.47 m/s**（max_vel=1.0 的 4.5 倍）
    ///             amax=**20.4 m/s²**（max_acc=1.0 的 20 倍）
    ///             min_clr 从输入 0.18 掉到 **0.0028**  → 被拒 → 折线
    ///
    /// @param max_gap 相邻点最大间距 [m]，建议 ≤ piece_len
    std::vector<Eigen::Vector3d> densifyPath(const std::vector<Eigen::Vector3d>& in,
                                             double max_gap) const {
        if (in.size() < 2 || max_gap <= 1e-6) return in;
        std::vector<Eigen::Vector3d> out;
        out.reserve(in.size() * 2);
        out.push_back(in.front());
        int inserted = 0;
        for (size_t i = 1; i < in.size(); ++i) {
            const Eigen::Vector3d& a = in[i - 1];
            const Eigen::Vector3d& b = in[i];
            const double d = (b - a).norm();
            if (d > max_gap) {
                const int n = static_cast<int>(std::ceil(d / max_gap));
                for (int k = 1; k < n; ++k) {
                    out.push_back(a + (b - a) * (static_cast<double>(k) / n));
                    ++inserted;
                }
            }
            out.push_back(b);
        }
        if (inserted > 0) {
            ROS_INFO_STREAM_THROTTLE(
                2.0, "[azj] densify: " << in.size() << " -> " << out.size()
                     << " pts (inserted " << inserted << ", max_gap " << max_gap << "m)");
        }
        return out;
    }

    /// 把**当前生效轨迹从此刻往后的剩余部分**发布成 /azj/path（2026-11-17）。
    ///
    /// 与 `sampleTrajAsPath()` 的区别：那个从 t=0 抽稀（显示整条轨迹，
    /// 含已飞过的部分）；本函数从**当前播放时刻**开始，所以 path 起点
    /// 永远是飞机该到达的参考点 —— 这正是用户在 rviz 里预期的"跟着飞机走"。
    void publishCommittedTrajAsPath() {
        const auto snap = snapTraj();
        if (!snap || !snap->traj.valid()) return;
        const double dur = snap->traj.duration();
        double t_now = (ros::Time::now() - snap->start).toSec();
        t_now = std::min(std::max(t_now, 0.0), dur);

        std::vector<Eigen::Vector3d> pts;
        Eigen::Vector3d last_p;
        bool have = false;
        for (double t = t_now; t <= dur + 1e-9; t += 0.02) {
            Eigen::Vector3d p, v, a;
            if (!snap->traj.sample(std::min(t, dur), p, v, a)) continue;
            if (!have || (p - last_p).norm() >= 0.10) {
                pts.push_back(p);
                last_p = p;
                have = true;
            }
        }
        Eigen::Vector3d pe, ve, ae;
        if (snap->traj.sample(dur, pe, ve, ae)) {
            if (pts.empty() || (pe - pts.back()).norm() > 1e-3) pts.push_back(pe);
        }
        if (pts.size() < 2) return;

        const std_msgs::Header header = makeHeader();
        nav_msgs::Path msg;
        msg.header = header;
        for (const auto& p : pts) {
            geometry_msgs::PoseStamped ps;
            ps.header = header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = p.z();
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_.publish(msg);
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            last_path_ = pts;
        }
    }

    /// 把已构建好的 PathTrajectory 装进快照并原子换上新参考。
    ///
    /// 单独抽出来的原因（2026-11-17）：轨迹现在有两个后端
    /// （B 样条+TOPP 走 buildTrajectoryImpl，MINCO 走 buildTrajectoryMinco），
    /// 但"生成 yaw 曲线 / 记 v0 / 原子换指针"这三步完全相同。
    /// 复制两份极易漂移（改一处忘另一处），所以统一到这里。
    bool commitTrajectory(PathTrajectory&& traj) {
        if (!traj.valid()) return false;
        auto snap = std::make_shared<TrajSnapshot>();
        snap->traj = std::move(traj);
        snap->v0 = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        snap->start = ros::Time::now();
        snap->id = ++trajectory_id_;

        // 生成预规划 yaw 曲线（照 SUPER 的 yaw_traj_opt 做法）。
        // start_yaw 传当前机头：让曲线起点与飞机实际朝向连续，
        // 避免轨迹一生成就要求转一个大角度。
        snap->yaw_prof = buildYawProfile(
            snap->traj, 0.5, -1.0,
            robot_state_.rcv ? robot_state_.yaw
                             : std::numeric_limits<double>::quiet_NaN());
        // ★ yaw 曲线摘要（2026-11-25）：验证 mode=0 是否真的跟随路径切线
        {
            const auto& yp = snap->yaw_prof;
            if (yp.valid()) {
                double lo = 1e9, hi = -1e9;
                for (double y : yp.yaw) { lo = std::min(lo, y); hi = std::max(hi, y); }
                ROS_INFO_STREAM_THROTTLE(
                    2.0, "[azj] yaw_prof: " << yp.t.size() << " waypoints, range ["
                             << lo * 180.0 / M_PI << "°, " << hi * 180.0 / M_PI
                             << "°] span " << (hi - lo) * 180.0 / M_PI << "° dur="
                             << snap->traj.duration() << "s");
            } else {
                ROS_WARN_STREAM_THROTTLE(2.0, "[azj] yaw_prof INVALID (only "
                                                  << yp.t.size() << " waypoints)");
            }
        }

        // 原子换上新快照。发布线程要么看到旧轨迹、要么看到新轨迹。
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            active_ = std::move(snap);
            has_trajectory_ = true;
        }
        return true;
    }

    /// MINCO 后端：直接从路径 + 起终点状态生成轨迹（2026-11-17）。
    ///
    /// 与 buildTrajectoryImpl 的区别：
    ///   buildTrajectoryImpl → B 样条形状优化 + TOPP 时间参数化，起点只约束 v
    ///   buildTrajectoryMinco → MINCO 联合优化形状与时间，起点硬约束 p/v/a/j
    ///
    /// @param path   A* 路径（会作为 MINCO 内点初值）
    /// @param head   起点状态。**a/j 必须来自旧轨迹**（这是 MINCO 的核心收益：
    ///               告诉优化器"我正以 a 加速/减速"，而不是每次从 a=0 重来）
    /// @param tail   终点状态（rolling 时 v 非零）
    /// 把一次 MINCO 调用写成文本文件，供离线分析（2026-11-25）。
    ///
    /// 只写文本（不写二进制）：可以直接 grep / 人工读，
    /// 也能被 azj_path_audit.py 解析。
    void dumpMincoCall(const ClearanceField& field,
                       const std::vector<Eigen::Vector3d>& in_path,
                       const StatePVAJ& head, const StatePVAJ& tail,
                       const MincoOptParams& mp,
                       const MincoOptResult* res,   // nullptr = 优化失败
                       double mc_check, double min_ok, double vmax_ok,
                       bool accepted,
                       // ★ 2026-11-25 追加：A* 路径的最差点 + 简化前的原始路径
                       const Eigen::Vector3d* astar_worst = nullptr,
                       double astar_worst_clr = -1.0,
                       const std::vector<Eigen::Vector3d>* raw_before_simplify = nullptr) {
        if (np_.minco_dump_dir.empty()) return;
        static int seq = 0;
        const std::string dir = np_.minco_dump_dir;
        ::mkdir(dir.c_str(), 0755);   // 已存在则失败，可忽略

        char fn[512];
        std::snprintf(fn, sizeof(fn), "%s/minco_%05d_t%.3f.txt",
                      dir.c_str(), ++seq, ros::Time::now().toSec());
        std::ofstream f(fn);
        if (!f) {
            ROS_WARN_STREAM_THROTTLE(2.0, "[azj] cannot write dump: " << fn);
            return;
        }
        f << std::fixed << std::setprecision(6);

        f << "# azj MINCO dump  seq=" << seq << "  accepted=" << (accepted ? 1 : 0) << "\n";
        f << "# wall=" << std::setprecision(9) << ros::Time::now().toSec()
          << std::setprecision(6) << "\n";

        // ---- 起点/终点状态 ----
        f << "[head]\n";
        f << "p " << head.p.transpose() << "  clr " << field.clearance(head.p) << "\n";
        f << "v " << head.v.transpose() << "\n";
        f << "a " << head.a.transpose() << "\n";
        f << "j " << head.j.transpose() << "\n";
        f << "[tail]\n";
        f << "p " << tail.p.transpose() << "  clr " << field.clearance(tail.p) << "\n";
        f << "v " << tail.v.transpose() << "\n";

        // ---- 参数 ----
        f << "[params]\n";
        f << "w_mid " << mp.w_mid << "  w_obs " << mp.w_obs
          << "  w_smooth " << mp.w_smooth << "  w_time " << mp.w_time
          << "  w_anchor " << mp.w_anchor << "\n";
        f << "max_vel " << mp.max_vel << "  max_acc " << mp.max_acc
          << "  safe_margin " << mp.safe_margin << "\n";
        f << "iterations " << mp.iterations << "  pieces " << mp.pieces
          << "  piece_len " << mp.piece_len << "\n";
        f << "lock_z " << (mp.lock_z ? 1 : 0) << "  lock_z_value "
          << mp.lock_z_value << "  z_bounds " << (mp.enable_z_bounds ? 1 : 0)
          << "  z[" << mp.z_min << "," << mp.z_max << "]\n";
        f << "stop_at_end " << (mp.stop_at_end ? 1 : 0) << "\n";

        // ---- 校验 ----
        f << "[check]\n";
        f << "mc " << mc_check << "  min_ok " << min_ok
          << "  vmax_ok " << vmax_ok << "  accepted " << (accepted ? 1 : 0) << "\n";

        // ---- 输入路径（A* 给的，逐点带 clearance）----
        f << "[in_path]  n=" << in_path.size() << "\n";
        double in_min = 1e9;
        for (size_t i = 0; i < in_path.size(); ++i) {
            const double c = field.clearance(in_path[i]);
            in_min = std::min(in_min, c);
            f << "  " << i << " " << in_path[i].transpose() << "  clr " << c << "\n";
        }
        f << "[in_path_min_clr] " << in_min << "\n";

        // ---- 输出轨迹（MINCO 结果）----
        if (res && res->success) {
            f << "[out_traj]  ok=1 len=" << res->length
              << " dur=" << res->duration
              << " min_clr=" << res->min_clearance
              << " vmax=" << res->max_vel
              << " amax=" << res->max_acc
              << " iters=" << res->iterations << "\n";
            double out_min = 1e9;
            Eigen::Vector3d wpt(0, 0, 0);
            for (double t = 0.0; t <= res->duration + 1e-9; t += 0.02) {
                Eigen::Vector3d p, v, a;
                if (!res->traj.sample(std::min(t, res->duration), p, v, a)) continue;
                const double c = field.clearance(p);
                if (c < out_min) { out_min = c; wpt = p; }
                f << "  " << t << " " << p.transpose()
                  << "  clr " << c << "  |v| " << v.norm() << "\n";
            }
            f << "[out_traj_worst] " << wpt.transpose() << "  clr " << out_min << "\n";
        } else {
            f << "[out_traj]  ok=0  msg="
              << (res ? res->msg : std::string("optimize not called")) << "\n";
        }

        // ══════════════════════════════════════════════════════════════
        //  ★ A* 路径最差点诊断（2026-11-25）
        // ══════════════════════════════════════════════════════════════
        //
        // 要回答的矛盾：A* 内部按 robot_radius=0.30 硬约束搜路，
        // 事后用同一个 field 沿路径采样却算出 0.0088。
        // 二者必有一错。这里把最差点钉死，并做三件事区分成因：
        //
        //   ① 打出最差点坐标 + 该点 ±3cm 的 clearance 分布
        //      → 判断是否落在障碍边界（插值/离散化误差）
        //   ② 打出**简化前**的原始路径（A* 栅格级，间距 0.1m）
        //      → 若原始路径也在该点附近且 clr 很低：A* 本身漏检
        //      → 若原始路径在那段有密集点而 clr 正常：simplifyPath 删错了
        //   ③ 统计原始路径与简化路径各自的 min_clr
        if (astar_worst && astar_worst_clr >= -0.5) {
            f << "[astar_worst] " << astar_worst->transpose()
              << "  clr " << astar_worst_clr << "\n";
            // 局部 ±3cm 网格（步长 1cm），看是不是卡在障碍表面
            f << "[astar_worst_local]\n";
            const double zq = plan_2d_active_ ? plan_2d_z_active_ : astar_worst->z();
            for (int ix = -3; ix <= 3; ++ix) {
                for (int iy = -3; iy <= 3; ++iy) {
                    const Eigen::Vector3d p(astar_worst->x() + 0.01 * ix,
                                            astar_worst->y() + 0.01 * iy, zq);
                    const double c = field.clearance(p);
                    if (!std::isfinite(c)) continue;
                    f << "L " << ix << " " << iy << " " << c << "\n";
                }
            }
        }

        if (raw_before_simplify && raw_before_simplify->size() >= 2) {
            const auto& rp = *raw_before_simplify;
            double raw_min = 1e9;
            Eigen::Vector3d raw_worst(0, 0, 0);
            for (const auto& q : rp) {
                const double c = field.clearance(q);
                if (c < raw_min) { raw_min = c; raw_worst = q; }
            }
            f << "[raw_path]  n=" << rp.size() << "  min_clr=" << raw_min
              << "  worst " << raw_worst.transpose() << "\n";
            for (size_t i = 0; i < rp.size(); ++i) {
                f << "  " << i << " " << rp[i].transpose()
                  << "  clr " << field.clearance(rp[i]) << "\n";
            }
            // 对比：简化前后
            double sim_min = 1e9;
            for (const auto& q : in_path) sim_min = std::min(sim_min, field.clearance(q));
            f << "[simplify_diff] raw_n=" << rp.size()
              << " sim_n=" << in_path.size()
              << " raw_min_clr=" << raw_min
              << " sim_min_clr=" << sim_min
              << " delta=" << (sim_min - raw_min) << "\n";
            if (sim_min < raw_min - 1e-3) {
                f << "# !! simplifyPath 让 clearance 变差了 —— 查删点逻辑\n";
            } else {
                f << "# simplifyPath 未使 clearance 变差（差值>=0）\n";
            }
        }

        // ---- 环境点云快照（只取路径附近，控制文件大小）----
        //
        // 这是最关键的一段：没有它就无法离线复现（点云随时间变化，
        // 而 bag 里只有 inf_occ 的最后一帧）。半径取 3m 覆盖整条路径周边。
        {
            Eigen::Vector3d lo = head.p, hi = head.p;
            for (const auto& q : in_path) {
                lo = lo.cwiseMin(q);
                hi = hi.cwiseMax(q);
            }
            const double R = 3.0;
            f << "[env_grid]\n";
            f << "bbox " << (lo.x() - R) << " " << (lo.y() - R) << " " << (lo.z() - 0.2)
              << " " << (hi.x() + R) << " " << (hi.y() + R) << " " << (hi.z() + 0.2) << "\n";
            // 以 0.10m 网格采样 clearance（与 ESDF 分辨率一致）
            // 点数控制在 ~(8/0.1)*(8/0.1) = 6400 per layer，单层即可
            const double zq = plan_2d_active_ ? plan_2d_z_active_ : head.p.z();
            f << "z " << zq << "\n";
            int cnt = 0;
            for (double x = lo.x() - R; x <= hi.x() + R; x += 0.10) {
                for (double y = lo.y() - R; y <= hi.y() + R; y += 0.10) {
                    const double c = field.clearance(Eigen::Vector3d(x, y, zq));
                    if (!std::isfinite(c)) continue;
                    if (c > 1.5) continue;   // 只记障碍附近，省空间
                    f << "g " << x << " " << y << " " << c << "\n";
                    ++cnt;
                }
            }
            f << "[env_grid_n] " << cnt << "\n";
        }
        f.close();
        ROS_INFO_STREAM_THROTTLE(5.0, "[azj] MINCO dump -> " << fn);
    }

    bool buildTrajectoryMinco(const std::vector<Eigen::Vector3d>& path, const StatePVAJ& head,
                              const StatePVAJ& tail, bool stop_at_end,
                              double vmax_allow = -1.0,
                              // 简化前的原始路径（仅用于 dump 诊断，可空）
                              const std::vector<Eigen::Vector3d>* raw_path = nullptr) {
        if (path.size() < 2) return false;

        // 传给 dump 的引用（空指针时给一个空 vector，保持调用简单）
        static const std::vector<Eigen::Vector3d> kEmptyPath;
        const std::vector<Eigen::Vector3d>& raw_path_ref =
            raw_path ? *raw_path : kEmptyPath;

        MincoOptParams mp;
        mp.safe_margin = sp_.robot_radius;
        mp.clearance_ref = sp_.clearance_ref;
        mp.clearance_cap = sp_.clearance_cap;
        mp.alpha = sp_.alpha;
        mp.w_mid = np_.minco_w_mid;
        mp.w_obs = np_.minco_w_obs;
        mp.w_smooth = np_.minco_w_smooth;
        // ★ 速度上限按 lag 衰减（2026-11-17，治"落后时参考照样跑满"）
        //
        // 只限制起点速度 v0 是不够的：MINCO 轨迹中段仍会优化到 max_vel，
        // 于是飞机落后时参考依然以满速跑 → lag 只是"不再恶化"，不会收敛。
        // 实测 lag 稳定在 0.87m 就是这个原因。
        //
        // 现在把整条轨迹的速度上限也降下来：飞机才有相对余量追上去。
        mp.max_vel = (vmax_allow > 1e-3) ? std::min(tp_.max_vel, vmax_allow)
                                        : tp_.max_vel;
        // rolling 段用"可跟踪加速度"，让轨迹本身落在 PX4 能跟上的范围内
        // （见 NodeParams::rolling_track_acc 的实测数据）
        mp.max_acc = (np_.rolling_track_acc > 1e-3) ? np_.rolling_track_acc : tp_.max_acc;
        mp.iterations = np_.minco_iterations;
        mp.stop_at_end = stop_at_end;
        mp.lock_z = plan_2d_active_;
        mp.lock_z_value = plan_2d_z_active_;
        if (np_.enforce_height_limits) {
            mp.enable_z_bounds = true;
            mp.z_min = np_.floor_z;
            mp.z_max = np_.ceil_z;
        }

        // ★★★ 长直线段插点（2026-11-25 实飞定位的根因）
        //
        // 传进来的 path 经过 simplifyPath，长直线只剩两端点。
        // MINCO 的 piece_len 决定内点密度，1.2m 段只 3 个点 →
        // 配 36° 转折 → 7 次多项式过冲 → vmax 4.47 / amax 20.4 → 被拒。
        //
        // 插到 ≤ piece_len（默认 0.40m），保证内点密度均匀。
        const std::vector<Eigen::Vector3d> path_dense =
            densifyPath(path, std::max(0.10, mp.piece_len));
        const std::vector<Eigen::Vector3d>& path_eff = path_dense;

        // 曲线场：与路径搜索用同一个（2D 模式自动带柱状投影）
        const ClearanceField::Ptr field = makeField(plan_2d_z_active_, head.p.z());
        const ros::WallTime t0 = ros::WallTime::now();
        MincoOptimizer opt;
        MincoOptResult r = opt.optimize(head, tail, path_eff, *field, mp);
        const double ms_first = (ros::WallTime::now() - t0).toSec() * 1e3;

        // ══════════════════════════════════════════════════════════════════
        //  ★★★ 安全不达标时**递进加重安全项重试**（2026-11-25）
        // ══════════════════════════════════════════════════════════════════
        //
        // 事故（/tmp/azj_planner.log，实飞）：
        //   t=43.85~44.96 连续 12 次 MINCO 拒绝，min_clr 在 0.002~0.008 之间，
        //   每次都"放弃 keep_old → B 样条"，B 样条也失败 → 落成折线 1.1s。
        //
        //   那些轨迹的形状是对的（len 4.2~5.0m、seg 4.3m），只是 clearance
        //   差了 0.26m。直接拒绝太浪费 —— B 样条分支早有这个机制
        //   （`w_obs ×10 → ×50` 递进重试），MINCO 一直没有。
        //
        // 现在补上：不达标就把 w_obs 加大重来，最多 2 次（×10、×50）。
        // 每次只多花 3~10ms（MINCO 单次实测 1~12ms），远小于"退回折线"
        // 带来的 1 秒抖动。
        {
            const double min_ok0 = sp_.robot_radius * 0.9;
            const double vmax_ok0 = tp_.max_vel * 1.25;
            for (const double boost : {10.0, 50.0}) {
                if (!r.success) break;
                const bool need_more =
                    (r.min_clearance < min_ok0) || (r.max_vel > vmax_ok0);
                if (!need_more) break;
                MincoOptParams mp2 = mp;
                mp2.w_obs = mp.w_obs * boost;
                mp2.iterations = std::max(mp.iterations, 80);
                mp2.projection_iters = std::max(mp.projection_iters, 4);
                mp2.projection_samples = std::max(mp.projection_samples, 400);
                MincoOptResult r2 = opt.optimize(head, tail, path_eff, *field, mp2);
                if (r2.success && r2.min_clearance > r.min_clearance) {
                    ROS_INFO_STREAM("[azj] MINCO retry w_obs x" << boost << ": min_clr "
                                    << r.min_clearance << " -> " << r2.min_clearance
                                    << "  vmax " << r.max_vel << " -> " << r2.max_vel);
                    // MincoOptResult 含不可拷贝成员（MincoS4NU 持裸指针的
                    // BandedSystem）→ 必须 move
                    r = std::move(r2);
                }
            }
        }
        const double ms = (ros::WallTime::now() - t0).toSec() * 1e3;

        if (!r.success) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] MINCO failed: " << r.msg);
            // 此处 astar_min_clr 还没算（在下方的校验块里），只 dump 基本内容
            dumpMincoCall(*field, path_eff, head, tail, mp, &r, 0.0, 0.0, 0.0, false,
                          nullptr, -1.0, &raw_path_ref);
            return false;
        }

        // ══════════════════════════════════════════════════════════════════
        //  ★★★ MINCO 输出安全校验（2026-11-17 严重 bug 修复）
        // ══════════════════════════════════════════════════════════════════
        //
        // 事故（/tmp/azj_flight_20261117_181457，用户报"控制器追不上"）：
        //
        //   [57.6] goal clearance=0.22m < robot_radius=0.3m at (16.65,1.65,0.85)
        //          ← 目标点落在障碍膨胀层内
        //   [59.2] MINCO ok: len=4.73m dur=1.26s min_clr=2.0e-14m vmax=7.23
        //   [60.2] MINCO ok: len=6.14m dur=1.38s min_clr=2.7e-14m vmax=10.27
        //
        // 三个问题同时出现：
        //   ① min_clr ≈ 0 —— 曲线**穿过了障碍**（2e-14 就是 0）
        //   ② vmax 7~10 m/s —— 超 max_vel(1.0) 十倍，飞机根本追不上
        //   ③ len 4.7~6.1m 而 seg 只有 0.76~1.09m —— 为了"居中"绕远 6 倍
        //
        // 根因是两件事叠加：
        //   · 目标贴障碍时，w_mid(30) 的居中收益远大于 w_anchor(0.10) 的
        //     保形力 → MINCO 宁可把路径绕到 6m 外去追求更大 clearance
        //   · 绕远后时间优化把 T 压到 1.26s（要"最短时间"）→ 平均 3.7 m/s
        //     → 7 次多项式系数爆到 1e+ 量级 → 速度曲线数值爆炸
        //
        // 而 B 样条分支有 `mc >= robot_radius*0.9` 的校验，MINCO 分支**没有**
        // → 这种轨迹被直接发布出去了。
        //
        // 这里补上同样的校验，并且**再加一条速度校验**（MINCO 的 max_vel 是
        // 软约束，w_vel 可能压不住时间优化的收益）。
        // 校验不过就返回 false → 调用方回退到 B 样条分支（那条是安全的）。
        // 段长（三道校验都要用，先算出来）
        const double seg_len_chk = [&] {
            double l = 0.0;
            for (size_t i = 1; i < path_eff.size(); ++i)
                l += (path_eff[i] - path_eff[i - 1]).norm();
            return l;
        }();
        {
            // ★★★ 校验阈值必须按**A* 路径的实测值**定，不能固定 robot_radius*0.9
            //     （2026-11-25 实飞定位的根因）
            //
            // 实测（/home/spadea/azj_flights/flight_20261125_171737）：
            //   场景是密集柱阵，S(-1.69,-0.05) → G(-0.15,-1.45) 的直线
            //   **宽度只有 0.10m**（机器人直径 0.60m），必须绕行。
            //   实测该场景 A* 的物理上限：
            //     robot_radius=0.30 → 能过，路径 min_clr 最高 **0.318m**
            //     robot_radius=0.36 → 无解（FAIL）
            //     robot_radius=0.42 → 无解（FAIL）
            //   也就是说 A* 已经把路径做到几何极限了。
            //
            //   而 MINCO 的 7 次多项式平滑**必然切角**，0.318 → 0.003，
            //   低于固定阈值 0.27 → 连续 18 次拒绝 → 折线 3.1 秒。
            //   这不是 MINCO 的错，是"要求平滑后还达到 0.27"在这个几何下无解。
            //
            // 正确语义：
            //   下限 = min(robot_radius*0.9, A*路径实测min_clr*0.85)
            //   即"不要求比 A* 更好"，只要求"不明显比 A* 更差"。
            //   A* 路径本身就是硬约束搜出来的，它代表该场景的可行极限。
            //   安全底线仍由 A* 的 robot_radius 保证（0.30m），
            //   MINCO 平滑后允许略低，但不得低于 85% 的 A* 水平。
            const double min_ok_hard = sp_.robot_radius * 0.9;
            double astar_min_clr = std::numeric_limits<double>::max();
            Eigen::Vector3d astar_worst_pt(0, 0, 0);
            {
                // 沿 path 密集采样（path 是 A* 简化后的折线）
                for (size_t i = 1; i < path_eff.size(); ++i) {
                    const Eigen::Vector3d& a = path_eff[i - 1];
                    const Eigen::Vector3d& b = path_eff[i];
                    const double len = (b - a).norm();
                    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.02)));
                    for (int k = 0; k <= n; ++k) {
                        const double u = static_cast<double>(k) / n;
                        const Eigen::Vector3d q = a + u * (b - a);
                        const double c = field->clearance(q);
                        if (c < astar_min_clr) {
                            astar_min_clr = c;
                            astar_worst_pt = q;
                        }
                    }
                }
            }
            if (!(astar_min_clr < 1e8)) astar_min_clr = min_ok_hard;
            // 允许 MINCO 比 A* 略低（平滑切角不可避免），但不低于 85%
            const double min_ok = std::min(min_ok_hard, astar_min_clr * 0.85);
            const double vmax_ok = tp_.max_vel * 1.25;
            // 排除末端一小段再判 clearance：目标点本身就贴障碍时，
            // 末段必然靠近它（与全量流程的 minClearanceAlongPathExcludingEnd 同义）
            std::vector<Eigen::Vector3d> pts;
            pts.reserve(64);
            for (double tt = 0.0; tt <= r.duration + 1e-9; tt += 0.02) {
                Eigen::Vector3d pp, vv, aa;
                if (r.traj.sample(tt, pp, vv, aa)) pts.push_back(pp);
            }
            const double mc = pts.size() >= 2
                                  ? minClearanceAlongPathExcludingEnd(
                                        pts, *field, sp_.robot_radius + 0.3, 0.02)
                                  : 0.0;
            // 速度超限比例：末段到点减速时允许短暂超一点
            if (mc < min_ok) {
                // 定位第一个越界点：这对判断"是层高/z 出界，还是真的贴障碍"至关重要
                double zmin = 1e9, zmax = -1e9;
                Eigen::Vector3d worst(0, 0, 0);
                double worst_c = 1e9;
                for (const auto& q : pts) {
                    zmin = std::min(zmin, q.z());
                    zmax = std::max(zmax, q.z());
                    const double c = field->clearance(q);
                    if (c < worst_c) { worst_c = c; worst = q; }
                }
                ROS_ERROR_STREAM(
                    "[azj] MINCO_REJECT reason=clearance min_clr=" << mc << "m < " << min_ok
                        << "m | len=" << r.length << "m seg=" << seg_len_chk << "m dur="
                        << r.duration << "s | z=[" << zmin << "," << zmax << "] ceil_z="
                        << np_.ceil_z << " | start_clr=" << field->clearance(head.p)
                        << " goal_clr=" << field->clearance(tail.p) << " | worst=("
                        << worst.transpose() << ") c=" << worst_c << " | start=("
                        << head.p.transpose() << ") goal=(" << tail.p.transpose() << ")");
                dumpMincoCall(*field, path_eff, head, tail, mp, &r, mc, min_ok,
                              vmax_ok, false, &astar_worst_pt, astar_min_clr,
                              &raw_path_ref);
                return false;
            }
            if (r.max_vel > vmax_ok) {
                ROS_ERROR_STREAM_THROTTLE(
                    0.5, "[azj] MINCO_REJECT reason=vmax vmax=" << r.max_vel << " > "
                             << vmax_ok << " m/s | len=" << r.length << "m dur="
                             << r.duration << "s seg=" << seg_len_chk << "m");
                dumpMincoCall(*field, path_eff, head, tail, mp, &r, mc, min_ok,
                              vmax_ok, false, &astar_worst_pt, astar_min_clr,
                              &raw_path_ref);
                return false;
            }
            // 绕远保护：优化后的长度不该超过段长的 2.5 倍
            if (seg_len_chk > 0.5 && r.length > seg_len_chk * 2.5) {
                ROS_ERROR_STREAM_THROTTLE(
                    0.5, "[azj] MINCO_REJECT reason=over_centering len=" << r.length
                             << "m > 2.5*seg=" << (seg_len_chk * 2.5) << "m (seg="
                             << seg_len_chk << "m) min_clr=" << r.min_clearance
                             << "m | goal_clr=" << field->clearance(tail.p));
                dumpMincoCall(*field, path_eff, head, tail, mp, &r, mc, min_ok,
                              vmax_ok, false, &astar_worst_pt, astar_min_clr,
                              &raw_path_ref);
                return false;
            }

            // 通过所有校验 → dump（accepted=1）供离线复核
            dumpMincoCall(*field, path_eff, head, tail, mp, &r, mc, min_ok,
                          vmax_ok, true, &astar_worst_pt, astar_min_clr,
                          &raw_path_ref);
        }
        // 诊断：轨迹前 1.2s 的速度分布（判断"起步是否过猛"）
        {
            char buf[256];
            int n = 0;
            buf[0] = 0;
            Eigen::Vector3d pp, vv, aa;
            for (double tt = 0.0; tt <= 1.2001 && n < 6; tt += 0.2, ++n) {
                if (!r.traj.sample(tt, pp, vv, aa)) break;
                char one[40];
                std::snprintf(one, sizeof(one), " %.2f:%.2f", tt, vv.norm());
                std::strncat(buf, one, sizeof(buf) - std::strlen(buf) - 1);
            }
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] MINCO ok: " << ms << "ms it=" << r.iterations << " len="
                                        << r.length << "m dur=" << r.duration << "s min_clr="
                                        << r.min_clearance << "m vmax=" << r.max_vel
                                        << " | v(t)=" << buf);
        }
        return commitTrajectory(std::move(r.traj));
    }

    bool buildTrajectoryImpl(const std::vector<Eigen::Vector3d>& path, TrajParams& tp,
                             const Eigen::Vector3d* start_p = nullptr,
                             const Eigen::Vector3d* start_v = nullptr) {
        // 起点速度：传**矢量**而不是标量。
        //
        // 标量的做法是 |v_current| × 新路径切向，会在重规划瞬间凭空生成
        // 竖直速度：飞机水平飞 0.8m/s、新路径起点切线朝下 45° 时，
        // 会给出 vz=-0.57 m/s → 控制器照做 → **瞬间掉高**（用户报的现象）。
        //
        // 传矢量后，trajectory 会把真实速度投影到新路径切向，
        // 只保留沿路径的推进分量，不製造与路径不符的竖直/横向速度。
        // ★ 初始速度要按"与路径起点的方向一致性"折扣（2026-11-14）。
        //
        // buildTrajectoryImpl 里 trajectory 会把 v_start_vec 投影到路径切向，
        // 但**投影只保留沿路径分量** —— 如果真实速度几乎垂直于路径，
        // 投影结果接近 0（这是对的）；如果真实速度与路径夹角 150°，
        // 投影会得到一个**沿路径反向**的大速度，轨迹就会要求飞机倒飞。
        //
        // 所以这里先做方向检查：
        //   夹角 <= 90° → 正常使用（投影后是前进分量）
        //   夹角 >  90° → 速度几乎与路径相反，把大小压到 30%，
        //                 让轨迹从低速起步，靠控制器慢慢转过去，
        //                 而不是要求它带着 1 m/s 反向冲。
        Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
        if (start_v) {
            v0 = *start_v;                       // 参考连续：用旧轨迹的速度
        } else if (vel_received_ || robot_state_.rcv) {
            v0 = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        }
        if (v0.head<2>().norm() > 1e-6 && path.size() >= 2) {
            const Eigen::Vector3d d0 = path[1] - path[0];
            if (d0.head<2>().norm() > 1e-6) {
                const double cosang = std::max(-1.0, std::min(1.0,
                    v0.head<2>().normalized().dot(d0.head<2>().normalized())));
                if (cosang < 0.0) {
                    const double ang_deg = std::acos(cosang) * 180.0 / M_PI;
                    v0 *= 0.30;
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] initial velocity vs path angle " << ang_deg
                             << "deg (>90) - scaling v_start to "
                             << v0.norm() << " m/s to avoid a reverse step");
                }
            }
        }
        if (v0.norm() > 1e-6) {
            tp.v_start_vec = v0;
            tp.has_v_start_vec = true;
            tp.v_start = v0.norm();
        } else {
            tp.v_start = 0.0;
        }
        tp.v_start = std::min(tp.v_start, tp.max_vel);
        // ★ 不再硬写 stop_at_end=true（2026-11-17）。
        //
        // 滚动窗口（rolling）的末点只是「前方 lookahead 米的点」，不是终点：
        // 若还按 stop_at_end=true 构建，轨迹会在局部目标处刹到 0
        // → 就是用户看到的"重规划后中途减速一下"。
        // 现在由调用方决定：常规全量规划传 true（到 goal 悬停），
        // rolling 局部段传 false（匀速穿过末点，下一周期继续向前滚）。
        // 默认值仍是 true（TrajParams::stop_at_end），旧行为不变。

        // ★ 先在**局部**构建（不持锁）：build 要做速度规划，耗时可能在毫秒级。
        //   若持锁构建，发布线程会被阻塞 → 又回到"断流"的老问题。
        auto snap = std::make_shared<TrajSnapshot>();
        bool built = false;
        if (start_p) {
            // ★ 优先用 Hermite（起点速度是完整矢量 → 能边走边转弯）
            // 见 buildSmoothFromState 的注释：这是对齐 SUPER/MINCO 的关键。
            built = snap->traj.buildSmoothFromState(*start_p, v0, path, tp);
            if (!built) {
                // 兜底：退回 TOPP
                built = snap->traj.buildFromState(*start_p, v0, path, tp);
            }
        } else {
            built = snap->traj.build(path, tp);
        }
        if (!built) return false;

        // 记下生成轨迹那一刻的实际速度，供输出淡入用
        snap->v0 = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        snap->start = ros::Time::now();
        snap->id = ++trajectory_id_;

        // 生成预规划 yaw 曲线（照 SUPER 的 yaw_traj_opt 做法）。
        // start_yaw 传当前机头：让曲线的起点与飞机实际朝向连续，
        // 避免轨迹一生成就要求转一个大角度（见 buildYawProfile 里的说明）。
        snap->yaw_prof = buildYawProfile(
            snap->traj, 0.5, -1.0,
            robot_state_.rcv ? robot_state_.yaw
                             : std::numeric_limits<double>::quiet_NaN());

        // 原子换上新快照。发布线程要么看到旧轨迹、要么看到新轨迹，
        // 绝不会看到半个（shared_ptr 赋值是原子的引用计数交换 + 指针写入，
        // 且这里持锁）。
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            active_ = std::move(snap);
            has_trajectory_ = true;
        }
        return true;
    }

    /// 生成一条**可控刹车轨迹**并原子换上新快照（2026-11-14，学 SUPER backup）。
    ///
    /// 与"停发参考让控制器急刹"的区别：
    ///   · 停发参考 → 控制器等 0.5s traj_timeout → PX4 位置环急刹 → 姿态抖
    ///   · 本函数   → 一条带速度规划的匀减速轨迹，控制器平滑跟踪
    ///
    /// 刹车方向 = 当前速度方向，减速度 = max_acc × 0.5（留控制余量）。
    /// 刹车距离 v²/(2a)：1 m/s 时约 0.33m。
    ///
    /// @return false 表示速度已很小（无需刹车）或构建失败
    bool tryBrakeReplan(const std::string& reason) {
        if (!np_.backup_enable) return false;
        if (!(vel_received_ || robot_state_.rcv)) return false;

        const Eigen::Vector3d bp = robot_state_.p;
        const Eigen::Vector3d bv = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        const double v = bv.norm();
        if (v < np_.backup_min_speed) {
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] brake skipped (v=" << v << " m/s < " << np_.backup_min_speed
                     << ") - too slow to matter, reason: " << reason);
            return false;
        }

        TrajParams tp = tp_;
        PathTrajectory brake;
        if (!PathTrajectory::buildBrake(bp, bv, tp, brake)) return false;

        auto snap = std::make_shared<TrajSnapshot>();
        snap->traj = brake;
        snap->start = ros::Time::now();
        snap->id = ++trajectory_id_;
        snap->v0 = bv;
        // 整条轨迹都是刹车段
        snap->backup_start = 0.0;
        snap->yaw_prof = buildYawProfile(
            snap->traj, 0.5, -1.0,
            robot_state_.rcv ? robot_state_.yaw
                             : std::numeric_limits<double>::quiet_NaN());

        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            active_ = std::move(snap);
            has_trajectory_ = true;
        }
        ROS_WARN_STREAM("[azj] CONTROLLED BRAKE engaged (reason: " << reason
                        << "): v=" << v << " m/s, dist=" << brake.duration()
                        << "s - tracking a planned deceleration instead of letting "
                        << "the controller time out");
        return true;
    }

    /// 取当前轨迹快照（持锁时间 = 一次 shared_ptr 拷贝，纳秒级）。
    /// 返回后**不再持锁**使用，调用方拿到的对象是不可变的。
    std::shared_ptr<const TrajSnapshot> snapTraj() const {
        std::lock_guard<std::mutex> lk(state_mtx_);
        return active_;
    }

    /// 沿轨迹预先规划 yaw 曲线。
    ///
    /// 直接对应 SUPER 的 `YawTrajOpt::getYawWaypointAllocation`
    /// （super_planner/src/traj_opt/yaw_traj_opt.cpp:55~90）：
    ///   对每个 waypoint，取 `pos(t) → pos(t + lookahead)` 的方向作 yaw。
    ///   前视的作用是把"绕柱时的瞬时侧移"平均掉，得到宏观飞行方向；
    ///   若只看瞬时速度，柱林里路径左右摆时机头就一直跟着摆。
    ///
    /// 与 SUPER 的差异：它用 minimumJerk 多项式，这里用 0.1s 间隔的
    /// 平滑插值（smoothstep）。实测 50Hz 下两者差别可以忽略，
    /// 而实现简单、无需求解器。
    YawProfile buildYawProfile(const PathTrajectory& traj,
                               double lookahead = 0.5,
                               double step = -1.0,
                               double start_yaw = std::numeric_limits<double>::quiet_NaN()) const {
        YawProfile prof;
        const double dur = traj.duration();
        if (!(dur > 1e-3)) return prof;

        // ★ waypoint 间隔必须**很稀疏** —— 这是 SUPER 不掉高的真正原因。
        //
        // SUPER 的 getYawTimeAllocation（yaw_traj_opt.cpp:29~51）：
        //     double interp_dt = M_PI / yaw_dot_max_;    // π/1.5 ≈ 2.09 秒
        //     times(i+1) = interp_t;                     // 每段 2.09s
        // 也就是说 **每 2.09 秒才取一个 yaw waypoint**，一条 7 秒的轨迹
        // 只有 3 个中间 waypoint；再用 5 阶 minimumJerk 多项式插值。
        //
        // 效果：yaw 参考是个**慢变量**，几秒内基本不变 → PX4 不需要
        // 持续输出偏航力矩 → 电机差速不占行程 → 推力充足 → 不掉高。
        //
        // 我第一版用 step=0.1s（每 0.1 秒一个点），yaw 参考仍在 0.1s 粒度
        // 上跟着路径摆动 → 持续跟踪误差 → 照样掉高。
        //
        // 这里直接采用 SUPER 的公式：step = π / yaw_dot_max，
        // 且至少 1.0s（防止 yaw_rate_max 设很大时间隔过小）。
        if (step <= 0.0) {
            // ★ 用**独立的** yaw_profile_dt，不再借用 yaw_rate_max。
            //
            // 为什么必须拆开（2026-11-09）：
            //   yaw_rate_max 有两个语义完全不同的用途：
            //     ① computeYaw 里每帧最大转角 —— 必须 <= PX4 能力(0.785)
            //     ② 这里决定 waypoint 稀疏度 —— SUPER 用 π/1.5 = 2.09s
            //   两者绑在一起时无法同时满足：0.6 rad/s 会让间隔变成 5.2s
            //   （一条 8 秒轨迹只有 1~2 个 waypoint，yaw 参考生硬）；
            //   而 1.5 rad/s 又超过 PX4 能力（掉高）。
            //
            // 现在 ① 用 yaw_rate_max（钳到 0.6），② 用 yaw_profile_dt（2.1s，
            // 与 SUPER 的 interp_dt 一致）。语义清晰、互不牵制。
            step = np_.yaw_profile_dt > 0.05 ? np_.yaw_profile_dt : 2.1;
        }

        auto posAt = [&](double t, Eigen::Vector3d& p) -> bool {
            Eigen::Vector3d v, a;
            return traj.sample(t, p, v, a);
        };

        const int N = std::max(2, static_cast<int>(std::ceil(dur / step)) + 1);
        prof.t.reserve(N);
        prof.yaw.reserve(N);

        double last = std::numeric_limits<double>::quiet_NaN();
        for (int k = 0; k < N; ++k) {
            const double t = std::min(dur, k * step);
            Eigen::Vector3d p0, p1;
            if (!posAt(t, p0)) continue;

            // 前视：末端附近自动收缩，避免读到越界值
            const double t1 = std::min(dur, t + lookahead);
            const double t0 = (t1 - t < 0.2) ? std::max(0.0, dur - lookahead) : t;
            if (!posAt(t1, p1) || !posAt(t0, p0)) continue;

            const Eigen::Vector3d d = p1 - p0;
            if (std::hypot(d.x(), d.y()) < 0.05) {
                // 方向不可靠（几乎原地）：沿用上一个 yaw
                if (!std::isnan(last)) { prof.t.push_back(t); prof.yaw.push_back(last); }
                continue;
            }
            double y = std::atan2(d.y(), d.x());
            // unwrap 成连续序列：与上一个差超过 pi 就补 2pi
            // （等价于 SUPER 的 normalizeNextYaw）
            if (!std::isnan(last)) {
                while (y - last > M_PI) y -= 2.0 * M_PI;
                while (y - last < -M_PI) y += 2.0 * M_PI;
            }
            last = y;
            prof.t.push_back(t);
            prof.yaw.push_back(y);
        }

        // ══════════════════════════════════════════════════════════════
        // ★ 速率预限幅 + 起点接续（2026-11-14）
        //
        // ① 第一个 waypoint 的 yaw 接到**当前机头**。
        //    原来 profile[0] = “轨迹起点前视 0.5s 的方向”，与当前机头可能
        //    相差 80°（yaw1 bag：0.21 → 1.57 rad）。computeYaw 只能以
        //    yaw_rate_max 硬追，整个飞行段都在饱和输出偏航力矩。
        //    接到当前机头后，参考从“现在”出发，转不过去就慢慢转，
        //    不会一上轨迹就要求大角度。
        //
        // ② 相邻 waypoint 的 yaw 变化按 yaw_rate_max 预限幅。
        //    保证参考曲线**自身**的斜率不超过 PX4 能跟的水平；
        //    computeYaw 的逐帧限幅就不会长期顶在饱和边界。
        //    注意：smoothstep 的峰值斜率是平均斜率的 1.5 倍，这里约束的是
        //    平均斜率，峰值由 computeYaw 兜住。
        // ══════════════════════════════════════════════════════════════
        if (prof.t.size() >= 2) {
            // 预限幅用与 computeYaw 一致的速率：飞行段取 yaw_rate_flight，
            // 否则曲线斜率超过实际允许值，指令会长期顶在饱和边界。
            double rate_cap = np_.yaw_rate_max;
            if (np_.yaw_rate_flight > 1e-6) {
                rate_cap = std::min(rate_cap, np_.yaw_rate_flight);
            }
            const double rate = std::max(1e-3, rate_cap);
            if (!std::isnan(start_yaw)) {
                double y = start_yaw;
                while (y - prof.yaw[0] > M_PI) y -= 2.0 * M_PI;
                while (y - prof.yaw[0] < -M_PI) y += 2.0 * M_PI;
                prof.yaw[0] = y;
            }
            for (size_t k = 1; k < prof.t.size(); ++k) {
                const double dt_k = std::max(1e-3, prof.t[k] - prof.t[k - 1]);
                const double max_d = rate * dt_k;
                const double d = prof.yaw[k] - prof.yaw[k - 1];
                if (std::abs(d) > max_d) {
                    prof.yaw[k] = prof.yaw[k - 1] + (d > 0.0 ? max_d : -max_d);
                }
            }
        }
        return prof;
    }

    /// 是否需要重规划。
    ///
    /// 为什么不能每帧无条件重规划：重规划会把轨迹时间基准重置为 0，
    /// 于是参考点永远停在"新轨迹的前 0.2s"，而轨迹前段速度又受 v_start 限制，
    /// 一旦速度反馈滞后，参考速度就永远上不去（实测 max|v| 卡在 0.03 m/s）。
    /// 正确做法是让每条轨迹**完整执行**，只在必要时重建。
    /// 重规划原因。区分原因是为了让**安全触发的重规划能绕过退避**（见 v1.3.2）。
    enum class ReplanReason {
        NONE,
        NO_TRAJ,      // 没有可用轨迹
        TRAJ_END,     // 轨迹快走完（常规重规划，可以退避）
        DEVIATED,     // 跟丢了参考点
        BLOCKED       // ★ 前方参考点被新障碍侵入 —— 安全触发，不可退避
    };

    ReplanReason needReplan(const Eigen::Vector3d& start) {
        // 取快照后已不持锁；发布线程可能同时换上更新的轨迹，
        // 这里用稍旧的一份做判断是安全的（判据只会略保守）。
        const auto snap = snapTraj();
        if (!snap || !snap->traj.valid()) return ReplanReason::NO_TRAJ;

        const double t = (ros::Time::now() - snap->start).toSec();
        // (a) 轨迹快走完 —— 只记账，不立即返回（原因见 (b) 的说明）
        const bool traj_ending = (t >= snap->traj.duration() - np_.replan_lead);

        // (b) 跟丢了：实际位置离参考点太远
        //
        // ⚠⚠ 这里**不能直接 return**（2026-11-14 安全修正）。
        //
        // 原来 (b) 在 (c) BLOCKED 之前无条件 return DEVIATED ——
        // 结果是：飞机因为障碍而偏离轨迹时，报的是"跟丢"而不是"被挡"，
        // BLOCKED 检查**根本没机会执行**。
        // 实测（/tmp/azj_flight_20261114_150005）：
        //   44.953 报了一次 BLOCKED，之后 46.154 / 47.434 / 49.255 全是 deviated
        //   → 旧路径被当成"可用"反复 realign → 沿着撞墙的路径飞 → 撞上。
        //
        // 正确语义：DEVIATED 只是"位置不一致"，它**不排除**"前方被挡"。
        // 所以先记账，继续往下做 BLOCKED 检查；BLOCKED 优先。
        Eigen::Vector3d ref_pos;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            ref_pos = last_ref_pos_;
        }
        // ★ 只看 XY（2026-11-14 修）。
        //
        // 原来用 3D 距离做“跟丢参考”判据。后果是**掉高会触发重规划**：
        // 飞机被偏航抢推力掉了 0.5m → 3D 偏差超限 → 重规划 → 新轨迹起点
        // = 当前低位置 → 参考点被拉回低处 → 飞机更追不上 → 再重规划，
        // 形成“越掉越重置、越重置越掉”的正反馈（yaw1 bag 实测 z 在
        // 0.85/0.01 之间来回跳、cmd vz 在 ±0.30 满幅跳）。
        //
        // 而重规划对 XY 偏离是必要的，对 Z 掉高是有害的：现有轨迹的 z 段
        // 本来就在向上爬，让它跑完反而能恢复高度。所以高度偏差交给
        // 轨迹自身 + 掉高保护处理，不在这里重置轨迹。
        const double dev_xy = std::hypot(start.x() - ref_pos.x(), start.y() - ref_pos.y());
        const bool deviated = (dev_xy > np_.replan_deviation);
        if (deviated) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] deviated from reference by "
                                              << dev_xy << "m (XY)");
        }

        // (c) 安全监控：前方参考点被新障碍侵入 → 必须重规划
        //     这是安全优先理念的兜底：不能因为“轨迹还没走完”就硬飞进障碍。
        //     2D 模式下同样用柱状投影，否则只查单层会漏掉头顶/脚下障碍。
        //
        // ★ 起点贴障碍时必须放行（2026-11-05 修正，治活锁）。
        //
        // 实测死循环：飞机贴在柱子上悬停（clearance=0.108m < 0.30m），
        // 本段从 ts=t 开始查 —— 而 t 对应的时间点就在飞机当前位置，
        // clearance 自然只有 0.108 < robot_radius → 第一个采样点就命中
        // “reference blocked ahead” → 重规划 → 又被安全校验拒 → 又回到这里。
        //
        // 语义修正：本项要抓的是“**原本安全的轨迹被新障碍挡住**”。
        // 如果起点本身就不安全，而前方并不比起点更差，那属于
        // “起点问题”（已由 A* 的 escape zone 与曲线起点豁免处理），
        // 不应当成新障碍侵入而反复重规划。
        // 只有当**前方真的比起点更槽**（正在变坏）才强制重规划。
        const ClearanceField::Ptr field_holder = makeField(goal_.z(), start.z());
        const ClearanceField& field = *field_holder;
        const double start_clr_now = field.clearance(start);
        const bool start_unsafe = start_clr_now < sp_.robot_radius;

        // ===== 分层阈值 + 滞回（2026-11-14，学 SUPER 的离散膨胀格语义）=====
        //
        // 旧判据 `c < robot_radius` 会在阈值上反复抖动，因为曲线优化的最优解
        // 本来就收敛到 c ≈ robot_radius（w_obs hinge 在 c < safe_margin 时才开始罚）。
        // 实测日志里 `min_clearance=0.3m (safe 0.3m)` 就是这个现象，
        // 于是每 3 秒触发一次 BLOCKED → 折线 → 平滑 → 曲线 → 再 BLOCKED。
        //
        // 新语义：只抓**真正侵入的硬障碍**。
        //   近段 near_frac(0.85)=0.255m：仍然敏感，能抓住刚扫出来的障碍
        //   远段 far_frac(0.55)=0.165m ：放松，因为远处 ESDF 可能还没建好
        //                               （unknown_clearance_cap=0.32 只比
        //                                 robot_radius 高 0.02，极易跌破）
        const double r = sp_.robot_radius;
        const double thr_near = r * std::max(0.1, np_.safety_near_frac);
        const double thr_far = r * std::max(0.1, np_.safety_far_frac);
        const double near_span = std::max(0.0, np_.safety_near_span);

        // 滞回：上次 BLOCKED 后，clearance 必须先恢复到 r×hyst 才允许再次触发。
        //
        // ⚠ 但**正在恶化**时必须立刻重新触发（2026-11-14 修正）。
        // 上一版的全局锁存有致命缺陷：一旦锁存，只要飞机还在障碍附近
        // （clearance 回不到 0.315），就**再也不触发 BLOCKED** ——
        // 哪怕它正在往障碍里飞。实测 15:00 那次全程只触发 1 次。
        //
        // 现在：滞回只用于"已经稳定/在改善"的情况；
        // 若 clearance 比锁存时更低（继续恶化），视为新事件，立即重触发。
        if (blocked_latched_) {
            if (start_clr_now < blocked_latch_clr_ - 1e-3) {
                // 继续恶化 → 立即解除锁存，允许再次触发
                blocked_latched_ = false;
            } else if (start_clr_now >= r * std::max(1.0, np_.safety_hysteresis)) {
                blocked_latched_ = false;      // 已恢复
            }
        }

        const double lookahead = std::min(t + np_.safety_lookahead, snap->traj.duration());
        for (double ts = t; ts <= lookahead; ts += 0.1) {
            Eigen::Vector3d p, v, a;
            if (!snap->traj.sample(ts, p, v, a)) continue;
            const double c = field.clearance(p);
            const double thr = (ts - t <= near_span) ? thr_near : thr_far;
            if (c < thr) {
                if (start_unsafe && c >= start_clr_now - 1e-6) continue;   // 不比起点更槽
                if (blocked_latched_) continue;                            // 滞回期内不重复触发
                blocked_latched_ = true;
                blocked_latch_clr_ = c;   // 记录锁存时的 clearance，用于"恶化则重触发"
                ROS_WARN_STREAM("[azj] reference blocked ahead (clearance="
                                    << c << "m < " << (thr == thr_near ? "near" : "far")
                                    << " threshold " << thr << "m; robot_radius="
                                    << r << "m) at dt=" << (ts - t) << "s, replanning");
                return ReplanReason::BLOCKED;
            }
        }
        // ── 按**安全优先级**返回（2026-11-14 重排）──
        //
        // BLOCKED 永远优先：它决定"会不会撞"。TRAJ_END / DEVIATED 只是
        // "该更新轨迹了"。原先 (a)(b) 在 (c) 之前无条件 return，
        // 导致"被挡"的信息被"走完/跟丢"吃掉 —— 这是撞墙的直接原因之一。
        //
        //   ① BLOCKED   前方有硬侵入 → 必须尽快重规划（快通道，不退避）
        //   ② TRAJ_END  轨迹走完     → 常规重规划
        //   ③ DEVIATED  跟丢参考     → 轻量重对齐（复用路径，~5ms）
        if (traj_ending) {
            ROS_INFO_STREAM_THROTTLE(2.0, "[azj] replanning: trajectory ending");
            return ReplanReason::TRAJ_END;
        }
        if (deviated) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] replanning: deviated (no blockage ahead)");
            return ReplanReason::DEVIATED;
        }
        return ReplanReason::NONE;
    }

    // ══════════════════════════════════════════════════════════════════
    //  滚动窗口重规划（2026-11-17，学 SUPER 的 receding horizon）
    // ══════════════════════════════════════════════════════════════════
    //
    // 设计对应 SUPER 的三处机制：
    //   SUPER                                 本实现
    //   ─────────────────────────────────────────────────────────────
    //   pos_init_state = 旧轨迹在未来 0.1s      rollingStartState()
    //   guide_path 只到前方 horizon             advanceEnd()（末点前探）
    //   setConditions 硬约束起点 (p,v,a)        buildTrajectoryImpl(p0,v0)
    //   每周期只优化 horizon 段                 seg = [p0, local_target]
    //
    // 额外的一条是 azj 自己的理念：末点每次重规划都做一遍 clearance 梯度
    // 上升（centerGuideEnd），把末点推到通道中轴 —— 这就是「末端引导居中」。
    // 因为末点在滚动，所以 ESDF 一变，下一次规划就立刻在末端生效。

    /// 在全局路径上找离 p 最近的顶点索引（O(n)，路径只有几百点）。
    int nearestIndexOnPath(const std::vector<Eigen::Vector3d>& path,
                          const Eigen::Vector3d& p) const {
        int best = 0;
        double best_d = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(path.size()); ++i) {
            const double d = (path[i] - p).squaredNorm();
            if (d < best_d) { best_d = d; best = i; }
        }
        return best;
    }

    /// 末端居中引导：把末点沿 ∇clearance 推到该横截面更宽处（= 两侧等距）。
    ///
    /// ★ 只在「通道较窄」时才做（clearance < clearance_ref）。
    ///   开阔地里 ∇c 指向很大且方向无意义，会把末点拖向随机方向。
    ///
    /// lateral_only 时把梯度投影到垂直于前进方向的平面：只做**横向回中**，
    /// 不改变沿路径的进度 —— 否则滚动窗口的长度会自相矛盾地伸缩。
    Eigen::Vector3d centerGuideEnd(const ClearanceField& field,
                                  const Eigen::Vector3d& pt,
                                  const Eigen::Vector3d& fwd) {
        if (np_.rolling_center_iters <= 0) return pt;
        const double c0 = field.clearance(pt);
        if (!std::isfinite(c0) || c0 >= sp_.clearance_ref) return pt;

        Eigen::Vector3d dir = fwd;
        dir.z() = 0.0;
        if (dir.norm() < 1e-6) dir = Eigen::Vector3d(1.0, 0.0, 0.0);
        dir.normalize();

        Eigen::Vector3d best = pt;
        double best_c = c0;
        Eigen::Vector3d q = pt;
        for (int it = 0; it < np_.rolling_center_iters; ++it) {
            Eigen::Vector3d g;
            if (!field.clearanceGradient(q, g)) break;
            if (np_.rolling_center_lateral_only) g -= g.dot(dir) * dir;
            if (!std::isfinite(g.norm()) || g.norm() < 1e-3) break;
            q += g.normalized() * np_.rolling_center_step;
            const double shift = (q - pt).norm();
            bool at_limit = false;
            if (shift > np_.rolling_center_max_shift) {
                q = pt + (q - pt) / shift * np_.rolling_center_max_shift;
                at_limit = true;
            }
            const double c = field.clearance(q);
            if (std::isfinite(c) && c > best_c + 1e-3) {
                best_c = c;
                best = q;
            }
            if (at_limit) break;
        }
        if (best_c > c0 + 1e-3) {
            ROS_INFO_STREAM_THROTTLE(
                2.0, "[azj] end centering: clearance " << c0 << " -> " << best_c
                     << " (shift " << (best - pt).norm() << "m)");
        }
        return best;
    }

    /// 全局路径缓存是否仍可用。
    ///
    /// 三个失效条件：未建立 / 目标变了 / 缓存过期。
    /// 「剩余段被新障碍切断」由 rollingReplan 里逐点检查（更便宜）。
    bool globalPathUsable(const Eigen::Vector3d& p0) const {
        if (!global_path_valid_ || global_path_.size() < 2) return false;
        if ((global_path_goal_ - goal_).norm() > np_.rolling_goal_refresh_dist) return false;
        if (global_path_wall_.isZero()) return false;
        if ((ros::WallTime::now() - global_path_wall_).toSec() > np_.rolling_global_ttl) {
            return false;
        }
        // ★ 起点偏离全局路径太多 → 缓存作废（2026-11-17 真 bug）
        //
        // 事故（/tmp/azj_roll_* 高频滚动首测）：
        //   全局路径是"从旧起点搜出来的"，飞机偏出 1.2m 后，
        //   seg 的第一段 [p0 -> path[i0+1]] 变成一条硬连直线，
        //   实测直接穿柱子：min_clr = 0.0034 / 0.0001 m，曲线被拒（curved=0）。
        //
        // 判据用 0.30m（≈ robot_radius）：超过它，"飞机→路径"的连线就
        // 不再能假定为畅通，必须重新搜索。
        // ★★★ 偏离判据必须用"到路径的垂距"，不能用"到最近顶点的距离"
        //     （2026-11-17 真 bug，导致全局路径频繁重建 → 绕行方案翻转）
        //
        // 事故（/home/spadea/azj_flights/azj_flight_20261117_212208，t=4.58）：
        //   旧路径 40 点 len=4.14m，沿 y≈-1.5 直穿到终点；
        //   新路径 50 点 len=5.16m，绕到 y=-2.45 再折回。
        //   相邻路径平均位移 0.58m、**最大 1.19m** —— 飞机完全追不上，
        //   这就是用户报的"后段路径贴合程度不高"。
        //
        //   触发原因：`nearestIndexOnPath` 返回的是"最近顶点"，而飞机往前
        //   飞了 0.32m 后，最近顶点还是**路径首点**（路径从那里开始），
        //   于是 `|path[0] - p0| = 0.32 > 0.30` 被判成"偏离"→ 重建全局路径。
        //   这跟"真的偏离"是两件事：沿路径前进不是偏离。
        //
        // 正确判据：p0 到**路径折线**的最短距离（垂距）。
        //   同时保留一个前提：若 p0 在首段的反向延长线上（已经走过路径起点），
        //   说明缓存确实太旧（飞机跑到路径"身后"了），也该重建。
        {
            const int i = nearestIndexOnPath(global_path_, p0);
            // 到相邻两条线段的垂距，取最小
            double best = (global_path_[i] - p0).norm();
            for (int k = std::max(0, i - 1); k < std::min<int>(i + 1, global_path_.size() - 1); ++k) {
                const Eigen::Vector3d a = global_path_[k], b = global_path_[k + 1];
                const Eigen::Vector3d ab = b - a;
                const double l2 = ab.squaredNorm();
                if (l2 < 1e-12) continue;
                double t = (p0 - a).dot(ab) / l2;
                t = std::min(1.0, std::max(0.0, t));
                best = std::min(best, (a + t * ab - p0).norm());
            }
            if (best > 0.30) return false;

            // 飞机是否跑到路径起点"身后"（沿首段投影为负且超出 0.5m）
            if (global_path_.size() >= 2) {
                const Eigen::Vector3d d0 = global_path_[1] - global_path_[0];
                if (d0.norm() > 1e-6) {
                    const double proj = (p0 - global_path_[0]).dot(d0.normalized());
                    if (proj < -0.50) return false;
                }
            }
        }
        return true;
    }

    /// 滚动窗口主流程。返回 true 表示已处理（无论成败），调用方直接 return。
    bool rollingReplan(const ClearanceField& field, ReplanReason why) {
        // 不管下面成不成功，都算"已经滚过一轮"—— 否则持续失败时
        // rolling_period 会每 tick 都放行，把主线程跑满。
        last_rolling_wall_ = ros::WallTime::now();
        // 无节流计数：用于确认滚动真实频率（日志 THROTTLE 会低估次数）
        static int roll_n = 0;
        static ros::WallTime roll_t0 = ros::WallTime::now();
        ++roll_n;
        {
            const double el = (ros::WallTime::now() - roll_t0).toSec();
            if (el > 5.0) {
                ROS_INFO("[azj] rolling RATE: %d calls in %.1fs = %.1f Hz (why=%d last)",
                         roll_n, el, roll_n / el, static_cast<int>(why));
                roll_n = 0;
                roll_t0 = ros::WallTime::now();
            }
        }

        // ─────────── 1) 起点：必须锚在**飞机自己**身上 ───────────
        //
        // ★ 事故（/tmp/azj_roll_20261117_110931）：最初这里取
        //   `snap->traj.sample(t_now + forward_dt)` —— 即**轨迹参考**的位置。
        //   后果：飞机落后参考时（实测 XY 偏差 0.52 -> 0.82 -> 1.22 -> 1.54
        //   -> 1.79 -> 2.21 -> 2.50m 持续放大），新轨迹起点也跟着跑在飞机
        //   前面 -> 参考越跑越远 -> 标准的正反馈。这与本文件多处记录的
        //   “15Hz 重对齐死循环”同源。
        //
        // 正确语义（用户反复强调的“起点必须跟着无人机”）：
        //   起点 = 飞机实际位置 + 当前速度 × forward_dt（前视），
        //   即直接复用 plan_anchor_（computeAnchor 已按 forward_dt 算好）。
        //   飞机落后多少，参考就从哪里起步，偏差自然收敛。
        //
        // 旧轨迹只剩一个用途：飞机已经跟得很紧（<0.20m）时，用它的速度
        // 方向比 odom 差分速度干净（odom 噪声会传进轨迹）。**绝不取它的位置**。
        // ══════════════════════════════════════════════════════════════
        //  起点：两个条件必须同时成立（2026-11-17 最终结论）
        // ══════════════════════════════════════════════════════════════
        //
        // 这里先后踩了两个相反的坑，都记在这里，别再单修一边：
        //
        // ── 坑 A：起点取旧轨迹位置，但重建很稀疏（4~5s 一次）
        //    （/tmp/azj_roll_20261117_110931）
        //    飞机落后参考后，新起点更靠前 → 参考越跑越远：
        //    XY 偏差 0.52 -> 0.82 -> 1.22 -> 1.54 -> 1.79 -> 2.21 -> 2.50m
        //
        // ── 坑 B：起点取飞机实际位置，重建很密（10Hz）
        //    （/tmp/azj_roll_* 高频首测）
        //    轨迹 t=0 就落在飞机身上，发布线程 t=now-start≈0 →
        //    **参考点永远贴着飞机** → 飞机追一个不动的点 →
        //    实测 |v0| 稳定在 0.13~0.16 m/s（max_vel=1.0），45s 只走 4.27m。
        //
        // 两个坑的共同结论：
        //   参考点必须**领先**飞机（否则飞机没有前进目标），
        //   而领先量必须**不累积**（否则飞机追不上）。
        //   → 起点 = 旧轨迹前沿（领先 forward_dt），且必须高频重建。
        //   这正是 SUPER 的完整语义（pos_init_state + 15Hz），缺一不可。
        //
        // 唯一的例外：飞机严重落后（>0.5m）时不取轨迹前沿，退回飞机位置。
        //   此时是控制器跟不上（仿真 PX4 参数问题），再把参考往前推只会
        //   加剧追赶压力；退回后靠着"速度仍用规划速度"保住自举。
        Eigen::Vector3d p0 = plan_anchor_valid_ ? plan_anchor_ : robot_state_.p;
        Eigen::Vector3d v0 = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        bool from_traj = false;
        double start_lag = 0.0;
        // 起点加速度 / jerk（MINCO 后端专用，2026-11-17）
        //
        // ★ 这是 MINCO 相对 B 样条的核心收益：把"我正以 a 加速"告诉优化器。
        //   B 样条只有 v 约束，每条新轨迹都从 a=0 重新加速 → rolling 每 0.1s
        //   重建一次，速度永远停在加速段（实测 |v0| 恒 0.285）。
        //   a 从旧轨迹取：旧轨迹本身是 C2 的，取到的 a 就是"当前真实规划加速度"。
        //   拿不到（无旧轨迹/新目标）时置 0 —— 那是从静止起步的合理语义。
        Eigen::Vector3d a0 = Eigen::Vector3d::Zero();
        Eigen::Vector3d j0 = Eigen::Vector3d::Zero();
        {
            const auto snap = snapTraj();
            if (snap && snap->traj.valid() && !goal_is_new_) {
                const double t_now = (ros::Time::now() - snap->start).toSec();
                if (t_now >= 0.0 && t_now <= snap->traj.duration() + 1e-6) {
                    const double tj = std::min(t_now + std::max(0.0, np_.rolling_forward_dt),
                                               snap->traj.duration());
                    Eigen::Vector3d pt, vt, at;
                    if (snap->traj.sample(tj, pt, vt, at) && pt.allFinite() &&
                        vt.allFinite() &&
                        // 前沿点必须仍可通行（否则 BLOCKED，本轮该拒绝）
                        field.clearance(pt) >= sp_.robot_radius * 0.6) {
                        start_lag = (pt - robot_state_.p).norm();
                        // ★ 位置跟飞机、速度用规划（2026-11-17 第三条结论）
                        //
                        // 三种取法都试过，只有这一种同时满足两个要求：
                        //   · 位置取轨迹前沿 → 参考点固定领先 0.69m（实测
                        //     lag 中位 0.69、峰值 1.03m），飞机一直在追一个
                        //     够不着的点
                        //   · 位置取飞机 + 速度取飞机 → 速度自举失败（0.13 m/s）
                        //   · **位置取飞机 + 速度取规划** → 参考点贴着飞机
                        //     （lag≈forward_dt=0.1m），而轨迹以规划速度推进，
                        //     飞机被正常拉着走，不蠕变也不空追
                        //
                        // 物理上等价于 SUPER 的 setConditions(headPVAJ)：
                        // 位置是"当前实际状态"，速度/加速度是"规划意图"。
                        // ★★ 位置取**轨迹前沿**（2026-11-17 最终结论，见下）
                        //
                        // 判据不是 lag 大小，而是"飞机能否正常前进"：
                        //   · 位置取前沿：lag 稳态 0.69m，飞机 40s 正常飞到 14m ✓
                        //   · 位置取飞机：lag 只有 0.05m，但飞机 30s 只走 0.78m ✗
                        //       （参考点贴着飞机 → 飞机没有前进目标 → 速度趋 0）
                        // 所以 lag 是"控制器跟踪误差"，不是故障；
                        // 而参考点必须领先，飞机才有得追。
                        p0 = pt;
                        if (at.allFinite()) a0 = at;
                        // ★★ 速度取"前方窗口峰值"而不是"当前瞬时值"（2026-11-17 诊断）
                        //
                        // 实测（/tmp/azj_roll_20261117_113332）：|v0| 恒定 0.285，
                        // 飞机 30s 只走 0.78m。原因是**轨迹永远停在加速段**：
                        //   · buildSmoothFromState 只约束起点速度 v，不约束加速度 a
                        //   · 每条轨迹从 v0 起步、靠 max_acc 慢慢加速到 max_vel
                        //   · 但 0.16s 后就重建 → 此刻轨迹速度才 0.285
                        //   · 新轨迹又以 0.285 起步 → 永远出不了加速段
                        //
                        // 这是 B 样条路径参数化的结构性缺陷（无 a/jerk 约束），
                        // 不是参数问题。MINCO 的 setConditions(headPVAJ) 正是解法。
                        // 在移植 MINCO 之前，先用"前方窗口峰值速度"绕开：
                        // 取 [tj, tj+0.4s] 内的最大速度作为起点速度，相当于
                        // 承认"规划意图是加速到 v*"，而不是重复从当前低速起步。
                        double vpeak = vt.norm();
                        for (double dts = 0.05; dts <= 0.40; dts += 0.05) {
                            Eigen::Vector3d pc, vc, ac;
                            if (snap->traj.sample(
                                    std::min(tj + dts, snap->traj.duration()), pc, vc, ac)) {
                                vpeak = std::max(vpeak, vc.norm());
                            }
                        }
                        if (vpeak > 1e-6) v0 = vt.normalized() * vpeak;
                        from_traj = true;

                        // ★★★ 参考速度不得超过飞机能力，并且要按 lag 反馈收敛
                        //     （2026-11-17 真 bug，实测 lag 0 -> 1.12 -> 1.63m 发散）
                        //
                        // 事故（/tmp/azj_flight_20261117_132140）：
                        //   t=35.05  lag=0     |v0|=0.017  （飞机静止，正常起步）
                        //   t=36.10  lag=1.12  |v0|=1.000  ← 参考已满速
                        //   t=37.19  lag=1.39  |v0|=0.894
                        //   t=39.35  lag=1.63  |v0|=0.809  ← 持续发散
                        //
                        // odom 实测：飞机 0.9s 走了 0.377m → 约 **0.42 m/s**。
                        // 而参考被"前方窗口峰值速度"顶到 1.0 → 每秒多跑 0.58m
                        // → 飞机永远追不上 → lag 单调增长 → 用户看到"路径跑太快"。
                        //
                        // 根源是 max_acc=1.5 与仿真 PX4 实际能力（约 0.4 m/s²）
                        // 差了近 4 倍：规划器以为飞机能猛加速，飞机做不到。
                        //
                        // 修法（两层，缺一不可）：
                        //   ① 上限：v0 <= 飞机速度 + max_acc*forward_dt
                        //      （一个周期内飞机物理上最多能追这么多）
                        //   ② lag 反馈：落后越多，允许的参考速度越低；
                        //      lag 大到一定程度 v0 会**低于**飞机速度 → 参考主动
                        //      减速等飞机 → lag 必然收敛（这是闭环）。

                        const double v_act = vel_received_ ? current_vel_.norm() : 0.0;
                        const double track_acc = (np_.rolling_track_acc > 1e-3)
                                                     ? np_.rolling_track_acc
                                                     : tp_.max_acc;
                        // 允许的速度超前量 [m/s]（2026-11-17 回退）
                        //
                        // 历史（三次尝试，实测对比）：
                        //   acc_allow = track_acc * max(0.05, forward_dt) = 0.05
                        //     v3.2.2 实飞：横向偏离 mean 0.120、跳变 1 次  ← 最好
                        //   acc_allow = sqrt(2*a*lag_budget) = 0.63
                        //     v3.2.3 实飞：横向偏离 mean 0.264、跳变 **6** 次、max 148m/s
                        //     ← 变差。原因：上限放宽后参考可以比飞机快 0.6 m/s，
                        //       真实 PX4 跟不上 → 飞机横向上忙于追赶 → 偏离反而变大
                        //
                        // 教训：这个上限不该按"理论上能追上"算，而该按
                        //   "真实伺服确实能跟上"取小值。仿真 PX4 的可用
                        //   速度余量实测只有 0.05~0.1 m/s 量级。
                        const double acc_allow =
                            track_acc * std::max(0.05, np_.rolling_v_lead);
                        double v_cap = v_act + acc_allow;
                        v_cap -= std::max(0.0, start_lag - np_.rolling_lag_free) *
                                 std::max(0.0, np_.rolling_lag_gain);
                        v_cap = std::max(np_.rolling_v_floor, v_cap);
                        if (v0.norm() > v_cap && v0.norm() > 1e-6) {
                            ROS_INFO_STREAM_THROTTLE(
                                1.0, "[azj] v0 capped " << v0.norm() << " -> " << v_cap
                                     << " (v_act=" << v_act << " lag=" << start_lag << ")");
                            v0 *= v_cap / v0.norm();
                        }
                    }
                }
            }
        }
        if (v0.norm() > tp_.max_vel && v0.norm() > 1e-6) {
            v0 *= tp_.max_vel / v0.norm();
        }
        // a0 也限幅：MINCO 的 max_acc 是软约束，实际 a 可能短时超限
        // （实测取到 2.19 m/s² 而 max_acc=1.5）。直接传给下一条轨迹的
        // head.a 会让超限被反复继承，所以在这里钳一次。
        if (a0.norm() > tp_.max_acc && a0.norm() > 1e-6) {
            a0 *= tp_.max_acc / a0.norm();
        }

        // 起点安全性兜底（与全量流程的 anchor 回退同义）
        if (field.clearance(p0) < sp_.robot_radius * 0.9) {
            const Eigen::Vector3d pn = robot_state_.p;
            if (field.clearance(pn) > field.clearance(p0)) {
                p0 = pn;
                v0 = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
                from_traj = false;
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] rolling start backed off to odom "
                                                    << p0.transpose());
            }
        }

        // ─────────── 2) 全局路径：只在必要时重跑 A* ───────────
        bool rebuilt = false;
        if (!globalPathUsable(p0)) {
            const Eigen::Vector3d v_dir =
                (v0.head<2>().norm() > 0.2) ? v0 : Eigen::Vector3d::Zero();
            const ros::WallTime t0 = ros::WallTime::now();
            // ★★★ A* 的 clearance 要留**平滑余量**（2026-11-25 真 bug）
            //
            // 事故（/tmp/azj_planner.log，实飞）：
            //   A* 按 robot_radius=0.30 规划，路径恰好 clearance>=0.30；
            //   而 MINCO 的 7 次多项式在段内会**切角**（平滑的必然代价），
            //   把 0.30 压到 0.0039 → 校验拒绝 → 连续 12 次 → 折线 1.1s。
            //
            //   实测那几次：start_clr=0.46~0.53（起点很安全）、
            //   goal_clr=0.8（终点也安全），但 worst 点固定落在
            //   (-1.45,-1.61) —— 正是 A* 路径贴着一根柱子过的位置。
            //
            // 修法：给 A* 多要一点余量（`rolling_astar_margin` × robot_radius），
            //   让折线离障碍更远，MINCO 再切角也有空间。
            //   找不到路时自动退回原值（窄缝场景不能因为余量而瘫痪）。
            SearchParams sp_astar = sp_;
            const double margin_mul =
                std::max(1.0, np_.rolling_astar_margin);
            sp_astar.robot_radius = sp_.robot_radius * margin_mul;
            SearchResult gs = astar_.search(field, p0, goal_, sp_astar, v_dir);
            if (!gs.success && margin_mul > 1.001) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] A* failed with clearance margin x" << margin_mul
                             << " - retry with nominal " << sp_.robot_radius << "m");
                gs = astar_.search(field, p0, goal_, sp_, v_dir);
            }
            const double ms = (ros::WallTime::now() - t0).toSec() * 1e3;
            if (!gs.success) {
                // 与全量流程同构的失败处理：BLOCKED → 刹车，否则保留旧轨迹
                global_path_valid_ = false;
                const bool old_known_bad = (why == ReplanReason::BLOCKED);
                if (old_known_bad) {
                    if (!tryBrakeReplan(gs.msg)) {
                        has_trajectory_ = false;
                        ROS_ERROR_STREAM_THROTTLE(
                            1.0, "[azj] SAFETY: rolling plan failed ("
                                     << gs.msg << ") AND brake failed - stopping");
                    }
                } else if (snapTraj() && snapTraj()->traj.valid()) {
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] rolling plan failed (" << gs.msg
                             << ") - keeping committed trajectory");
                } else if (!tryBrakeReplan(gs.msg)) {
                    has_trajectory_ = false;
                }
                return true;   // 已处理
            }
            // ══════════════════════════════════════════════════════════
            //  ★★★ A* 结果必须用**当前** ESDF 复验（2026-11-25 根因修复）
            // ══════════════════════════════════════════════════════════
            //
            // 事故（flight_20261125_201602，seq33→38）：
            //   同一个点 (-1.4412,-1.4000) 的 clearance 在 0.6 秒内：
            //     t=65.036  0.3887   ← A* 搜路时用的值
            //     t=65.138  0.1239
            //     t=65.259  0.0088   ← MINCO 校验时已经变了
            //     t=65.638  0.0025
            //
            //   `updateESDF3D` 每 140ms 全量重算，而 rolling 每 100ms 规划一次。
            //   A* 搜索期间缓存 `g.clr`，一次调用若跨过 ESDF 更新，
            //   缓存的就是旧值 → 硬约束 0.30 通过 → 路径经过实际 0.03 的格子。
            //
            //   这不是"参数没调好"，是**快照不一致**：A* 在旧地图上找路，
            //   MINCO 在新地图上校验，两者必然矛盾。
            //
            // 修法：A* 出路径后立刻用**当前** field 复验；不达标就以更大余量重搜。
            //   重搜有次数上限（防 ESDF 持续变化时活锁），
            //   超过上限仍不达标就如实放弃（让上层走 keep_old / B 样条）。
            //
            // 复验阈值用 `robot_radius`（不是 0.9 倍）——这里判的是"A* 路径是否
            // 真的满足硬约束"，必须严格。MINCO 那边的平滑余量另行放宽。
            std::vector<Eigen::Vector3d> path_verified = gs.path;
            {
                const double need = sp_.robot_radius;
                const int kMaxResearch = 2;
                for (int attempt = 0; ; ++attempt) {
                    // 沿路径密集采样找最差点
                    double worst = std::numeric_limits<double>::max();
                    Eigen::Vector3d worst_pt(0, 0, 0);
                    for (size_t i = 1; i < path_verified.size(); ++i) {
                        const Eigen::Vector3d& a = path_verified[i - 1];
                        const Eigen::Vector3d& b = path_verified[i];
                        const double len = (b - a).norm();
                        const int n = std::max(1, static_cast<int>(std::ceil(len / 0.02)));
                        for (int k = 0; k <= n; ++k) {
                            const Eigen::Vector3d q =
                                a + (b - a) * (static_cast<double>(k) / n);
                            const double c = field.clearance(q);
                            if (c < worst) { worst = c; worst_pt = q; }
                        }
                    }
                    if (worst >= need) break;      // 通过

                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] A* path STALE: worst clr=" << worst << "m < "
                                 << need << "m at (" << worst_pt.transpose()
                                 << ") — ESDF changed during search (attempt "
                                 << attempt + 1 << "/" << kMaxResearch << ")");
                    if (attempt >= kMaxResearch) {
                        // 放弃：交给上层（keep_old 保留旧轨迹 / B 样条兜底）
                        ROS_ERROR_STREAM_THROTTLE(
                            1.0, "[azj] A* path still unsafe after "
                                     << kMaxResearch << " re-searches — giving up this cycle");
                        global_path_valid_ = false;
                        return true;   // 已处理（不更新 global_path_）
                    }
                    // 加大余量重搜（当前 ESDF）
                    SearchParams sp2 = sp_astar;
                    sp2.robot_radius = need * (1.0 + 0.20 * (attempt + 1));
                    const SearchResult gs2 =
                        astar_.search(field, p0, goal_, sp2, v_dir);
                    if (!gs2.success) {
                        ROS_ERROR_STREAM_THROTTLE(
                            1.0, "[azj] A* re-search failed (" << gs2.msg
                                     << ") — giving up this cycle");
                        global_path_valid_ = false;
                        return true;
                    }
                    path_verified = gs2.path;
                }
            }

            global_path_ = path_verified;
            global_path_goal_ = goal_;
            global_path_wall_ = ros::WallTime::now();
            global_path_valid_ = true;
            rebuilt = true;
            ROS_INFO_STREAM_THROTTLE(2.0, "[azj] rolling: rebuilt global path, "
                                               << global_path_.size() << " pts in " << ms
                                               << "ms");
        }

        // ─────────── 3) 末点前探（receding horizon）───────────
        const int i0 = nearestIndexOnPath(global_path_, p0);
        double remain = 0.0;
        for (int i = i0; i + 1 < static_cast<int>(global_path_.size()); ++i) {
            remain += (global_path_[i + 1] - global_path_[i]).norm();
        }
        const bool to_goal = remain <= np_.rolling_lookahead + np_.rolling_lookahead_min;
        double look = to_goal ? remain : np_.rolling_lookahead;
        look = std::max(look, 0.0);

        int i1 = i0;
        double acc = 0.0;
        while (i1 + 1 < static_cast<int>(global_path_.size())) {
            const double d = (global_path_[i1 + 1] - global_path_[i1]).norm();
            if (acc + d > look) break;
            acc += d;
            ++i1;
        }
        if (to_goal) i1 = static_cast<int>(global_path_.size()) - 1;

        // ══════════════════════════════════════════════════════════════════
        //  ★★★ 引导段继承（学 SUPER 的 guide_path / receding_dis）
        //      ——这是解决"重规划后路径横向突变"的核心机制
        // ══════════════════════════════════════════════════════════════════
        //
        // 事故（/home/spadea/azj_flights/azj_flight_20261117_213624，6 次跳变）：
        //   跳变全是**横向**的 —— 速度方向完全不变，位置却垂直瞬移：
        //     t=5.61  Δ=(-0.23,+0.20)   速度方向 90°(+y)
        //     t=13.90 Δ=(+0.74,+0.09)   速度方向 -90°(-y) ← 几乎全在 +x
        //   0.74m 的横向瞬移，正好等于 A\* 重搜后"绕行方案翻转"的量级。
        //
        // 根因：我们每轮都从 `global_path_`（A\* 全量结果）取段。
        //   A\* 的目标函数里有"居中惩罚"，起点只要挪一点，两个等价的
        //   绕行方案（从柱子左边绕 vs 右边绕）代价可能翻转 → 路径整体横跳。
        //
        // SUPER 的做法（`super_planner.cpp:547` 起）：
        //   ```cpp
        //   double split_dis = cfg_.receding_dis;
        //   if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0) {
        //       split_dis = std::numeric_limits<double>::max();   // 整条畅通→不切
        //   }
        //   // 沿**旧轨迹**采样，遇障碍就截断；这段直接当 guide_path 的前段
        //   while (map_ptr_->isOccupiedInflate(temp_pt) ||
        //          (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
        //       last_exp_traj_time_pos.pop_back();
        //   }
        //   ```
        //   也就是**新轨迹的前半段就是旧轨迹本身**，A\* 只补剩下的部分。
        //   于是新旧轨迹在切换时刻空间几何完全相同 → 横向不可能突变。
        //
        // 移植到这里：
        //   1) 从旧轨迹取 [t_now, t_now+lookahead] 的采样点，逐点查 clearance，
        //      遇障碍立即截断（安全语义与 SUPER 的 isOccupiedInflate 一致）
        //   2) 引导段末端接到全局路径的最近后续点，继续取到末点
        //   3) 若引导段与全局路径横向差太大（>0.5m），说明地图确实变了
        //      → 放弃引导段，退回纯全局路径（这不是抖动，是真变化）
        std::vector<Eigen::Vector3d> seg;
        {
            std::vector<Eigen::Vector3d> guide;
            const double need_clr = sp_.robot_radius * 0.9;
            const auto sn = snapTraj();
            if (np_.rolling_receding_dis > 1e-3 && sn && sn->traj.valid() &&
                !goal_is_new_) {
                const double t_now0 = (ros::Time::now() - sn->start).toSec();
                if (t_now0 >= 0.0 && t_now0 <= sn->traj.duration()) {
                    Eigen::Vector3d prev = p0;
                    for (double tt = t_now0; ; tt += 0.05) {
                        const double tlim = std::min(
                            sn->traj.duration(),
                            t_now0 + np_.rolling_lookahead);
                        if (tt > tlim + 1e-9) break;
                        Eigen::Vector3d p, v, a;
                        if (!sn->traj.sample(tt, p, v, a)) break;
                        if ((p - prev).norm() < 0.05 && !guide.empty()) continue;
                        // 逐点安全校验（与 SUPER 的 isOccupiedInflate 同义）
                        if (field.clearance(p) < need_clr) break;
                        if (!guide.empty() &&
                            (p - guide.back()).norm() + (guide.back() - p0).norm()
                                > np_.rolling_receding_dis + 0.3) {
                            break;   // 已达继承长度
                        }
                        guide.push_back(p);
                        prev = p;
                        if ((p - p0).norm() >= np_.rolling_receding_dis) break;
                    }
                    // 太短没意义（<0.3m）：引导段必须是"真正的一段"，
                    // 否则拼接处会产生折角
                    if ((guide.empty() ? 0.0 : (guide.back() - p0).norm()) < 0.30) {
                        guide.clear();
                    }
                }
            }

            if (!guide.empty()) {
                // 引导段末端接全局路径：找最近后续点
                const int i_join = nearestIndexOnPath(global_path_, guide.back());
                const double gap = (global_path_[i_join] - guide.back()).norm();
                if (gap < 0.50) {
                    seg.push_back(p0);
                    for (const auto& q : guide) seg.push_back(q);
                    // 从 i_join 之后继续取，到原定前探终点 i1
                    const int from = std::max(i_join, i0);
                    for (int i = from + 1; i <= i1; ++i) seg.push_back(global_path_[i]);
                    ROS_INFO_STREAM_THROTTLE(
                        1.0, "[azj] rolling: guide inherited " << (guide.back() - p0).norm()
                                 << "m from old traj (join gap " << gap << "m)");
                } else {
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] rolling: guide vs global path gap="
                                 << gap << "m (>0.5) - map really changed, using global path");
                }
            }

            if (seg.size() < 2) {
                // 兜底：纯全局路径（原来的逻辑）
                seg.push_back(p0);
                for (int i = i0 + 1; i <= i1; ++i) seg.push_back(global_path_[i]);
                if (seg.size() < 2 || (seg.back() - seg.front()).norm() < 0.15) {
                    seg.assign(global_path_.begin() + i0, global_path_.end());
                    if (seg.empty() || (seg.front() - p0).norm() > 1e-9) {
                        seg.insert(seg.begin(), p0);
                    }
                }
            }
        }

        // ─────────── 4) 末端引导居中（ESDF 一变立即生效）───────────
        if (!to_goal && seg.size() >= 2) {
            const Eigen::Vector3d fwd = seg.back() - seg[seg.size() - 2];
            seg.back() = centerGuideEnd(field, seg.back(), fwd);
        }

        // 去重 + 保形简化（与全量流程一致）
        {
            std::vector<Eigen::Vector3d> dedup;
            dedup.reserve(seg.size());
            for (const auto& q : seg) {
                if (dedup.empty() || (q - dedup.back()).norm() > 0.01) dedup.push_back(q);
            }
            if (dedup.size() >= 2) seg.swap(dedup);
        }
        std::vector<Eigen::Vector3d> path = simplifyPath(seg);

        // ══════════════════════════════════════════════════════════════
        //  ★★★ rolling 段也要用**当前** ESDF 复验（2026-11-25 修正）
        // ══════════════════════════════════════════════════════════════
        //
        // v3.4.6 只给「全局路径重建」加了复验，但 rolling 每周期用的是
        // 从 global_path_ 切出来的 `seg`（局部段）——复验完全没覆盖到。
        //
        // 实测（flight_20261125_202250）：STALE 触发 **0 次**，
        // 而 dump 里 `raw_path min_clr=0.025`（远低于 robot_radius 0.30），
        // 证明这段路径确实没被复验过。
        //
        // 现在在这里补一次：只有通过复验的 seg 才进 MINCO。
        // 不通过时**不放宽阈值、不强行规划**，而是保留旧轨迹等下一周期
        // （下一周期 A* 会用新 ESDF 重搜全局路径）。
        {
            const double need = sp_.robot_radius;
            double worst = std::numeric_limits<double>::max();
            Eigen::Vector3d worst_pt(0, 0, 0);
            for (size_t i = 1; i < path.size(); ++i) {
                const Eigen::Vector3d& a = path[i - 1];
                const Eigen::Vector3d& b = path[i];
                const double len = (b - a).norm();
                const int n = std::max(1, static_cast<int>(std::ceil(len / 0.02)));
                for (int k = 0; k <= n; ++k) {
                    const Eigen::Vector3d q = a + (b - a) * (static_cast<double>(k) / n);
                    const double c = field.clearance(q);
                    if (c < worst) { worst = c; worst_pt = q; }
                }
            }
            if (worst < need) {
                ++seg_stale_streak_;
                global_path_valid_ = false;
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] rolling seg STALE (" << seg_stale_streak_ << " in a row): worst clr="
                             << worst << "m < " << need << "m at (" << worst_pt.transpose()
                             << ") — ESDF changed, skip this cycle (keep committed traj)");
                // ★ 连续 STALE 保护（2026-11-25）
                //
                // 如果 ESDF 持续显示该区域不可通行，而 A* 每周期都还是搜出
                // 同一条路（因为目标没变、起点没大变），就会陷入
                // "搜路 → 复验失败 → 跳过 → 再搜路"的活动锁，飞机停摆。
                //
                // 连续 5 次（约 0.5s）仍 STALE，就**不再跳过**，而是让 MINCO
                // 尽力优化（它至少会做安全投影），并让上层自然走 keep_old/折线。
                // 安全上仍由 MINCO 的校验兜底 —— 不会发布不安全的轨迹。
                if (seg_stale_streak_ >= 5) {
                    ROS_ERROR_STREAM("[azj] rolling seg STALE " << seg_stale_streak_
                                         << " times — accepting MINCO's best effort instead "
                                            "(safety still enforced by MINCO checks)");
                    seg_stale_streak_ = 0;
                    // 落到下面的正常流程
                } else {
                const bool old_known_bad = (why == ReplanReason::BLOCKED);
                if (old_known_bad) {
                    if (!tryBrakeReplan("rolling seg stale")) {
                        has_trajectory_ = false;
                    }
                } else if (snapTraj() && snapTraj()->traj.valid()) {
                    // 保留旧轨迹，本周期不更新
                } else if (!tryBrakeReplan("rolling seg stale")) {
                    has_trajectory_ = false;
                }
                return true;   // 已处理
                }   // end if (seg_stale_streak_ >= 5) else
            } else {
                seg_stale_streak_ = 0;
            }
        }

        // ─────────── 5) 轨迹生成 ───────────
        //
        // 后端二选一（2026-11-17）：
        //   MINCO      → 形状与时间联合优化，起点硬约束 p/v/a/j
        //   B 样条+TOPP → 旧的 curve_optimizer 路径（保留可回退）
        const double seg_len = pathLength(path);

        // ★ 极短段不走 MINCO（2026-11-17）
        //
        // 实测 seg=0.05m 时 MINCO 输出 dur=0.1s / vmax=1.71 m/s：
        // 段数被 clamp 到 2，每段只有 2.5cm，多项式系数在数值上完全病态
        // （和 B 样条那边"路径 <0.8m 时曲率估计不可靠"是同一类问题）。
        // 0.35m 以内本来就是"最后一步到位"，直线轨迹完全够用。
        const bool seg_too_short_for_minco = seg_len < 0.35;
        if (np_.traj_backend == 1 && !seg_too_short_for_minco) {
            StatePVAJ hd;
            hd.p = p0;
            hd.v = v0;
            hd.a = a0;
            hd.j = j0;
            hd.v = hd.v.cwiseMax(-tp_.max_vel).cwiseMin(tp_.max_vel);
            if (hd.a.norm() > tp_.max_acc) hd.a *= tp_.max_acc / hd.a.norm();

            StatePVAJ tl;
            tl.p = path.back();
            if (!to_goal) {
                // 局部目标：沿末段切向保持速度 → "不减速穿过"
                Eigen::Vector3d tan = path.back() - path[path.size() - 2];
                if (tan.norm() > 1e-6) tl.v = tan.normalized() * v0.norm();
            }

            // 落后越多，轨迹整体速度上限越低 → 飞机有相对余量追上去
            // ★ 速度压制必须有下限，否则会"越落后越慢"死锁（2026-11-17）
            //
            // 事故（unknown-region bench）：lag=5.35m 时
            //   v_decay = (5.35-0.35)*0.5 = 2.5
            //   vmax_allow = max(0.25, 1.0-2.5) = **0.25**
            //   而轨迹长度 4.8m → dur=**24s**，参考速度 0.2 m/s。
            //   飞机若被压到 0.2 m/s，lag 反而更大 → 下一轮压得更狠 → 死锁。
            //
            // 压制的作用是"让飞机有机会追上"，但压到低于飞机能力就没意义了。
            // 下限取 max_vel 的 55%：既明显低于巡航（给追赶余量），
            // 又远高于"几乎停住"（避免死锁）。
            const double v_floor_eff =
                std::max(np_.rolling_v_floor, tp_.max_vel * 0.55);
            const double v_decay = std::max(0.0, start_lag - np_.rolling_lag_free) *
                                   std::max(0.0, np_.rolling_lag_vel_gain);
            const double vmax_allow =
                std::max(v_floor_eff, tp_.max_vel - v_decay);

            if (buildTrajectoryMinco(path, hd, tl, to_goal, vmax_allow, &seg)) {
                // ★★★ 可视化必须用**实际轨迹**，不能存 A* 折线（2026-11-17 真 bug）
                //
                // 事故（用户报"规划出来是折线"）：这里原来写 `last_path_ = path`，
                // 而 path 是 A* 折线（simplifyPath 之后只有几个顶点）。
                // publishPathViz() 画的就是 last_path_ →
                // **rviz 里显示折线，而飞机实际跟的是 MINCO 平滑曲线**。
                // 于是"看着像折线、飞着却打滑" —— 完全是可视化与实际不一致。
                //
                // B 样条分支没这个问题，因为那边 `path = pts`（曲线重采样点）。
                // 现在从 MINCO 输出的 traj 按固定弧长抽稀，得到真正要飞的形状。
                last_path_ = sampleTrajAsPath(0.10);
                if (last_path_.size() < 2) last_path_ = path;   // 兜底
                last_path_length_ = seg_len;
                last_min_clearance_ = minClearanceAlongPath(path, field, 0.02);
                goal_is_new_ = false;
                publishPathViz();
                ROS_INFO_STREAM_THROTTLE(
                    1.0, "[azj] rolling[minco]: seg=" << seg_len << "m min_clr="
                             << last_min_clearance_ << "m start_from_traj="
                             << (from_traj ? 1 : 0) << " lag=" << start_lag
                             << "m |v0|=" << v0.norm() << " |a0|=" << a0.norm()
                             << " to_goal=" << (to_goal ? 1 : 0));
                return true;
            }
            // ══════════════════════════════════════════════════════════════
            //  ★★ MINCO 失败时**保持旧轨迹**，不要回退 B 样条（2026-11-17）
            // ══════════════════════════════════════════════════════════════
            //
            // 事故（/home/spadea/azj_flights/azj_flight_20261117_211548）：
            // 从 bag 的 /azj/path 反推出三种形态在 0.7 秒内切换了三次：
            //
            //   t=0.00~3.90  MINCO     (点间距 0.107m，正常)
            //   t=4.02       BSPLINE   (点间距 0.050m，97 点，len=4.78m)
            //   t=4.20       POLYLINE  (点间距 0.319m，8 点，len=3.74m)
            //   t=4.75       MINCO     (恢复)
            //
            // 相邻路径整体位移 **0.348m（最大 0.47m）**——因为 MINCO 和
            // B 样条/折线给出的几何形状差 0.45m。参考点每帧被拽到新形状上，
            // 飞机只能来回追，表现为：
            //   · 横向偏离 mean 0.144m（本该 <0.05）
            //   · 速度比 v_act/v_ref = 1.304（抄近道加速追）
            //   · 参考推进速率 p50 只有 0.33（大量时间在"换形状"而非"前进"）
            //
            // 正确语义与全量流程的"plan failed 时保留 committed trajectory"一致：
            //   **一条已验证安全的旧轨迹 ≫ 一条形状突变的新轨迹。**
            // MINCO 偶尔被安全校验拒绝是正常的（那是保护），
            // 但拒绝之后不该把飞机甩到另一条完全不同的曲线上。
            //
            // 兜底：旧轨迹剩余时间不足以再等一轮（<0.25s）时，
            // 仍走 B 样条 —— 否则会断参考。BLOCKED（安全事件）也走 B 样条，
            // 因为那时旧轨迹已不可信。
            // ★ 保留旧轨迹的前提：它必须还"活着"并且**能撑过下一次规划**。
            //
            // 事故（/home/spadea/azj_flights/azj_flight_20261117_214632）：
            //   参考速度在 t=8.4~9.5 塌到 **0.00**，飞机还在 0.22~0.49 滑行。
            //   机制：`keep_old` 只查了"剩余 >0.25s"就在**规划时刻**返回，
            //   但之后如果 MINCO 连续被拒，旧轨迹会在 0.25s 后**过期**；
            //   而 publishTick 采样 t>duration 时按 `ends_at_rest_` 返回 v=0
            //   → 参考归零、飞机滑行 —— 正是用户看到的"轨迹不跟无人机走"。
            //
            // 现在要求剩余时间至少覆盖 **2 个滚动周期**，否则宁可走 B 样条
            // 兜底也不能让参考断掉。B 样条形状差一点 << 参考速度归零。
            const double need_remain =
                std::max(0.35, 3.0 * std::max(0.05, np_.rolling_period));
            const bool keep_old =
                (why != ReplanReason::BLOCKED) && !seg_too_short_for_minco &&
                [&] {
                    const auto sn = snapTraj();
                    if (!sn || !sn->traj.valid()) return false;
                    const double el = (ros::Time::now() - sn->start).toSec();
                    return (sn->traj.duration() - el) > need_remain;
                }();
            if (keep_old) {
                ++minco_reject_streak_;
                // ★ keep_old 必须**发布旧轨迹的剩余部分**（2026-11-17 真 bug）
                //
                // 事故（~/azj_flights/azj_flight_20261117_215742）：
                //   t=4.64 发布 38 点曲线 → 之后 **1.89s 没有新的 /azj/path**
                //   → t=6.52 才出现 6 点折线。
                //   用户原话："轨迹起点没有跟着飞机走，轨迹被静止了一样，
                //   然后等到重规划突然轨迹又开始跟着无人机跑"。
                //
                // 原因：keep_old 只是 `return true`，既不改 `last_path_`
                //   也不调 `publishPathViz()`。于是 rviz 里显示的仍是
                //   1.89s 前那条轨迹（起点固定在生成时刻的位置），
                //   而飞机已经飞走 —— 视觉上就是"轨迹静止、起点不跟"。
                //
                // 修法：把**旧轨迹从当前时刻往后的剩余部分**发布出去。
                //   这样 path 起点始终等于飞机当前应该到达的参考点。
                publishCommittedTrajAsPath();
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] MINCO rejected (" << minco_reject_streak_
                             << " in a row) - keeping committed trajectory; republishing its "
                                "remaining part as /azj/path so the viz keeps following.");
                // ★ 连续被拒超过 N 次就放弃 keep_old，让 B 样条接手
                //
                // 否则 MINCO 在当前场景（典型：未知建图区）持续失效时，
                // 会一直保留旧轨迹直到它过期 → 期间 path 只能显示旧内容。
                // 连续拒绝是"这个场景 MINCO 就是不适用"的明确信号。
                if (minco_reject_streak_ < np_.minco_max_reject_streak) {
                    return true;
                }
                ROS_WARN_STREAM("[azj] MINCO rejected " << minco_reject_streak_
                                    << " times in a row - giving up keep_old, using B-spline");
            } else {
                minco_reject_streak_ = 0;
            }
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[azj] MINCO backend failed AND old trajectory too short "
                     "- falling back to B-spline");
        }

        bool curved = false;
        if (np_.curve_enable && seg_len >= 0.8) {
            CurveOptParams cp = cp_;
            cp.safe_margin = sp_.robot_radius;
            cp.clearance_ref = sp_.clearance_ref;
            cp.clearance_cap = sp_.clearance_cap;
            cp.alpha = sp_.alpha;
            cp.lock_z = plan_2d_active_;
            cp.lock_z_value = plan_2d_z_active_;
            if (np_.enforce_height_limits) {
                cp.enable_z_bounds = true;
                cp.z_min = np_.floor_z;
                cp.z_max = np_.ceil_z;
            }
            cp.w_height = np_.w_height;
            cp.height_scale = sp_.height_scale;
            // BLOCKED（安全事件）用快速迭代：宁可先有一条合格曲线，
            // 也不要为了精修让飞机沿旧轨迹多飞 0.5s。
            const bool quick = (why == ReplanReason::BLOCKED) &&
                               np_.curve_iterations_quick > 0;
            if (quick) cp.iterations = std::min(cp.iterations, np_.curve_iterations_quick);

            const ros::WallTime t_cv0 = ros::WallTime::now();
            const CurveOptResult cr = curve_opt_.optimize(path, field, cp);
            const double ms_cv = (ros::WallTime::now() - t_cv0).toSec() * 1e3;
            if (cr.success) {
                std::vector<Eigen::Vector3d> pts, tgs;
                if (cr.spline.sampleByArcLength(np_.curve_sample_step, pts, tgs) &&
                    pts.size() >= 2) {
                    // 安全检查（排除末端：末点可能刻意靠近障碍以进入通道）
                    const double mc = minClearanceAlongPathExcludingEnd(
                        pts, field, sp_.robot_radius + 0.3, 0.02);
                    if (mc >= sp_.robot_radius * 0.9) {
                        path = pts;
                        curved = true;
                        last_max_curvature_ = cr.max_curvature;
                    } else {
                        ROS_WARN_STREAM_THROTTLE(
                            1.0, "[azj] rolling curve rejected: min_clr="
                                     << mc << "m < " << (sp_.robot_radius * 0.9)
                                     << "m - keeping A* polyline");
                    }
                }
            }
            ROS_INFO_STREAM_THROTTLE(
                2.0, "[azj] rolling T {astar_rebuild=" << (rebuilt ? 1 : 0)
                         << " curve=" << ms_cv << "ms seg=" << seg_len
                         << "m to_goal=" << (to_goal ? 1 : 0) << " curved="
                         << (curved ? 1 : 0) << "}");
        }

        // ─────────── 6) 构建轨迹并原子换快照 ───────────
        //
        // ★ stop_at_end = to_goal：局部目标处**不减速**。
        //   这是「同一段路径不会中途减速」的关键 —— 若局部末点也刹到 0，
        //   每 1/15s 一次就变成持续顿挫（用户之前的体验）。
        TrajParams tp = tp_;
        tp.stop_at_end = to_goal;
        if (!buildTrajectoryImpl(path, tp, &p0, &v0)) {
            const bool old_known_bad = (why == ReplanReason::BLOCKED);
            if (old_known_bad) {
                if (!tryBrakeReplan("rolling trajectory build failed")) {
                    has_trajectory_ = false;
                    ROS_ERROR_STREAM_THROTTLE(
                        1.0, "[azj] SAFETY: rolling build failed AND brake failed");
                }
            } else if (snapTraj() && snapTraj()->traj.valid()) {
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] rolling build failed - keeping committed");
            } else {
                has_trajectory_ = false;
            }
            return true;
        }

        last_path_ = path;
        last_path_length_ = pathLength(path);
        last_min_clearance_ = minClearanceAlongPath(path, field, 0.02);
        goal_is_new_ = false;
        publishPathViz();

        ROS_INFO_STREAM_THROTTLE(
            1.0, "[azj] rolling: seg=" << last_path_length_ << "m min_clr="
                     << last_min_clearance_ << "m start_from_traj=" << (from_traj ? 1 : 0)
                     << " lag=" << start_lag << "m |v0|=" << v0.norm()
                     << " remain=" << remain << "m");
        return true;
    }

    void replanCallback(const ros::TimerEvent&) {
        // ★ 运行时 yaw_mode 切换（2026-11-14 新增）。
        //
        // 为什么需要：做 A/B 对照时，旧办法要重启 azj 节点（连带重建 rog_map），
        // 而"不同 SITL 实例 / HTE 学习状态不同"会污染对比 —— 实测三次实验里
        // A 组是已悬停状态起录（z_min 全程 0.842），B/C 是起飞即机动，
        // 根本不可比。有了运行时切换，**一次飞行内**就能交替两种模式，
        // 飞机状态、地图、HTE 全都不变。
        //
        // 用法（无需重启）：
        //   rosparam set /azj_planner/yaw_mode 0    # 不转
        //   rosparam set /azj_planner/yaw_mode 3    # 朝速度
        // 用**绝对名**读，理由同 plan_2d（私有句柄会多解析一层）。
        {
            int want = np_.yaw_mode;
            if (ros::param::get("/azj_planner/yaw_mode", want)) {
                const int cur = yaw_mode_rt_.load();
                if (want != cur) {
                    yaw_mode_rt_.store(want);
                    ROS_WARN("[azj] yaw_mode switched %d -> %d at runtime "
                             "(no restart needed; for A/B tests)", cur, want);
                }
            }
        }

        if (!has_goal_) return;

        // ══════════════════════════════════════════════════════════════════
        // ★★ 每次进入规划周期都重置"规划起始状态"（2026-11-14 关键修正）
        //
        // 事故（/tmp/azj_flight_20261114_154400）：
        //   t=46.651  extrapolated 1.5s: pos -2.43227 -0.153794 -> -3.39576
        //   t=48.644  extrapolated 1.5s: pos -2.43227 -0.153794 -> -3.39576
        //   —— 完全相同的坐标，说明 plan_start_pos_ 是**很久以前**设的。
        //
        // 原因：plan_start_* 原来只在"完整规划"路径里赋值（A* 之前），
        //   而 `realign only (DEVIATED)` 分支在那之前就 return 了 →
        //   realign 用的是上一次完整规划的时间戳（已过去 >1.5s）
        //   → dt_plan 被 clamp 到 1.5s → 沿 1.5 秒前的速度外推 1.4m
        //   → anchor 落在完全错误的位置 → 轨迹起步速度方向错 → 掉高。
        //
        // 现在：进周期就记，任何分支（realign / 完整规划 / 刹车）都用它。
        // ══════════════════════════════════════════════════════════════════
        plan_start_wall_ = ros::WallTime::now();
        plan_start_pos_ = robot_state_.p;
        plan_start_vel_ = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();

        // 到达后锁定，直到收到新目标。
        // 否则惯性/漂移会把飞机拉出 arrive_tol，触发"再次规划→再次到达"的来回蹭。
        // 到点后的位置保持交给控制器（navigationIso/Contour 自带位置环）。
        if (arrived_) return;
        if (!robot_state_.rcv) {
            ROS_WARN_STREAM_THROTTLE(2.0, "[azj] no odom yet, skip planning");
            return;
        }

        Eigen::Vector3d start = robot_state_.p;
        Eigen::Vector3d goal = goal_;

        // ---------- 层高约束：自动钳到地图范围 + 高度保持 ----------
        //
        // ⚠ 关键概念：rog_map 的 virtual_ground/ceil_height **只是点云裁剪边界**，
        //   它们不约束规划！用户把 virtual_ceil_height 改小，A* 完全不受影响，
        //   因为 A* 只看 clearance 场，不看建图参数。
        //   真正约束规划高度的是 azj/floor_z 与 azj/ceil_z（这里）。
        //
        // 但两者必须协调：ceil_z 若超过地图的 ceil_eff，那片区域是 UNKNOWN，
        // 而 unknown_clearance_cap(0.5) > robot_radius(0.3) 会被判为"可通行"
        // → 路径往上跑进没有地图的地方（用户说的"从很上面规划"）。
        // 所以这里把规划边界钳进地图边界。
        if (np_.enforce_height_limits) {
            clampHeightLimitsToMap();
            // 高度保持参考。
            //
            // ★ 必须用**目标高度**，不能用"起终点平均"！
            //   实测 bug：飞机在地面 z=0.03 打点去 0.80 时，
            //   平均值 = 0.415，z_band=0.40 → 允许 [0.015, 0.815]，
            //   于是路径可以合法地掉到 0.043（地面附近 UNKNOWN 被高估成宽敞）。
            //   这正是"明明周围都是膨胀点云，路径还是往下钻"的原因 ——
            //   约束本身被当前高度拉低了。
            //
            //   用 goal.z 则高度带始终围绕"要飞到的高度"，
            //   起飞/掉高恢复的过渡由轨迹层处理（起点 z 不受 z_band 限制，
            //   A* 对起点格有例外）。
            sp_.w_height = np_.w_height;
            sp_.height_ref = goal.z() + np_.height_ref_offset;
            sp_.height_scale = std::max(0.2, np_.height_scale);
            sp_.height_band = std::max(0.0, np_.height_band);
            sp_.w_height_band = std::max(0.0, np_.w_height_band);
            sp_.z_band = std::max(0.0, np_.z_band);
        } else {
            sp_.w_height = 0.0;
            sp_.w_height_band = 0.0;
            sp_.z_band = 0.0;
        }

        // 起点高度显著低于目标时，明确说明"路径会从低处起"——
        // 否则用户会以为规划器把路径往地下拽（实测就是这样被误解的）。
        if (goal.z() - start.z() > 0.25) {
            ROS_WARN_STREAM_THROTTLE(
                2.0, "[azj] start z=" << start.z() << " is " << (goal.z() - start.z())
                                      << "m below goal z=" << goal.z()
                                      << "; path will climb from the low start (not a bug)");
        }

        // 层高钳制：飞机在地面（z 略低于 floor_z）时，把规划起点抬到允许高度。
        //
        // 不做这一步的话，加了层高硬约束后起点会被判成"在障碍里"，
        // 规划器直接拒绝 —— 而这时候飞行器只是还没起飞或刚落地。
        // 钳制后路径从 floor_z 开始，控制器会先把它带上去，是合理行为。
        if (np_.enforce_height_limits) {
            const double zlo = np_.floor_z, zhi = np_.ceil_z;
            if (start.z() < zlo) {
                ROS_WARN_STREAM_THROTTLE(2.0, "[azj] start z=" << start.z()
                                                               << " below floor_z=" << zlo
                                                               << ", clamped for planning");
                start.z() = zlo;
            } else if (start.z() > zhi) {
                ROS_WARN_STREAM_THROTTLE(2.0, "[azj] start z=" << start.z() << " above ceil_z="
                                                               << zhi << ", clamped");
                start.z() = zhi;
            }
            if (goal.z() < zlo || goal.z() > zhi) {
                ROS_WARN_STREAM_THROTTLE(2.0, "[azj] goal z=" << goal.z() << " outside ["
                                                              << zlo << "," << zhi
                                                              << "], clamped");
                goal.z() = std::min(std::max(goal.z(), zlo), zhi);
            }

            // ══════════════════════════════════════════════════════════
            //  ★★ 贴地起点要抬到"真正有间隙"的高度（2026-11-14 重大修正）
            //
            // 旧实现只抬 0.05m，实测完全不够，导致规划器瘫痪 15 秒：
            //
            //   /tmp/azj_flight_20261114_214929（飞机停在原点）：
            //     t=36.993  start z=2.2e-06 too close to floor_z=0; lifted to 0.05
            //     t=36.994  start diag: pos=(0,0,0.05) esdf=0.000 unknown=0
            //               clearance_used=0.000
            //     → `path_search.cpp:229` 的 `start_clr > 0.0` 为假
            //     → `plan failed: start is inside an obstacle` **连续 15 次**
            //       （t=36.99 ~ 51.99，每秒一次）→ 完全无法规划
            //
            // 为什么 0.05 不够：飞机趴在地面时，起点处同时有
            //   · 地面点云（z≈0）
            //   · 机体自反射（雷达/深度相机看到自己）
            // 两者叠加 → ESDF 在 (0,0,0.05) 依然是 **0.000**。
            //
            // 正确做法：**逐步抬升直到 clearance 真的 > 0**（最多抬到
            // ground_lift_max，默认 0.40m）。这样：
            //   · 贴地/地面噪声 → 抬几档就够，规划能开始
            //   · 真的被障碍包住（四面墙）→ 抬到上限仍为 0 → 拒绝（正确）
            // ══════════════════════════════════════════════════════════
            const double kClear = 0.05;
            if (start.z() - zlo < kClear) {
                const double lift_max = std::max(kClear, np_.ground_lift_max);
                double lift = kClear;
                double best_lift = kClear;
                double best_clr = -2.0;
                while (lift <= lift_max + 1e-9) {
                    const Eigen::Vector3d probe(start.x(), start.y(), zlo + lift);
                    const double c = clearanceAt(probe);
                    if (c > best_clr) { best_clr = c; best_lift = lift; }
                    if (c > 1e-3) { best_lift = lift; break; }
                    lift += 0.05;
                }
                ROS_WARN_STREAM_THROTTLE(
                    2.0, "[azj] start z=" << start.z() << " too close to floor_z=" << zlo
                                          << "; lifted to " << (zlo + best_lift)
                                          << " (clearance=" << best_clr
                                          << ", searched up to +" << lift_max << "m)");
                start.z() = zlo + best_lift;
            }
            if (zhi - start.z() < kClear) {
                start.z() = zhi - kClear;
            }
        }

        if ((start - goal).norm() <= np_.arrive_tol) {
            // ★ 到达判定必须同时看速度（2026-11-05，治“到点过冲”）。
            //
            // 只看距离的后果（实测）：飞机进 0.15m 球时速度还有 0.41 m/s，
            // 规划器立刻发 COMPLETED、停止输出轨迹 → 只剩 PX4 位置环刹车，
            // 但 PX4 位置环的增益不足以在 0.15m 内刹住 0.4m/s，
            // 于是冲过目标 0.15~0.25m，再被拉回来。
            // 用户看到的就是“到点过冲”。
            //
            // 正确语义：到点 = 到达且停下。速度未降时**继续输出轨迹**
            // （轨迹本身已经在减速，因为 goal 就在眼前），让它自己刹住。
            //
            // arrive_vel_tol 取 0.20 m/s：实测轨迹末端的自然末速就很小，
            // 这个阀值不会拖延到达，只是拒绝“带着明显速度”就判到位。
            const bool slow_enough = current_vel_.head<2>().norm() <= np_.arrive_vel_tol;
            if (!slow_enough) {
                // 速度还大：不判到达，继续规划/跟轨迹
                // （不 return，让下面的 needReplan 正常走）
            } else if (!arrived_) {
                arrived_ = true;
                has_trajectory_ = false;
                ROS_INFO("[azj] goal reached (dist=%.3f, |v|=%.2f)",
                         (start - goal).norm(), current_vel_.head<2>().norm());
                return;
            } else {
                return;
            }
        }

        // ---------- 2D / 3D 模式（支持运行时切换）----------
        // 必须在 needReplan 之前解析：切换模式要让本周期立刻重建轨迹。
        //
        // 注意用**绝对名**读：节点私有句柄是 /azj_planner，pnh_.getParam("azj/plan_2d")
        // 会解析成 /azj_planner/azj/plan_2d（多一层），而我们在
        // publishParamsToServer 里写的、用户 rosparam set 的都是 /azj_planner/plan_2d。
        // 早期版本就是这里对不上，导致切换完全没反应。
        bool want_2d = np_.plan_2d;
        ros::param::get("/azj_planner/plan_2d", want_2d);
        if (want_2d != plan_2d_active_) {
            plan_2d_active_ = want_2d;
            has_trajectory_ = false;   // 强制重建
            ROS_WARN("[azj] plan mode switched to %s", want_2d ? "2D (XY only)" : "3D");
        }
        sp_.plan_2d = plan_2d_active_;
        plan_2d_z_active_ = goal.z();
        z_plane_ = plan_2d_z_active_;

        // ★ MINCO dump 目录支持运行时切换（2026-11-25）
        //
        // 为什么必须运行时读：取证脚本 `azj_shadow.sh` 是在规划器**已经起来之后**
        // 才 set 这个参数的（它不知道你什么时候起 raicom_azj.sh）。
        // 如果只在 loadParams 里读一次，取证就永远抓不到 dump。
        //
        // 用绝对名读（与 yaw_mode / plan_2d 同理）：私有句柄会多解析一层
        // → /azj_planner/azj/minco_dump_dir，而 rosparam set 写的是
        // /azj_planner/minco_dump_dir。
        {
            std::string want_dump = np_.minco_dump_dir;
            if (ros::param::get("/azj_planner/minco_dump_dir", want_dump)) {
                if (want_dump != np_.minco_dump_dir) {
                    np_.minco_dump_dir = want_dump;
                    if (want_dump.empty()) {
                        ROS_WARN("[azj] MINCO dump DISABLED");
                    } else {
                        ROS_WARN("[azj] MINCO dump ENABLED -> %s", want_dump.c_str());
                    }
                }
            }
        }

        ReplanReason why = needReplan(start);

        // ★★ rolling 持续滚动（2026-11-17）
        //
        // 为什么需要：rolling 的末点前探 + 末端居中必须**持续**重算。
        // 若只在 needReplan 报 TRAJ_END 时才算（旧行为实测 4~5s 一次），
        // 等于"每 4 秒才看一眼前方" —— 雷达扫出新障碍后要等很久才居中。
        //
        // 为什么不能无限快（比如每 tick 15Hz）：这个代码库亲身验证过
        // 15Hz 重建轨迹 → 参考点每周期被"重置到前方" → 速度指令递增
        // → 掉高（见下面 DEVIATED 分支的长注释与
        // /tmp/azj_flight_20261114_154400）。
        // 现在 anchor 已改成"飞机位置 + 按耗时外推"（v1.7.2），前提不再成立，
        // 但仍取 0.5s 这个保守周期：足够 "实时"，也远离那个振荡区。
        if (np_.rolling_enable && np_.rolling_period > 1e-3 &&
            why == ReplanReason::NONE) {
            const double since = last_rolling_wall_.isZero()
                                     ? 1e9
                                     : (ros::WallTime::now() - last_rolling_wall_).toSec();
            if (since >= std::max(0.05, np_.rolling_period)) {
                why = ReplanReason::TRAJ_END;   // 借道"常规重规划"分支
            }
        }

        // ★ 快通道折线的**主动**平滑（2026-11-14 新增）。
        //
        // 背景：BLOCKED 时故意跳过曲线优化（30ms 出折线，保住安全性），
        // 日志说 "next replan will smooth it"。但那个"下一次"原先是靠
        // TRAJ_END / DEVIATED 偶然触发的，实测要等 4.2 秒，而且可能又失败。
        //
        // 这里改为机制保证：fast path 之后过了 fast_smooth_delay 秒，
        // 只要当前**不再**是 BLOCKED，就主动发起一次完整规划（走曲线）。
        //
        // 为什么不用更短：强制重规划会把轨迹时间基准重置为 0，
        // 太频繁会让参考点一直停在轨迹前段（v 上不去）。
        // 0.6s 足够让飞机脱离刚扫出的障碍，又不至于让折线长期在线。
        // 这里借用 ReplanReason::TRAJ_END 走"常规重规划"分支，
        // 因此会受 min_replan_period 退避保护，不会活锁。
        bool smooth_forced = false;
        if (why == ReplanReason::NONE && !last_fast_path_wall_.isZero()) {
            const double since = (ros::WallTime::now() - last_fast_path_wall_).toSec();
            if (since > np_.fast_smooth_delay && since < 10.0) {
                why = ReplanReason::TRAJ_END;
                smooth_forced = true;
            }
        }
        if (smooth_forced) {
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] smoothing replan (fast-path polyline is "
                         << (ros::WallTime::now() - last_fast_path_wall_).toSec()
                         << "s old) - running full curve optimization");
            last_fast_path_wall_ = ros::WallTime();   // 用掉就清，避免反复触发
        }

        if (why == ReplanReason::NONE) return;

        // ══════════════════════════════════════════════════════════════════
        // ★★ 轻量重对齐：DEVIATED 不跑 A*/曲线（2026-11-14，学 SUPER 的 NO_NEED）
        //
        // 实测问题（/tmp/azj_flight_20261114_134950）：
        //   12 秒内 8 次重规划（tid 1→9），每次曲线优化 0.9~1.2 秒
        //   → 飞机有一半时间在"等规划"，而前方其实一直是通的。
        //
        // SUPER 为什么快 3000 倍（super_planner.log：
        //   `average computation time: 0.305 ms` vs 我们的 850~1200ms）：
        //   它的 `generateExpTraj` 有大量 NO_NEED 早退 ——
        //   沿旧轨迹采样检查膨胀格，**全通就 return NO_NEED，根本不规划**。
        //   （super_planner.cpp:441/470/483 三处早退）
        //
        // 这里做等价的事，但语义更严格：
        //   DEVIATED = 飞机跟丢参考点，**不等于路径失效**。
        //   路径有效性已经由 needReplan 的 BLOCKED 检查独立保证（刚通过）。
        //   所以只需用**同一条路径**重新做时间参数化（几毫秒），
        //   不需要重跑 A*，更不需要重跑曲线优化。
        //
        // 效果：这类重规划从 ~1000ms 降到 ~5ms，且轨迹形状完全不变
        //       （不会出现"重规划后突然转向"那种掉高事故）。
        // ══════════════════════════════════════════════════════════════════
        // ══════════════════════════════════════════════════════════════════
        // ★★ DEVIATED 时**什么都不做**（2026-11-14，学 SUPER 的 NO_NEED）
        //
        // 上一版在这里做 `buildTrajectory(last_path_)` 轻量重对齐，实测灾难：
        //
        //   /tmp/azj_flight_20261114_154400（15Hz）：
        //     rel_t=9.88  轨迹 2→3
        //     rel_t=9.98  3→4   ← 每 0.1 秒重建一次
        //     rel_t=10.09 4→5
        //     rel_t=10.23 5→6
        //     rel_t=10.34 6→7
        //     rel_t=10.45 7→8
        //     rel_t=10.55 8→9   ← 0.1 秒内 7 次！
        //     新轨迹速度一路递增 (0.11,-0.21)→(0.25,-0.28)→…→(0.69,-0.56)
        //     rel_t=11.0  z 从 0.588 掉到 0.499（vz=-0.74）
        //
        // 为什么会死循环：重对齐用 anchor（**旧轨迹参考**在 t_now 的状态）起步，
        // 而飞机落后参考 0.5m → 新轨迹的参考仍在飞机前方 0.5m
        // → 下一周期 still deviated → 又重对齐 → 参考又被"重置到前方"
        // → 每周期给控制器一个越来越激进的速度指令 → 叠加 → 掉高。
        //
        // SUPER 的语义很清楚：**飞机跟不上参考是控制器的事，不是规划器的事**。
        // 只要轨迹本身安全（BLOCKED 没触发），就让轨迹继续跑完，
        // 到 TRAJ_END 自然重规划（那时从飞机实际位置出发，会真正对齐）。
        // ══════════════════════════════════════════════════════════════════
        // ══════════════════════════════════════════════════════════════════
        // ★★ DEVIATED 必须重规划（2026-11-16 改正）
        //
        // v1.6.1 我把它改成"什么都不做"，理由是避免 15Hz 下的死循环。
        // 但那个死循环的**真正根因**是 anchor 用了旧轨迹的**参考状态**
        // （飞机落后参考 0.5m → 新轨迹参考仍在 0.5m 前 → 永远 deviated）。
        //
        // 现在 anchor 已经改成"飞机自身位置 + 按耗时外推"（v1.7.2），
        // 新轨迹的参考点就落在飞机身上 —— 重规划后**不会**立刻又 deviated。
        // 所以死循环的前提消失了。
        //
        // 而"什么都不做"的代价实测很大
        // （/tmp/azj_flight_20261116_213108）：
        //   t=6.5~8.4  参考点超前飞机 0.54 → **1.42m** 且持续增大
        //   t=8.5      轨迹跑完才重规划，参考点**倒退 1.51m**（75 m/s 等效）
        //   → 控制器先被拖着追 1.4m 外的参考（sp 打满、方向乱摆）
        //   → 再吃到 1.5m 的反向跳变 → roll -23° → 掉高
        //
        // 用户看到的"规划的路径不跟着无人机走、起点落在已飞过的路径上"
        // 就是这条链：参考跑前面不纠正 → 最后一次性倒退。
        // ══════════════════════════════════════════════════════════════════
        // ══════════════════════════════════════════════════════════════════
        // ★★★ 统一在这里算一次 anchor —— 所有分支共用（2026-11-16 真 bug）
        //
        // 事故（/tmp/azj_flight_20261116_214415）：
        //   t=101.476  完整规划 → plan_anchor_ = 飞机位置（此刻）
        //   t=102.035  deviated 0.504m → realign 1.26ms
        //   t=103.048  deviated 0.726m → realign 1.39ms   ← 偏差还在扩大
        //   t=104.433  deviated 0.579m → realign 1.42ms
        //   t=106.114  deviated 0.582m → realign 1.36ms
        //
        //   原因：`plan_anchor_` 原来只在 **A* 分支**里赋值。
        //   而 realign 在 A* 之前就 return 了 → 用的是**上一次完整规划时的
        //   anchor**（t=101.5，飞机那时在 0.5m 后面）。
        //   → realign 虽然 1.3ms 就完成，但重建出的轨迹起点仍在旧位置
        //   → 参考点不回退，偏差自然不收敛
        //   → 这就是用户说的"重规划起点根本没在当前 odom"。
        //
        // 修法：在 `why` 确定之后、任何分支之前算一次 anchor。
        //   此刻 spinner 刚处理完上一批回调，`robot_state_` 是**最新的**。
        //
        //   前视时长：
        //     DEVIATED（realign，1~2ms）→ look = 0，就是当前位置
        //     完整规划（~270ms）        → look = plan_duration_ema_
        // ══════════════════════════════════════════════════════════════════
        {
            // rolling 模式：前视固定用 rolling_forward_dt（=SUPER 的
            // replan_forward_dt）。不用 plan_duration_ema_ 的原因：
            // 那是为“单次规划 0.5~1s、期间飞机已飞很远”设计的补偿；
            // rolling 单次只需 1~5ms，用 0.1s 前视就够，
            // 用 EMA 反而会把起点推到飞机前面（哪怕只 0.2m 也会累积成偏差）。
            const double look =
                np_.rolling_enable
                    ? (why == ReplanReason::DEVIATED ? 0.0 : np_.rolling_forward_dt)
                    : (why == ReplanReason::DEVIATED
                           ? 0.0
                           : std::max(0.0, plan_duration_ema_));
            computeAnchor(look, plan_anchor_, plan_anchor_v_);
            plan_anchor_valid_ = true;
        }

        if (why == ReplanReason::DEVIATED && !np_.rolling_enable) {
            // 有旧轨迹可复用时走"轻量重对齐"（只重做时间参数化，~5ms）；
            // 否则落到下面的完整规划。
            //
            // 轻量重对齐在 v1.6.1 被判定会死循环 —— 那同样是因为 anchor
            // 用参考状态。现在 anchor 用飞机位置，重对齐后参考点立刻回到
            // 飞机附近，下一周期不再 deviated。
            // ★★ realign 只在"旧路径仍贴近飞机"时才复用（2026-11-16 真 bug）
            //
            // 事故（/tmp/azj_flight_20261116_220405）：
            //   t=41.642  deviated 0.557m → realign 1.6ms
            //             C1 transition **0.815m**, start tangent (-0.009, 0.011)
            //   t=45.023  deviated 0.517m → realign 1.7ms
            //             C1 transition **0.885m**, start tangent (-0.367,-0.015)
            //
            //   为什么：realign 复用 `last_path_`（上一次完整规划产出的路径），
            //   但飞机已经偏离那条路径 0.8m（掉高 + 被挤开）。
            //   于是 buildTrajectory 要插一个 **0.8m 的 C1 过渡段**，
            //   而过渡段起点切向 ≈ 当前速度方向（飞机几乎停住）→ 方向近乎为零
            //   → 轨迹形状是"从一个点突然横移 0.8m 再走路径"
            //   → 控制器猛打 → 掉高。循环。
            //
            //   静止时为什么没事：v=0、飞机就在路径起点上，gap≈0。
            //   这正是用户说的"静止时规划没问题，重规划下起点判定有问题"。
            //
            // 修法：算一下旧路径离飞机多近。太远就**不 realign**，
            //       落到下面的完整规划（A* 会从当前位置重新搜）。
            bool realign_ok = (last_path_.size() >= 2 && has_trajectory_);
            if (realign_ok) {
                const Eigen::Vector3d p_now =
                    robot_state_.rcv ? robot_state_.p : Eigen::Vector3d::Zero();
                double best = std::numeric_limits<double>::infinity();
                for (const auto& q : last_path_) {
                    best = std::min(best, (q.head<2>() - p_now.head<2>()).norm());
                }
                if (best > np_.realign_max_gap) {
                    realign_ok = false;
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] skip realign: the old path is " << best
                                 << "m away from the aircraft (> " << np_.realign_max_gap
                                 << "m). Reusing it would inject a " << best
                                 << "m transition step. Running a full replan.");
                }
            }
            if (realign_ok) {
                const ros::WallTime t_re0 = ros::WallTime::now();
                if (buildTrajectory(last_path_)) {
                    const double ms_re =
                        (ros::WallTime::now() - t_re0).toSec() * 1e3;
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] realign (deviated): reference re-anchored to the "
                                 "aircraft, " << ms_re << " ms");
                    last_replan_wall_ = ros::WallTime::now();
                    return;
                }
            }
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] realign unavailable - running a full replan");
        }

        // ★ 重规划退避（2026-11-05 引入，2026-11-05 v1.3.2 修正）。
        //
        // 退避的目的只有一个：防止“规划→被拒→重规划”活锁把主线程跑满。
        // 它**绝不能**拦安全触发的重规划 —— 那是把防活锁机制变成了安全隐患。
        //
        // 实测问题（用户报“墙刚被扫出来但路径不避开”）：
        //   雷达扫出新墙 → needReplan 报 BLOCKED → 但正好落在 0.3s 退避窗口内
        //   → return，继续沿**旧轨迹飞向那面墙** → 再叠加 0.33~1.31s 的规划耗时，
        //   总延迟可达 1.6s（1m/s 下飞 1.6m）→ 穿墙。
        //
        // 现在按原因分级：
        //   BLOCKED  → 退避上限压到 blocked_replan_period（默认 0.10s，
        //              只用于防止同一帧内重复进入，不再是安全延迟）
        //   其他原因 → 用 min_replan_period（防活锁的本来用途）
        {
            const ros::WallTime now = ros::WallTime::now();
            double limit = (why == ReplanReason::BLOCKED)
                               ? np_.blocked_replan_period
                               : np_.min_replan_period;
            // ★ rolling 模式：退避降到 rolling_period（2026-11-17）
            //
            // min_replan_period=0.30 是为"单次规划 0.5~1s 的全量流程"设计的
            // 防活锁机制。rolling 单次只要 1~4ms，0.3s 的退避反而成了
            // 「滚动频率天花板」—— 实测只有 2.3~2.8Hz（/tmp/azj_roll_*，
            // 用无节流计数器测得；日志 THROTTLE 会让人误判成 0.8Hz）。
            //
            // 现在 rolling 用 rolling_period（0.1s → 10Hz）做退避，
            // CPU 占用实测 10Hz × 4ms = 4%，不存在活锁风险。
            if (np_.rolling_enable && np_.rolling_period > 1e-3) {
                limit = std::min(limit, std::max(0.05, np_.rolling_period));
            }
            if (!last_replan_wall_.isZero()) {
                const double dt = (now - last_replan_wall_).toSec();
                if (dt < limit) {
                    if (why == ReplanReason::BLOCKED) {
                        // 安全触发被退避：明确打日志，便于现场判断“是不是这里慢了”
                        ROS_WARN_STREAM_THROTTLE(
                            0.5, "[azj] BLOCKED but throttled by replan period (dt="
                                     << dt << "s < " << limit
                                     << "s) - still flying old trajectory!");
                    }
                    return;
                }
            }
            last_replan_wall_ = now;
        }

        const ClearanceField::Ptr field_holder = makeField(plan_2d_z_active_, start.z());
        const ClearanceField& field = *field_holder;

        // 起点诊断：失败时把现场状态打全，避免再花一轮去猜。
        {
            static int diag_cnt = 0;
            if (diag_cnt++ % 50 == 0) {
                const rog_map::Vec3f sq = start;
                double sd = 0.0;
                if (esdf_map_->insideLocalMap(sq)) esdf_map_->evaluateEDT(start, sd);
                ROS_INFO("[azj] start diag: pos=(%.2f,%.2f,%.2f) esdf=%.3f unknown=%d "
                         "clearance_used=%.3f robot_radius=%.2f",
                         start.x(), start.y(), start.z(), sd,
                         static_cast<int>(isUnknown(sq)), field.clearance(start),
                         sp_.robot_radius);
            }
        }

        // 目标点 clearance 诊断：目标本身贴障碍时，路径末端必然贴着它。
        // 用户容易误判为"规划器把路径贴着障碍走"。
        //
        // ★ 诊断要打全中间量。只打 `clearance=0` 会让人误以为是"目标贴障碍"，
        //   实际可能是 insideLocalMap 出界、UNKNOWN 分支、或 evaluateEDT 拿到
        //   未初始化格点。三种成因的处理方式完全不同，必须能一眼区分。
        {
            const double gc = field.clearance(goal);
            if (gc < sp_.robot_radius) {
                const rog_map::Vec3f gq = goal;
                const bool in_map = esdf_map_->insideLocalMap(gq);
                double raw_edt = -123.0;
                bool have_edt = false;
                if (in_map) {
                    esdf_map_->evaluateEDT(goal, raw_edt);
                    have_edt = true;
                }
                const bool unk = isUnknown(gq);
                ROS_WARN_STREAM_THROTTLE(
                    2.0, "[azj] goal clearance="
                             << gc << "m < robot_radius=" << sp_.robot_radius
                             << "m at (" << goal.x() << "," << goal.y() << "," << goal.z()
                             << ") | inside_map=" << in_map << " edt_raw="
                             << (have_edt ? std::to_string(raw_edt) : std::string("n/a"))
                             << " unknown=" << unk << " floor_z=" << np_.floor_z
                             << " ceil_z=" << np_.ceil_z);
            }
        }

        // ══════════════════════════════════════════════════════════════
        //  滚动窗口重规划（rolling，2026-11-17，对齐 SUPER）
        // ══════════════════════════════════════════════════════════════
        //
        // 与下面原有全量流程的区别：
        //   全量：anchor → goal 的 A* + 整条曲线优化，末点永远在 goal
        //   rolling：起点跟旧轨迹、末点只前探 lookahead 米，只优化这一小段
        //
        // 三个收益（对应用户的三条要求）：
        //   ① 末端引导居中 —— 末点每次重规划都重新做 clearance 梯度上升，
        //      ESDF 一变，下一次规划立刻在「末端」体现，不必整条重优化
        //   ② 速度连续 —— 起点硬约束 = 旧轨迹在未来 forward_dt 的 (p,v)，
        //      且 stop_at_end=false（末点不减速）→ 不出现「中途减速一下」
        //   ③ 快 —— A* 只在 goal 变/TTL 过期/剩余段被挡时重跑，
        //      曲线只优化 4m 而非 15m
        if (np_.rolling_enable && rollingReplan(field, why)) {
            return;
        }

        // ══════════════════════════════════════════════════════════════
        //  热启动：沿旧轨迹取引导段（学 SUPER 的 receding horizon）
        // ══════════════════════════════════════════════════════════════
        //
        // 详见 hot_init_reuse 参数的注释。要点：
        //   · 旧轨迹在这段是**优化过的平滑曲线**，直接作为初值 →
        //     曲线优化几步收敛，而不是从 45° 折线冷启动 60 次迭代
        //   · A* 搜索空间从「当前位置→目标」缩到「引导段末端→目标」
        //   · 引导段必须逐点复核 clearance（用真实飞行半径，不是监控阈值）
        std::vector<Eigen::Vector3d> hot_guide;
        {
            const auto snap_now = snapTraj();
            if (np_.hot_init_reuse > 1e-3 && !goal_is_new_ &&
                snap_now && snap_now->traj.valid()) {
                const double t_now = (ros::Time::now() - snap_now->start).toSec();
                const double t_end = std::min(t_now + np_.hot_init_reuse,
                                              snap_now->traj.duration());
                const double need_clr = sp_.robot_radius *
                                        std::max(0.1, np_.hot_init_clear_frac);
                // ★ 引导段最多吃掉"到目标距离的 50%"（2026-11-14）。
                //
                // 否则当目标较近时（实测 3.5m），引导段取满 hot_init_max_len=3.0m，
                // A* 只剩 0.5m 可规划 → 路径碎成 0.6m / 0.067m 的碎片，
                // 曲线优化在这么短的路径上 kmax 会爆到 40093（R=2.5e-5m）
                // → 反复被拒 → 退回折线 → 又触发新一轮。
                const double dist_to_goal = (start - goal).norm();
                const double guide_cap =
                    std::min(np_.hot_init_max_len, dist_to_goal * 0.5);
                if (guide_cap < np_.hot_init_min_len) {
                    // 目标太近，不需要热启动（A* 会直接给出整条路径）
                } else {
                Eigen::Vector3d prev = start;
                double acc = 0.0;
                for (double ts = t_now + 0.1; ts <= t_end + 1e-9; ts += 0.1) {
                    Eigen::Vector3d p, v, a;
                    if (!snap_now->traj.sample(ts, p, v, a)) break;
                    if (field.clearance(p) < need_clr) break;   // 前方被挡 → 引导段到此为止
                    acc += (p - prev).norm();
                    if (acc > guide_cap) break;
                    prev = p;
                    hot_guide.push_back(p);
                }
                if (!hot_guide.empty() &&
                    (acc < np_.hot_init_min_len || hot_guide.size() < 3)) {
                    hot_guide.clear();          // 太短没意义
                }
                if (!hot_guide.empty()) {
                    ROS_INFO_STREAM_THROTTLE(
                        2.0, "[azj] hot init: reusing " << acc
                             << "m of the committed trajectory as guide ("
                             << hot_guide.size() << " pts, cap "
                             << guide_cap << "m)");
                }
                }   // end if (guide_cap >= min_len)
            }
        }

        // ══════════════════════════════════════════════════════════════════
        //  A* 起点 = 新轨迹起点 = anchor（学 SUPER 的 pos_init_state）
        // ══════════════════════════════════════════════════════════════════
        //
        // SUPER 的做法：`pos_init_state`（旧轨迹在未来 replan_forward_dt 的状态）
        // 同时作为 **guide_path 起点** 和 **轨迹优化的起点约束** —— 是同一个点。
        //
        // 我们此前 A* 从"旧轨迹此刻位置"、而 buildTrajectory 的 anchor 用
        // "旧轨迹未来位置"，两者差 0.5~1.0m（正好是规划耗时飞过的距离）
        // → 路径起点落后 anchor → C1 过渡段要把 anchor 拉回 → 折返。
        //
        // 现在统一：A* 和轨迹都从 anchor 出发 → gap = 0 → 结构上不可能折返。
        // ══════════════════════════════════════════════════════════════════
        // ★ 复用回调开头算好的统一 anchor（2026-11-16）
        Eigen::Vector3d anchor = plan_anchor_valid_ ? plan_anchor_
                                                    : (robot_state_.rcv ? robot_state_.p
                                                                        : Eigen::Vector3d::Zero());
        Eigen::Vector3d anchor_v = plan_anchor_valid_ ? plan_anchor_v_
                                                      : Eigen::Vector3d::Zero();

        // anchor 必须在可通行区域（否则 A* 起点被判在障碍里）
        //
        // 外推点可能落在障碍里（转弯时直线外推会偏）。这时沿外推方向
        // 逐步回退到"最远的仍安全的点"，而不是直接退回飞机位置 —— 既尽量
        // 补偿飞行距离，又保证起点安全。
        Eigen::Vector3d astar_from = anchor;
        {
            const double need = sp_.robot_radius * 0.9;
            if (field.clearance(anchor) < need) {
                const Eigen::Vector3d p_now =
                    robot_state_.rcv ? robot_state_.p : Eigen::Vector3d::Zero();
                Eigen::Vector3d best = p_now;
                double best_c = field.clearance(best);
                for (double f = 1.0; f > 0.0; f -= 0.15) {
                    const Eigen::Vector3d cand = p_now + (anchor - p_now) * f;
                    const double c = field.clearance(cand);
                    if (c >= need) { best = cand; best_c = c; break; }
                    if (c > best_c) { best = cand; best_c = c; }
                }
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] anchor clearance too low - backed off to "
                             << best.transpose() << " (clearance " << best_c << ")");
                anchor = best;
                astar_from = best;
                plan_anchor_ = best;          // 让 buildTrajectory 也用这个
                anchor_v = plan_anchor_v_;
            }
        }

        const ros::WallTime t_astar0 = ros::WallTime::now();
        // ★ 把入场速度方向传给 A*，让它把首段拉直到速度方向
        //   （治栅格 45° 量化造成的急转，2026-11-16）
        const Eigen::Vector3d v_dir_in =
            (anchor_v.head<2>().norm() > 0.2) ? anchor_v : Eigen::Vector3d::Zero();
        const SearchResult res = astar_.search(field, astar_from, goal, sp_, v_dir_in);
        const double ms_astar = (ros::WallTime::now() - t_astar0).toSec() * 1e3;
        if (!res.success) {
            // 规划失败。
            //
            // ★ 2026-11-14 改进：若旧轨迹仍然有效（needReplan 刚判过前畅），
            //   **保留旧轨迹继续飞**，不要清 has_trajectory_。
            //   原来直接清标志 → 控制器 0.5s 后超时刹车 → 悬停等待 →
            //   用户看到"时不时停一下"。SUPER 这里有 backup traj 兜底，
            //   语义相同：不要因为新规划失败就把安全的旧轨迹丢掉。
            const bool old_known_bad = (why == ReplanReason::BLOCKED);
            if (old_known_bad) {
                // 旧轨迹已知有障碍 → 立刻生成**可控刹车轨迹**替换它。
                //
                // ★ 这是学 SUPER backup 的关键收益（2026-11-14）：
                //   原来这里是 `has_trajectory_ = false`（停发参考），
                //   控制器 0.5s 后超时 → PX4 自己急刹 → 姿态抖、可能掉高。
                //   现在主动生成一条**有速度规划**的刹车轨迹：
                //     沿当前速度方向、以 max_acc*0.5 匀减速到静止。
                //   控制器跟踪它 → 减速过程平滑可控。
                if (!tryBrakeReplan(res.msg)) {
                    has_trajectory_ = false;
                    ROS_ERROR_STREAM_THROTTLE(
                        1.0, "[azj] SAFETY: plan failed (" << res.msg
                             << ") AND brake generation failed - stopping reference");
                }
            } else if (snapTraj() && snapTraj()->traj.valid()) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] plan failed (" << res.msg
                         << ") - keeping the committed trajectory (still safe)");
            } else if (!tryBrakeReplan(res.msg)) {
                has_trajectory_ = false;
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] plan failed: " << res.msg);
            }
            return;
        }

        // ---------- 折线 → 曲线 ----------
        //
        // 之前只有 simplify + smoothPath（顶点移动平均），本质上还是折线：
        // 实测转角中位 45°、最大 92°，且加大平滑迭代无效（因为顶点间仍是直线）。
        // 现在改用 ESDF 梯度驱动的三次 B 样条优化，得到 C2 连续曲线。
        //
        // ══════════════════════════════════════════════════════════════════
        // ★★ 不再把 hot_guide 拼进路径（2026-11-14 重大修正）
        //
        // 上一版把 hot_guide（旧轨迹采样）+ A* 段拼成优化初值，理由是"初值平滑"。
        // 但实测这个拼接会**错位**并制造反向 S 弯：
        //
        //   /tmp/azj_flight_20261114_152514：
        //     hot_guide 从 t=43.199（规划开始）的旧轨迹取 1.27m
        //     但规划耗时 1.35s，结束时飞机已前进 1.3m
        //     → hot_guide 整体落后 1.3m
        //     → anchor（旧轨迹在 t_now 的状态）落在 hot_guide **之外**
        //     → C1 过渡段要把 anchor 拉回 hot_guide 起点 → 反向 S 弯
        //     → `curve retry: kmax 11503 -> 5.78`（11503 就是 S 弯的曲率爆炸）
        //     → 额外一次完整优化 → 单次规划 1347ms
        //     → 新路径起点切向往后，与速度 1 m/s 反向 → v_start 投影成 0
        //     → 轨迹从静止起步但飞机有动量 → 掉高
        //
        // 现在：A* **直接从 anchor 规划**，路径从 anchor 出发朝目标，
        // 天然没有折返。hot_guide 只保留一个用途 —— 判断"前方是否畅通"
        // （决定要不要走 fast_path），不再参与几何拼接。
        //
        // 代价：A* 搜索空间从 ~2m 回到 ~4m，实测 A* 只要 2~4ms，可忽略。
        // 收益：kmax 不再爆炸、不再需要 retry、单次规划回到 ~900ms。
        // ══════════════════════════════════════════════════════════════════
        std::vector<Eigen::Vector3d> merged_raw = res.path;

        // ★★ 路径首点强制设为 anchor（2026-11-14，消除横向 gap）
        //
        // A* 的 path[0] 是**起点栅格中心**，与 anchor 相差最多 0.07m
        // （resolution 0.1 的对角）。这 0.07m 的横向偏差必须由轨迹的
        // 首段"横移"吸收，而 0.07m / 0.07s 需要 1.4 m/s 的横向速度 ——
        // 物理上不可能（纵向还要保持 1 m/s）→ 速度回摆 vy=-0.34。
        //
        // 实测（单元测试 [12]/[13]）：gap 0.07m 时 vy 出现 -0.34 的凹陷；
        // 而 gap = 0 时完全平滑。
        //
        // 所以：保留 A* 的后续点，只把首点替换成精确的 anchor。
        // 这是 SUPER 的 guide_path[0] = pos_init_state.p 的直接对应。
        if (plan_anchor_valid_ && merged_raw.size() >= 2) {
            merged_raw[0] = plan_anchor_;
        }
        (void)0;   // use_guide 已废弃：A* 从旧轨迹当前位置规划

        // 中间点去重（相邻点间距 < 1cm 合并），避免 B 样条初值退化
        std::vector<Eigen::Vector3d> dedup_raw;
        dedup_raw.reserve(merged_raw.size());
        for (const auto& q : merged_raw) {
            if (dedup_raw.empty() || (q - dedup_raw.back()).norm() > 0.01) {
                dedup_raw.push_back(q);
            }
        }
        if (dedup_raw.size() < 2) dedup_raw = merged_raw;

        std::vector<Eigen::Vector3d> path = simplifyPath(dedup_raw);

        // ★ 安全快通道（v1.3.2）：新障碍侵入轨迹时跳过曲线优化。
        //
        // 实测耗时对比（同一场景）：
        //     A*          13 ~ 25 ms
        //     曲线优化    327 ~ 1308 ms     ← 40~80 倍
        //
        // 用户现场问题：“雷达下一帧扫出来了，路径还是没及时避开”。
        // 延迟链路：退避 0.3s + 规划 1.3s ≈ 1.6s（1m/s 下飞 1.6m）→ 穿墙。
        //
        // 而本项目**只有单线程 spinner**（ros::spin()），规划在 replanCallback 里
        // 同步执行，期间 pos_cmd 断流、点云回调也停摆（实测断流 >1s 有 17 次）。
        // 所以缩短这段同步时间本身就是安全收益。
        //
        // 取舍：急时就别管平滑了。折线转角大（实测中位 45°），但**不撞**优先。
        // 这条快路径跑完后，下一次常规重规划（TRAJ_END）会出完整曲线替换它。
        // 可用 `azj/blocked_fast_path: false` 关掉，退回旧行为。
        // ★ 有热启动引导段时**不走** fast path（2026-11-14）。
        //
        // 这是消除"折线/曲线震荡"的最后一块：
        //   fast_path 的意义是"紧急时跳过 0.9~1.1s 的曲线优化，30ms 出折线"。
        //   但有了热启动，曲线优化的初值已经是旧轨迹（平滑），
        //   35 次迭代实测只要 0.3~0.6s —— 快通道省下的时间已经没那么关键，
        //   而代价是用户看到折线、然后 0.7s 后又被曲线替换（就是震荡）。
        //
        // 所以：能把引导段拿到（说明旧轨迹前方畅通到某处）→ 直接做快速曲线；
        //       拿不到（真正的新场景/近处被挡）→ 才走 fast path 折线。
        const bool fast_path = np_.blocked_fast_path && (why == ReplanReason::BLOCKED) &&
                               hot_guide.empty();

        // ★ 极短路径跳过曲线优化（2026-11-14）。
        //
        // 实测 `curve rejected: kmax=40093.7 (R=2.49e-05m)` 出现在
        // `fallback polyline: len=0.0669m` 这种碎路径上 ——
        // 路径只有 6.7cm（3~4 个采样点），曲率估计的差分在数值上完全不可靠，
        // 必然报超大 kmax → 被拒 → 退回折线 → 又因为"被拒"触发新重规划。
        //
        // 0.8m 以内的路径本身就是"最后一段直线减速"，曲线优化没有收益。
        const bool path_too_short = pathLength(path) < 0.8;
        if (path_too_short) {
            ROS_INFO_STREAM_THROTTLE(
                2.0, "[azj] path is " << pathLength(path)
                         << "m (< 0.8m) - skipping curve opt (numerically unstable kmax)");
        }

        if (np_.curve_enable && !fast_path && !path_too_short) {
            cp_.safe_margin = sp_.robot_radius;
            cp_.clearance_ref = sp_.clearance_ref;
            cp_.clearance_cap = sp_.clearance_cap;
            cp_.alpha = sp_.alpha;
            cp_.lock_z = plan_2d_active_;   // 2D：曲线 z 必须恒定
            cp_.lock_z_value = plan_2d_z_active_;   // 锁到规划平面（不是飞机当前高度）
            // 层高硬边界也要传进曲线优化：否则平滑/居中会把曲线推出
            // A* 的可行域（实测 floor_z=-0.10 时曲线顶到 -0.103）
            if (np_.enforce_height_limits) {
                cp_.enable_z_bounds = true;
                cp_.z_min = np_.floor_z;
                cp_.z_max = np_.ceil_z;
            }

            // ★ 曲线优化也要有高度保持（2026-11-05 新增）。
            //
            // 只给 z_min/z_max 硬边界是不够的：优化会为了更大 clearance
            // 一路把 z 推到天花板边界（实测 path z=[0.05,0.999289]），
            // 飞机跟着冲到 1.0076m 再被拽回 —— 用户报的"打点后冲高再掉高"。
            //
            // 这里与 A* 用**同一套**高度参数（w_height / height_ref / height_scale），
            // 保证两级规划的高度意图一致：A* 压住 0.85，曲线也就不会推走。
            cp_.w_height = np_.w_height;
            // ★ height_ref 故意**不设**（保持 NaN）：让曲线优化锚定 A* 的 z 形状，
            //   而不是绝对目标高度。见 curve_optimizer.cpp 里的详细说明 ——
            //   设成 goal.z 会把起点在低处的路径强行拉高，kmax 爆炸到 781
            //   （限值 20），全部被拒并回落折线。
            cp_.height_scale = sp_.height_scale;

            // ★ 快通道后的平滑用"快速版"迭代数（2026-11-14）。
            //
            // 为什么：曲线优化 60 次迭代实测 0.9~1.1 秒，而 fast path 之后的
            // 平滑是**补救性**的 —— 飞机此刻还在跟折线飞，越早出曲线越好。
            // 用快速版把耗时压到 ~0.4s，先让一条合格曲线尽快上线；
            // 后续常规重规划（TRAJ_END）会用完整迭代数精修。
            //
            // 只在 smooth_forced 时生效，常规规划仍用 curve_iterations。
            // ★ BLOCKED（安全事件）也用快速迭代（2026-11-14）。
            //
            // 实测：BLOCKED 时用完整 60 次迭代耗时 900~1347ms，
            // 期间飞机前进 0.9~1.3m，外推必然失真（实测误差 0.96m）。
            // 快速版 35 次约 450~550ms，把失真压到一半。
            // 后续常规重规划（TRAJ_END）仍会用 60 次精修。
            //
            // 这是"安全响应速度"与"轨迹平滑度"的权衡 ——
            // 对 BLOCKED 场景，响应速度优先（安全优先原则）。
            const bool quick = (smooth_forced || why == ReplanReason::BLOCKED) &&
                               np_.curve_iterations_quick > 0;
            const int iters_backup = cp_.iterations;
            if (quick) {
                cp_.iterations = std::min(cp_.iterations,
                                          np_.curve_iterations_quick);
                ROS_INFO_STREAM("[azj] quick smoothing: iterations "
                                    << cp_.iterations << " (fast-path recovery)");
            }

            // ---- 优化 ----
            const ros::WallTime t_cv0 = ros::WallTime::now();
            CurveOptResult cr = curve_opt_.optimize(path, field, cp_);
            cp_.iterations = iters_backup;
            const double ms_cv = (ros::WallTime::now() - t_cv0).toSec() * 1e3;
            ROS_INFO_STREAM_THROTTLE(5.0, "[azj] T {astar=" << ms_astar
                                    << "ms curve=" << ms_cv << "ms ncp="
                                    << cr.iterations << "}");

            // kmax 超限时先"加重平滑"重试一次，而不是直接退回折线。
            // 很多情况下加大 w_smooth 就能把曲率压进限制；直接 fallback
            // 会让用户看到难看的锯齿折线（实测 raicom 紧缝路径被拒成 8 点折线）。
            // ★ 只在"真不可接受"时才重试（2026-11-14 性能修正）。
            //
            // 原来条件是 kmax > max_curvature(20) 就无条件重试。
            // 但 v1.4.4 起，kmax <= max_curvature×curvature_tolerance(60)
            // 是**可以接受**的（保留曲线 + 降速跟踪）。
            // 所以 kmax=36.97 这种本来就能用，却白白多跑一次优化 ——
            // 实测 44.953 那次：curve retry 让耗时从 ~550ms 翻到 **1100ms**，
            // 而飞机在 1.1 秒里飞了 0.96m，导致新轨迹起点直接过期。
            const double k_accept = (np_.max_curvature > 1e-6)
                                        ? np_.max_curvature *
                                              std::max(1.0, np_.curvature_tolerance)
                                        : 0.0;
            if (cr.success && np_.max_curvature > 1e-6 &&
                cr.max_curvature > k_accept) {
                CurveOptParams cp2 = cp_;
                cp2.w_smooth = cp_.w_smooth * 6.0;
                const CurveOptResult cr2 = curve_opt_.optimize(path, field, cp2);
                if (cr2.success && cr2.max_curvature < cr.max_curvature) {
                    ROS_INFO_STREAM_THROTTLE(
                        1.0, "[azj] curve retry (w_smooth x6): kmax "
                                 << cr.max_curvature << " -> " << cr2.max_curvature);
                    cr = cr2;
                }
            }

            if (cr.success) {
                std::vector<Eigen::Vector3d> pts, tgs;
                const bool resampled =
                    cr.spline.sampleByArcLength(np_.curve_sample_step, pts, tgs) &&
                    pts.size() >= 2;
                if (!resampled) {
                    ROS_WARN_THROTTLE(1.0, "[azj] curve resample failed, keep polyline");
                    applyPolylineFallback(res, field, &dedup_raw);
                    return;
                }

                // ---------- ① 安全校验（最高优先级）----------
                // 曲线优化对安全只用 hinge 软惩罚，平滑/居中项会把曲线拽到
                // 障碍附近（实测出现过 min_clearance=1.4mm）。
                // 排除终点前一段：目标点是用户指定的硬约束，它本身贴障碍时
                // 路径必须能到达，不能因此判整条曲线不安全。
                auto safeOf = [&](const std::vector<Eigen::Vector3d>& q) {
                    return minClearanceAlongPathExcludingEnd(
                        q, field, sp_.robot_radius + 0.3, 0.02);
                };
                double mc = safeOf(pts);

                // ★ 安全失败不直接放弃，而是递进加重安全项重试。
                //
                // 原来一旦 `mc < robot_radius*0.9` 就退回折线。但长路径下
                // （实测 15m → 56 控制点，固定 150 迭代不够）曲线只是没收敛，
                // 否定它退回的折线反而更不安全（实测折线 min_clr=0.153m
                // < robot_radius=0.30m）。
                //
                // 递进重试：w_obs × 10 → × 50，同时投影迭代加倍并加密采样，
                // 每次都用同一套安全标准复核；仍不过才退回折线。
                // 代价：最多 2 次额外优化（百毫秒量级），只在不安全时触发。
                if (mc < sp_.robot_radius * 0.9) {
                    for (const double boost : {10.0, 50.0}) {
                        CurveOptParams cpb = cp_;
                        cpb.w_obs = cp_.w_obs * boost;
                        cpb.projection_iters =
                            std::max(cp_.projection_iters * 2, 6);
                        cpb.projection_samples =
                            std::max(cp_.projection_samples, 400);
                        const CurveOptResult cb = curve_opt_.optimize(path, field, cpb);
                        if (!cb.success) continue;
                        std::vector<Eigen::Vector3d> pb, tb;
                        if (!cb.spline.sampleByArcLength(np_.curve_sample_step, pb, tb) ||
                            pb.size() < 2)
                            continue;
                        const double mcb = safeOf(pb);
                        if (mcb > mc) {
                            ROS_INFO_STREAM_THROTTLE(
                                1.0, "[azj] curve safety retry w_obs x"
                                         << boost << ": min_clr " << mc << " -> " << mcb);
                            cr = cb;
                            pts = pb;
                            tgs = tb;
                            mc = mcb;
                        }
                        if (mc >= sp_.robot_radius * 0.9) break;
                    }
                }

                if (mc < sp_.robot_radius * 0.9) {
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] curve REJECTED: min_clearance="
                                 << mc << "m < robot_radius*0.9=" << (sp_.robot_radius * 0.9)
                                 << "m - falling back to polyline");
                    applyPolylineFallback(res, field, &dedup_raw);
                    return;
                }

                // ---------- ② 曲率校验（2026-11-14 改为分级）----------
                //
                // ★ 原来这里是硬拒绝：kmax > max_curvature 就退回折线。
                //   实测代价（raicom 3D，/tmp/azj_flight_20261114_130456）：
                //
                //     curve retry (w_smooth x6): kmax 65.7018 -> 22.0075
                //     curve rejected: kmax=22.0075 > limit 20 (R=0.0454m)
                //       - falling back to polyline
                //     fallback polyline: len=1.55702m min_clr=0.11082
                //
                //   只超 10% 就把整条曲线丢掉，换回来的折线 clearance 只有
                //   **0.111m**（robot_radius=0.30）—— 比曲线危险得多。
                //   而曲线本身已经通过了 ① 的安全校验（mc >= 0.27m）。
                //
                //   这是自相矛盾的取舍：为了"曲率好看"牺牲安全指标。
                //   而且配置注释自己就写了"高曲率本身不等于不安全"——
                //   trajectory.cpp 按 `v = sqrt(max_lat_acc/kappa)` 自动降速，
                //   kmax=22 → v=0.23 m/s。慢，但对 1.5m 短路径完全可接受。
                //
                // 分级：
                //   kmax <= limit                → 正常通过
                //   limit < kmax <= limit*K_TOL  → 警告但**保留曲线**（降速跟踪）
                //   kmax >  limit*K_TOL          → 真跟不了，退回折线
                // K_TOL=3 时：limit 20 → 容差 60（R=1.7cm），再小确实没法跟。
                if (np_.max_curvature > 1e-6 && cr.max_curvature > np_.max_curvature) {
                    const double k_tol = std::max(1.0, np_.curvature_tolerance);
                    if (cr.max_curvature > np_.max_curvature * k_tol) {
                        ROS_WARN_STREAM_THROTTLE(
                            1.0, "[azj] curve rejected: kmax="
                                     << cr.max_curvature << " > limit*" << k_tol << "="
                                     << np_.max_curvature * k_tol << " (R="
                                     << 1.0 / cr.max_curvature
                                     << "m) - too tight to track, falling back to polyline");
                        applyPolylineFallback(res, field, &dedup_raw);
                        return;
                    }
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] curve kept despite kmax="
                                 << cr.max_curvature << " > limit " << np_.max_curvature
                                 << " (R=" << 1.0 / cr.max_curvature
                                 << "m): safety passed, speed will be limited to ~"
                                 << std::sqrt(tp_.max_lat_acc / cr.max_curvature)
                                 << " m/s. (polyline fallback would have clearance "
                                 << res.min_clearance << "m)");
                }

                // ---------- 通过校验 ----------
                path = pts;
                last_max_curvature_ = cr.max_curvature;

                {
                    double zmin = 1e9, zmax = -1e9;
                    for (const auto& q : pts) {
                        zmin = std::min(zmin, q.z());
                        zmax = std::max(zmax, q.z());
                    }
                    ROS_INFO_STREAM_THROTTLE(
                        1.0, "[azj] path z=[" << zmin << "," << zmax << "] span="
                                              << (zmax - zmin) << " lock_z="
                                              << static_cast<int>(cp_.lock_z));
                }
                ROS_INFO_STREAM_THROTTLE(1.0, "[azj] curve: iters="
                                                  << cr.iterations << " len=" << cr.length
                                                  << "m min_clr=" << cr.min_clearance
                                                  << "m kmax=" << cr.max_curvature);
            } else {
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] curve opt failed: "
                                                  << cr.msg << " - falling back to polyline");
                applyPolylineFallback(res, field, &dedup_raw);
                return;
            }
        } else {
            path = smoothPath(path, field, sp_.robot_radius, 3, 0.35);
            if (fast_path) {
                last_fast_path_wall_ = ros::WallTime::now();
                ROS_WARN_THROTTLE(
                    0.5, "[azj] BLOCKED -> fast path (skip curve opt, ~30ms instead of ~1.3s); "
                         "smoothing replan scheduled");
            }
        }

        const double min_c = minClearanceAlongPath(path, field, 0.02);
        last_min_clearance_ = min_c;
        last_path_length_ = pathLength(path);
        last_path_ = path;
        // 已产出可用路径 → 后续重规划可正常热启动复用旧轨迹
        goal_is_new_ = false;

        // ★ 记录本次规划总耗时，供下次计算"前视时长"（2026-11-14，学 SUPER）
        //
        // SUPER 用固定 replan_forward_dt=0.1s，因为它规划只要 0.3ms。
        // 我们单次 0.5~1.0s → 必须用**实测耗时**当前视，
        // 否则新轨迹起点还是落在"飞机已经飞过的地方"。
        {
            const double dt_now = plan_start_wall_.isZero()
                                      ? 0.0
                                      : (ros::WallTime::now() - plan_start_wall_).toSec();
            if (dt_now > 1e-3 && dt_now < 3.0) {
                plan_duration_ema_ = (plan_duration_ema_ < 1e-6)
                                         ? dt_now
                                         : (0.7 * plan_duration_ema_ + 0.3 * dt_now);
                ROS_INFO_STREAM_THROTTLE(
                    2.0, "[azj] plan duration: this=" << dt_now
                             << "s  ema=" << plan_duration_ema_
                             << "s  (used as replan look-ahead)");
            }
        }

        if (!buildTrajectory(path)) {
            // ★★ 能不能"保留旧轨迹"取决于旧轨迹是否仍然安全（2026-11-14 修正）
            //
            // 事故背景（/tmp/azj_flight_20261114_150005）：
            //   t=44.953 `reference blocked ahead` → 旧轨迹已被判定有障碍
            //   t=46.154 重规划出的路径被拒 → 走这个分支 → **保留旧轨迹**
            //   → 飞机沿**已知撞墙**的旧轨迹继续飞 → 撞上
            //
            // "保留旧轨迹"只在旧轨迹**没被判 blocked** 时才成立。
            // 安全优先的原则很清楚：
            //   一个"形状不完美但能绕开障碍"的新路径
            //   ≫ 一个"平滑但会撞墙"的旧轨迹。
            const bool old_known_bad = (why == ReplanReason::BLOCKED);
            if (old_known_bad) {
                // 旧轨迹已知有障碍：换成可控刹车（见 tryBrakeReplan 的说明）
                if (!tryBrakeReplan("new path rejected")) {
                    has_trajectory_ = false;
                    ROS_ERROR_STREAM_THROTTLE(
                        1.0, "[azj] SAFETY: new path rejected AND brake failed - stopping");
                }
            } else if (snapTraj() && snapTraj()->traj.valid()) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] trajectory build rejected - keeping the committed one "
                         "(it was not reported blocked)");
            } else {
                has_trajectory_ = false;
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] trajectory build failed");
            }
            return;
        }
        has_trajectory_ = true;
        publishPathViz();

        {
            const auto snap = snapTraj();
            ROS_INFO_STREAM_THROTTLE(
                1.0, "[azj] len=" << last_path_length_ << "m  min_clearance=" << min_c
                                  << "m (safe " << sp_.robot_radius << "m)  traj="
                                  << (snap ? snap->traj.duration() : 0.0) << "s  vpk="
                                  << (snap ? snap->traj.maxVelocity() : 0.0) << "m/s");
        }
    }

    void publishTick() {
        // 到达后继续发 COMPLETED，而不是静默停止。
        // 原因：api_3d.cpp 收到 COMPLETED 才会走 nav_hold_active_ 分支
        // （锁存当前位置悬停、速度归零）。静默不发则控制器要等 0.5s
        // traj_timeout 才刹车，这期间会按最后的参考速度继续飞 —— 实测过冲 0.176m。
        // 本函数运行在**独立发布线程**，与规划线程并发。
        // 所有跨线程状态都在这里一次性快照，之后完全不持锁。
        bool arrived_local = false;
        bool has_goal_local = false;
        std::shared_ptr<const TrajSnapshot> snap;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            arrived_local = arrived_;
            has_goal_local = has_goal_;
            snap = active_;
        }

        if (arrived_local) {
            publishCompleted();
            return;
        }
        // snap 为 nullptr 等价于"无可用轨迹"：静默返回（控制器会自行超时刹车）
        if (!has_goal_local || !snap || !snap->traj.valid()) return;

        const double t = (ros::Time::now() - snap->start).toSec();
        Eigen::Vector3d p, v, a;
        if (!snap->traj.sample(t, p, v, a)) return;

        rog_map::Vec3f goal_local;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            goal_local = goal_;
        }

        // ★★★ 轨迹过期兜底（2026-11-17 严重 bug）
        //
        // 事故（/home/spadea/azj_flights/azj_flight_20261117_214632）：
        //   t=8.4~9.5 参考速度塌到 **0.00**，飞机还在 0.22~0.49 滑行，
        //   用户报"中间有一段时间轨迹不跟着无人机走了"。
        //
        // 机制：控制器 type_mask=3043 → **XY 是速度控制**
        //   （PX/PY 被 IGNORE）。也就是说飞机只认 cmd.velocity，
        //   位置误差完全不起作用。
        //   而轨迹一旦跑完（t >= duration），`sample()` 按 ends_at_rest
        //   返回 v=0 → 飞机收到"零速"指令 → **原地停住**，
        //   哪怕离目标还有 2 米。
        //
        //   轨迹为什么会跑完而没到目标：这个场景是 `to_goal=true`
        //   （stop_at_end），MINCO 把终点定在 goal 且末速为 0。
        //   飞机实际落后参考 0.2~0.4m，等轨迹时间轴走完时飞机还没到位
        //   → 参考已经"到站归零" → 飞机停在半路。
        //
        // 修法：过期且未到达时，**不要发零速**，而是按"到目标的剩余方向"
        //   给一个保底速度，让飞机继续靠近；同时打日志提示重规划失败。
        //   这不是"接管控制"，只是防止规划器断流导致飞机停摆 ——
        //   语义与 SUPER 的 backup/stop 段一致（保证永远有一条可行参考）。
        if (!arrived_local && t >= snap->traj.duration() - 1e-3) {
            const Eigen::Vector3d to_goal(goal_local.x() - p.x(),
                                          goal_local.y() - p.y(),
                                          goal_local.z() - p.z());
            const double dh = std::hypot(to_goal.x(), to_goal.y());
            if (dh > 0.05) {
                const double vcap = std::max(0.05, np_.traj_expired_min_vel);
                const double vmag = std::min(vcap, dh * 0.8);
                v = Eigen::Vector3d(to_goal.x() / dh * vmag,
                                    to_goal.y() / dh * vmag, 0.0);
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] trajectory EXPIRED but not arrived (t=" << t << "s / "
                             << snap->traj.duration() << "s, dist=" << dh
                             << "m) - publishing fallback velocity " << vmag
                             << " m/s toward goal instead of 0 (XY is velocity-controlled, "
                             << "a zero command would park the drone mid-air)");
            }
        }

        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            last_ref_pos_ = p;
        }

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = np_.frame_id.empty() ? cfg_.frame_id : np_.frame_id;
        if (cmd.header.frame_id.empty()) cmd.header.frame_id = "map";

        cmd.position.x = p.x();
        cmd.position.y = p.y();
        cmd.position.z = p.z();
        // ---- 输出速度：z 限幅 + 重规划淡入 ----
        //
        // 为什么：控制器的 type_mask 未设 IGNORE_VZ（nav_mode=3 时
        // position.z 与 velocity.z 同时有效），规划器给的 velocity.z
        // 会被当作速度前馈叠加。重规划瞬间若 vz 阶跃，
        // 飞机会被"拽一下" —— 表现为瞬间掉高。
        //
        // 两道保护：
        //   ① 硬限幅：|vz| <= max_vel_z_out
        //   ② 淡入：新轨迹前 vel_blend_time 秒内，把参考速度
        //      从"生成时的实际速度"线性过渡到轨迹速度，消除阶跃。
        Eigen::Vector3d v_out = v;

        if (np_.max_vel_z_out > 1e-6) {
            v_out.z() = std::min(std::max(v_out.z(), -np_.max_vel_z_out),
                                 np_.max_vel_z_out);
        }

        // ★★★ 滚动模式下禁用速度淡入（2026-11-17 真 bug）
        //
        // 事故（/home/spadea/azj_flights/azj_flight_20261117_214632）：
        //   参考速度在中段塌到 0.03~0.06，而飞机还在 0.49~0.70 滑行 ——
        //   用户报"中间有一段时间轨迹不跟着无人机走了"。
        //   实测 v_cmd/v_act 中位 0.76（正常应 ≈1.0）。
        //
        // 根因：`vel_blend_time = 0.35`，而 rolling 每 **0.1s** 重建一次轨迹。
        //   每条新轨迹的 tnow 最多到 0.1s，于是
        //       blend = tnow / 0.35 ≈ 0~0.29（均值 0.14）
        //       v_out = blend × 轨迹速度 + (1-blend) × snap->v0
        //   而 `snap->v0` 是**飞机实际速度**（commitTrajectory 里赋值）。
        //   → 参考速度被永久拽向"飞机现在多快"，规划想让飞机加速的部分
        //     有 86% 被丢掉 → 参考永远提不起速 → 飞机只能滑行。
        //
        //   blend 的原始用途（见下方历史注释）是给 **B 样条**路径用的：
        //   那条路起点速度可能阶跃。而 MINCO 用 `setConditions(headPVAJ)`
        //   把起点 p/v/a/j 全部硬约束，轨迹起点速度**就是**上一帧的规划速度，
        //   本身已经连续 → blend 此时不但无用，还会主动破坏连续性。
        //
        // 判据：只要 rolling 生效且用 MINCO 后端，就不做淡入。
        const bool blend_redundant = np_.rolling_enable && (np_.traj_backend == 1);

        // 淡入：t 从 0 到 vel_blend_time
        double blend = 1.0;
        if (np_.vel_blend_time > 1e-6 && !blend_redundant) {
            const double tnow = (ros::Time::now() - snap->start).toSec();
            blend = std::min(1.0, std::max(0.0, tnow / np_.vel_blend_time));
        }
        if (blend < 1.0) {
            // 生成轨迹时记下的起点速度，向轨迹速度插值
            v_out = blend * v_out + (1.0 - blend) * snap->v0;
            if (np_.max_vel_z_out > 1e-6) {
                v_out.z() = std::min(std::max(v_out.z(), -np_.max_vel_z_out),
                                     np_.max_vel_z_out);
            }
        }

        // ══════════════════════════════════════════════════════════════════
        // ★★★ 输出速度参考的**变化率限制**（2026-11-16，治重规划阶跃）
        //
        // 事故（/tmp/azj_flight_20261116_220405）：
        //   t=40.20  refV (-0.22,-0.97)  tid=1
        //   t=40.25  refV (-0.74,-0.48)  tid=2   ← 方向差 41°，0.05s 内
        //   → 变化率 (0.52,0.49)/0.05 ≈ **14 m/s²**，远超 max_acc = 1.5
        //   → 控制器 sp 从 (0.79,-0.93) 翻到 (-1.43,-0.10)
        //   → roll 6.5° → -17.5°，推重比 1.32 撑不住
        //   → z 0.774 → 0.001（1 秒落地）
        //
        // 根因链：新轨迹的起点速度方向取决于**新路径首段切向**
        //   （A* 栅格 45° 增量 + anchor_v 投影），而飞机实际运动方向
        //   可能在别的方位 → 两者夹角几十度 → 参考速度方向突变。
        //
        // 无论轨迹怎么生成，**发布出去的参考速度必须是物理可达的**。
        // 飞机的加速度能力就是 max_acc —— 参考跑得比飞机能跟的更快，
        // 控制器只能饱和输出，姿态被拽偏，掉高。
        //
        // 所以在这里做**参考整形（reference shaping）**：
        //   |v_out - v_prev_pub| <= acc_limit * dt
        // 这是标准做法：先保证参考可行，再谈跟踪精度。
        //
        // ⚠ 上限用 max_acc × 1.5 而不是 1.0：留一点余量，
        //   避免把正常轨迹的加速段也削平（轨迹本身已按 max_acc 规划）。
        if (np_.ref_vel_rate_limit > 0.0 && last_pub_vel_valid_) {
            static ros::Time last_lim_t;
            const double dt_lim = last_lim_t.isZero()
                                      ? (1.0 / std::max(1.0, np_.publish_rate))
                                      : std::max(1e-3, std::min(0.05,
                                            (ros::Time::now() - last_lim_t).toSec()));
            last_lim_t = ros::Time::now();
            const double dmax = np_.ref_vel_rate_limit * dt_lim;
            const Eigen::Vector3d dv = v_out - last_pub_vel_;
            const double dn = dv.norm();
            if (dn > dmax) {
                v_out = last_pub_vel_ + dv * (dmax / dn);
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] reference velocity step limited: was "
                             << dn << "m in " << dt_lim << "s (= " << (dn / dt_lim)
                             << " m/s^2), clamped to " << np_.ref_vel_rate_limit
                             << " m/s^2. A trajectory switch would otherwise drag "
                             << "the attitude and lose altitude.");
            }
        }
        last_pub_vel_ = v_out;
        last_pub_vel_valid_ = true;

        cmd.velocity.x = v_out.x();
        cmd.velocity.y = v_out.y();
        cmd.velocity.z = v_out.z();
        // ══════════════════════════════════════════════════════════════════
        // ★★★ 加速度前馈必须钳制到物理能力内（2026-11-16 真 bug）
        //
        // 事故（/tmp/azj_flight_20261116_224359）：
        //   t=35.650  refAx  1.57  refAy  0.44  tid=1
        //   t=35.675  refAx  9.55  refAy **13.40**  tid=2   ← 轨迹切换瞬间
        //   t=35.875  refAx **-21.69** refAy -11.76
        //
        //   控制器 `nav_ff_acc_ * planner_cmd_.acceleration` 把它当前馈，
        //   于是 spVy 在 0.05s 内从 -1.25 跳到 **+1.11**（2.36 m/s 阶跃）
        //   → roll 从 5.1° 转到 **-48.6°** → cos(48.6°)=0.66
        //   → 垂直推力 23.36×0.66 = 15.4N < 重力 17.72N → 必然掉高。
        //
        // 为什么会有 13/21：`acc_` 是采样点差分来的，
        //   buildSmoothFromState 里还把 `vel_.back()` 强制置 0
        //   → 若前一采样点速度非零，0.02s 内跳 1 m/s → acc = 50。
        //   单元测试实测（/tmp/tacc2.cpp）：
        //     反向入180°  max|a| = **402.6**
        //     斜向入/急转  max|a| = 63~64
        //   一律超标 40~270 倍。
        //
        // 双保险：
        //   ① 这里钳制（无论轨迹层怎么生成，控制器都不会看到超能力前馈）
        //   ② 轨迹层修端点（见 trajectory.cpp）
        // ══════════════════════════════════════════════════════════════════
        Eigen::Vector3d a_out = a;
        if (np_.acc_feedforward_limit > 0.0) {
            const double an = a_out.norm();
            if (an > np_.acc_feedforward_limit) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] acceleration feed-forward clamped: |a|=" << an
                             << " m/s^2 -> " << np_.acc_feedforward_limit
                             << ". A spike this large at a trajectory switch drags the "
                             << "attitude (sp saturation) and loses altitude.");
                a_out *= np_.acc_feedforward_limit / an;
            }
        }
        cmd.acceleration.x = a_out.x();
        cmd.acceleration.y = a_out.y();
        cmd.acceleration.z = a_out.z();

        // dt 用实际发布间隔
        static ros::Time last_pub_t;
        const ros::Time now_t = cmd.header.stamp;
        double dt = last_pub_t.isZero() ? (1.0 / std::max(1.0, np_.publish_rate))
                                        : (now_t - last_pub_t).toSec();
        last_pub_t = now_t;
        dt = std::min(std::max(dt, 1e-3), 0.5);   // 防异常跳变

        // ★ 掉高保护（2026-11-14）：参考 z 比实际高出一个阈值 → 本轮不转机头。
        // 这是“安全优于航向”的兑底：飞机正在掉高时，任何额外的偏航力矩
        // 都在和爬升抢推力。高度恢复后 computeYaw 会自动继续转向。
        bool alt_sag = false;
        if (np_.yaw_alt_guard_drop > 1e-3 && robot_state_.rcv) {
            alt_sag = (p.z() - robot_state_.p.z()) > np_.yaw_alt_guard_drop;
        }
        // ── 几何切线：当前参考点 → 前方 0.5m 的方向（2026-11-25）──
        //
        // 为什么在这里算而不用 yaw_prof：rolling 每 0.1s 重建轨迹，
        // traj_t 永远停在 0，yaw_prof 的时间索引结构性失效（详见 computeYaw）。
        //
        // 前视距离取 0.5m（不是 0.5s）：与速度解耦，慢速时也是"看前方半米"，
        // 转弯前会提前转。太小（<0.2m）会被轨迹的微摆带着抖；
        // 太大（>1m）会在急弯处切角、机头超前于实际路径。
        double tang_yaw = std::numeric_limits<double>::quiet_NaN();
        {
            const double dur = snap->traj.duration();
            const double look = std::min(dur, t + 0.5);   // 近似 0.5m（v≈1m/s）
            Eigen::Vector3d pc, vc, ac, pa, va, aa;
            if (snap->traj.sample(t, pc, vc, ac) &&
                snap->traj.sample(look, pa, va, aa)) {
                const double dx = pa.x() - pc.x();
                const double dy = pa.y() - pc.y();
                if (std::hypot(dx, dy) > 0.05) {   // 方向可靠才用
                    tang_yaw = std::atan2(dy, dx);
                }
            }
        }

        cmd.yaw = computeYaw(v, dt, goal_local, &snap->yaw_prof, t, alt_sag, tang_yaw);
        cmd.yaw_dot = 0.0;

        // backup（刹车）段的诊断标记（2026-11-14，学 SUPER 的 on_backup_traj）。
        // 语义上仍是 READY（速度控制），不改 flag —— 改 flag 会让控制器切模式。
        if (snap->onBackup(t)) {
            ROS_WARN_THROTTLE(1.0,
                "[azj] executing BACKUP (controlled brake) segment: t=%.2fs / %.2fs",
                t, snap->traj.duration());
        }

        cmd.trajectory_id = snap->id;
        // ★ trajectory_flag：轨迹跑完也不能无条件发 COMPLETED（2026-11-14 修）
        //
        // 这里原本是 `if (t >= duration) COMPLETED`，**绕过了
        // `publish_completed_flag`**。后果：即使把该参数设成 false，
        // 只要轨迹时间跑完而规划器还没判“到达”（arrive_tol=0.15m + 速度门限），
        // 就会发 COMPLETED → controller api_3d.cpp:794 切到 HOLD 位置控制
        // （mask=3064），与飞行段的 3015 速度控制反复切换。
        //
        // 实测（yaw1_before_rate_fix.bag）：23s 内 mask 在 3015/3064 间切换
        // 十余次；每次 HOLD 都用 planner_cmd_.position.z 做位置环目标，
        // 而重规划刚把轨迹重置到起点（掉高后起点 z≈0.05）→ 位置环把飞机
        // 按向地面。这是“掉高后起不来”的直接机制。
        //
        // SUPER 从不发 COMPLETED（fsm_ros1.hpp:155 只发 1/2），这里对齐它：
        // 默认只发 READY；到点由 publishCompleted() 用 READY + 零速保持。
        if (np_.publish_completed_flag && t >= snap->traj.duration()) {
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        } else {
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        }

        cmd_pub_.publish(cmd);
    }

    void publishCompleted() {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = np_.frame_id.empty() ? cfg_.frame_id : np_.frame_id;
        if (cmd.header.frame_id.empty()) cmd.header.frame_id = "map";

        // goal_ 由规划线程写（goalCallback 在 spinner 上），
        // 本函数在发布线程跑 → 必须加锁快照。
        rog_map::Vec3f goal_local;
        uint32_t traj_id_local = 0;
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            goal_local = goal_;
            traj_id_local = active_ ? active_->id : 0;
        }

        cmd.position.x = goal_local.x();
        cmd.position.y = goal_local.y();
        cmd.position.z = goal_local.z();
        // 速度/加速度显式归零，避免控制器把上一帧的残留当参考
        cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0;
        cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0;

        // ★ 到点后仍然要让 computeYaw 跑（2026-11-05）。
        //
        // 旧写法直接 `cmd.yaw = last_yaw_`，机头在到点那一刻就冻住了。
        // 那样 mode=3（飞行朝速度、到点朝指定朝向）**永远转不到指定朝向** ——
        // 到点瞬间速度归零，而转向必须发生在到点之后。
        //
        // 传零速度调用：mode 1/3 在飞行段的"朝速度"分支自然不触发（spd=0 < 门限），
        // 只有 arrived_ 后的 msg 朝向分支会生效，语义正好。
        static ros::Time last_hover_t;
        const ros::Time now_h = cmd.header.stamp;
        double dt_h = last_hover_t.isZero()
                          ? (1.0 / std::max(1.0, np_.publish_rate))
                          : (now_h - last_hover_t).toSec();
        last_hover_t = now_h;
        // 同 computeYaw 里的理由：断流后 dt 累积会导致 yaw 阶跃。
        // 上限同样收紧到 1 个发布周期（见 computeYaw 里的实测说明）。
        dt_h = std::min(std::max(dt_h, 1e-3), 1.0 / std::max(1.0, np_.publish_rate));

        bool alt_sag_h = false;
        if (np_.yaw_alt_guard_drop > 1e-3 && robot_state_.rcv) {
            alt_sag_h = (goal_local.z() - robot_state_.p.z()) > np_.yaw_alt_guard_drop;
        }
        cmd.yaw = computeYaw(Eigen::Vector3d::Zero(), dt_h, goal_local, nullptr, 0.0,
                             alt_sag_h);
        cmd.yaw_dot = 0.0;
        cmd.trajectory_id = traj_id_local;

        // ★ trajectory_flag：默认**发 READY，不发 COMPLETED**（2026-11-09，对齐 SUPER）
        //
        // 这是 yaw 掉高的真正根因。控制器 api_3d.cpp:794 的分支：
        //     if (!planner_cmd_received_ || traj_timeout || traj_completed) { HOLD }
        // 收到 COMPLETED 就进 HOLD 分支，而两个分支的**控制模式完全不同**：
        //
        //   HOLD（COMPLETED）: mask=3064 → 位置控制（PX/PY/PZ 有效，V* 全忽略）
        //   主分支（READY）  : mask=3015 → 速度控制（VX/VY/VZ 有效，P* 全忽略）
        //
        // 实测 azj 有 **65%** 的帧在发 COMPLETED（htfix_yaw0.bag: READY 35%、
        // COMPLETED 65%），意味着控制器 65% 的时间在位置控制、35% 在速度控制，
        // 反复切换。位置控制模式下 PX4 用位置环算姿态+推力，此时再要求转 yaw
        // 就要和位置保持争夺电机行程 → 掉高。这解释了为什么 yaw_mode=0
        // （不转）看不出问题，一开 yaw 就掉。
        //
        // SUPER 从不发 COMPLETED（fsm_ros1.hpp:155 `on_backup_traj ? 2 : 1`，
        // 连它的 stop_cmd 也是 flag=1）→ 控制器 100% 走速度控制 → 不掉高。
        //
        // 到点后的语义与 SUPER 的 stop_cmd 完全一致：
        //   position = 目标点（给控制器做误差计算）
        //   velocity = 0（速度环保持零速）
        //   flag     = READY
        // 控制器在 rviz 模式下本来就不判到达（nav_rviz_mode_ 永不到达），
        // 所以不发 COMPLETED 不影响任务流程。
        //
        // 需要旧行为的场景（非 rviz 的点到点任务靠 COMPLETED 判到达）
        // 把 `azj/publish_completed_flag` 设 true 即可。
        cmd.trajectory_flag = np_.publish_completed_flag
                                  ? quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED
                                  : quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd_pub_.publish(cmd);
    }

    /// yaw 模式（见 np_.yaw_mode 的说明）
    ///
    /// 2026-11-25 精简为两个模式（原 0~4 五个模式已废弃）：
    ///
    ///   0 = YAW_PATH_TANGENT  跟随**路径切线**；到点后朝打点/消息给的 yaw
    ///   1 = YAW_HOLD          全程不发 yaw 变化量（保持当前机头）
    ///
    /// 关于"不发 yaw 指令"的技术现实：
    ///   `quadrotor_msgs/PositionCommand` 的 yaw 字段是必填项，
    ///   控制器（api_3d.cpp）的 type_mask 只 IGNORE_YAW_RATE、
    ///   **不 IGNORE_YAW** —— 也就是说 yaw 一定会被当作参考使用。
    ///   所以"不发"在协议层做不到。mode=1 的实际语义是：
    ///   **每帧发当前机头（last_yaw_），让控制器看不到任何 yaw 变化**
    ///   → 飞机不产生偏航力矩 → 不抢电机行程。
    ///   这与早期实测的"yaw_mode=0 全程不转不掉高"完全等价。
    enum YawMode {
        YAW_PATH_TANGENT = 0,
        YAW_HOLD         = 1
    };

    /// yaw 参考：固定值优先；否则按 yaw_mode 决定。
    ///
    /// 三个安全约束（缺一个就会出问题）：
    ///   ① 速度门限/距离门限 —— 参考方向不可靠时不动
    ///   ② 死区 —— 抑制路径微摆带来的机头抖动
    ///   ③ 速率限制 —— 绝不阶跃（历史 bug：无条件 atan2 导致 180° 急转掉机）
    /// 低速下朝速度方向会让 yaw 抖动（速度方向本身噪声大）。
    /// yaw 参考。原则：**绝不阶跃**。
    ///
    /// 历史 bug（仿真里表现为"打点后立刻旋转降落"）：
    ///   原来写 `last_yaw_ = atan2(v.y(), v.x())`，目标在身后时 yaw 瞬间跳
    ///   180°，PX4 以最大角速率急转 —— 电机输出出现 [1000,1648,1035,2000]
    ///   这种极端组合，姿态/高度失控后掉落。
    ///   SUPER 那边有 yaw_dot_max: 1.5 限速（用户 2026-08-03 自己调过），
    ///   我这里漏了。
    ///
    /// 现在的策略：
    ///   1. goal_yaw 给了固定值 → 用固定值（同样受速率限制）
    ///   2. yaw_follow_velocity=false（默认）→ 机头保持打点那一刻的朝向，不转
    ///   3. yaw_follow_velocity=true → 朝速度方向，但每帧最多转 yaw_rate_max*dt
    /// @param goal_local  调用方在 state_mtx_ 下快照的 goal（保留参数，当前模式未用）
    ///
    /// 注意：本函数运行在**发布线程**，而 goal_ 由规划线程（goalCallback）写，
    /// 所以 goal 必须由调用方快照后传入，不能在这里直接读 goal_。
    /// @param yaw_prof   预规划 yaw 曲线（可能为空 → 退回旧行为）
    /// @param traj_t     当前轨迹时刻 [s]，用于查预规划曲线
    double computeYaw(const Eigen::Vector3d& v, double dt, const rog_map::Vec3f& goal_local,
                      const YawProfile* yaw_prof = nullptr, double traj_t = 0.0,
                      bool alt_sag = false,
                      double tangent_yaw = std::numeric_limits<double>::quiet_NaN()) {
        // 首次调用用飞机当前机头初始化，避免从 0 开始猛转
        if (!yaw_init_) {
            last_yaw_ = robot_state_.rcv ? robot_state_.yaw : 0.0;
            yaw_init_ = true;
        }

        // ── 决定目标朝向 ──
        //
        // 优先级：固定 goal_yaw 最高（显式指定的就是要服从），
        // 其次才看 yaw_mode。这样老配置里写了 goal_yaw 的行为完全不变。
        double target = last_yaw_;
        bool have_target = false;

        if (!std::isnan(np_.goal_yaw)) {
            target = np_.goal_yaw;
            have_target = true;
        } else {
            // 读运行时值（可由 rosparam set 切换），而非启动时的 np_.yaw_mode
            int mode = yaw_mode_rt_.load();

            // ★ 旧模式号兼容（2026-11-25 精简为 0/1 后）
            //
            // 旧语义：0=不转 1=朝速度 2=朝目标 3=朝速度+到点朝消息 4=不转+到点朝消息
            // 新语义：0=路径切线+到点朝消息  1=不转
            //
            // 老配置里常见的 3（默认值）与新的 0 语义最接近 → 映射到 0，只提示一次。
            // 0/4（不转）→ 1。其余（1/2）没有直接对应，也按 0 处理并明确告警，
            // 避免静默改变行为。
            if (mode < 0 || mode > 4) {
                ROS_WARN_STREAM_THROTTLE(
                    5.0, "[azj] yaw_mode=" << mode << " unknown, falling back to 0 "
                             "(path tangent)");
                mode = YAW_PATH_TANGENT;
            } else if (mode != YAW_PATH_TANGENT && mode != YAW_HOLD) {
                const int mapped = (mode == 0 || mode == 4) ? YAW_HOLD : YAW_PATH_TANGENT;
                ROS_WARN_STREAM_THROTTLE(
                    5.0, "[azj] yaw_mode=" << mode
                             << " is a legacy value (modes are now 0/1 only); mapping to "
                             << mapped << ". Please set yaw_mode to 0 or 1.");
                mode = mapped;
            }

            // 老参数 yaw_follow_velocity 的兼容：打开它就等价于"飞行中要转"，
            // 即新语义的 0。仅在用户显式设成 true 时生效（默认 false）。
            if (np_.yaw_follow_velocity && mode == YAW_HOLD) {
                ROS_WARN_STREAM_THROTTLE(
                    5.0, "[azj] yaw_follow_velocity=true overrides yaw_mode=1 -> 0");
                mode = YAW_PATH_TANGENT;
            }

            switch (mode) {
                case YAW_PATH_TANGENT: {
                    // 到点后优先朝消息给的方向（投放/拍照要求机头对正）
                    if (arrived_ && msg_yaw_valid_) {
                        target = msg_yaw_;
                        have_target = true;
                        break;
                    }
                    // 飞行中：跟随**路径切线**。
                    //
                    // ★★★ 必须用 tangent_yaw（几何切线），不能用 yaw_prof[traj_t]
                    //     （2026-11-25 实测定位的结构 bug）
                    //
                    // 事故：`yaw_prof->at(traj_t)` 永远返回 0°。
                    //   日志：`yaw decision: mode=0 ... target=0.00°
                    //          prof_valid=1 traj_t=0.011`
                    //   而同一时刻 `yaw_prof: 4 waypoints, range [0°, 30°]`。
                    //
                    //   ？因为 rolling 每 0.1s 重建一次轨迹，每次 `start = now`
                    //   → `traj_t = now - start` 永远停在 0~0.1s
                    //   → 永远读到第一个 waypoint（起始机头，≈0°）
                    //   → yaw 完全不跟随路径，飞行中转不转都一样。
                    //
                    //   yaw_prof 的时间索引在"轨迹一次性跑完"的模式下才成立
                    //   （SUPER 就是这样：一条轨迹飞到底）。我们是滚动重建，
                    //   绝对时间索引结构性失效。
                    //
                    // 修法：用**几何切线** —— 取当前参考点前方 0.5m 处的方向。
                    //   与时间索引无关，rolling 重建多少次都不影响。
                    //   0.5m 前视是平滑的关键（不是逐帧速度方向），
                    //   避免柱林里速度噪声导致的机头抖动。
                    if (std::isfinite(tangent_yaw)) {
                        target = tangent_yaw;
                        have_target = true;
                    } else if (yaw_prof && yaw_prof->valid()) {
                        // 兜底：几何切线不可用时退回预规划曲线
                        target = yaw_prof->at(traj_t);
                        have_target = true;
                    } else {
                        // 兜底：曲线不可用时退回瞬时速度方向 + 速度门限
                        const double spd = std::hypot(v.x(), v.y());
                        if (spd > np_.yaw_vel_thresh) {
                            target = std::atan2(v.y(), v.x());
                            have_target = true;
                        }
                    }
                    break;
                }
                case YAW_HOLD:
                default:
                    // 不产生 yaw 变化量：have_target 保持 false →
                    // 函数末尾 `if (!have_target) return last_yaw_`
                    // → 每帧都发当前机头，控制器看不到变化。
                    break;
            }
        }

        ROS_INFO_STREAM_THROTTLE(
            2.0, "[azj] yaw decision: mode=" << yaw_mode_rt_.load()
                     << " arrived=" << (arrived_ ? 1 : 0)
                     << " msg_yaw_valid=" << (msg_yaw_valid_ ? 1 : 0)
                     << " have_target=" << (have_target ? 1 : 0)
                     << " target=" << (have_target ? target * 180.0 / M_PI : 0.0)
                     << "° last_yaw=" << last_yaw_ * 180.0 / M_PI << "°"
                     << " prof_valid=" << ((yaw_prof && yaw_prof->valid()) ? 1 : 0)
                     << " traj_t=" << traj_t);

        if (!have_target) return last_yaw_;

        // 角度差归一到 [-pi, pi]
        double diff = target - last_yaw_;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;

        // 死区：夹角很小时不动，避免"路径微摆 → 机头跟着摆"的抖动。
        // 注意这次比较用**绝对值**，不改变 last_yaw_，所以不会累积误差。
        if (std::abs(diff) <= np_.yaw_deadband) return last_yaw_;

        // 速率限制：**绝不阶跃**。这是历史 bug 的教训 ——
        // 早期无条件 atan2 且无限速，目标在身后时 yaw 瞬间跳 180°，
        // PX4 以最大角速率急转 → 姿态/高度失控 → 掉落。
        //
        // ⚠ 2026-11-05 第二次事故：dt 不能直接用，必须钳住！
        //
        // 调用方传进来的 dt 是**两次发布之间的间隔**，而规划器单线程，
        // 规划一次要 0.3~1.4s，期间根本不发布 → 恢复后 dt 累积到 0.5s
        // → max_step = 0.5(rad/s) × 0.5(s) = 0.25 rad = **14.3° 的阶跃**。
        // 实测单帧最大跳变就是 0.2500 rad，而 50Hz 下应为 0.010 rad，
        // 跳变 25 倍。PX4 会用最大角加速度追这个阶跃，推力被挤占 → 掉高。
        //
        // 正确做法：dt 上限取**一个**正常发布周期，而不是 3 个。
        // 断流期间飞机本来也没收到指令，恢复后只该走一小步，剩下的
        // 靠后续帧慢慢追 —— 这才是“绝不阶跃”的真正含义。
        //
        // ★ 2026-11-14 从 3 个周期收紧到 1 个周期：
        //   bag 实测（/tmp/azj_flight_20261114_090419）分档生效的段
        //   （|vxy|=1.0、rate_cap=0.5）逐帧精确 0.50 rad/s；但 15~20s
        //   出现 max **6.25 rad/s** 的单帧尖尖 —— 就是 dt_ctrl 允许
        //   rate_cap×0.06 的结果，随后一帧只隔 5ms → 表观 6 rad/s。
        //   上限收成 1 个周期后，max_step = rate_cap×0.02，
        //   无论实际间隔多小，表观速率都不超 rate_cap。
        const double dt_nom = 1.0 / std::max(1.0, np_.publish_rate);
        const double dt_ctrl = std::min(std::max(dt, 1e-3), dt_nom);

        // ★ 飞行中 / 到点后分档限速（2026-11-14）
        //
        // 判据用的是**是否已到点**（arrived_），不是水平速度。
        //
        // 为什么不用速度：旧写法 `|v_xy| > yaw_vel_thresh(0.15)` 在悬停、
        // 慢速修正、糊在地面上时都判 false → 退回 yaw_rate_max=1.0。
        // bag 实测（/tmp/azj_flight_20261114_090419）：
        //   0~5s   in_flight 96% → 速率精确 0.50 ✓
        //   15~20s in_flight 15% → 速率 1.00~1.33，max 6.25 ✗
        // 而飞机在悬停时一样需要推力维持高度，转 yaw 一样抢电机行程，
        // 所以“速度小就可以快转”是错的。正确语义是：
        //   还在跟踪轨迹（会机动、要保高）→ 用慢档 yaw_rate_flight
        //   已到点悬停（无位置需求）    → 才允许用 yaw_rate_max 快速对准
        const bool in_flight = !arrived_;
        double rate_cap = np_.yaw_rate_max;
        if (in_flight && np_.yaw_rate_flight > 1e-6) {
            rate_cap = std::min(rate_cap, np_.yaw_rate_flight);
        }

        // ★★ 倾角自适应降速（2026-11-14 第二次实测后新增）
        //
        // 实测（/tmp/azj_flight_20261114_123622/flight.bag，yaw_mode=3）：
        //   第一点（路径近乎直线，yaw 速率 0.00~0.19）→ z 稳在 0.85~0.89，不掉
        //   到点悬停（vxy=0，yaw 速率 0.99）        → Δz=-0.003，不掉
        //   第二点（路径转弯，yaw 速率顶在 0.50）    → z 从 0.891 掉到 0.013
        //
        // 掉高瞬间 tilt 只有 3~7°（cos≈0.996），**不是倾角导致的垂直推力损失**。
        // 是偏航差速在"悬停油门 0.871 只剩 12.9% 行程"的空间里撞顶：
        // 对角电机一组加速一组减速，任一侧到 1.0 就限幅，净推力随之下降。
        //
        // 所以：倾角越大 → 垂直分量越小 + 水平控制也在吃行程 → 偏航必须更慢。
        // 线性衰减，30° 时降到 30%：
        //     tilt  0° → 1.00 × yaw_rate_flight
        //     tilt 10° → 0.77 ×
        //     tilt 20° → 0.53 ×
        //     tilt 30° → 0.30 ×
        if (in_flight && robot_state_.rcv) {
            const auto& q = robot_state_.q;
            // 机体 z 轴在世界系的 z 分量 = R33 = 1 - 2(qx²+qy²) = cos(tilt)
            double cos_tilt = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
            cos_tilt = std::min(1.0, std::max(-1.0, cos_tilt));
            const double tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;
            const double scale = std::max(0.30, 1.0 - (tilt_deg / 30.0) * 0.70);
            rate_cap *= scale;
        }
        // ★ 掉高保护（安全优先）—— 实际高度明显低于参考时，
        // 本轮直接不转：先把高度保住，恢复后再继续对准。
        if (alt_sag) {
            return last_yaw_;
        }
        const double max_step = std::max(1e-6, rate_cap) * dt_ctrl;
        const double step = std::min(std::abs(diff), max_step);
        last_yaw_ += (diff > 0.0 ? step : -step);

        // 再次归一到 (-pi, pi]，避免长期运行后数值无限增长
        while (last_yaw_ > M_PI) last_yaw_ -= 2.0 * M_PI;
        while (last_yaw_ < -M_PI) last_yaw_ += 2.0 * M_PI;
        return last_yaw_;
    }

    /// 目标点可视化：rviz 打点后能看到目标落在哪（不加这个，打点后界面上没反馈，
    /// 容易误以为点没发出去）。
    void publishGoalMarker() {
        visualization_msgs::MarkerArray arr;
        const std_msgs::Header header = makeHeader();

        visualization_msgs::Marker m;
        m.header = header;
        m.ns = "azj_goal";
        m.id = 0;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.pose.position.x = goal_.x();
        m.pose.position.y = goal_.y();
        m.pose.position.z = goal_.z();
        m.scale.x = m.scale.y = m.scale.z = 0.30;
        m.color.r = 1.0f;
        m.color.g = 0.6f;
        m.color.b = 0.0f;
        m.color.a = 0.85f;
        arr.markers.push_back(m);

        visualization_msgs::Marker t;
        t.header = header;
        t.ns = "azj_goal_text";
        t.id = 1;
        t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::Marker::ADD;
        t.pose.orientation.w = 1.0;
        t.pose.position.x = goal_.x();
        t.pose.position.y = goal_.y();
        t.pose.position.z = goal_.z() + 0.45;
        t.scale.z = 0.30;
        t.color.r = t.color.g = t.color.b = 1.0f;
        t.color.a = 0.9f;
        t.text = "AZJ GOAL";
        arr.markers.push_back(t);

        goal_marker_pub_.publish(arr);
    }

    void publishPathViz() {
        if (last_path_.empty()) return;
        const std_msgs::Header header = makeHeader();

        nav_msgs::Path msg;
        msg.header = header;
        msg.poses.reserve(last_path_.size());
        for (const auto& p : last_path_) {
            geometry_msgs::PoseStamped ps;
            ps.header = header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = p.z();
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_.publish(msg);

        visualization_msgs::MarkerArray arr;
        visualization_msgs::Marker line;
        line.header = header;
        line.ns = "azj_path";
        line.id = 0;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.r = 0.1f;
        line.color.g = 1.0f;
        line.color.b = 0.4f;
        line.color.a = 0.9f;
        line.points.reserve(last_path_.size());
        for (const auto& p : last_path_) {
            geometry_msgs::Point gp;
            gp.x = p.x();
            gp.y = p.y();
            gp.z = p.z();
            line.points.push_back(gp);
        }
        arr.markers.push_back(line);
        marker_pub_.publish(arr);
    }

    std_msgs::Header makeHeader() const {
        std_msgs::Header h;
        h.frame_id = np_.frame_id.empty() ? cfg_.frame_id : np_.frame_id;
        if (h.frame_id.empty()) h.frame_id = "map";
        h.stamp = ros::Time::now();
        return h;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber goal_sub_;
    ros::Subscriber vel_sub_;
    ros::Subscriber odom_vel_sub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher goal_marker_pub_;
    ros::Publisher cmd_pub_;
    ros::Timer replan_timer_;
    /// 独立发布线程。
    ///
    /// 为什么必须是线程而不是 timer：ROS 的 timer 由 spinner 调度，
    /// 而规划（replanCallback）也在 spinner 上同步执行 0.3~1.4s。
    /// 单线程 spinner 被规划占住时，timer 根本轮不到 → pos_cmd 断流
    /// → 控制器 0.5s 超时进 HOLD、恢复后 dt 累积导致 yaw 阶跃掉机。
    ///
    /// 这个线程只做三件事：拿快照、采样、发布。全部无锁（快照是不可变的），
    /// 所以规划再慢也不影响它。
    void startPublishThread() {
        pub_run_ = true;
        last_pub_wall_ = ros::WallTime::now();
        pub_thread_ = std::thread([this]() {
            const double dt = 1.0 / std::max(1.0, np_.publish_rate);
            ros::WallRate rate(1.0 / dt);
            while (ros::ok() && pub_run_) {
                publishTick();
                rate.sleep();
            }
        });
        ROS_INFO("[azj] publish thread started (%.0f Hz, decoupled from planner)",
                 std::max(1.0, np_.publish_rate));
    }

    void stopPublishThread() {
        pub_run_ = false;
        if (pub_thread_.joinable()) pub_thread_.join();
    }

public:
    ~AzjPlannerNode() override { stopPublishThread(); }

private:

    ClearanceAStar astar_;
    CurveOptimizer curve_opt_;
    double last_max_curvature_{0.0};
    SearchParams sp_;
    CurveOptParams cp_;
    TrajParams tp_;
    NodeParams np_;

    // ── 滚动窗口的全局路径缓存（2026-11-17）──
    //
    // 为什么缓存：15Hz 下每秒重跑 15 次全量 A* 是纯粹浪费 ——
    // 目标没变、地图没断的时候，上一次的全局路径仍然是最优的。
    // 只有 goal 变了 / 过期 / 剩余段被挡时才重建。
    std::vector<Eigen::Vector3d> global_path_;
    ros::WallTime global_path_wall_;
    Eigen::Vector3d global_path_goal_{Eigen::Vector3d::Zero()};
    bool global_path_valid_{false};
    int minco_reject_streak_{0};
    int seg_stale_streak_{0};   // rolling seg 连续 STALE 次数（防活动锁）   // MINCO 连续被拒次数（超过阈值就放弃 keep_old）
    ros::WallTime last_rolling_wall_;   // 上次滚动重规划时刻（rolling_period 用）

    bool has_goal_{false};
    bool plan_2d_active_{false};      // 当前生效的模式（含运行时切换）
    double plan_2d_z_active_{0.0};    // 2D 模式当前锁定的平面高度
    double z_plane_{0.0};             // 当前规划平面（makeField 用）
    ros::WallTime last_replan_wall_;  // 上次实际进入规划的时刻（重规划退避用）
    /// 上次走 BLOCKED 快通道（出折线）的时刻。
    ///
    /// 为什么要记：快通道的日志承诺 "next replan will smooth it"，
    /// 但那个"下一次"原来只能靠 TRAJ_END（轨迹跑完）或 DEVIATED（跟丢）偶然触发。
    /// 实测（raicom 3D）折线从 t=37.4 一直走到 t=41.6 才被替换 —— 4.2 秒，
    /// 而且那次替换的曲线又被 kmax 拒掉，等于从未平滑过。
    /// 现在用这个时间戳驱动一次**主动**平滑重规划（见 replanCallback）。
    ros::WallTime last_fast_path_wall_;
    /// BLOCKED 滞回锁存（2026-11-14）。见 needReplan 里的说明。
    bool blocked_latched_{false};
    /// 本次规划的起始时刻与状态（用于补偿"单线程 spinner 期间状态冻结"）。
    /// 见 replanCallback 里的 plan_start_wall_ 赋值处注释。2026-11-14
    ros::WallTime plan_start_wall_;
    /// 单次规划耗时的滑动平均 [s]（2026-11-14）。
    /// 用作新轨迹起点的"前视时长" —— 等价 SUPER 的 replan_forward_dt，
    /// 但按实际耗时自适应（我们规划 0.5~1.0s，SUPER 只要 0.3ms）。
    double plan_duration_ema_{0.0};
    /// 上一帧发布的参考速度 + 速率限制状态（2026-11-16）
    Eigen::Vector3d last_pub_vel_{Eigen::Vector3d::Zero()};
    bool last_pub_vel_valid_{false};

    /// 本次规划的起点状态（(p,v) 整组取自旧轨迹未来时刻，学 SUPER）。
    /// 由 replanCallback 算**一次**，A* 与 buildTrajectory 共用 —— 保证 gap=0。
    Eigen::Vector3d plan_anchor_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d plan_anchor_v_{Eigen::Vector3d::Zero()};
    bool plan_anchor_valid_{false};
    Eigen::Vector3d plan_start_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d plan_start_vel_{Eigen::Vector3d::Zero()};
    /// 锁存时的 clearance（用于判断"是否仍在恶化"）
    double blocked_latch_clr_{0.0};
    /// 当前目标是否"刚打的新点"（此时禁止热启动复用旧轨迹）。2026-11-14
    bool goal_is_new_{false};
    bool arrived_{false};
    rog_map::Vec3f goal_{rog_map::Vec3f::Zero()};

    Eigen::Vector3d current_vel_{Eigen::Vector3d::Zero()};
    // 打点消息里带的朝向（yaw_mode=FROM_MSG 用）。
    // rviz "2D Nav Goal" 按住拖方向就会带 orientation；不拖则 w=1,theta=0。
    double msg_yaw_{0.0};
    bool msg_yaw_valid_{false};
    bool vel_received_{false};

    // ────────────────────────────────────────────────────────────────
    // 轨迹快照 + 跨线程共享状态（2026-11-05 并发改造）
    //
    // 为什么不用 AsyncSpinner：
    //   rog_map 的 `evaluateEDT` **不加锁**读 `distance_buffer`，
    //   而 `updateESDF3D` 会全量重写它。单线程 spin 时二者天然串行，
    //   一旦开多线程 spinner，规划线程可能读到"半更新"的 ESDF ——
    //   那会算错 clearance，直接导致路径穿墙。用户明确不改 rog_map 源码，
    //   所以**必须保持 ros::spin() 单线程**。
    //
    // 那 pos_cmd 断流怎么办（规划 0.3~1.4s 期间控制器 0.5s 就超时）：
    //   解耦"规划"与"发布"。规划仍在 spinner 线程里同步跑，
    //   发布挪到**独立线程**，它只读不可变的轨迹快照。
    //   于是规划再慢，pos_cmd 也照常 50Hz 发出，控制器不超时、
    //   dt 不累积、yaw 不阶跃。
    //
    // 线程安全设计：
    //   · 轨迹用 shared_ptr<const> 快照，发布线程拿到后**不再持锁**，
    //     旧轨迹在最后一个引用释放时自动销毁 → 无锁竞争、无需拷贝。
    //   · replanCallback 先在局部变量里 build（不持锁），
    //     再把新快照原子换上 → build 期间发布线程继续用旧轨迹，
    //     **发布永不阻塞**。
    //   · 其它跨线程小状态（arrived_/has_goal_/goal_/traj_v0_/last_ref_pos_）
    //     由 state_mtx_ 保护，持锁时间都是几行赋值。
    // ────────────────────────────────────────────────────────────────
    std::shared_ptr<const TrajSnapshot> active_;   // nullptr = 无可用轨迹
    mutable std::mutex state_mtx_;

    std::thread pub_thread_;
    std::atomic<bool> pub_run_{false};
    /// 运行时 yaw_mode（可由 `rosparam set /azj_planner/yaw_mode N` 切换）。
    /// 用 atomic 因为发布线程读、规划线程写。
    std::atomic<int> yaw_mode_rt_{1};
    std::mutex pub_start_mtx_;      // 保护 traj_start 记录的"首次发布时刻"
    ros::WallTime last_pub_wall_;   // 发布线程用的墙上时间基准

    bool has_trajectory_{false};
    double last_replan_gap_{0.0};                        // 重规划时路径起点与当前位置的偏差
    uint32_t trajectory_id_{0};

    std::vector<Eigen::Vector3d> last_path_;
    double last_min_clearance_{0.0};
    double last_path_length_{0.0};

    Eigen::Vector3d last_ref_pos_{Eigen::Vector3d::Zero()};

    double last_yaw_{0.0};
    bool yaw_init_{false};
};

}  // namespace azj_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "azj_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cfg_path;
    pnh.param<std::string>("config_file", cfg_path, "");
    if (cfg_path.empty()) {
        cfg_path = ros::package::getPath("azj_planner") + "/config/azj_planner.yaml";
    }

    try {
        azj_planner::AzjPlannerNode node(nh, pnh, cfg_path);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("[azj] init failed: %s", e.what());
        return 1;
    }
    return 0;
}
