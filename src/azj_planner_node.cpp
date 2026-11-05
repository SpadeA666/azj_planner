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

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <rog_map_ros/rog_map_ros1.hpp>
#include <yaml-cpp/yaml.h>

#include "azj_planner/curve_optimizer.h"
#include "azj_planner/pillar_clearance_field.h"
#include "azj_planner/path_search.h"
#include "azj_planner/trajectory.h"

namespace azj_planner {

class AzjPlannerNode : public rog_map::ROGMapROS {
public:
    AzjPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                   const std::string& cfg_path)
        : rog_map::ROGMapROS(nh, cfg_path), nh_(nh), pnh_(pnh) {
        if (!cfg_.esdf_en || !esdf_map_) {
            throw std::runtime_error(
                "rog_map ESDF disabled: set rog_map/esdf/enable true in the yaml");
        }
        loadParams(cfg_path);

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
        publish_timer_ = nh_.createTimer(ros::Duration(1.0 / np_.publish_rate),
                                         &AzjPlannerNode::publishCallback, this);

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
        ROS_INFO("[azj] yaw: follow_velocity=%d rate_max=%.2f rad/s (0->no turn)",
                 static_cast<int>(np_.yaw_follow_velocity), np_.yaw_rate_max);
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
        double goal_yaw{NAN};          // 固定目标 yaw；NaN = 按 yaw_follow_velocity 决定
        // 是否让机头朝速度方向。**默认 false**：打点只要求"飞过去"，不该顺带转向。
        // 早期版本无条件朝速度方向且无速率限制，打点瞬间 yaw 阶跃 180°，
        // PX4 以最大角速率急转 → 姿态/高度失控 → 掉落。
        bool yaw_follow_velocity{false};
        // yaw 参考的最大变化率 [rad/s]。对齐 SUPER 的 yaw_dot_max: 1.5。
        // 即使开启跟随，也只允许缓慢转，不会阶跃。
        double yaw_rate_max{1.5};
        // 目标高度覆盖。NaN = 用消息里的 z（真机地面站会带正确高度）。
        // 仿真 rviz 的 2D Nav Goal 在地面平面上点击，z≈0，直接用会掉到地面，
        // 所以仿真里必须设成 fly_height。
        double goal_z_override{NAN};
        double arrive_tol{0.25};       // 判定到达的距离
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
        // 重规划最小间隔 [s]。机制性的防活锁兑底：
        // 实测一次完整规划 1.5~1.7s，所以这个值不会拖慢正常重规划；
        // 它只防止“规划→被拒→重规划”循环把主线程跑满。
        double min_replan_period{0.30};
    };

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
                bln("yaw_follow_velocity", np_.yaw_follow_velocity);
                dbl("yaw_rate_max", np_.yaw_rate_max);
                dbl("goal_z_override", np_.goal_z_override);
                dbl("arrive_tol", np_.arrive_tol);
                dbl("replan_lead", np_.replan_lead);
                dbl("replan_deviation", np_.replan_deviation);
                dbl("safety_lookahead", np_.safety_lookahead);
                dbl("min_replan_period", np_.min_replan_period);

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
        pnh_.param("azj/yaw_follow_velocity", np_.yaw_follow_velocity,
                   np_.yaw_follow_velocity);
        pnh_.param("azj/yaw_rate_max", np_.yaw_rate_max, np_.yaw_rate_max);
        pnh_.param("azj/goal_z_override", np_.goal_z_override, np_.goal_z_override);
        pnh_.param("azj/arrive_tol", np_.arrive_tol, np_.arrive_tol);
        pnh_.param("azj/replan_lead", np_.replan_lead, np_.replan_lead);
        pnh_.param("azj/replan_deviation", np_.replan_deviation, np_.replan_deviation);
        pnh_.param("azj/safety_lookahead", np_.safety_lookahead, np_.safety_lookahead);
        pnh_.param("azj/min_replan_period", np_.min_replan_period,
                   np_.min_replan_period);

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
        ros::param::set("/azj_planner/curve_w_mid", cp_.w_mid);
        ros::param::set("/azj_planner/curve_w_obs", cp_.w_obs);
        ros::param::set("/azj_planner/curve_w_anchor", cp_.w_anchor);
        ros::param::set("/azj_planner/robot_radius", sp_.robot_radius);
        ros::param::set("/azj_planner/clearance_weight", sp_.clearance_weight);
        ros::param::set("/azj_planner/goal_z_override", np_.goal_z_override);
        ros::param::set("/azj_planner/yaw_follow_velocity", np_.yaw_follow_velocity);
        ros::param::set("/azj_planner/yaw_rate_max", np_.yaw_rate_max);
        ros::param::set("/azj_planner/curve_iterations", cp_.iterations);
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
    void applyPolylineFallback(const SearchResult& res, const ClearanceField& field) {
        // 折线兜底也要"能看"：直接用 A* 的原始栅格路径（每 0.1m 一点）
        // 是锯齿状折线，视觉上是乱的。
        // 这里先 simplify 删共线点，再做**保形**轻平滑（平滑后必须复核 clearance
        // 不下降，否则退回未平滑版本）。
        std::vector<Eigen::Vector3d> path = simplifyPath(res.path);
        const double mc_before = minClearanceAlongPath(path, field, 0.02);
        {
            const auto sm = smoothPath(path, field, sp_.robot_radius * 0.9, 2, 0.25);
            if (sm.size() >= 2) {
                const double mc_after = minClearanceAlongPath(sm, field, 0.02);
                if (mc_after >= mc_before - 1e-6) path = sm;
            }
        }
        if (np_.enforce_height_limits) {
            for (auto& q : path) {
                q.z() = std::min(std::max(q.z(), np_.floor_z), np_.ceil_z);
            }
        }
        if (path.size() < 2) {
            has_trajectory_ = false;
            ROS_WARN_THROTTLE(1.0, "[azj] fallback: A* path too short (%zu)", path.size());
            return;
        }

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
        goal_ = rog_map::Vec3f(msg->pose.position.x, msg->pose.position.y, gz);
        has_goal_ = true;
        arrived_ = false;
        has_trajectory_ = false;
        publishGoalMarker();
        ROS_INFO("[azj] goal received (%.2f, %.2f, %.2f)", goal_.x(), goal_.y(), goal_.z());
    }

    /// 把路径转成时间参数化轨迹。
    bool buildTrajectory(const std::vector<Eigen::Vector3d>& path) {
        // 重规划连续性检查：新轨迹起点应当接近飞机当前位置。
        // 若差太多（比如 A* 路径从别处开始），t=0 的参考点会让飞机
        // 收到一个突兀的跳变目标 —— 表现为重规划瞬间"被拽一下"。
        if (vel_received_ || robot_state_.rcv) {
            const Eigen::Vector3d cur = robot_state_.p;
            const double gap = (path.front() - cur).norm();
            if (gap > 0.5) {
                ROS_WARN_STREAM_THROTTLE(
                    1.0, "[azj] new trajectory starts " << gap
                                                        << "m away from current position ("
                                                        << path.front().transpose()
                                                        << " vs " << cur.transpose() << ")");
            }
            // 起点强制对齐到当前位置，保证 t=0 的参考 = 飞机现在的位置。
            // 用 z 也一起对齐：高度由 z_band 约束管，不靠这里。
            // 注意：只在偏差较小时强制，避免把路径整体拉变形。
            if (gap > 1e-6 && gap <= 0.5) {
                // 只改起点，后续点不动（B 样条已生成，这里改的是采样折线）
                // 用一个短过渡段，避免 0.3m 的突进
                std::vector<Eigen::Vector3d> fixed;
                fixed.reserve(path.size() + 2);
                fixed.push_back(cur);
                const int N = std::max(2, static_cast<int>(std::ceil(gap / 0.10)));
                for (int k = 1; k <= N; ++k) {
                    const double t = static_cast<double>(k) / N;
                    fixed.push_back(cur + t * (path.front() - cur));
                }
                for (size_t i = 1; i < path.size(); ++i) fixed.push_back(path[i]);
                last_replan_gap_ = gap;
                TrajParams tp = tp_;
                return buildTrajectoryImpl(fixed, tp);
            }
        }
        TrajParams tp = tp_;
        return buildTrajectoryImpl(path, tp);
    }

    bool buildTrajectoryImpl(const std::vector<Eigen::Vector3d>& path, TrajParams& tp) {
        // 起点速度：传**矢量**而不是标量。
        //
        // 标量的做法是 |v_current| × 新路径切向，会在重规划瞬间凭空生成
        // 竖直速度：飞机水平飞 0.8m/s、新路径起点切线朝下 45° 时，
        // 会给出 vz=-0.57 m/s → 控制器照做 → **瞬间掉高**（用户报的现象）。
        //
        // 传矢量后，trajectory 会把真实速度投影到新路径切向，
        // 只保留沿路径的推进分量，不製造与路径不符的竖直/横向速度。
        if (vel_received_) {
            tp.v_start_vec = current_vel_;
            tp.has_v_start_vec = true;
            tp.v_start = current_vel_.norm();   // 兜底
        } else {
            tp.v_start = 0.0;
        }
        tp.v_start = std::min(tp.v_start, tp.max_vel);
        tp.stop_at_end = true;

        if (!traj_.build(path, tp)) return false;
        // 记下生成轨迹那一刻的实际速度，供输出淡入用
        traj_v0_ = vel_received_ ? current_vel_ : Eigen::Vector3d::Zero();
        traj_start_ = ros::Time::now();
        trajectory_id_++;
        return true;
    }

    /// 是否需要重规划。
    ///
    /// 为什么不能每帧无条件重规划：重规划会把轨迹时间基准重置为 0，
    /// 于是参考点永远停在"新轨迹的前 0.2s"，而轨迹前段速度又受 v_start 限制，
    /// 一旦速度反馈滞后，参考速度就永远上不去（实测 max|v| 卡在 0.03 m/s）。
    /// 正确做法是让每条轨迹**完整执行**，只在必要时重建。
    bool needReplan(const Eigen::Vector3d& start) {
        if (!has_trajectory_ || !traj_.valid()) return true;

        const double t = (ros::Time::now() - traj_start_).toSec();
        // (a) 轨迹快走完
        if (t >= traj_.duration() - np_.replan_lead) return true;

        // (b) 跟丢了：实际位置离参考点太远
        if ((start - last_ref_pos_).norm() > np_.replan_deviation) {
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] deviated from reference by "
                                              << (start - last_ref_pos_).norm() << "m, replanning");
            return true;
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
        const double lookahead = std::min(t + np_.safety_lookahead, traj_.duration());
        for (double ts = t; ts <= lookahead; ts += 0.1) {
            Eigen::Vector3d p, v, a;
            if (!traj_.sample(ts, p, v, a)) continue;
            const double c = field.clearance(p);
            if (c < sp_.robot_radius) {
                if (start_unsafe && c >= start_clr_now - 1e-6) continue;   // 不比起点更槽
                ROS_WARN_STREAM_THROTTLE(1.0, "[azj] reference blocked ahead (clearance="
                                                  << c << "m < robot_radius="
                                                  << sp_.robot_radius
                                                  << "m), replanning");
                return true;
            }
        }
        return false;
    }

    void replanCallback(const ros::TimerEvent&) {
        if (!has_goal_) return;
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

            // 起点离地面/天花板太近时抬一点。
            // 为什么：飞机停在地面（z=0.03）时，地面点云就在附近，
            // ESDF clearance ≈ 0 → 起点被判"在障碍里" → 永远 NO_PATH。
            // 这是物理事实（贴地），不是规划器缺陷，但规划必须能开始。
            // 抬起量与 robot_radius 无关，只要求 > 0 即可（escape 逻辑会接管）。
            const double kClear = 0.05;
            if (start.z() - zlo < kClear) {
                ROS_WARN_STREAM_THROTTLE(
                    2.0, "[azj] start z=" << start.z() << " too close to floor_z=" << zlo
                                          << "; lifted to " << (zlo + kClear));
                start.z() = zlo + kClear;
            }
            if (zhi - start.z() < kClear) {
                start.z() = zhi - kClear;
            }
        }

        if ((start - goal).norm() <= np_.arrive_tol) {
            if (!arrived_) {
                arrived_ = true;
                has_trajectory_ = false;
                ROS_INFO("[azj] goal reached (dist=%.3f)", (start - goal).norm());
            }
            return;
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

        if (!needReplan(start)) return;

        // ★ 重规划频率硬上限（2026-11-05）。
        //
        // 机制保证而非依赖上面的逻辑正确：即使将来又出现“规划→被拒→重规划”
        // 这类活锁，也绝不会把主线程跑成 100% CPU、导致 /planning/pos_cmd 断流。
        // 实测一次完整规划（A* + 曲线，含安全重试）约 1.5~1.7s，
        // 所以 0.30s 的下限不会拖慢正常重规划，只在异常时兑底。
        {
            const ros::WallTime now = ros::WallTime::now();
            if (!last_replan_wall_.isZero()) {
                const double dt = (now - last_replan_wall_).toSec();
                if (dt < np_.min_replan_period) {
                    return;   // 退避：上一条轨迹还在，控制器自己接着飞
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

        const ros::WallTime t_astar0 = ros::WallTime::now();
        const SearchResult res = astar_.search(field, start, goal, sp_);
        const double ms_astar = (ros::WallTime::now() - t_astar0).toSec() * 1e3;
        if (!res.success) {
            // 规划失败：不发新参考，让控制器按 traj_timeout 自行刹车。
            // 但把已有轨迹标记失效，避免继续沿旧轨迹飞向未知区域。
            has_trajectory_ = false;
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] plan failed: " << res.msg);
            return;
        }

        // ---------- 折线 → 曲线 ----------
        //
        // 之前只有 simplify + smoothPath（顶点移动平均），本质上还是折线：
        // 实测转角中位 45°、最大 92°，且加大平滑迭代无效（因为顶点间仍是直线）。
        // 现在改用 ESDF 梯度驱动的三次 B 样条优化，得到 C2 连续曲线。
        std::vector<Eigen::Vector3d> path = simplifyPath(res.path);
        if (np_.curve_enable) {
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

            // ---- 优化 ----
            const ros::WallTime t_cv0 = ros::WallTime::now();
            CurveOptResult cr = curve_opt_.optimize(path, field, cp_);
            const double ms_cv = (ros::WallTime::now() - t_cv0).toSec() * 1e3;
            ROS_INFO_STREAM_THROTTLE(5.0, "[azj] T {astar=" << ms_astar
                                    << "ms curve=" << ms_cv << "ms ncp="
                                    << cr.iterations << "}");

            // kmax 超限时先"加重平滑"重试一次，而不是直接退回折线。
            // 很多情况下加大 w_smooth 就能把曲率压进限制；直接 fallback
            // 会让用户看到难看的锯齿折线（实测 raicom 紧缝路径被拒成 8 点折线）。
            if (cr.success && np_.max_curvature > 1e-6 &&
                cr.max_curvature > np_.max_curvature) {
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
                    applyPolylineFallback(res, field);
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
                    applyPolylineFallback(res, field);
                    return;
                }

                // ---------- ② 曲率校验 ----------
                if (np_.max_curvature > 1e-6 && cr.max_curvature > np_.max_curvature) {
                    ROS_WARN_STREAM_THROTTLE(
                        1.0, "[azj] curve rejected: kmax="
                                 << cr.max_curvature << " > limit " << np_.max_curvature
                                 << " (R=" << 1.0 / cr.max_curvature
                                 << "m) - falling back to polyline");
                    applyPolylineFallback(res, field);
                    return;
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
                applyPolylineFallback(res, field);
                return;
            }
        } else {
            path = smoothPath(path, field, sp_.robot_radius, 3, 0.35);
        }

        const double min_c = minClearanceAlongPath(path, field, 0.02);
        last_min_clearance_ = min_c;
        last_path_length_ = pathLength(path);
        last_path_ = path;

        if (!buildTrajectory(path)) {
            has_trajectory_ = false;
            ROS_WARN_STREAM_THROTTLE(1.0, "[azj] trajectory build failed");
            return;
        }
        has_trajectory_ = true;
        publishPathViz();

        ROS_INFO_STREAM_THROTTLE(
            1.0, "[azj] len=" << last_path_length_ << "m  min_clearance=" << min_c
                              << "m (safe " << sp_.robot_radius << "m)  traj="
                              << traj_.duration() << "s  vpk=" << traj_.maxVelocity()
                              << "m/s");
    }

    void publishCallback(const ros::TimerEvent&) {
        // 到达后继续发 COMPLETED，而不是静默停止。
        // 原因：api_3d.cpp 收到 COMPLETED 才会走 nav_hold_active_ 分支
        // （锁存当前位置悬停、速度归零）。静默不发则控制器要等 0.5s
        // traj_timeout 才刹车，这期间会按最后的参考速度继续飞 —— 实测过冲 0.176m。
        if (arrived_) {
            publishCompleted();
            return;
        }
        if (!has_goal_ || !has_trajectory_ || !traj_.valid()) return;

        const double t = (ros::Time::now() - traj_start_).toSec();
        Eigen::Vector3d p, v, a;
        if (!traj_.sample(t, p, v, a)) return;

        last_ref_pos_ = p;

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

        // 淡入：t 从 0 到 vel_blend_time
        double blend = 1.0;
        if (np_.vel_blend_time > 1e-6 && has_trajectory_) {
            const double tnow = (ros::Time::now() - traj_start_).toSec();
            blend = std::min(1.0, std::max(0.0, tnow / np_.vel_blend_time));
        }
        if (blend < 1.0 && vel_received_) {
            // 生成轨迹时记下的起点速度（traj_v0_），向轨迹速度插值
            v_out = blend * v_out + (1.0 - blend) * traj_v0_;
            if (np_.max_vel_z_out > 1e-6) {
                v_out.z() = std::min(std::max(v_out.z(), -np_.max_vel_z_out),
                                     np_.max_vel_z_out);
            }
        }

        cmd.velocity.x = v_out.x();
        cmd.velocity.y = v_out.y();
        cmd.velocity.z = v_out.z();
        cmd.acceleration.x = a.x();
        cmd.acceleration.y = a.y();
        cmd.acceleration.z = a.z();

        // dt 用实际发布间隔
        static ros::Time last_pub_t;
        const ros::Time now_t = cmd.header.stamp;
        double dt = last_pub_t.isZero() ? (1.0 / std::max(1.0, np_.publish_rate))
                                        : (now_t - last_pub_t).toSec();
        last_pub_t = now_t;
        dt = std::min(std::max(dt, 1e-3), 0.5);   // 防异常跳变

        cmd.yaw = computeYaw(v, dt);
        cmd.yaw_dot = 0.0;

        cmd.trajectory_id = trajectory_id_;
        // 轨迹跑完 → COMPLETED，控制器据此判定到点（api_3d.cpp:790）
        if (t >= traj_.duration()) {
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
            // 到点后保持发 COMPLETED 一小段，避免控制器刚收到就超时
            // （实际悬停由控制器自己的到达判定接管）
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

        cmd.position.x = goal_.x();
        cmd.position.y = goal_.y();
        cmd.position.z = goal_.z();
        // 速度/加速度显式归零，避免控制器把上一帧的残留当参考
        cmd.velocity.x = cmd.velocity.y = cmd.velocity.z = 0.0;
        cmd.acceleration.x = cmd.acceleration.y = cmd.acceleration.z = 0.0;
        cmd.yaw = std::isnan(np_.goal_yaw) ? last_yaw_ : np_.goal_yaw;
        cmd.yaw_dot = 0.0;
        cmd.trajectory_id = trajectory_id_;
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        cmd_pub_.publish(cmd);
    }

    /// yaw 参考：固定值优先；否则速度足够大时朝速度方向，低速时保持上次值。
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
    double computeYaw(const Eigen::Vector3d& v, double dt) {
        // 首次调用用飞机当前机头初始化，避免从 0 开始猛转
        if (!yaw_init_) {
            last_yaw_ = robot_state_.rcv ? robot_state_.yaw : 0.0;
            yaw_init_ = true;
        }

        double target = last_yaw_;
        if (!std::isnan(np_.goal_yaw)) {
            target = np_.goal_yaw;
        } else if (np_.yaw_follow_velocity && std::hypot(v.x(), v.y()) > 0.15) {
            target = std::atan2(v.y(), v.x());
        }

        // 角度差归一到 [-pi, pi]，然后按最大变化率逼近
        double diff = target - last_yaw_;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;

        const double max_step = std::max(1e-6, np_.yaw_rate_max) * std::max(dt, 1e-3);
        if (std::abs(diff) <= max_step) {
            last_yaw_ += diff;
        } else {
            last_yaw_ += (diff > 0.0 ? max_step : -max_step);
        }
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
    ros::Timer publish_timer_;

    ClearanceAStar astar_;
    CurveOptimizer curve_opt_;
    double last_max_curvature_{0.0};
    SearchParams sp_;
    CurveOptParams cp_;
    TrajParams tp_;
    NodeParams np_;

    bool has_goal_{false};
    bool plan_2d_active_{false};      // 当前生效的模式（含运行时切换）
    double plan_2d_z_active_{0.0};    // 2D 模式当前锁定的平面高度
    double z_plane_{0.0};             // 当前规划平面（makeField 用）
    ros::WallTime last_replan_wall_;  // 上次实际进入规划的时刻（重规划退避用）
    bool arrived_{false};
    rog_map::Vec3f goal_{rog_map::Vec3f::Zero()};

    Eigen::Vector3d current_vel_{Eigen::Vector3d::Zero()};
    bool vel_received_{false};

    PathTrajectory traj_;
    bool has_trajectory_{false};
    ros::Time traj_start_;
    Eigen::Vector3d traj_v0_{Eigen::Vector3d::Zero()};   // 轨迹生成时的实际速度
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
