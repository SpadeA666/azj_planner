#pragma once

#include <Eigen/Core>
#include <string>
#include <vector>

#include "azj_planner/clearance_field.h"

namespace azj_planner {

/// 搜索参数。
///
/// 与 EGO / SUPER 的根本差别在这里：那两个框架把 clearance 当**阈值**用
/// （`dist > dist0` 时代价恒为 0，于是解由最短路径决定，必然贴边）。
/// Azj 把 clearance 当**连续收益**用：代价 ≈ 路径长度 / clearance^alpha，
/// 越宽的地方越便宜，解自然落在通道中轴。
struct SearchParams {
    /// 栅格分辨率 [m]。
    double resolution{0.1};

    /// 硬安全半径 [m]。clearance 小于它的格子**不可通行**（起点/终点例外）。
    /// 这是安全底线，不是软惩罚——低于它直接拒绝，符合「安全优先」。
    double robot_radius{0.25};

    /// 代价归一化基准 [m]。clearance == ref 时单位长度代价恰为 1。
    double clearance_ref{1.0};

    /// clearance 上限 [m]。超过它不再额外奖励，防止为了更宽而绕大圈。
    double clearance_cap{2.0};

    /// 居中强度权重 w（次要目标）。
    ///
    /// 代价 = len * (1 + w * (ref / clamp(c, ·, cap))^alpha)
    /// w = 0 → 退化成纯最短路径，也就是 EGO/SUPER 的贴边行为。
    ///
    /// 实测调参参考（离线场景，见 test/offline_clearance_test.cpp）：
    ///   长通道 10m：w=0.05 → 横向偏移 0.236m 降到 0.008m，长度仅 +0.75%
    ///   短缝 1.6m ：w=0.20 才能把偏移从 0.481m 降到 0.027m，长度 +2.5%
    /// 惩罚沿路径**累积**，所以窄而短的通道需要更大的 w。
    /// 默认 0.10 是两者的折中：长通道近乎完美居中，短缝也明显改善，
    /// 长度代价 <1.6%。
    double clearance_weight{0.10};

    /// 居中强度指数。alpha 越大越「死板地走正中」。
    double alpha{2.0};

    /// 最大扩展节点数，防止病态地图卡死。
    int max_expansions{400000};

    /// 搜索框相对 起点-终点 包围盒的外扩 [m]。
    Eigen::Vector3d box_margin{2.0, 2.0, 1.5};

    /// 距终点小于该值即视为到达 [m]。
    double goal_tolerance{0.30};

    /// true = 只在 XY 平面规划（2D 模式），z 由调用方锁定。
    ///
    /// 语义：
    ///   - 搜索只走 8 邻域（dz 恒为 0），栅格 z 方向退化为单层
    ///   - 终点的 z 被强制成起点的 z（纯平面规划）
    ///   - clearance 的语义由调用方决定：**2D 模式下应该传"该 (x,y) 列上
    ///     z 范围内的最小 clearance"**，而不是单层 ESDF ——
    ///     否则桌子底下、天花板上的障碍会被漏掉，安全性就没了。
    ///     这个包装在节点层做（PillarClearanceField），算法层不感知。
    bool plan_2d{false};

    /// 高度保持权重与参考高度。
    ///
    /// 为什么必须有：z 方向原本完全自由，A* 只要不越界就往低处（或高处）
    /// 抄近路 —— 表现为"路径从地板下/天花板上绕"。光加硬边界不够，
    /// 因为贴着边界走仍然是合法且更短的。
    ///
    /// 代价形如：len * w_height * (dz / height_scale)^2
    /// w_height=0 关闭；0.3~1.0 通常够用。
    double w_height{0.0};
    /// 高度参考（一般是起终点平均高度）
    double height_ref{0.0};
    /// dz 归一化尺度 [m]，dz 等于它时代价增量 = w_height
    double height_scale{1.0};

    /// 高度豁免带宽 [m]。|z - height_ref| <= band 时不额外罚。
    ///
    /// ★ 为什么需要它（实测失衡）：
    ///   仅靠 w_height 的二次项压不住"往 clearance 大的地方钻"。
    ///   实测 raicom 场景：飞机 0.52、目标 0.80，
    ///     · clearance 0.3 → 0.5，代价因子 2.11 → 1.40（省 0.71/m）
    ///     · 高度偏离 0.6m，w_height=0.5 只罚 0.18/m
    ///   净省 0.53/m → A* 必然下降到 z=0.04（地面附近 UNKNOWN 被高估）。
    double height_band{0.30};

    /// 高度硬约束带 [m]。|z - height_ref| > z_band 的格子**直接判不可通行**。
    ///
    /// ★ 为什么软惩罚不够（实测）：
    ///   造一个"矮墙 z0.6~1.0 挡住目标高度、下方空旷"的场景，
    ///   A* 宁可从墙下钻（z=0.346，到地面 clearance 0.346，代价因子 1.84）
    ///   也不绕行 —— 因为贴墙代价因子高达 5.44（clearance 0.15）。
    ///   差距 3.6/m，而 w_height_band=20 在 excess=0.154 时只罚 0.47/m。
    ///   加大到 100 也不够（2.37/m），再大就会压死真实升降需求。
    ///   → 必须用**硬约束**。
    ///
    /// 0 = 关闭（退回纯软约束）。
    double z_band{0.0};

    /// 超出 height_band 后的强罚权重。
    /// excess = |dz| - band，代价 += len * w_height_band * excess^2。
    /// 给 20 时 excess=0.3 罚 1.8/m，足以压过 clearance 收益。
    double w_height_band{20.0};

    /// 起点逃生半径 [m]。
    /// 飞机可能已经贴在障碍旁（clearance < robot_radius），此时若严格按
    /// 安全半径拒绝所有邻居，就永远无法规划——因为无人机已处在不安全状态。
    /// 该半径内的格子只要求「不在障碍内」（clearance > 0），让规划器能把
    /// 飞机先引出来。区域内的低 clearance 会在结果里如实报告。
    double escape_radius{0.60};
};

/// 单次搜索结果。
struct SearchResult {
    bool success{false};
    std::string msg;

    /// 世界坐标路径（含起点与终点）。
    std::vector<Eigen::Vector3d> path;

    /// 与 path 等长的 clearance 曲线，便于画图核对「是否居中」。
    std::vector<double> clearance_along_path;

    double length{0.0};
    double min_clearance{0.0};    ///< 路径上的最小 clearance（瓶颈）
    double mean_clearance{0.0};   ///< 路径上的平均 clearance
    int expansions{0};
};

/// 以 clearance 为收益的 3D A*。
///
/// 代价模型：
///     edge_cost = ||Δp|| * (clearance_ref / clamp(c, ·, clearance_cap))^alpha
/// 其中 c 取相邻两格 clearance 的较小值（保守）。
class ClearanceAStar {
public:
    /// @param v_dir 入场速度方向（非零时用于"起点引出"，避免首段
    ///              被栅格 45° 量化成急转）。传 Zero 则关闭该功能。
    SearchResult search(const ClearanceField& field,
                        const Eigen::Vector3d& start,
                        const Eigen::Vector3d& goal,
                        const SearchParams& params,
                        const Eigen::Vector3d& v_dir = Eigen::Vector3d::Zero()) const;
};

/// 保形简化：只删除几乎共线的中间点，**绝不拉直**。
///
/// 为什么不能「视线拉直」：拉直的判据只能是「仍满足安全下界」，
/// 而贴近障碍的直线往往恰好满足下界——于是拉直会把 A* 优化出来的
/// 居中形状重新压回贴墙直线，居中代价就白算了。
std::vector<Eigen::Vector3d> simplifyPath(const std::vector<Eigen::Vector3d>& path,
                                          double collinear_cos_tol = 0.995);

/// 端点固定的移动平均平滑；任何一步若破坏 clearance 下界就保留原点。
std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path,
                                        const ClearanceField& field,
                                        double min_clearance,
                                        int iterations = 3,
                                        double weight = 0.35);

/// 沿路径重采样 clearance 曲线（顶点采样，与 path 等长）。
std::vector<double> sampleClearance(const std::vector<Eigen::Vector3d>& path,
                                    const ClearanceField& field);

/// 沿路径**按固定步长密集采样**，返回整条折线（含线段中间）的最小 clearance。
///
/// 不能用顶点采样代替：简化后的路径顶点可能稀疏，刚好跳过最窄处，
/// 会把「门内 0.27m」错报成「1.99m」，直接误导安全判断。
double minClearanceAlongPath(const std::vector<Eigen::Vector3d>& path,
                             const ClearanceField& field,
                             double step = 0.05);

/// 路径几何长度。
double pathLength(const std::vector<Eigen::Vector3d>& path);

}  // namespace azj_planner
