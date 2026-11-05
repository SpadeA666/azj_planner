#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "azj_planner/clearance_field.h"

namespace azj_planner {

/// 把 3D clearance 场"柱状投影"成 2D 场。
///
/// ★ 为什么不能只用单层 z 的 ESDF：
///   2D 规划固定在某个飞行高度，但障碍物可能在上方（天花板、横梁）或
///   下方（桌子、矮墙）。只看 z=0.8 那一层，"桌子底下"会显示为可通行，
///   2D 规划器就会直直穿过去 —— 安全性直接丢掉。
///
/// 本类改成：在 (x,y) 固定的前提下，沿 z 在 [z_min, z_max] 内采样若干点，
/// 取**最小** clearance 作为该 (x,y) 的 2D clearance。
/// 语义上等价于"把 ±z 范围内的障碍都压扁投影到同一个平面"。
///
/// 采样点越密越准确，但每次查询开销线性增长。A* 一格一次查询，
/// 7 个采样点是性价比合适的默认值。
class PillarClearanceField : public ClearanceField {
public:
    PillarClearanceField(ClearanceField::Ptr base, double z_center, double z_half_range,
                         int z_samples)
        : base_(std::move(base)) {
        z_center_ = z_center;
        z_half_ = std::max(0.0, z_half_range);
        n_ = std::max(2, z_samples);
    }

    double clearance(const Eigen::Vector3d& p) const override {
        if (z_half_ < 1e-9) return base_->clearance(Eigen::Vector3d(p.x(), p.y(), z_center_));

        double mn = std::numeric_limits<double>::max();
        int valid = 0;
        for (int i = 0; i < n_; ++i) {
            const double t = static_cast<double>(i) / (n_ - 1);
            const double z = z_center_ - z_half_ + 2.0 * z_half_ * t;
            const double c = base_->clearance(Eigen::Vector3d(p.x(), p.y(), z));
            // 跳过"无数据"层。
            //
            // 为什么必须跳过而不是取 min：柱状范围通常会伸到地图 z 边界之外
            // （例如 z_center=0.8、半范围 1.6 时下探到 z=-0.8，低于
            // virtual_ground_height），那里的 ESDF 返回 0 或 kUnknown(=-1)。
            // 直接 min 会让**每一列**都变成 0/负数，2D 规划全线报
            // "start is inside an obstacle"。实测就是这个原因。
            //
            // 语义上这是对的：地图外的层没有障碍信息，不该当作障碍；
            // 真正要防的是"有数据但很近"的层。
            if (c <= kUnknownClearance + 1e-9) continue;   // 无数据
            mn = std::min(mn, c);
            ++valid;
        }
        // 整列都没有数据 → 交给调用方的未知策略（返回 kUnknownClearance）
        return valid > 0 ? mn : kUnknownClearance;
    }

    /// 柱状梯度：取 z 采样里 clearance 最小的那一层，用它的梯度。
    ///
    /// 为什么这样做：2D 规划的"约束来源"就是那一层最紧的障碍，
    /// 居中/避让都应该针对它。把 7 层梯度平均会稀释掉真正危险的障碍。
    bool clearanceGradient(const Eigen::Vector3d& p, Eigen::Vector3d& grad) const override {
        if (z_half_ < 1e-9) {
            return base_->clearanceGradient(Eigen::Vector3d(p.x(), p.y(), z_center_), grad);
        }

        double best_c = std::numeric_limits<double>::max();
        Eigen::Vector3d best_g = Eigen::Vector3d::Zero();
        bool found = false;
        for (int i = 0; i < n_; ++i) {
            const double t = static_cast<double>(i) / (n_ - 1);
            const double z = z_center_ - z_half_ + 2.0 * z_half_ * t;
            const Eigen::Vector3d q(p.x(), p.y(), z);
            const double c = base_->clearance(q);
            if (c <= kUnknownClearance + 1e-9) continue;   // 无数据层，跳过
            Eigen::Vector3d g;
            if (!base_->clearanceGradient(q, g)) continue;
            if (c < best_c) {
                best_c = c;
                best_g = g;
                found = true;
            }
        }
        if (!found) return false;
        grad = best_g;
        return true;
    }

private:
    ClearanceField::Ptr base_;
    double z_center_{0.0};
    double z_half_{0.0};
    int n_{7};
};

}  // namespace azj_planner
