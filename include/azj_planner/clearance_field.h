#pragma once

#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

namespace azj_planner {

/// 清障场（clearance field）抽象。
///
/// 语义：返回世界坐标点到**最近障碍表面**的距离，单位米。
///  - 自由空间：> 0
///  - 障碍内部：<= 0
///  - 未知区域：按调用方策略返回（默认实现要求返回 <= 0，以保守处理）
///
/// 这一层刻意不依赖 ROS / rog_map，因此核心算法可以离线单测。
class ClearanceField {
public:
    using Ptr = std::shared_ptr<ClearanceField>;
    virtual ~ClearanceField() = default;

    virtual double clearance(const Eigen::Vector3d& p) const = 0;

    /// clearance 对位置的梯度 ∇c，**指向 clearance 增大的方向**
    /// （也就是远离最近障碍的方向）。
    ///
    /// 默认实现用中心差分兜底。派生类若能从 ESDF 直接解析求梯度
    /// （rog_map 的 evaluateFirstGrad）就应覆盖它 —— 曲线优化每个迭代
    /// 要算 thousands 次梯度，数值差分是 6 倍开销。
    ///
    /// 返回 false 表示该点无有效梯度（出界 / 未知），调用方应跳过该采样点。
    virtual bool clearanceGradient(const Eigen::Vector3d& p, Eigen::Vector3d& grad) const {
        const double h = 0.02;
        Eigen::Vector3d g;
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d d = Eigen::Vector3d::Zero();
            d[i] = h;
            const double cp = clearance(p + d);
            const double cm = clearance(p - d);
            g[i] = (cp - cm) / (2.0 * h);
            if (!std::isfinite(g[i])) return false;
        }
        grad = g;
        return true;
    }

    /// 约定：未知/无数据返回该值，调用方按「不可通行」处理。
    static constexpr double kUnknownClearance = -1.0;
};

/// 用一个可调用对象实现 ClearanceField（测试与快速接入都方便）。
/// 可选地传入解析梯度（例如 rog_map 的 evaluateFirstGrad），
/// 省掉 6 倍数值差分开销。
class FunctionClearanceField : public ClearanceField {
public:
    using ValueFn = std::function<double(const Eigen::Vector3d&)>;
    using GradFn = std::function<bool(const Eigen::Vector3d&, Eigen::Vector3d&)>;

    explicit FunctionClearanceField(ValueFn f) : f_(std::move(f)) {}
    FunctionClearanceField(ValueFn f, GradFn g) : f_(std::move(f)), g_(std::move(g)) {}

    double clearance(const Eigen::Vector3d& p) const override { return f_(p); }

    bool clearanceGradient(const Eigen::Vector3d& p, Eigen::Vector3d& grad) const override {
        if (g_) return g_(p, grad);
        return ClearanceField::clearanceGradient(p, grad);
    }

private:
    ValueFn f_;
    GradFn g_;
};

/// 把多个场按「取最小」合成：常用于「静态 ESDF + 动态障碍排斥」叠加。
class MinClearanceField : public ClearanceField {
public:
    void add(ClearanceField::Ptr field) { fields_.push_back(std::move(field)); }

    double clearance(const Eigen::Vector3d& p) const override {
        double best = std::numeric_limits<double>::max();
        for (const auto& f : fields_) best = std::min(best, f->clearance(p));
        return fields_.empty() ? kUnknownClearance : best;
    }

private:
    std::vector<ClearanceField::Ptr> fields_;
};

}  // namespace azj_planner
