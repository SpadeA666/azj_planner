// 离线验证：Azj Planner 的「居中」性质是否真的成立。
//
// 不依赖 ROS、不依赖地图、不需要 Bag——纯解析 clearance 场，
// 目的只有一个：把「贴边」和「从正中心过」的差别**量化**出来。

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "azj_planner/clearance_field.h"
#include "azj_planner/path_search.h"

using Eigen::Vector3d;
using namespace azj_planner;

namespace {

/// 轴对齐盒子的有符号距离（外正内负）。
double boxSdf(const Vector3d& p, const Vector3d& center, const Vector3d& half) {
    const Vector3d d = (p - center).cwiseAbs() - half;
    const Vector3d dmax = d.cwiseMax(0.0);
    const double outside = dmax.norm();
    const double inside = std::min(std::max(d.x(), std::max(d.y(), d.z())), 0.0);
    return outside + inside;
}

/// 门洞场景：上下两段**长墙**之间留 y ∈ [-0.3, 0.3] 的缝，门中心 y = 0。
/// 墙刻意延伸到搜索框之外，确保「绕过去」在几何上不可行——
/// 否则最短路径会直接绕行，根本不会经过门，测不出居中。
struct GateScene {
    Vector3d lower_c{4.5, -10.15, 0.0};
    Vector3d lower_h{0.5, 9.85, 10.0};   // x∈[4,5], y∈[-20,-0.3], z∈[-10,10]
    Vector3d upper_c{4.5, 10.15, 0.0};
    Vector3d upper_h{0.5, 9.85, 10.0};   // x∈[4,5], y∈[0.3,20],  z∈[-10,10]

    double clearance(const Vector3d& p) const {
        return std::min(boxSdf(p, lower_c, lower_h), boxSdf(p, upper_c, upper_h));
    }
};

/// 不对称走廊：上墙 y = 0，下墙 y = -3，中心 y = -1.5。
struct CorridorScene {
    double clearance(const Vector3d& p) const {
        const double d_upper = 0.0 - p.y();   // 走廊在 y<0 一侧
        const double d_lower = p.y() + 3.0;   // 下墙在 y=-3
        return std::min(d_upper, d_lower);
    }
};

/// 路径与平面 x = probe 的交点 y（线性插值）。找不到返回 NaN。
double yAtX(const std::vector<Vector3d>& path, double probe) {
    for (size_t i = 1; i < path.size(); ++i) {
        const Vector3d& a = path[i - 1];
        const Vector3d& b = path[i];
        if ((a.x() - probe) * (b.x() - probe) <= 0.0 && std::abs(b.x() - a.x()) > 1e-9) {
            const double t = (probe - a.x()) / (b.x() - a.x());
            return a.y() + t * (b.y() - a.y());
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}

struct Row {
    std::string label;
    bool success{false};
    std::string msg;
    double y_probe{0.0};
    double length{0.0};
    double min_clr{0.0};
    double mean_clr{0.0};
    int expansions{0};
};

Row runScene(const std::string& label, const ClearanceField& field, const Vector3d& start,
             const Vector3d& goal, double clearance_weight, double alpha, double robot_radius,
             double probe_x, int max_expansions = 4000000) {
    Row r;
    r.label = label;

    SearchParams p;
    p.resolution = 0.05;
    p.robot_radius = robot_radius;
    p.clearance_ref = 1.0;
    p.clearance_cap = 2.0;
    p.clearance_weight = clearance_weight;
    p.alpha = alpha;
    p.box_margin = Vector3d(1.0, 3.0, 1.0);
    p.goal_tolerance = 0.20;
    p.max_expansions = max_expansions;

    ClearanceAStar planner;
    const SearchResult res = planner.search(field, start, goal, p);

    r.success = res.success;
    r.msg = res.msg;
    if (!res.success) return r;

    const auto simplified = simplifyPath(res.path);
    const auto smoothed = smoothPath(simplified, field, robot_radius, 3, 0.35);

    r.y_probe = yAtX(smoothed, probe_x);
    r.length = pathLength(smoothed);
    // 必须沿线段密集采样：顶点稀疏时会跳过最窄处，报出虚假的高 clearanc
    r.min_clr = minClearanceAlongPath(smoothed, field, 0.02);
    double sum = 0.0;
    for (const auto& q : smoothed) sum += field.clearance(q);
    r.mean_clr = sum / static_cast<double>(smoothed.size());
    r.expansions = res.expansions;
    return r;
}

void printRow(const Row& r, double center) {
    if (!r.success) {
        std::printf("  %-30s 规划失败: %s\n", r.label.c_str(), r.msg.c_str());
        return;
    }
    std::printf("  %-30s  y=%-7.3f (偏中心%6.3f)  len=%-6.2f  min=%-6.3f  mean=%-6.3f\n",
                r.label.c_str(), r.y_probe, std::abs(r.y_probe - center), r.length, r.min_clr,
                r.mean_clr);
}

/// 绕 z 轴旋转的盒子 SDF（用于竖直墙）。
/// 胶囊墙建在 z=0 平面，两端是圆的，规划器可以往 z 方向偏一点来拉大
/// clearance（“从墙端头绕过去”）。真实墙体在 z 方向是延伸的，所以这里用
/// 竖直旋转盒更贴近实际，也能避开这种不真实的 z 逃逸。
double rotBoxSdf(const Vector3d& p, const Vector3d& center, const Vector3d& half, double yaw) {
    const Vector3d d = p - center;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    const Vector3d l(c * d.x() - s * d.y(), s * d.x() + c * d.y(), d.z());
    const Vector3d q = l.cwiseAbs() - half;
    const Vector3d qmax = q.cwiseMax(0.0);
    return qmax.norm() + std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0);
}

/// 路径到某点的最小距离��必须沿线段密集采样）。
/// 只遍历顶点会漏掉“穿越点落在段中间”的情形，导致误报很大或很小的距离。
double minDistToPoint(const std::vector<Vector3d>& path, const Vector3d& target,
                      double step = 0.05) {
    if (path.empty()) return std::numeric_limits<double>::quiet_NaN();
    double best = (path.front() - target).norm();
    for (size_t i = 1; i < path.size(); ++i) {
        const Vector3d& a = path[i - 1];
        const Vector3d& b = path[i];
        const double len = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
        for (int k = 0; k <= n; ++k) {
            best = std::min(best, ((a + (b - a) * (static_cast<double>(k) / n)) - target).norm());
        }
    }
    return best;
}

/// 胶囊体（线段 + 半径）的有符号距离。
/// 用它拼连续斜墙：相邻胶囊首尾相接，缝里没有空隙可钻。
struct Capsule {
    Vector3d a, b;
    double r{0.25};

    double sdf(const Vector3d& p) const {
        const Vector3d pa = p - a;
        const Vector3d ba = b - a;
        const double bb = ba.dot(ba);
        double h = bb > 1e-12 ? pa.dot(ba) / bb : 0.0;
        h = std::min(std::max(h, 0.0), 1.0);
        return (pa - h * ba).norm() - r;
    }
};

struct ChainScene {
    std::vector<Capsule> caps;

    double clearance(const Vector3d& p) const {
        double m = std::numeric_limits<double>::max();
        for (const auto& c : caps) m = std::min(m, c.sdf(p));
        return m;
    }

    /// 沿方向 dir 从 base 铺一串首尾相接的胶囊，形成连续墙。
    void addChain(const Vector3d& base, const Vector3d& dir, double total_len, double seg_len,
                  double radius) {
        const Vector3d u = dir.normalized();
        const int n = std::max(1, static_cast<int>(std::ceil(total_len / seg_len)));
        for (int i = 0; i < n; ++i) {
            Capsule c;
            c.a = base + u * (i * seg_len);
            c.b = base + u * ((i + 1) * seg_len);
            c.r = radius;
            caps.push_back(c);
        }
    }
};

/// 规划并返回平滑后的路径（失败返回空）。
std::vector<Vector3d> planPath(const ClearanceField& field, const Vector3d& start,
                               const Vector3d& goal, double w, double alpha, double robot_radius,
                               const Vector3d& margin) {
    SearchParams p;
    p.resolution = 0.05;
    p.robot_radius = robot_radius;
    p.clearance_ref = 1.0;
    p.clearance_cap = 2.0;
    p.clearance_weight = w;
    p.alpha = alpha;
    p.box_margin = margin;
    p.goal_tolerance = 0.25;
    p.max_expansions = 4000000;

    ClearanceAStar planner;
    const SearchResult r = planner.search(field, start, goal, p);
    if (!r.success) {
        std::printf("    规划失败: %s\n", r.msg.c_str());
        return {};
    }
    return smoothPath(simplifyPath(r.path), field, robot_radius, 3, 0.35);
}

/// 路径在「沿通道轴向进度 = t_target」处的横向偏移（带符号，沿法向 n）。
double lateralOffsetAt(const std::vector<Vector3d>& path, const Vector3d& origin,
                       const Vector3d& axis, const Vector3d& normal, double t_target) {
    if (path.empty()) return std::numeric_limits<double>::quiet_NaN();
    double best_err = 1e18;
    double best_off = 0.0;
    for (const auto& p : path) {
        const Vector3d rel = p - origin;
        const double t = rel.dot(axis);
        const double err = std::abs(t - t_target);
        if (err < best_err) {
            best_err = err;
            best_off = rel.dot(normal);
        }
    }
    return best_off;
}

/// ASCII 俯视图（z = 0 切面）：'#' 障碍、'*' 路径、'S' 起点、'G' 终点、'.' 空地。
void drawTopView(const ClearanceField& field, const std::vector<Vector3d>& path,
                 const Vector3d& s, const Vector3d& g, double x0, double x1, double y0,
                 double y1) {
    const double dx = 0.25;
    const double dy = 0.50;   // 终端字符高约为宽的两倍，dy 取两倍才不失真
    const int nx = static_cast<int>((x1 - x0) / dx) + 1;
    const int ny = static_cast<int>((y1 - y0) / dy) + 1;

    std::vector<std::string> grid(ny, std::string(nx, '.'));

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Vector3d q(x0 + i * dx, y0 + j * dy, 0.0);
            if (field.clearance(q) <= 0.02) grid[j][i] = '#';
        }
    }

    auto put = [&](const Vector3d& p, char ch) {
        const int i = static_cast<int>(std::lround((p.x() - x0) / dx));
        const int j = static_cast<int>(std::lround((p.y() - y0) / dy));
        if (i >= 0 && i < nx && j >= 0 && j < ny) grid[j][i] = ch;
    };

    // 沿线段插值填充，避免稀疏路径画成断点
    for (size_t i = 1; i < path.size(); ++i) {
        const Vector3d& a = path[i - 1];
        const Vector3d& b = path[i];
        const double len = (b - a).norm();
        const int n = std::max(1, static_cast<int>(std::ceil(len / 0.05)));
        for (int k = 0; k <= n; ++k) put(a + (b - a) * (static_cast<double>(k) / n), '*');
    }
    put(s, 'S');
    put(g, 'G');

    for (int j = ny - 1; j >= 0; --j) {
        std::printf("    %6.2f |%s|\n", y0 + j * dy, grid[j].c_str());
    }
    std::printf("           +");
    for (int i = 0; i < nx; ++i) std::printf("-");
    std::printf("+\n");
}

}  // namespace

int main() {
    std::printf("\n============== Azj Planner 离线核心验证 ==============\n");

    // ---------------- 场景 1：门洞 ----------------
    {
        std::printf("\n[场景 1] 门洞：缝 y∈[-0.3,0.3]，中心 y=0，净宽 0.6m\n");
        std::printf("         起终点都在 y=-1.5 —— 直线必须穿过障碍，只能从门过。\n");
        std::printf("         robot_radius=0.10，探针 x=4.5（门中心）\n");
        std::printf("         理想：w=0 贴门边(y≈-0.2)，w 增大后趋近 y=0\n\n");

        const GateScene scene;
        const FunctionClearanceField field([&](const Vector3d& q) { return scene.clearance(q); });
        const Vector3d start(0.0, -1.5, 0.0);
        const Vector3d goal(9.0, -1.5, 0.0);

        for (double w : {0.0, 0.005, 0.02, 0.05, 0.2}) {
            char label[64];
            std::snprintf(label, sizeof(label), "w=%-5.3f alpha=2", w);
            printRow(runScene(label, field, start, goal, w, 2.0, 0.10, 4.5), 0.0);
        }
    }

    // ---------------- 场景 2：不对称走廊 ----------------
    {
        std::printf("\n[场景 2] 不对称走廊：上墙 y=0，下墙 y=-3，中心 y=-1.5\n");
        std::printf("         起终点贴上方墙 y=-0.2 —— 纯最短路径全程贴墙\n");
        std::printf("         robot_radius=0.10，探针 x=5.0\n\n");

        const CorridorScene scene;
        const FunctionClearanceField field([&](const Vector3d& q) { return scene.clearance(q); });
        const Vector3d start(0.0, -0.2, 0.0);
        const Vector3d goal(10.0, -0.2, 0.0);

        for (double w : {0.0, 0.02, 0.2}) {
            char label[64];
            std::snprintf(label, sizeof(label), "w=%-5.3f alpha=2", w);
            printRow(runScene(label, field, start, goal, w, 2.0, 0.10, 5.0), -1.5);
        }
    }

    // ---------------- 场景 3：硬安全半径必须真的拦得住 ----------------
    {
        std::printf("\n[场景 3] 硬安全半径：robot_radius=0.50 > 门半宽 0.30\n");
        std::printf("         期望 success=false（门过不去就明确拒绝，不硬挤）\n\n");

        const GateScene scene;
        const FunctionClearanceField field([&](const Vector3d& q) { return scene.clearance(q); });
        const Row r = runScene("robot_radius=0.50", field, Vector3d(0, -1.5, 0),
                               Vector3d(9, -1.5, 0), 0.02, 2.0, 0.50, 4.5, 4000000);
        std::printf("  success=%s  msg=%s\n", r.success ? "true" : "false", r.msg.c_str());
    }

    // ---------------- 场景 4：起点贴墙不能崩 ----------------
    {
        std::printf("\n[场景 4] 起点贴墙：起点 y=-0.05，clearance=0.05 < robot_radius=0.10\n");
        std::printf("         期望：起点放行，路径尽快离开贴墙区\n\n");

        const CorridorScene scene;
        const FunctionClearanceField field([&](const Vector3d& q) { return scene.clearance(q); });
        const Row r = runScene("起点贴墙", field, Vector3d(0, -0.05, 0), Vector3d(10, -0.05, 0),
                               0.02, 2.0, 0.10, 5.0);
        printRow(r, -1.5);
        if (r.success) std::printf("  msg=%s\n", r.msg.c_str());
        else std::printf("  msg=%s\n", r.msg.c_str());
    }

    // ---------------- 场景 5：斜向长走廊（对角线通道） ----------------
    {
        std::printf("\n[场景 5] 斜向走廊（45°）：两侧连续胶囊墙，半净宽 0.60m\n");
        std::printf("         起终点各偏在一侧 —— 纯最短会贴一侧墙，居中应走斜向中轴\n");
        std::printf("         robot_radius=0.30\n\n");

        const Vector3d A(2.0, 2.0, 0.0);
        const Vector3d B(8.0, -4.0, 0.0);
        const Vector3d axis = (B - A).normalized();          // (0.7071, -0.7071)
        const Vector3d normal(-axis.y(), axis.x(), 0.0);     // (0.7071,  0.7071)
        const double W = 0.60;    // 半净宽
        const double R = 0.25;    // 墙半径
        const double ext = 1.5;   // 两端延长，确保不能从端头绕过去
        const double wall_len = 2.0 * ext + (B - A).norm();

        ChainScene scene;
        const Vector3d base0 = A - axis * ext;
        scene.addChain(base0 + normal * (W + R), axis, wall_len, 0.5, R);
        scene.addChain(base0 - normal * (W + R), axis, wall_len, 0.5, R);

        const FunctionClearanceField field(
            [&](const Vector3d& q) { return scene.clearance(q); });

        const Vector3d start = A - axis * 1.0 + normal * 0.25;   // 偏在 +n 侧
        const Vector3d goal = B + axis * 1.0 - normal * 0.25;    // 偏在 -n 侧
        const double channel_len = (B - A).norm();

        std::printf("  ---- w 扫描：通道内横向偏移 vs 长度代价 ----\n");
        std::printf("     %-7s %-11s %-11s %-9s %-10s\n", "w", "t=.25 偏/m", "t=.50 偏/m",
                    "len/m", "增量/m");
        std::vector<Vector3d> path_w0, path_best;
        double len0 = 0.0;
        for (double w : {0.0, 0.01, 0.02, 0.05, 0.10, 0.20}) {
            const auto path =
                planPath(field, start, goal, w, 2.0, 0.30, Vector3d(1.5, 1.5, 0.6));
            if (path.empty()) {
                std::printf("     %-7.3f  规划失败\n", w);
                continue;
            }
            const double len = pathLength(path);
            if (w == 0.0) len0 = len;
            std::printf("     %-7.3f %+11.3f %+11.3f %-9.3f %+10.3f\n", w,
                        lateralOffsetAt(path, A, axis, normal, 0.25 * channel_len),
                        lateralOffsetAt(path, A, axis, normal, 0.50 * channel_len), len,
                        len - len0);
            if (w == 0.0) path_w0 = path;
            if (std::abs(w - 0.10) < 1e-9) path_best = path;
        }

        if (!path_w0.empty()) {
            std::printf("\n  ---- w=0.000 纯最短（EGO 式）俯视图 ----\n");
            drawTopView(field, path_w0, start, goal, 0.0, 10.0, -6.5, 3.5);
        }
        if (!path_best.empty()) {
            std::printf("\n  ---- w=0.100 居中（默认值）俯视图 ----\n");
            drawTopView(field, path_best, start, goal, 0.0, 10.0, -6.5, 3.5);
        }
    }

    // ---------------- 场景 6：斜置竖直墙留斜缝（斜门） ----------------
    {
        std::printf("\n[场景 6] 斜置竖直墙留缝（45° 斜门）：净缝宽 1.60m，缝心 (4,-1)\n");
        std::printf("         墙在 z 方向延伸 ±2m，防止从墙顶/墙底绕过去\n");
        std::printf("         起终点在墙两侧 —— 应从斜缝正中心穿过\n\n");

        const Vector3d C(4.0, -1.0, 0.0);
        const double yaw = M_PI / 4.0;
        const Vector3d wd(std::cos(yaw), std::sin(yaw), 0.0);   // 墙延伸方向
        const Vector3d normal(-wd.y(), wd.x(), 0.0);           // 墙法向
        const double half_gap = 0.80;    // 缝半净宽（缝心 clearance = 0.80）
        const double thick_half = 0.10;
        const double L_half = 7.0;    // 墙沿自身方向半长
        const double H_half = 2.0;    // 墙高半尺寸（z 方向延伸，堵住绕顶）
        const Vector3d half(L_half, thick_half, H_half);

        // 缝是「同一堵墙中间缺一段」：两段墙沿 wd 方向各退开 half_gap，
        // 各自延伸到远处。沿厚度方向偏移会变成「平行通道」，那是另一回事。
        const double off = half_gap + L_half;
        const Vector3d wallA = C + wd * off;
        const Vector3d wallB = C - wd * off;

        const FunctionClearanceField field([&](const Vector3d& q) {
            return std::min(rotBoxSdf(q, wallA, half, yaw), rotBoxSdf(q, wallB, half, yaw));
        });

        // 起终点各沿墙方向偏 0.50m —— 直线过缝处会偏到距缝心 0.50m，
        // 那里 clearance 只剩 0.30m（刚好卡在安全线上），居中优化有发挥空间。
        // 若起终点正对称，直线本来就在缝心，测不出居中能力。
        const Vector3d start = C - normal * 3.5 + wd * 0.50;
        const Vector3d goal = C + normal * 3.5 + wd * 0.50;

        std::printf("   (缝心 clearance = %.2f m，robot_radius = 0.30 m)\n", half_gap);
        std::printf("   (起终点各沿墙向偏 0.50m，直线过缝处 clearance 仅 0.30m，必须绕回缝心)\n\n");

        // w 扫描：看清「居中效果」与「长度代价」的权衡，便于按实际场景定参
        std::printf("  ---- w 扫描：居中效果 vs 长度代价 ----\n");
        std::printf("     %-7s %-10s %-10s %-9s %-10s\n", "w", "偏缝心/m", "min_clr/m", "len/m",
                    "增量/m");
        std::vector<Vector3d> path_w0, path_w002;
        double len0 = 0.0;
        for (double w : {0.0, 0.01, 0.02, 0.05, 0.10, 0.20, 0.40}) {
            const auto path =
                planPath(field, start, goal, w, 2.0, 0.30, Vector3d(2.0, 2.0, 0.6));
            if (path.empty()) {
                std::printf("     %-7.3f  规划失败\n", w);
                continue;
            }
            const double len = pathLength(path);
            if (w == 0.0) len0 = len;
            std::printf("     %-7.3f %-10.3f %-10.3f %-9.3f %+10.3f\n", w,
                        minDistToPoint(path, C, 0.02), minClearanceAlongPath(path, field, 0.02),
                        len, len - len0);
            if (w == 0.0) path_w0 = path;
            if (std::abs(w - 0.02) < 1e-9) path_w002 = path;
        }

        if (!path_w0.empty()) {
            std::printf("\n  ---- w=0.000 纯最短（EGO 式）俯视图 ----\n");
            drawTopView(field, path_w0, start, goal, -0.5, 9.0, -5.5, 3.5);
        }
        if (!path_w002.empty()) {
            std::printf("\n  ---- w=0.100 居中（默认值）俯视图 ----\n");
            drawTopView(field, path_w002, start, goal, -0.5, 9.0, -5.5, 3.5);
        }
    }

    std::printf("\n=====================================================\n\n");
    return 0;
}
