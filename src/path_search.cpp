#include "azj_planner/path_search.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <queue>

namespace azj_planner {
namespace {

constexpr double kEps = 1e-6;
constexpr size_t kMaxCells = 6000000;  // 6M 格上限，约 80MB，防止病态参数炸内存

struct Grid {
    Eigen::Vector3d origin{0, 0, 0};
    int nx{0}, ny{0}, nz{0};
    double res{0.1};

    std::vector<float> clr;    // NaN 表示尚未求值（惰性求 ESDF）
    std::vector<float> g;
    std::vector<int> parent;
    std::vector<uint8_t> closed;

    inline size_t idx(int i, int j, int k) const {
        return static_cast<size_t>(i) + static_cast<size_t>(nx) *
                                          (static_cast<size_t>(j) +
                                           static_cast<size_t>(ny) * static_cast<size_t>(k));
    }
    inline size_t size() const {
        return static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nz);
    }

    Eigen::Vector3d pos(int i, int j, int k) const {
        return origin + Eigen::Vector3d((i + 0.5) * res, (j + 0.5) * res, (k + 0.5) * res);
    }

    void clampIndex(int& i, int& j, int& k) const {
        i = std::min(std::max(i, 0), nx - 1);
        j = std::min(std::max(j, 0), ny - 1);
        k = std::min(std::max(k, 0), nz - 1);
    }

    bool toIndex(const Eigen::Vector3d& p, int& i, int& j, int& k) const {
        const Eigen::Vector3d d = (p - origin) / res;
        i = static_cast<int>(std::floor(d.x()));
        j = static_cast<int>(std::floor(d.y()));
        k = static_cast<int>(std::floor(d.z()));
        const bool inside = (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz);
        clampIndex(i, j, k);
        return inside;
    }
};

/// 惰性查询 + 缓存。ESDF 查询是 O(1) 插值，但同一格会被反复访问。
inline double getClr(Grid& g, int i, int j, int k, const ClearanceField& field) {
    float& c = g.clr[g.idx(i, j, k)];
    if (std::isnan(c)) {
        double v = field.clearance(g.pos(i, j, k));
        if (!std::isfinite(v)) v = ClearanceField::kUnknownClearance;
        c = static_cast<float>(v);
    }
    return static_cast<double>(c);
}

struct OpenEntry {
    float f;
    int idx;
    bool operator>(const OpenEntry& o) const { return f > o.f; }
};

/// 邻居表。2D 模式下只返回 dz=0 的 8 个方向，
/// 保证搜索严格限制在一个平面内（不会靠 z 方向绕障碍）。
struct NeighborTable {
    std::array<std::array<int, 3>, 26> dirs{};
    int count{0};
};

const NeighborTable& neighbors(bool plan_2d) {
    static const NeighborTable k3d = [] {
        NeighborTable t;
        int n = 0;
        for (int di = -1; di <= 1; ++di)
            for (int dj = -1; dj <= 1; ++dj)
                for (int dk = -1; dk <= 1; ++dk)
                    if (di || dj || dk) t.dirs[n++] = {di, dj, dk};
        t.count = n;
        return t;
    }();
    static const NeighborTable k2d = [] {
        NeighborTable t;
        int n = 0;
        for (int di = -1; di <= 1; ++di)
            for (int dj = -1; dj <= 1; ++dj)
                if (di || dj) t.dirs[n++] = {di, dj, 0};
        t.count = n;
        return t;
    }();
    return plan_2d ? k2d : k3d;
}

bool segmentClearanceOk(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                        const ClearanceField& field, double min_clearance,
                        int samples_per_meter) {
    const double len = (b - a).norm();
    const int n = std::max(2, static_cast<int>(std::ceil(len * samples_per_meter)));
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const Eigen::Vector3d p = a + t * (b - a);
        if (field.clearance(p) < min_clearance) return false;
    }
    return true;
}

}  // namespace

SearchResult ClearanceAStar::search(const ClearanceField& field,
                                   const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& goal,
                                   const SearchParams& params,
                                   const Eigen::Vector3d& v_dir_in) const {
    const bool has_v_dir = (v_dir_in.head<2>().norm() > 0.2);
    const Eigen::Vector3d v_dir = has_v_dir ? v_dir_in.normalized()
                                            : Eigen::Vector3d::Zero();
    SearchResult res;

    if (!(params.resolution > 0.0)) {
        res.msg = "resolution must be > 0";
        return res;
    }
    if (!start.allFinite() || !goal.allFinite()) {
        res.msg = "start or goal contains NaN/Inf";
        return res;
    }

    const double res_step = params.resolution;
    // 2D 模式：所有采样点压到【目标高度】平面上。
    //
    // 为什么用 goal.z 而不是 start.z：
    //   2D 规划的含义是"在目标飞行高度上做平面避障"。如果用当前高度，
    //   飞机一旦掉高就锁在低处再也爬不回去。用 goal.z 则控制器会先
    //   过渡到目标高度，再在平面上避障。
    //   代价：过渡段（当前高度→目标高度）的垂直路径本身不做避障，
    //   所以调用方应在 2D 模式下用足够大的 z 采样范围让 clearance
    //   覆盖整个过渡段（见节点里的 PillarClearanceField 配置）。
    Eigen::Vector3d s_use = start;
    Eigen::Vector3d g_use = goal;
    if (params.plan_2d) {
        s_use.z() = g_use.z();
    }

    Eigen::Vector3d bmin = s_use.cwiseMin(g_use) - params.box_margin;
    Eigen::Vector3d bmax = s_use.cwiseMax(g_use) + params.box_margin;

    // 2D 模式：z 方向的框必须**紧贴规划平面**，不能套用 box_margin。
    // 否则 nz 虽然是 1，但那一层的格子中心在 origin.z + 0.5*res，
    // 而 origin.z = bmin.z = z_plane - box_margin.z —— 实测会跑到 z=-0.65，
    // 整条路径掉到地下（几何上完全错，且 clearance 全取错位置）。
    if (params.plan_2d) {
        bmin.z() = s_use.z() - 0.5 * res_step;
        bmax.z() = s_use.z() + 0.5 * res_step;
    }

    Grid g;
    g.origin = bmin;
    g.res = res_step;
    g.nx = std::max(1, static_cast<int>(std::ceil((bmax.x() - bmin.x()) / res_step)));
    g.ny = std::max(1, static_cast<int>(std::ceil((bmax.y() - bmin.y()) / res_step)));
    g.nz = params.plan_2d
               ? 1
               : std::max(1, static_cast<int>(std::ceil((bmax.z() - bmin.z()) / res_step)));

    const size_t total = g.size();
    if (total > kMaxCells) {
        res.msg = "search box too large (" + std::to_string(total) + " cells > limit " +
                  std::to_string(kMaxCells) + "); reduce box_margin or increase resolution";
        return res;
    }

    g.clr.assign(total, std::numeric_limits<float>::quiet_NaN());
    g.g.assign(total, std::numeric_limits<float>::infinity());
    g.parent.assign(total, -1);
    g.closed.assign(total, 0);

    int si, sj, sk, gi, gj, gk;
    g.toIndex(s_use, si, sj, sk);
    g.toIndex(g_use, gi, gj, gk);
    const int s_idx = static_cast<int>(g.idx(si, sj, sk));
    const int t_idx = static_cast<int>(g.idx(gi, gj, gk));

    // 最小单位代价（clearance 取到上限时），用作 A* 启发式的 admissible 系数。
    // 因为 c 被 clamp 到 cap，单位代价下界就是 clearance=cap 时的取值。
    const double min_unit =
        1.0 + params.clearance_weight *
                  std::pow(params.clearance_ref / std::max(params.clearance_cap, 1e-3),
                           params.alpha);

    auto heuristic = [&](const Eigen::Vector3d& p) {
        return (p - g_use).norm() * min_unit;
    };

    // 核心代价：长度是主目标，clearance 是有界修正项。
    // 这是与 EGO/SUPER 的分水岭——它们的碰撞项在 clearance 充足时恒为 0，
    // 这里则在任何 clearance 下都留有一个小梯度，把路径往通道中轴推。
    auto edgeCost = [&](double len, double c, double z) {
        const double cc = std::min(std::max(c, 1e-3), params.clearance_cap);
        const double penalty = std::pow(params.clearance_ref / cc, params.alpha);
        double cost = len * (1.0 + params.clearance_weight * penalty);

        // 高度保持：偏离 height_ref 越远越贵。
        // 不加这一项时，z 方向完全自由 —— A* 会贴着 floor_z / ceil_z 走，
        // 因为那样更短。加了之后路径倾向维持在起终点高度附近，
        // 只有真的需要才上下移动。
        if (params.w_height > 1e-9 || params.w_height_band > 1e-9) {
            const double dz_abs = std::abs(z - params.height_ref);
            const double dz = (z - params.height_ref) / std::max(1e-3, params.height_scale);
            cost += len * params.w_height * dz * dz;

            // 超出豁免带后强罚：这一项才是真正压住"往 clearance 大的地方
            // （往往是地面附近的 UNKNOWN 区）钻"的机制。
            const double band = std::max(0.0, params.height_band);
            if (params.w_height_band > 1e-9 && dz_abs > band) {
                const double excess = dz_abs - band;
                cost += len * params.w_height_band * excess * excess;
            }
        }
        return cost;
    };

    // 起点例外：飞机当前所在位置必须是可规划的，否则整个规划器瘫痪。
    //
    // ★★ 判据从 `> 0` 放宽到 `>= 0`（2026-11-14 重大修正）
    //
    // `clearance == 0` 的语义是"**贴着障碍表面**"，不是"在障碍内部"。
    // 飞机趴在地面 / 紧贴柱子时，起点 clearance 就是 0。此时拒绝规划
    // 意味着**规划器彻底瘫痪**，飞机完全没有参考 —— 比"从不安全位置
    // 规划出来"危险得多。
    //
    // 实测（/tmp/azj_flight_20261114_214929）：
    //   t=36.994  start diag: pos=(0,0,0.05) esdf=0.000 unknown=0
    //             clearance_used=0.000
    //   → 本判据为假 → `plan failed: start is inside an obstacle`
    //     **连续 15 次**（36.99~51.99 每秒一次）→ 飞机无参考 → 掉高
    //
    // 注意 2D 模式的特殊性：`PillarClearanceField` 在 z_half=0 时
    // **完全忽略 p.z**，直接用规划平面（plane=goal.z）查 clearance。
    // 所以"抬高起点 z"对 2D 无效 —— 只能靠放宽这个判据。
    // （3D 模式下 replanCallback 的逐步抬升仍然有用。）
    //
    // 逃逸能力由下面的 escape 逻辑保证：起点 clearance < robot_radius 时，
    // 允许走向"不比起点更差"且"在 escape_radius 内"的邻居。
    const double start_clr = getClr(g, si, sj, sk, field);
    const bool start_passable = start_clr >= -1e-9;   // kUnknownClearance(-1) 仍拒绝
    if (!start_passable) {
        res.msg = "start is deeply inside an obstacle (clearance=" +
                  std::to_string(start_clr) + "); refusing to plan";
        return res;
    }
    if (start_clr < 1e-9) {
        // ⚠ azj_core 是纯算法库，**不能依赖 ROS**（否则链接不过）。
        // 用 fprintf + 静态计数做节流；核心信息同时体现在 res.msg 里。
        static int touch_cnt = 0;
        if (touch_cnt++ % 20 == 0) {
            std::fprintf(stderr,
                "[azj-plansearch] start touches an obstacle (clearance=%.3f m) - "
                "planning via the escape rule (robot_radius=%.2f, escape_radius=%.2f)\n",
                start_clr, params.robot_radius, params.escape_radius);
        }
    }

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> open;
    g.g[s_idx] = 0.0f;
    open.push({static_cast<float>(heuristic(s_use)), s_idx});

    int expansions = 0;
    int found = -1;

    while (!open.empty()) {
        const OpenEntry e = open.top();
        open.pop();
        if (g.closed[e.idx]) continue;
        g.closed[e.idx] = 1;
        ++expansions;

        if (expansions > params.max_expansions) {
            res.msg = "hit max_expansions (" + std::to_string(params.max_expansions) +
                      "); search truncated";
            break;
        }

        int i = e.idx % g.nx;
        const int rem = e.idx / g.nx;
        int j = rem % g.ny;
        int k = rem / g.ny;

        const Eigen::Vector3d p = g.pos(i, j, k);
        if ((p - g_use).norm() <= params.goal_tolerance || e.idx == t_idx) {
            found = e.idx;
            break;
        }

        const double c_cur = getClr(g, i, j, k, field);

        const NeighborTable& nbt = neighbors(params.plan_2d);
        for (int di = 0; di < nbt.count; ++di) {
            const auto& d = nbt.dirs[di];
            const int ni = i + d[0], nj = j + d[1], nk = k + d[2];
            if (ni < 0 || ni >= g.nx || nj < 0 || nj >= g.ny || nk < 0 || nk >= g.nz) continue;

            const size_t nidx = g.idx(ni, nj, nk);
            if (g.closed[nidx]) continue;

            const double cn = getClr(g, ni, nj, nk, field);
            if (cn <= 0.0) continue;                       // 障碍内部
            const Eigen::Vector3d np = g.pos(ni, nj, nk);
            const bool is_goal_cell = (static_cast<int>(nidx) == t_idx);

            // 高度硬约束：偏离参考高度超过 z_band 直接不走。
            //
            // 例外（三者都是必要的）：
            //   ① 终点格：用户指定，必须能到达
            //   ② 起点格
            //   ③ **起点爬升通道**：飞机在地面/低处时（z 低于 height_ref-z_band），
            //      允许在起点正上方垂直爬升到允许区。否则起点连不通 → 永远 NO_PATH。
            //      限制在水平半径 escape_radius 内，且只允许朝参考高度方向走，
            //      不会变成"从低处绕行"。
            if (params.z_band > 1e-6) {
                const double dz_ref = std::abs(np.z() - params.height_ref);
                const bool is_start_cell = (static_cast<int>(nidx) == s_idx);
                if (!is_goal_cell && !is_start_cell && dz_ref > params.z_band) {
                    // 起点爬升通道
                    const bool low_side = (s_use.z() < params.height_ref);
                    const double horiz = std::hypot(np.x() - s_use.x(), np.y() - s_use.y());
                    const bool toward_ref =
                        low_side ? (np.z() >= s_use.z() - 1e-6 &&
                                    np.z() <= params.height_ref + 1e-6)
                                 : (np.z() <= s_use.z() + 1e-6 &&
                                    np.z() >= params.height_ref - 1e-6);
                    if (horiz <= params.escape_radius && toward_ref) {
                        // 允许：这是从起点爬升/下降到规划高度
                    } else {
                        continue;
                    }
                }
            }
            if (!is_goal_cell && cn < params.robot_radius) {
                // 起点逃生区：飞机可能已贴障碍，必须允许它先飞出来，
                // 否则「已处于不安全状态」会让规划器永远无解。
                //
                // ★ 必须带两个条件，否则 robot_radius 形同虚设：
                //
                //   ① 只在**起点本身就不安全**时启用。
                //      早期版本无条件启用，于是起点在房间中央（clearance 0.45）
                //      时，0.6m 球内所有 clearance<0.3 的格子照样可走
                //      → 实测路径贴着 clearance=0.12 的墙走，
                //      表现为"robot_radius 和膨胀层不生效、专门往点云密集处走"。
                //
                //   ② 逃生区内也要求 clearance **不比起点更差**（单调不降）。
                //      否则会从起点"往更危险的方向"钻。
                if (start_clr >= params.robot_radius) continue;      // ① 起点安全 → 严约束
                if (cn < start_clr - 1e-6) continue;                 // ② 不许更糟
                if ((np - s_use).norm() > params.escape_radius) continue;
            }

            const double step = res_step * std::sqrt(static_cast<double>(d[0] * d[0] +
                                                                        d[1] * d[1] +
                                                                        d[2] * d[2]));
            const double cmin = std::min(c_cur, cn);       // 保守取两侧较小值
            const double ng = static_cast<double>(g.g[e.idx]) + edgeCost(step, cmin, np.z());
            if (ng < static_cast<double>(g.g[nidx])) {
                g.g[nidx] = static_cast<float>(ng);
                g.parent[nidx] = e.idx;
                open.push({static_cast<float>(ng + heuristic(g.pos(ni, nj, nk))),
                           static_cast<int>(nidx)});
            }
        }
    }

    res.expansions = expansions;

    if (found < 0) {
        if (res.msg.empty()) res.msg = "no feasible path (fully blocked within safety radius)";
        return res;
    }

    // 回溯
    std::vector<int> ids;
    for (int cur = found; cur >= 0; cur = g.parent[cur]) {
        ids.push_back(cur);
        if (cur == s_idx) break;
    }
    if (ids.empty() || ids.back() != s_idx) {
        res.msg = "path backtracking failed (start unreachable)";
        return res;
    }
    std::reverse(ids.begin(), ids.end());

    std::vector<Eigen::Vector3d> raw;
    raw.reserve(ids.size() + 2);
    raw.push_back(s_use);
    for (int id : ids) {
        const int i = id % g.nx;
        const int rem = id / g.nx;
        const int j = rem % g.ny;
        const int k = rem / g.ny;
        const Eigen::Vector3d q = g.pos(i, j, k);
        if ((q - raw.back()).norm() > 0.25 * res_step) raw.push_back(q);
    }
    if ((g_use - raw.back()).norm() > 1e-9) raw.push_back(g_use);

    // ══════════════════════════════════════════════════════════════════
    // ★★ 起点定向：首段沿**入场速度方向**，避免栅格 45° 量化造成的急转
    //    （2026-11-16，用户"接着你的思路改"的正解）
    //
    // 问题：A* 的 path[1] 是相邻栅格中心，与 path[0] 只差 0.1~0.14m。
    //   若入场速度方向是 30°，而 path[1]-path[0] 量化到 45° 或 0°，
    //   这 0.1m 的首段就要承担 15~30° 的转角
    //   → 曲率 = 2·θ/L ≈ 2×0.5/0.1 = 10 m⁻¹
    //   → vlim = sqrt(1.2/10) = 0.35 m/s → 减速 → 参考速度方向仍突变。
    //
    // 修法：在 path[0] 之后插入一个"引出点"：
    //   方向 = 入场速度方向（可选时）
    //   长度 = clamp(0.15, 2×res, 0.35) —— 至少 2 个栅格，够拉开转角
    //   且**必须安全**（clearance 不低于起点），否则不插。
    //
    // 这个点只用于改善轨迹形状，A* 的可行性/最优性完全不变
    // （路径仍经过它，只是首段被拉直）。
    // ══════════════════════════════════════════════════════════════════
    if (has_v_dir && raw.size() >= 2) {
        const Eigen::Vector3d d0 = raw[1] - raw[0];
        const double nd0 = d0.head<2>().norm();
        if (nd0 > 1e-6) {
            const Eigen::Vector2d t0 = d0.head<2>() / nd0;
            const double c = std::max(-1.0, std::min(1.0,
                t0.dot(v_dir.head<2>())));
            const double ang = std::acos(c) * 180.0 / M_PI;
            if (ang > 12.0) {          // 首段方向与入场方向差 >12° 才引出
                // ★ 引出段长度 0.35 → 0.45m（2026-11-17）
                //
                // 实测（/tmp/tv0.cpp）：首段长度决定"转向时能否保住速度"
                //   lead=0.15 + 90° 转角 → 起点速度能被砍到 0.619
                //   lead=0.35 + 90° 转角 → 1.000
                // 而首段太短时，即使起点速度保持，**中段速度也会掉**：
                //   Hermite 在首段内完成 55° 转向 → 速度矢量必须绕过原点
                //   → 实测 0.086m 首段时 t=0.12s 的 |v| 掉到 0.245
                // 0.45m 让 55° 转向分摊到足够长的弧长上，|v| 保持 >0.8。
                const double lead = std::min(0.45, std::max(0.20, 4.5 * params.resolution));
                Eigen::Vector3d lp = raw[0] + Eigen::Vector3d(v_dir.x(), v_dir.y(), 0.0) * lead;
                // 保持与路径同一高度层（2D 下尤其重要）
                lp.z() = raw[0].z();
                const double lc = field.clearance(lp);
                const double sc = field.clearance(raw[0]);
                if (lc >= std::max(0.0, sc - 0.02) &&
                    (lp - raw[1]).norm() > 0.05) {
                    raw.insert(raw.begin() + 1, lp);
                    res.msg = "ok (start direction aligned)";
                }
            }
        }
    }

    res.path = std::move(raw);
    res.success = true;
    if (res.msg.empty()) res.msg = "ok";

    res.length = pathLength(res.path);
    res.clearance_along_path = sampleClearance(res.path, field);
    if (!res.clearance_along_path.empty()) {
        double mn = std::numeric_limits<double>::max();
        double sum = 0.0;
        for (double c : res.clearance_along_path) {
            mn = std::min(mn, c);
            sum += c;
        }
        res.min_clearance = mn;
        res.mean_clearance = sum / static_cast<double>(res.clearance_along_path.size());
    }
    return res;
}

std::vector<Eigen::Vector3d> simplifyPath(const std::vector<Eigen::Vector3d>& path,
                                          double collinear_cos_tol) {
    if (path.size() < 3) return path;

    std::vector<Eigen::Vector3d> out;
    out.reserve(path.size());
    out.push_back(path.front());

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const Eigen::Vector3d d1 = path[i] - path[i - 1];
        const Eigen::Vector3d d2 = path[i + 1] - path[i];
        const double n1 = d1.norm();
        const double n2 = d2.norm();
        if (n1 < 1e-9 || n2 < 1e-9) continue;
        // 只删「几乎笔直」的点；有任何实际拐弯都保留
        if (d1.dot(d2) / (n1 * n2) < collinear_cos_tol) out.push_back(path[i]);
    }

    out.push_back(path.back());
    return out;
}

std::vector<Eigen::Vector3d> smoothPath(const std::vector<Eigen::Vector3d>& path,
                                        const ClearanceField& field,
                                        double min_clearance,
                                        int iterations,
                                        double weight) {
    if (path.size() < 3) return path;

    std::vector<Eigen::Vector3d> cur = path;
    std::vector<Eigen::Vector3d> next = cur;

    for (int it = 0; it < iterations; ++it) {
        for (size_t i = 1; i + 1 < cur.size(); ++i) {
            const Eigen::Vector3d avg = 0.5 * (cur[i - 1] + cur[i + 1]);
            const Eigen::Vector3d cand = (1.0 - weight) * cur[i] + weight * avg;

            // 平滑的判据不能用「不低于安全下界」——那会容忍路径一路劣化到
            // 刚好卡在下界上。这里要求：候选点与两端线段都不得明显低于
            // **本段原有的** clearance 水平。
            const double local_min = std::min({field.clearance(cur[i - 1]),
                                               field.clearance(cur[i]),
                                               field.clearance(cur[i + 1])});
            const double thr = std::max(min_clearance, local_min * 0.95);

            const bool ok = field.clearance(cand) >= thr &&
                            segmentClearanceOk(cur[i - 1], cand, field, thr, 20) &&
                            segmentClearanceOk(cand, cur[i + 1], field, thr, 20);
            next[i] = ok ? cand : cur[i];
        }
        next.front() = cur.front();
        next.back() = cur.back();
        cur.swap(next);
    }
    return cur;
}

std::vector<double> sampleClearance(const std::vector<Eigen::Vector3d>& path,
                                    const ClearanceField& field) {
    std::vector<double> out;
    out.reserve(path.size());
    for (const auto& p : path) out.push_back(field.clearance(p));
    return out;
}

double minClearanceAlongPath(const std::vector<Eigen::Vector3d>& path,
                             const ClearanceField& field,
                             double step) {
    if (path.empty()) return 0.0;
    if (step <= 1e-6) step = 0.05;

    double mn = field.clearance(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
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

double pathLength(const std::vector<Eigen::Vector3d>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) len += (path[i] - path[i - 1]).norm();
    return len;
}

}  // namespace azj_planner
