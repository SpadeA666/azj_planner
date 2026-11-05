# Azj Planner 版本记录

## v1.0 —— 2026-11-04

首个可用版本。安全优先（max-min clearance）的 3D 局部规划器，
建图复用 rog_map，不修改其源码。

### 快照

`~/azj_planner_releases/azj_planner_v1.0_20261104_192543.tar.gz`

解包恢复：
```bash
tar xzf ~/azj_planner_releases/azj_planner_v1.0_*.tar.gz -C ~/catkin_ws/src/
cd ~/catkin_ws && catkin build azj_planner
```

### v1.0 包含的功能

| 模块 | 状态 |
|---|---|
| clearance 加权 A*（居中为核心目标） | ✓ |
| 三次 B 样条曲线优化（C2 连续，ESDF 梯度） | ✓ |
| 时间参数化（TOPP 简化版，曲率/加速度限速） | ✓ |
| `quadrotor_msgs/PositionCommand` 输出（接 as_controller） | ✓ |
| 2D / 3D 模式切换（运行时 `rosparam set`） | ✓ |
| 高度约束（z_band 硬带 + 软保持） | ✓ |
| 调试工具 `azj_debug.py` / `azj_record.py` | ✓ |
| raicom 场景验收测试（含纯几何可达性分析） | ✓ |

### v1.0 已验证的结论（实测数据）

**居中效果**（45° 通道，半净宽 1.0m，起终点各偏 0.7m）：

| 指标 | 折线 | 曲线 |
|---|---|---|
| 转角中位 | 45.0° | 0.05° |
| 中段横向偏移 | 0.467 m | **0.0002 m** |
| 中段两侧距离差 | 0.93 m | **0.0004 m** |
| 最小 clearance | 0.262 m | 0.300 m |
| 长度 | 8.635 m | 8.649 m（+0.16%） |

**raicom 场景验收**（R=0.25）：3D 4 点 + 2D 2 点全 PASS。

### v1.0 的关键修复记录

1. **`p_miss` 必须 < 0.5** —— 默认 0.7 导致 `l_miss=+0.847`（正数），
   raycast 的“空闲”格子反而增加占据概率 → 整幅地图被填实。
2. **UNKNOWN ≠ 障碍** —— 早期直接返回 -1，飞机永远“在障碍里”。
3. **2D 用单层真实 ESDF，不做柱状投影** —— 投影会把框孔压成实心。
4. **`height_ref` 用目标高度**，不用“起终点平均” —— 否则飞机在地面时
   高度带被拉低，路径可以合法地掉到地面附近。
5. **起点逃生区必须带条件** —— 否则 `robot_radius` 形同虚设，
   路径贴着 clearance=0.12 的墙走。
6. **`unknown_clearance_cap` 不封顶** —— UNKNOWN 格子的 ESDF 是可信的，
   封顶会让 A\* 认为墙边比空旷区更宽敞。
7. **yaw 绝不能阶跃** —— 曾导致打点瞬间转 180°、PX4 失控掉落。
8. **曲线安全/曲率双重校验** —— 曲线对安全只有软惩罚，会突破到 1.4mm。

### v1.0 已知限制

- **未实飞验证**（全部证据来自离线测试 + 隔离 master 的消息级验证）
- 动态障碍未处理
- 栅格 A\* 单次 0.1~0.3s（按需重规划后稳态够用）
- 真实点云稀疏时，A\* 可能在缺失处“看到假缝”而穿墙（建图固有局限）
- raicom 横墙以南区域在 R=0.30 下物理不可达（柱缝只有 0.40~0.50m）

## v1.1 —— 2026-11-04（v1.0 + 掉高修复）

### 修复：重规划瞬间掉高

**根因不在本包，在 `as_controller` 的 type_mask。**

`nav_default_mode: 3`（`NAV_LEVEL`）时：

```cpp
const bool use_z = (m == NAV_FULL || m == NAV_Z_ONLY);   // m=3 → false
...
if (use_z) target_position.type_mask |= IGNORE_PZ;
// use_z=false → 不加 IGNORE_PZ，position.z 有效
// 但也没加 IGNORE_VZ，velocity.z 同样有效
```

`type_mask = 3011`，于是 PX4 同时收到：

| 字段 | 值 | 环 |
|---|---|---|
| `position.z` | `fly_height` (0.8) | PX4 位置环 |
| `velocity.z` | 规划器 vz + 误差项 | 速度前馈 |

重规划瞬间 `planner_cmd_.velocity.z` 突变 → 速度前馈把飞机拽一下 → 掉高。

### 本包的缓解（不碰 as_controller）

**① 起点速度用矢量而非标量**

原实现 `v = |v_current| × 新路径切向`，会在重规划时**凭空制造竖直速度**：

```
飞机水平飞 0.8 m/s、新路径起点切线朝下 45°
→ 生成 vz = -0.57 m/s   （飞机根本没在下降！）
```

现在把真实速度**投影到新路径切向**，只保留沿路径的推进分量。

**② 输出 vz 硬限幅**（`max_vel_z_out: 0.30`）

**③ 重规划后速度淡入**（`vel_blend_time: 0.35`）

新轨迹前 0.35s 内，参考速度从"生成时的实际速度"线性过渡到轨迹速度，
消除阶跃。

**④ 轨迹起点对齐当前位置**

新轨迹起点与飞机位置差 < 0.5m 时，插入一段 0.1m/点 的过渡，
避免 t=0 的参考点突变。

### 建议同时在 as_controller 侧修（更彻底）

```cpp
// navigationCore 里 use_z=false 时也应忽略 velocity.z
if (use_z)
    target_position.type_mask |= IGNORE_PZ;
else
    target_position.type_mask |= IGNORE_VZ;   // ← 补这一行
```

或把 `nav_default_mode` 改成 **0（NAV_FULL）**，让规划器管 Z，
避免位置环和速度前馈打架。

### v1.1 验证

- 重规划 vz 最大跳变 **0.0000 m/s**
- raicom 单点验收 PASS（126 点、z 跨度 0、clearance 0.2466）
- 离线 4 项全 PASS

## v1.3 —— 2026-11-05

修掉"密集柱阵场景规划全是折线"的**根源**，不是调参数绕过去。
症状：15m 目标打点后 `curve REJECTED` → 回落折线，而且折线本身
`min_clearance=0.153m < robot_radius=0.30m`（贴着柱子飞）。

### 快照

`~/azj_planner_releases/azj_planner_v1.3_20261105_173359.tar.gz`

### 四个真实缺陷（都已定位到具体代码行）

1. **ESDF 有效盒太小，目标在盒外被假值骗**（最关键）
   - `esdf/local_update_box: [8,8,4]` 只有半径 4m，后来调到 12m 仍不够 ——
     用户会打到 15m 外。
   - `updateESDF3D` 每帧只重算以 odom 为中心的盒子，盒外
     `distance_buffer` **从未被写过**，而 `vector<double>::resize` 填的是 **0**。
   - 于是盒外 `evaluateEDT` 插值读到一片 0 → `clearance=0`。
     日志表现：`goal clearance=0m < robot_radius`，看着像"目标贴障碍"，
     实际目标离最近柱子 0.96m。A* 因此报 `fully blocked` 或绕路。
   - 修：`clearanceAt` / `gradientAt` 用 `getUpdatedBbox()` 判定有效范围，
     **盒外一律返回 `kUnknownClearance`（-1，不可通行）**。
     不再用"乐观 cap 0.32"—— 那会把曲线骗进未建图区（实测 `worst vertex
     at 12.11 ... c=0.32` 正是盒边界外的假值）。
   - `local_update_box` 改 `[32, 32, 2]`（半径 16m）。z 只要 2m
     （实际 map z bounds ≈ [-0.2,1.6]），因为 z 是全量重算的体积因子。

2. **安全硬投影的成果被整个丢弃**（`curve_optimizer.cpp`）
   - `projectSafe` 返回的是**最后一轮 rep 的**越界点数。第一轮就修好时
     返回 0 → `if (viol > 0)` 为假 → 投影结果被扔掉。
     **投影越成功越容易被丢弃。**
   - 修：改用"控制点是否真的被改动"（`(Qproj-Q).squaredNorm() > 1e-18`）
     判断，并且投影后无条件接受、同步刷新 `best_cost` 基准。
     安全不是可交易的代价项。

3. **采样分辨率不随路径长度自适应**
   - 固定 120 采样在 15m 上是 12.5cm 一格，看不见"被平滑切进去的柱子边"。
   - 修：`S = clamp(ceil(total_len / 0.10), samples, 1500)`，
     投影采样同样按 0.10m 自适应。

4. **折线兜底本身不安全也不说**
   - 已加"安全失败 → 递进加重 `w_obs`（×10、×50）+ 加密投影"重试两次，
     仍不过才回落折线。

### 性能（笔记本，24 核，实测）

| 目标 | 点数 | min_clearance | 越界(<0.30m) | A* | 曲线优化 |
|---|---|---|---|---|---|
| (15, 0) | 313 | +0.649 | 0/313 | 360ms | 1699ms |
| (8.55, 5.64) | 222 | +0.731 | 0/222 | 250ms | 1407ms |
| (6.95, -2.03) | 154 | +0.805 | 0/154 | 122ms | 1501ms |

单次规划合计 **1.5~1.7s**（v1.2 时 15m 目标是 7.8s）。

**副作用**：为压耗时，`curve_iterations` 150 → 60、`curve_projection_iters`
3 → 2、线搜索回溯 12 → 4。清晰度指标与 150 迭代一致（复现过两轮）。

### 试过但无效，别再试

- **迭代次数自适应**（`max(iters, 6*ncp)`）：15m → 336 次迭代，
  耗时炸到 21.5s 且仍然 `REJECTED`。采样分辨率才是关键，不是迭代数。
- **OpenMP 并行采样循环**：N 只有 150 左右，fork/join 开销超过收益，
  **反而变慢** 1.7s → 2.6s。已在代码里注释留档。

### 已知不足

- 单次规划 1.5~1.7s，不是 10Hz 实时。适合起飞前打点 / 低频重规划。
  要点：旧轨迹在重规划期间继续飞，所以只要旧轨迹仍安全就能接受。
  要真正实时需要缓存 ESDF 查询或引入增量优化，未做。
- 目标超出 ESDF 有效盒（距 odom > 16m）会报不可达 —— **这是刻意的**，
  没建图的区域不能假装安全。

## v1.3.1 —— 2026-11-05

### 修的致命 bug：规划器活锁（"卡住了"）

现象：飞机飞到贴着柱子的位置后，`/planning/pos_cmd` 变成每 3.9s 才发一条，
连发 40+ 次，飞机停住不动。主线程 100% CPU。

日志呈完美循环：
```
fallback polyline: min_clr=0.1078 (start_clr=0.1078)
reference blocked ahead (clearance=0.136), replanning
curve REJECTED: min_clearance=0.1078 < robot_radius*0.9=0.27
```

**根因**：飞机贴柱悬停（clearance 0.108m < robot_radius 0.30m），而：

1. `minClearanceAlongPathExcludingEnd` **只豁免终点，没豁免起点** →
   从贴柱位置出发的任何路径，起点 clearance 必然 0.108，永远被拒。
2. `needReplan` 的安全监控**从 `t`（当前位置）开始查** →
   当前位置就 0.108 < 0.30，第一个采样点命中"前方受阻"，立刻重规划。

两者互为因果。A* 那层本来有起点逃生区（`path_search.cpp:324`），
但下游两处都没配套，导致永远走不到。

**修法（三处）**：
1. 起点豁免：仅在**起点本身已不安全**时跳过起点前 `robot_radius+0.3` 一段。
   起点安全时仍严格校验全路径。
2. 安全监控语义修正：起点不安全时，若前方**不比起点更差**就不算"新障碍侵入"。
3. 新增 `min_replan_period: 0.30` 重规划退避 —— 机制性兜底，
   即便将来又出现类似逻辑缺陷，也不会打满 CPU 让 pos_cmd 断流。

**验证**（用现场真实数值 clearance=0.108）：
```
旧判定（不豁免起点）  min_clr = 0.108  → REJECTED   ← 死循环成因
新判定（豁免起点）    min_clr = 0.463  → PASS       ← 打破

反向：起点贴柱 + 中段撞另一根柱 → min_clr=0.211 → REJECTED  ← 安全性未削弱
```
实测 75s 飞行：`curve REJECTED` **0 次**（改前连续出现），断流 >2s 仅 1 次。

### 控制器：加速度前馈（nav_ff_acc）

见 `as_controller` 的 `nav_ffacc_20261105` 改动。规划器的
`PositionCommand.acceleration` 一直被填充但控制器从未使用，补上
`v += k_a * a_ref` 补偿 0.18s 相位滞后。

未饱和段的实测效果（75s，柱阵）：

| 指标 | 无前馈 | k_a=0.30 | 变化 |
|---|---|---|---|
| 切向滞后 中位 | +0.203 | +0.155 | **-24%** |
| XY 误差均值 | 0.225 | 0.198 | -12% |
| 速度误差 | 0.233 | 0.181 | -22% |
| 法向偏离 中位 | 0.052 | 0.064 | +23%（切角略增，绝对差 1.2cm） |

### 教训：限幅不能当"安全阀"收窄

第一次调参我把 `nav_max_vel` 从 1.4 降到 1.25（想减少饱和），**这是错的**：
加加速度前馈后指令需求 p90=1.53/p95=1.64，1.25 让饱和率从 15% 飙到 29%，
把前馈的收益全吃掉（饱和帧滞后 0.291 vs 未饱和 0.155）。

已定为 **1.60**（饱和约 6%）。实测 `v_act` max 仅 1.17 —— 实际飞行速度由
`v_ref`(1.0) 决定，限幅只限制**追赶瞬态**。
