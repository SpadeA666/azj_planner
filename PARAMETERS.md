# Azj Planner 参数手册

> 2026-11-26 逐项核对 `config/azj_planner.yaml`（1051 行）与源码后编写。
> 表中「默认」列取自当前 yaml 实际生效值，不是历史值。
> 若发现文档与代码不一致，**以代码为准**，并回头改这份文档。

配置分两段：`rog_map:` 是建图（第三方库，我们只读不写源码），
`azj:` 是规划器本身（134 个参数，本文重点）。

---

## 0. 先回答最常问的：怎么调速度

「调速度」实际上有 **五个不同的旋钮**，作用层次完全不同。改错层会有反效果。

### 0.1 想要飞得更快 / 更慢 —— 改 `max_vel`

```yaml
azj:
  max_vel: 1.0        # 最大水平速度 [m/s]
```

**它的作用方式取决于后端，这一点必须知道：**

| 后端 | `traj_backend` | max_vel 的性质 |
|---|---|---|
| MINCO（默认） | `1` | **软约束**：罚函数 `w_vel·hinge(|v|-vmax)²`，可能被突破 |
| B 样条 + TOPP | `0` | **硬约束**：TOPP 时间参数化直接按 `vlim = max_vel` 规划 |

MINCO 是软约束，所以节点加了三道保险：

1. 校验阈值 `max_vel × 1.25` —— 优化结果超过它就**拒绝重试**（最多三轮，加重 w_obs）
2. `rolling_track_acc` 限制起点速度
3. `rolling_lag_vel_gain` 落后时整体降速

> 实测参考（v3.1.0）：`max_vel=1.0` 时输出 `vmax` 落在 0.92~1.42。
> 想严格不超，把 `max_vel` 设成你能接受上限的 0.8 倍。

**改完必须重启节点**（见第 12 节）。

### 0.2 想要加速更猛 / 更柔和 —— 改 `max_acc`

```yaml
  max_acc: 1.5        # 最大水平加速度 [m/s²]
  max_acc_z: 1.0      # 最大垂直加速度
  max_lat_acc: 1.2    # 最大向心加速度，决定转弯限速
```

- `max_acc` —— 直线段加速度。MINCO 后端下同样是软约束（`w_acc` 罚）
- `max_lat_acc` —— 转弯限速：`v = sqrt(max_lat_acc / kmax)`。调大转弯更快但更易过冲
- `max_acc_z` —— 垂直方向

**但真正卡住"实际能飞多快"的往往不是这些值**，而是下面这个：

### 0.3 ★ 卡住实际速度的通常是 `rolling_track_acc`

```yaml
  rolling_track_acc: 1.0    # rolling 段的可跟踪加速度 [m/s²]
```

这是**仿真 PX4 实测出来的真实能力**（bag 数据：0.82s 从静止到 0.92 m/s → ≈1.12 m/s²）。
`max_acc=1.5` 是"机体理论上限"，而仿真里 PX4 跟不到 1.5。

如果你把 `max_vel` 调到 2.0 但飞机还是在 1.0 附近徘徊，
**先看这个参数，不是 max_vel 没生效**。

### 0.4 想让它别"跑太快追不上" —— 改这三个

之前你报过"路径比飞机飞得快"，那一族的参数：

```yaml
  rolling_v_lead: 0.05        # 参考允许领先飞机多少 [m/s]
  rolling_lag_vel_gain: 0.50  # 落后时整体降速：v_allow = max_vel - (lag-free)*gain
  rolling_lag_free: 0.35      # 这个 lag 以内不降速（正常跟踪误差）
  rolling_v_floor: 0.25       # 降速的下限，别降到停
```

实测（v3.1.0 横扫）：`lag_vel_gain` 0.0→0.5 的对比是
"冲过目标 1.56m" → "正常停在 14.05m"，同时消除历史遗留的过冲问题。

### 0.5 输出侧的两道硬钳制（不要随便放开）

这两项是**防掉高的最后防线**，跟上面几项独立：

```yaml
  acc_feedforward_limit: 1.5   # 加速度前馈硬钳 [m/s²]
  ref_vel_rate_limit: 2.25     # 参考速度变化率硬钳 [m/s²]（= max_acc × 1.5）
  max_vel_z_out: 0.30          # 输出 vz 硬限幅
```

`ref_vel_rate_limit` 是最关键的一个：实测 2026-11-16 那次的掉高根因就是
重规划瞬间参考速度方向在 0.05s 内转了 41°（变化率 **14 m/s²**，是 max_acc 的 9 倍）
→ 控制器只能饱和 → 姿态被拽偏 → z 从 0.774 掉到 0.001。

**设 0 = 关闭该保护。不要关。**

---

## 1. 速度与加速度完整链路

从"你想飞多快"到"控制器收到什么"，中间经过这些环节：

```
max_vel / max_acc          ← 你调的（轨迹层目标）
      ↓
traj_backend=1: MINCO 软约束（w_vel / w_acc 罚）
traj_backend=0: TOPP 硬约束
      ↓
校验：vmax > max_vel*1.25 → 拒绝重试
      ↓
rolling_track_acc          ← rolling 段起点速度上限
rolling_v_lead             ← 允许领先飞机的量
rolling_lag_vel_gain       ← 落后时整体降速
      ↓
acc_feedforward_limit      ← 加速度前馈硬钳
ref_vel_rate_limit         ← 参考速度变化率硬钳
max_vel_z_out              ← vz 硬限幅
      ↓
/planning/pos_cmd → 控制器
```

| 参数 | 默认 | 单位 | 作用 |
|---|---|---|---|
| `max_vel` | 1.0 | m/s | 最大水平速度 |
| `max_vel_z` | 0.6 | m/s | 最大垂直速度 |
| `max_acc` | 1.5 | m/s² | 最大水平加速度 |
| `max_acc_z` | 1.0 | m/s² | 最大垂直加速度 |
| `max_lat_acc` | 1.2 | m/s² | 向心加速度，转弯限速 |
| `rolling_track_acc` | 1.0 | m/s² | rolling 段可跟踪加速度（**实际瓶颈**） |
| `rolling_v_lead` | 0.05 | m/s | 参考允许领先量 |
| `rolling_v_floor` | 0.25 | m/s | 降速下限 |
| `rolling_lag_free` | 0.35 | m | 该 lag 内不降速 |
| `rolling_lag_gain` | 1.0 | — | lag 降速的起点增益 |
| `rolling_lag_vel_gain` | 0.50 | 1/s | 落后时速度上限衰减率 |
| `acc_feedforward_limit` | 1.5 | m/s² | 加速度前馈硬钳 |
| `ref_vel_rate_limit` | 2.25 | m/s² | 参考速度变化率硬钳 |
| `max_vel_z_out` | 0.30 | m/s | 输出 vz 硬限幅 |
| `max_curvature` | 20.0 | 1/m | 曲率上限（超了降速跟踪） |
| `curvature_tolerance` | 3.0 | 倍 | 曲率容差倍数，超过则拒绝曲线 |

---

## 2. 安全（最不能随便动的）

| 参数 | 默认 | 含义 |
|---|---|---|
| `robot_radius` | 0.30 | **硬安全半径**。clearance 低于它直接拒绝通行，不是软惩罚 |
| `escape_radius` | 0.60 | 起点逃生区。飞机已贴障碍时允许先飞出来，否则永远无解 |
| `safety_near_frac` | 0.95 | 近距安全触发阈值（× robot_radius） |
| `safety_far_frac` | 0.90 | 远距安全触发阈值 |
| `safety_near_span` | 0.5 | 近/远分界跨度 |
| `safety_hysteresis` | 1.05 | 滞回系数：触发后 clearance 需恢复到此倍才再次允许 |
| `safety_lookahead` | 1.50 | 前方安全监控前瞻 [s] |

### ⚠ robot_radius 的现实问题

它是你**手填的一个数，和真实机体外廓没有任何绑定**。

按 9 寸桨算：桨尖半径 = 轴距/2 + 桨半径 = 190 + 114.3 = **304mm**，
比 `robot_radius=0.30`（300mm）还大 4mm。

地面站 3D 视图里的「机体」分组能实时算出你当前配置的真实桨尖半径，
用它对照 `robot_radius` 检查是否匹配。

### escape_radius 解决什么

飞机停得离墙 0.05m、而安全半径 0.3m 时，严格按安全半径拒绝所有邻居
→ **永远无法规划**。逃生区内只要求 clearance > 0，让它先飞出来。

---

## 3. 居中（核心理念，azj 存在的理由）

| 参数 | 默认 | 含义 |
|---|---|---|
| `clearance_weight` | 0.15 | w。**0 = 纯最短（退化成 EGO 行为）** |
| `clearance_ref` | 1.0 | 归一化基准 [m] |
| `clearance_cap` | 2.0 | 超过此 clearance 不再奖励，防绕远 |
| `alpha` | 2.0 | 居中锐度 |

代价 = `长度 × (1 + w × (ref / clamp(c, ·, cap))^alpha)`

**w 的实测调参表**（离线 `offline_clearance_test`）：

| 场景 | w | 横向偏移 | 长度增量 |
|---|---|---|---|
| 长通道 10m | 0（纯最短） | 0.236 m | — |
| 长通道 10m | 0.05 | **0.008 m** | +0.75% |
| 长通道 10m | 0.10 | 0.005 m | +1.3% |
| 长通道 10m | 0.15（当前） | 更居中 | 略增 |

调大 → 更居中但路径更长；调小 → 更短但贴边。

### MINCO 后端的居中权重是独立的

`traj_backend=1`（默认）时，实际居中由 `minco_w_mid` 主导，**不是** `clearance_weight`：

| 参数 | 默认 | 含义 |
|---|---|---|
| `minco_w_mid` | 30.0 | MINCO 居中权重 = jerk 能量的 30 倍 |
| `minco_w_obs` | 50.0 | 安全硬下界的软权重 |
| `minco_w_smooth` | 1.0 | jerk 泛函权重 |
| `minco_iterations` | 60 | 迭代次数 |
| `minco_dump_dir` | `""` | 取证 dump 目录。空 = 关闭（生产用）。**支持运行时 set** |

`minco_w_mid=30` 是横扫实测最优（直通道半宽 1.5m、初值偏 0.8m，
60 迭代做到横向不平衡 0.05m、min_clr 1.473/1.5）。

**注意 `w_vel` / `w_acc` 是代码里的固定值 2.0，yaml 没暴露。**
想加强速度约束目前只能改 `src/minco_optimizer` 或让我加参数。

---

## 4. 轨迹后端

| 参数 | 默认 | 含义 |
|---|---|---|
| `traj_backend` | 1 | **1 = MINCO（默认）**，0 = B 样条 + TOPP |

出问题改回 0 即回到旧行为，无需改代码。

MINCO 的核心收益：`head.a`（起点加速度）**从旧轨迹继承**，而不是从 odom 差分。
这让滚动重建时加速度连续，不会每条轨迹都从 a=0 重新加速。

已知局限：
- `max_vel` / `max_acc` 是软约束
- 极短段（`seg < 0.35m`）会回退到 B 样条分支（数值病态）

---

## 5. 滚动重规划（rolling）

| 参数 | 默认 | 含义 |
|---|---|---|
| `rolling_enable` | true | 总开关。**关掉 = 回到 v1.9 全量流程**（最快的回退手段） |
| `rolling_period` | 0.10 | 滚动周期 [s]，0.1 = 10Hz |
| `rolling_forward_dt` | **0.0** | ★ 必须为 0（见下） |
| `rolling_lookahead` | 4.0 | 末点前探距离 [m]，= 滚动窗口长度 |
| `rolling_lookahead_min` | 1.5 | 剩余小于它就直奔 goal |
| `rolling_global_ttl` | 4.0 | 全局路径缓存有效期 [s] |
| `rolling_goal_refresh_dist` | 0.50 | 目标移动超过它才重建全局路径 |
| `rolling_astar_margin` | 1.30 | A* 的 clearance 余量倍数 |
| `rolling_receding_dis` | 2.0 | 从旧轨迹继承的引导段长度 |
| `rolling_snap_end` | true | 末点吸附到自由格 |

### ★ rolling_forward_dt 必须为 0（2026-11-17 定论）

这个值是"每次重建时参考点前跳的时长"。设 0.1 时实测每 0.21s 参考前跳 0.08m
（瞬时 4 m/s），等效额外推进 0.38 m/s → **参考永远比飞机快 → lag 稳在 0.3~0.6 不收敛**。

改 0.0 后：跳变 82 次 → **1 次**，lag 0.30~0.60 → **0.15**。

SUPER 能用 0.1 是因为它同时**延后新轨迹 start_WT** 保持时间轴连续，我们没有那个机制。

### 末端居中

| 参数 | 默认 | 含义 |
|---|---|---|
| `rolling_center_iters` | 8 | 末端居中梯度上升迭代数（0=关闭） |
| `rolling_center_step` | 0.08 | 每次步长 [m] |
| `rolling_center_max_shift` | 0.60 | 末端最大横移 [m] |
| `rolling_center_lateral_only` | true | 只在垂直前进方向回中 |

---

## 6. 机头朝向（yaw）

| 参数 | 默认 | 含义 |
|---|---|---|
| `yaw_mode` | 1 | **0** = 跟路径切线 + 到点朝消息 yaw；**1** = 全程保持机头 |
| `yaw_rate_max` | 1.0 | yaw 参考最大变化率 [rad/s] |
| `yaw_rate_flight` | 0.25 | 飞行中实际限速 [rad/s]（≈14°/s） |
| `yaw_deadband` | 0.05 | 角度死区 [rad]（≈2.9°），抑制抖动 |
| `yaw_alt_guard_drop` | 0.35 | 掉高超此值就停转 yaw [m]，0=关闭 |
| `yaw_vel_thresh` | 0.15 | 退回"朝速度方向"时的速度门限 |
| `yaw_profile_dt` | 2.1 | 预规划 yaw 曲线 waypoint 间隔 [s] |
| `yaw_follow_velocity` | false | 仅兼容老配置：设 true 会把 `yaw_mode=1` 覆盖成 0 |
| `px4_yaw_auto_max_deg` | 180 | PX4 的 `MPC_YAWRAUTO_MAX`，用于钳制 |
| `goal_yaw` | NaN | 固定目标 yaw [rad]。**设了它 yaw_mode 完全失效** |

### ★ 为什么 yaw 不能跟"预规划曲线的时间索引"

`yaw_prof->at(traj_t)` 在 rolling 下**结构性失效**：
每 0.1s 重建轨迹、`start = now` → `traj_t` 永远停在 0~0.14s
→ 永远读到第一个 waypoint（≈0°）→ yaw 完全不跟路径。

所以 `yaw_mode=0` 用的是**几何切线**（当前点 → 前方 0.5m 的 atan2），与时间索引解耦。

### ★ 为什么必须限速 yaw

PX4 四旋翼每个电机叠加 throttle/roll/pitch/yaw 四个分量。
飞行中 cmd.yaw 持续变化 → 实测 yaw **稳态落后指令 26~30°**
→ 姿态环持续输出偏航力矩 → 电机差速占用行程 → 推力下降 → **掉高**。

关键不是"转得快"，而是"跟踪误差持续存在"。

`yaw_rate_max=1.5` 会掉机（2026-11-05 实测事故：转 1 秒后 z 从 0.83 掉到地面）。

---

## 7. 高度与 2D/3D

| 参数 | 默认 | 含义 |
|---|---|---|
| `plan_2d` | true | **true = 只在 XY 规划，z 锁在目标高度** |
| `plan_2d_slab_half` | 0.0 | 薄层厚度。0 = 单层 ESDF，不投影 |
| `plan_2d_z_samples` | 9 | slab 模式下的 z 采样点 |
| `enforce_height_limits` | true | 层高硬约束总开关 |
| `floor_z` | 0.00 | 规划可通行高度下限 |
| `ceil_z` | 1.00 | 上限 |
| `z_band` | 0.20 | 高度硬约束带 |
| `w_height` | 2.00 | 高度保持软权重 |
| `height_ref_offset` | 0.0 | 目标高度的整体偏移 [m]（标定用） |
| `height_scale` | 1.0 | 目标高度的缩放系数（标定用） |
| `height_band` | 0.15 | 软带宽度 |
| `w_height_band` | 20.0 | 超出软带后的强罚 |
| `ground_lift_max` | 0.40 | 贴地起点最大抬升 [m] |
| `min_goal_z` | 0.30 | 目标最低高度 |

### ★ 2D 模式的两个真实约束

1. **A\* 所有邻居都被强制设到规划平面**（`goal.z()`），起点也不例外
2. `ceil_z` 是硬上限：打点高度被 clamp 到 1.00 时，若那一层 ESDF 是 0（贴天花板），
   目标会被判在障碍里 → 拒绝规划

地面测试时若发现"打点没反应"，先看 `/tmp/planner_azj.log` 里的
`goal clearance=... unknown=...` —— `unknown=1` 表示那层没被雷达扫到。

---

## 8. 重规划触发

| 参数 | 默认 | 含义 |
|---|---|---|
| `replan_rate` | 15.0 | 重规划检查频率 [Hz]（只在需要时真重建） |
| `min_replan_period` | 0.30 | 最小间隔，防活锁兜底 |
| `blocked_replan_period` | 0.10 | 安全触发时的退避（比上面短） |
| `replan_lead` | 0.10 | 轨迹剩余时间小于它就重规划 [s] |
| `replan_deviation` | 0.50 | 偏离阈值 [m]。**现在不触发重规划**（学 SUPER 的 NO_NEED） |
| `blocked_fast_path` | true | 安全触发走快通道（跳过曲线优化） |
| `fast_smooth_delay` | 0.6 | 快通道后多久再做平滑 [s] |

> 实测：`min_replan_period=0.30` 曾把 rolling 压到 2.3~2.8Hz。
> rolling 模式会用 `rolling_period`（0.1s）做退避，不再受 0.30 限制。

---

## 9. 起点 / anchor（重规划连续性的核心）

| 参数 | 默认 | 含义 |
|---|---|---|
| `anchor_from_trajectory` | **false** | anchor 取飞机自身状态（false）还是旧轨迹前视点（true） |
| `anchor_max_dev` | 0.30 | 开启上面那项时的偏离上限 |
| `plan_anchor_catchup_max` | 0.5 | anchor 过期补偿上限 [m] |
| `realign_max_gap` | 0.25 | 复用旧路径的最大偏离 [m] |
| `ref_start_max_dev` | 1.5 | 新起点允许偏离已提交参考的最大距离 |
| `fallback_center_enable` | true | 折线兜底也做居中修正 |
| `fallback_center_skip_clear` | 1.2 | 已经这么宽就不动 [m] |

### ★ anchor_from_trajectory 必须是 false

`true` 是 SUPER 式（旧轨迹未来时刻的 pos+vel）。
**只在"规划耗时 <100ms"时才成立** —— SUPER 靠 0.3ms 的规划速度让 0.1s 前视 ≈ 飞机当前位置。

我们规划要 0.7~1.8s，前视就变成"前方 0.7~1.0m"，实测断层 0.91~1.51m：

```
t=3.40  tid 1→2  cmd(-2.22,-1.21) 飞机(-2.48,-0.33) 断层 0.914m
t=7.0   断层 1.510m 且 |cmdV|=0
```

→ 飞机必须加速去追断层 → 速度不平滑、机身抖动。这就是你早前报的"重规划断层"。

---

## 10. 失败兜底

| 参数 | 默认 | 含义 |
|---|---|---|
| `brake_angle_deg` | 55.0 | 目标方向与当前速度夹角超此值先刹车 [度]。180=关闭 |
| `hermite_preferred` | true | 优先用三次 Hermite 做 C1 过渡（关掉 = 直线连接，会断层） |
| `brake_angle_min_speed` | 0.3 | 低于此速度不做急转判断 [m/s] |
| `backup_enable` | true | 规划失败时生成可控刹车轨迹 |
| `backup_min_speed` | 0.15 | 低于此速度不生成刹车段 [m/s] |
| `hot_init_reuse` | 1.5 | 复用旧轨迹热启动的时长 [s]，0=关闭 |
| `hot_init_min_len` | 0.5 | 引导段最小长度 [m] |
| `hot_init_max_len` | 3.0 | 引导段最大长度 [m] |
| `hot_init_clear_frac` | 0.95 | 引导段要求的 clearance（× robot_radius） |

### 刹车段解决什么

飞机全速冲向目标、目标却在身后时，直接转向 → 加速度阶跃 → 掉高。
先拼一段**沿当前速度的刹车段**（减速到静止）再接路径 —— 这是
SUPER 的 "guide 段 + backup" 在 azj 上的等价实现。

---

## 11. 搜索 / 建图 / 输出
### A* 搜索

| 参数 | 默认 | 含义 |
|---|---|---|
| `resolution` | 0.10 | A* 栅格分辨率 [m] |
| `box_margin_x` | 2.5 | 搜索框 x 方向外扩 [m] |
| `box_margin_y` | 2.5 | 搜索框 y 方向外扩 [m] |
| `box_margin_z` | 0.8 | z 方向外扩 |
| `goal_tolerance` | 0.30 | 目标吸附容差 [m] |
| `max_expansions` | 400000 | 搜索节点上限（防病态场景炸内存） |

### 建图（rog_map 段，我们只读）

| 参数 | 默认 | 含义 |
|---|---|---|
| `cloud_topic` | `/super_cloud` | 点云输入 |
| `odom_topic` | `/gazebo_truth/odom` | 位姿输入。**必须与点云变换同源** |
| `local_update_box` | `[32, 32, 2]` | ESDF 更新盒 [m]。太小会出现"目标在盒外 → clearance=0" |
| `resolution` | 0.10 | 栅格分辨率 |
| `unknown_as_free` | false | 未知区当自由空间（**危险**，别开） |
| `free_clearance_if_unknown` | 0.5 | 上面开启时的假设 clearance |
| `unknown_clearance_cap` | 0.32 | 未知区 clearance 上限 |

> `local_update_box` 曾是小 `[8,8,4]`（半径 4m），而用户常打 15m 外 →
> 盒外 `distance_buffer` 是 0 → `clearance=0` → "fully blocked"。
> 现在 `[32,32,2]`，代价是全量重算变慢。

### 输出

| 参数 | 默认 | 含义 |
|---|---|---|
| `cmd_topic` | `/planning/pos_cmd` | **与 SUPER/EGO 共用，一次只能跑一个** |
| `publish_rate` | 50.0 | 参考发布频率 [Hz]。**必须 > 2**（控制器超时 0.5s） |
| `path_topic` | `/azj/path` | 规划路径（nav_msgs/Path，latched） |
| `marker_topic` | `/azj/path_marker` | 可视化 marker |
| `goal_topic` | `/move_base_simple/goal` | 目标输入（与 SUPER 相同） |
| `frame_id` | `map` | |
| `vel_topic` | `""` | 空 = 从建图 odom 的 twist 取速度（推荐，位姿/速度同源） |
| `odom_twist_body` | false | 建图 odom 的 twist 是否在**机体系**（实机 faster_lio 是，仿真 gazebo 不是） |
| `arrive_tol` | 0.15 | 到达判定距离 [m] |
| `arrive_vel_tol` | 0.20 | 到达判定速度上限 [m/s] |
| `vel_blend_time` | 0.35 | 速度淡入时间 [s]。**rolling 下自动禁用** |
| `publish_completed_flag` | false | 到点是否发 COMPLETED（发它会触发控制器切模式 → yaw 掉高） |
| `snap_goal_out_of_inflation` | true | 目标落在膨胀层内时自动吸出 |
| `goal_snap_search_radius` | 4.0 | 吸附搜索半径 [m] |
| `min_goal_clearance` | 0.40 | 吸附点要求的最小 clearance [m] |

### 曲线优化（`traj_backend=0` 或 MINCO 回退时用）

| 参数 | 默认 | 含义 |
|---|---|---|
| `curve_enable` | true | 曲线优化开关 |
| `curve_w_mid` | 1.0 | 居中权重 |
| `curve_w_obs` | 50.0 | 避障权重 |
| `curve_w_smooth` | 1.0 | 平滑权重 |
| `curve_w_anchor` | 0.5 | 保形权重（别偏离 A* 太多） |
| `curve_w_len` | 0.05 | 长度权重 |
| `curve_samples` | 140 | 采样点数 |
| `curve_iterations` | 120 | 迭代次数（默认路径，长） |
| `curve_iterations_quick` | 80 | 快通道迭代次数 |
| `curve_stall_iters` | 8 | 连续无改善多少次提前退出 |
| `curve_control_point_spacing` | 0.30 | 控制点间距 [m] |
| `curve_enable_projection` | true | 安全投影开关 |
| `curve_projection_iters` | 2 | 投影迭代 |
| `curve_projection_samples` | 240 | 投影采样 |
| `curve_enable_local_centering` | true | 局部居中 |
| `curve_local_scan_radius` | 2.0 | 局部扫描半径 [m] |
| `curve_sample_step` | 0.05 | 采样步长 [m] |

---

## 11.5 ★ odom 速度的坐标系陷阱（实机移植必读）

`vel_topic` 留空时，速度从建图 odom 的 `twist` 取。
但**不同来源的 twist 坐标系不一样**，搞错会让飞机偏航后速度方向全错。

### 三类 odom 源的实际语义（均已核代码）

| 来源 | twist 坐标系 | 依据 |
|---|---|---|
| 仿真 `/gazebo_truth/odom` | **世界系** ENU | `gz_truth_bridge.py:92` 直接透传 gazebo `msg.twist[i]` |
| 实机 `/Odometry_highrate`（faster_lio） | **机体系** | `laser_mapping.cc:809`：<br>`vel_body = s.rot.conjugate() * s.vel`<br>（注释原话“转到 body 系，更符合 child_frame_id”）|
| `/mavros/local_position/velocity_local` | **世界系** ENU | mavros 约定 |

### 为什么 TF 救不了这个问题

板卡上用 `static_transform_publisher 0 0 0 0 0 0 map odom`（零变换）
解决的是**TF 树连通性**——让不同坐标系名字能连上，rviz 能对齐。

但 twist 坐标系问题是**消息字段的语义标注错误**：

```
飞机机头朝北（yaw=90°），实际向前飞 0.5 m/s
faster_lio 发:  twist.linear = (0.5, 0, 0)   ← 机体系 x = 前向
azj 当成世界系读 → 认为飞机在向东飞 0.5 m/s   ← 错
正确世界系应该是: (0, 0.5, 0)                ← 向北
```

整个过程**没有任何 TF 查询**（直接读消息字段），所以 TF 无从介入。
这不是坐标系变换问题，是协议约定问题。

### 为什么只有 azj 需要处理

| 节点 | 速度来源 | 受影响？ |
|---|---|---|
| **SUPER** | 通过 rog_map 拿 odom | **否**。`rog_map_ros1.hpp:74` 的 odomCallback **只读 pose**，不碰 twist；<br>`ROGMap::updateRobotState`（`rog_map.cpp:268`）也只填 `p/q/yaw`，`v` 永远是 0 |
| **as_controller** | `/mavros/local_position/velocity_local` | **否**。这是 ENU 世界系（`api_3d.cpp:2465`）|
| **azj（修前）** | 自订 `/Odometry_highrate` 的 twist | **是**。直接当世界系用 → 偏航后速度方向错 |

> 换句话说：**SUPER 从来没这个问题，是因为它从来不用速度**（起点速度恒为 0）。
> 这也侧面解释了为什么 SUPER 的轨迹总是“从静止起步”——它本来就没做速度继承。

### 三种解决方式的对比

| 方案 | 同源？ | 侵入性 | 评价 |
|---|---|---|---|
| **`odom_twist_body: true`**（当前） | ✓ 位姿与速度同一帧消息 | 一个小参数 | **推荐**。只多一次旋转，数学无损 |
| `vel_topic: /mavros/local_position/velocity_local` | ✗ 位姿 faster_lio、速度 PX4 EKF | 不改代码 | 可行但违背“严格同源”设计；两源可能有 2~9cm 错动与延迟差 |
| 清空 twist 当 0 | — | — | 不可取。重规划时每次都从静止起步（azj 早期就是这个毛病）|

### 验证方法

启动日志会明确标注实际走的是哪条路：

```
[azj] velocity from odom twist (/Odometry_highrate) [body->world converted]
                                                  ↑ true 时有这个标记
[azj] velocity from odom twist (/gazebo_truth/odom)
                                                  ↑ 仿真（世界系，不需转换）
```

旋转公式已做数值验证（6 个边界用例全通过，含 yaw=0/±90/180、侧向飞、垂直飞、
零四元数保护）：

```cpp
Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
if (quat.norm() > 1e-6) v = quat.normalized() * v;   // body → world
```

---

## 12. 哪些参数能运行时改

**只有 3 个**支持 `rosparam set` 后立即生效，其余**必须重启节点**：

```bash
rosparam set /azj_planner/plan_2d true         # 2D / 3D 切换
rosparam set /azj_planner/yaw_mode 0           # 机头模式
rosparam set /azj_planner/minco_dump_dir /tmp/x  # MINCO 取证
```

**改 `max_vel` / `max_acc` / `robot_radius` 等，必须重启节点。**

### 重启节点的正确方式

```bash
bash ~/catkin_ws/scripts/switch_planner.sh azj
```

不要直接 `pkill azj_planner_node` —— 切换脚本会顺带处理清理，
直接杀会留下孤儿注册。

### ⚠ 改 yaml 后注意生成副本

`sh/raicom_azj.sh` 会把 yaml **复制一份**到 `/tmp/azj_planner_sim.yaml` 再改几项
（odom 话题前缀、`yaw_mode` 等），节点读的是**副本**。
所以：

- 用 `switch_planner.sh` 启动 → 读**源 yaml**，改完重启即生效
- 用 `raicom_azj.sh` 启动 → 读**副本**，改完要重启整个脚本

---

## 13. 按症状查参数

| 症状 | 先查 | 再查 |
|---|---|---|
| 飞太慢 | `max_vel` | `rolling_track_acc`、`rolling_lag_vel_gain` |
| 追不上路径 | `rolling_v_lead`、`rolling_lag_vel_gain` | `rolling_forward_dt` 必须为 0 |
| 掉高 | `ref_vel_rate_limit`、`yaw_rate_flight` | `MPC_THR_HOVER`（PX4 侧）、`max_vel_z_out` |
| 路径贴边 | `minco_w_mid`（MINCO）/ `clearance_weight`（B样条） | `rolling_astar_margin` |
| 出折线 | `minco_w_mid` 是否过大 | `rolling_astar_margin`、`traj_backend` 回 0 对比 |
| 路径不跟飞机走 | `rolling_period`、`rolling_enable` | `anchor_from_trajectory` 必须 false |
| 打点没反应 | `goal clearance=... unknown=...`（日志） | `floor_z`/`ceil_z`、`plan_2d` |
| 重规划抖动 | `plan_anchor_catchup_max` | `realign_max_gap`、`ref_start_max_dev` |
| 起点被判障碍 | `escape_radius` | `ground_lift_max`、`robot_radius` |

---

## 14. 调参工作流（建议）

1. **一次只改一个参数**，记下改前的值
2. **改完必须重启节点**（除非是第 12 节那三个）
3. **用取证脚本录数据**，不要靠感觉判断：

```bash
# 终端 A：起飞（你自己操作）
bash ~/catkin_ws/src/as_gcs/ui/restart_all.sh raicom --planner azj

# 终端 B：旁路记录（只观测，不干预）
bash ~/catkin_ws/scripts/azj_shadow.sh
```

产物在 `~/azj_flights/flight_<ts>/`，含 `planner.log` / `flight.bag` /
`audit/report.txt` + CSV。

4. **看这些关键指标**：

| 指标 | 正常范围 | 含义 |
|---|---|---|
| `[azj] MINCO ok: Xms` | 1~20ms | 优化耗时 |
| `min_clr` | ≥ `robot_radius` | 轨迹最小间隙 |
| `rolling: seg=` 的 `min_clr` | ≥ 0.30 | A* 段间隙 |
| lag | 0.1~0.3m 且能回落 | 参考超前量 |
| XY 误差 p90 | < 0.7m | 跟踪质量 |
| Z 偏差 | ±0.07m | 高度稳定性 |

5. **出问题最快的回退**：
   - `rolling_enable: false` → 回 v1.9 全量流程
   - `traj_backend: 0` → 回 B 样条 + TOPP
   - `yaw_mode: 1` → 完全不转

---

## 15. 文档一致状态

本文编写时（2026-11-26）已逐项比对 yaml，**134 个参数全覆盖**。
同时修正了 README 中两处过时值：

- `clearance_weight`：README 原写 0.10 → 已改 **0.15**（yaml 实际值）
- `yaw_rate_max`：README 原写 1.5 → 已是 **1.0**（1.5 会掉机，见第 6 节）

### 仍存在的"文档覆盖不到"的地方

`w_vel` / `w_acc`（MINCO 速度/加速度权重，代码默认 **2.0**）
**未在 yaml 暴露**。虽然 `PARAMETERS.md` 提到了它们无法调，
但如果确实需要调（比如想让 max_vel 更"硬"），得改 `include/azj_planner/minco_optimizer.h`
或给它加两个 yaml 参数。

发现新的一致性问题时请更新本文，别让文档继续漂移。
