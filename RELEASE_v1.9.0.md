# azj_planner v1.9.0 —— 2026-11-17 存档（用户确认可用基线）

## 状态：可用
- **不掉高**
- **顺利规划**
- 已知遗留：飞行途中会「减速一下」（轨迹切换处速度不连续），目标是做到 SUPER 那样连续

## 这一版解决了什么（按时间倒序）
1. **加速度前馈 402 m/s²**（掉高真凶）—— 五次 Hermite 加速度公式分组错误
   `a = (e0·p0+e3·p1)/h² + (e1·v0+e4·v1)/h + (e2·a0+e5·a1)`
   旧写法三项统一除 h²，v 项变 v/h → 50~402 m/s²。已退回**三次 Hermite**。
2. **节点切线每段独立**（共享切线在相邻段时长差 5× 时速度不连续）
3. **转向时间下限** `dt ≥ |Δv|/max_acc`
4. **发布侧双钳制**：`acc_feedforward_limit=1.5`、`ref_vel_rate_limit=2.25`
5. **A\* 起点定向**：`path[0]` 后插引出点，沿入场速度方向
6. **realign 门槛** `realign_max_gap=0.25`（旧路径离飞机太远就不复用）
7. **DEVIATED 恢复重规划**（v1.6.1 曾改成"什么都不做"→ 参考点跑偏 1.4m）
8. **统一 anchor**：从当前 odom 出发 + 按规划耗时外推（`computeAnchor`）
9. **MPC_THR_HOVER=0.871** 写进 PX4 的 `px4-rc.params`（推力基准，出厂 0.5 不足）

## 关键参数（config/azj_planner.yaml）
```yaml
replan_rate: 15.0
curve_samples: 70          # 采样间隔 ≥ ESDF 分辨率 0.10m
curve_iterations: 35       # 894ms → 363ms
acc_feedforward_limit: 1.5 # 发布侧硬钳制
ref_vel_rate_limit: 2.25   # 参考速度变化率上限
realign_max_gap: 0.25
anchor_from_trajectory: false   # 从当前 odom，不用旧轨迹未来点
safety_near_frac: 0.95
safety_far_frac: 0.90
```

## 验证
- 离线测试：trajectory **61 PASS** / curve 16 / plan_mode 22
- 飞行验证：用户确认可用（2026-11-16 夜间最后一次）
- 加速度指标：5 场景 max|a| 全部 = 1.50（= max_acc 精确上限）

## 下一步
把「中途减速」做到 SUPER 那样连续 —— 见 `SUPER_CONTINUITY.md`
