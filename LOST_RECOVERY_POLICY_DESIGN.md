# LOST Recovery Policy Design

## 1. 本轮实现

Phase 0.8 新增默认关闭的 `localization.lost_recovery`：

```json
"lost_recovery": {
  "enable": false,
  "relocalization_period_frames": 5,
  "recovery_stable_frames": 3,
  "max_lost_frames_before_pause": 3,
  "write_trajectory_while_lost": false
}
```

实验启用后：

- LOST 后不会每帧都发起 relocalization query。
- 前若干 LOST 帧仍可连续 query，之后按 `relocalization_period_frames` 周期尝试。
- 非 query 帧写入 CSV：`relocalization_query_reason=lost_recovery_period_wait`。
- 没有候选时仍使用 smoother guard 的 hold prior 防止 fixed-lag smoother 欠约束。
- `write_trajectory_while_lost=false` 时，LOST/hold 期间不写正常 trajectory。

## 2. 当前没有实现的部分

当前仍未实现真正的 skip-frame 或 smoother reset，因为 `glim::OdometryEstimationIMU::insert_frame()` 属于上游路径，本轮禁止改上游。当前策略仍是保护 fixed-lag smoother，而不是正式定位恢复。

当前没有把 publisher 的 pose/odom 输出完全暂停；`write_trajectory()` 中会先更新 custom data，再根据配置跳过轨迹文件写入。在线输出侧的 pose suppression 仍需要后续在 publisher 或 odometry result contract 中实现。

## 3. 为什么没有选择 skip frame

理想方案是 LOST 且无候选时不向 smoother 插入当前 frame。但当前 subclass 只能覆盖 `create_factors()` 和 `update_frames()`，上游 IMU odometry 已经创建了 frame/state。强行返回空 factors 可能继续留下欠约束 X/V/B 状态，这正是 Phase 0.5/0.6 观察到的 GTSAM underconstrained 风险。

因此本轮保持 `hold_last_pose_prior`，但明确标记为：

```text
lost_pause_no_candidate_hold_prior
```

并关闭 LOST 期间正常 trajectory 写入，避免把冻结 pose 误读为成功定位。

## 4. 推荐正式方案

下一阶段正式方案建议二选一：

1. 在 localization odometry module 与上游 `insert_frame()` 之间增加 frame insertion policy。
   - LOST/no candidate 时可安全 skip 当前 frame。
   - 不创建新的欠约束 X/V/B 状态。
   - 仍发布 LOST status 和 debug CSV。

2. 在 relocalization 成功后执行 smoother reset。
   - 清空或重建 fixed-lag smoother 局部窗口。
   - 用 relocalized pose 作为新窗口 anchor。
   - 进入 RECOVERING，连续 N 帧 accepted 后再 TRACKING。

## 5. 当前风险

`hold_last_pose_prior` 是安全保护，不是定位成功。长期保留可能让内部 smoother 稳定但外部系统误以为定位仍有 pose。必须结合 status、CSV 和 trajectory suppression 使用。
