# Smoother Guard Design

## 选择方案

Phase 0.6 选择方案 B：`hold_last_pose_prior`，默认关闭。

没有选择方案 A 的原因：不改上游 GLIM 时，`OdometryEstimationIMU::insert_frame()` 已经把当前 frame、`X/V/B` new values 和 IMU factors 准备好，并在 `create_factors()` 返回后立即更新 smoother。若只在子类里“跳过 smoother update”，下一帧仍会把这个未进入 smoother 的 frame 当作 last frame 查询 `X(last)`，存在破坏上游状态序列的风险。

## 配置

```json
"smoother_guard": {
  "enable": false,
  "mode": "hold_last_pose_prior",
  "max_lost_frames_without_prior": 0
}
```

## 触发条件

当前实现只在 relocalization 查询无候选时尝试触发：

```text
status enters RELOCALIZING
AND candidates == 0
AND no scan-to-map pose prior will be added
AND smoother_guard.enable == true
AND last accepted pose exists
```

触发后：

- 当前状态仍保持 LOST / RELOCALIZING 语义，不伪装为 TRACKING。
- 当前 frame pose 被冻结到 last accepted pose。
- 仅添加 `PriorFactor<Pose3>(X(current))`，用于避免 pose 欠约束。
- CSV 写入 `smoother_guard_action=hold_last_pose_prior`。

## 安全边界

这个 guard 只是防止 fixed-lag smoother 在 LOST 后继续接收欠约束 pose 状态，不代表定位恢复，也不应作为长期定位策略。后续更正确的方案仍是设计 localization-only 的 LOST/RELOCALIZING smoother reset 或 frame insertion policy。

## Phase 0.6 实验结果

15 秒片段：

- guard off：GTSAM warning 0，LOST 8，registration rejection 13。
- smoother guard on：GTSAM warning 0，LOST 8，registration rejection 13，`hold_last_pose_prior` 触发 1 次。

30 秒组合实验：

- target rebuild guard + smoother guard：GTSAM warning 0，`hold_last_pose_prior` 触发 84 次。
- 轨迹不再飞到数百米/上千米，`suspicious_divergence=false`，但 LOST/RELOCALIZING 很多，不能视为稳定 baseline。

## 下一步建议

P0.x 继续：先保留默认关闭，用 debug CSV 和更长片段确认它能否稳定抑制 `x1445/x2968` 类 GTSAM warning。进入 Phase 1 前，不建议默认开启。
