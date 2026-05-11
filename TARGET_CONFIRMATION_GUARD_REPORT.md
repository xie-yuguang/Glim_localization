# Target Confirmation Guard Report

## 实现摘要

Phase 0.7 将 Phase 0.6 的 target rebuild guard 从“窗口内大 correction 单帧拒绝”推进为“多帧 confirmation 实验策略”。

新增配置仍默认关闭：

```json
"target_rebuild_guard": {
  "enable": false,
  "confirmation_frames": 2,
  "near_threshold_ratio": 0.8,
  "force_degraded_on_large_correction": true,
  "consistency_translation": 1.0,
  "consistency_angle": 0.3,
  "cooldown_frames": 3
}
```

## 当前策略

- target map rebuild 后进入 confirmation window。
- window 内如果 `delta_t > max_pose_correction_translation * near_threshold_ratio`：
  - 不写入 smoother；
  - 不更新 last accepted pose；
  - reject reason 为 `target_rebuild_large_correction` 或 `target_rebuild_confirmation_pending`；
  - 缓存 pending correction，并记录是否与上一次 pending correction 一致；
  - 当前版本只记录一致性，不直接接受连续大 correction。
- window 内如果连续小 correction 达到 `confirmation_frames`，退出 confirmation window。

Debug CSV 新增字段：

- `target_rebuild_guard_action`
- `confirmation_window_remaining`
- `pending_correction_delta_t`
- `pending_correction_consistent`

## 15s 对比

| case | frames | LOST | rejected | total distance | max step | suspicious |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| guard off | 151 | 1 | 13 | 43.776m | 5.553m | false |
| confirmation guard | 151 | 1 | 10 | 28.271m | 3.174m | false |

关键帧：

```text
126 TRACKING->DEGRADED delta_t=5.015 reject=large_translation_correction target_rebuild_guard_action=confirmation_window_started
127 DEGRADED->DEGRADED delta_t=2.785 reject=target_rebuild_large_correction target_rebuild_guard_action=pending_large_correction
128 DEGRADED->TRACKING delta_t=0.697 target_rebuild_guard_action=confirmation_small_correction
129 TRACKING->TRACKING target_rebuild_guard_action=confirmed_small_corrections
```

结论：confirmation guard 继续稳定拦住 frame 127 的近阈值错误 correction，并显著降低 15s 轨迹总路程和最大单步。

## 30s 对比

| case | trajectory frames | CSV frames | LOST | max step | suspicious | 说明 |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| confirmation only | 301 | 301 | 129 | 17.559m | true | LOST 后预测轨迹仍继续写出，出现大漂移 |
| confirmation + smoother guard | 251 | 301 | 50 | 1.977m | false | hold frame 不写 trajectory，状态仍为 LOST |

结论：target confirmation guard 能解释和缓解 frame 126/127，但不能单独解决 30s 后段长期 LOST。30s 可控失败需要 smoother guard 边界配合。

## 风险

- 该策略默认关闭，仍是实验保护。
- 当前不接受“连续一致的大 correction”，只记录 pending 一致性，避免把错误跳变写入 smoother。
- 过严的 confirmation 可能让系统更早进入 DEGRADED/LOST；本轮 15s 未观察到明显副作用，但 30s 后段仍会 LOST。

## 下一步建议

P0.8 建议把 pending correction 的一致性用于 relocalization 验证，而不是直接恢复 TRACKING；同时补充候选验证日志，解释 frame 252 后 no-candidate 长段。
