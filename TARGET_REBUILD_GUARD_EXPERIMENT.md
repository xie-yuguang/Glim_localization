# Target Rebuild Guard Experiment

## 目标

frame 126 显示 target map rebuild 后出现高 score 但大 correction：

```text
frame 126: delta_t=5.015, score=0.935, reason=large_translation_correction
frame 127: delta_t=2.785, accepted
frame 128: delta_t=2.815, accepted
```

Phase 0.6 增加默认关闭的实验性 guard，验证 rebuild 后短窗口内拒绝近阈值 correction 是否能降低失稳。

## 配置

```json
"target_rebuild_guard": {
  "enable": false,
  "confirmation_frames": 2,
  "near_threshold_ratio": 0.8,
  "force_degraded_on_large_correction": true
}
```

当前最小实现：

```text
target map just rebuilt OR rebuild confirmation window active
AND registration accepted
AND delta_t > max_pose_correction_translation * near_threshold_ratio
=> reject, reason=target_rebuild_large_correction
```

默认 `max_pose_correction_translation=3.0`、`near_threshold_ratio=0.8`，因此 2.4m 以上 correction 会在 rebuild 确认窗口内被拒绝。

## 15 秒结果

| 实验 | frames | rejection | LOST | total distance | max step | GTSAM warning |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| guard off | 151 | 13 | 8 | 43.808m | 5.553m | 0 |
| target rebuild guard on | 151 | 10 | 6 | 28.271m | 3.174m | 0 |

关键 frame：

```text
126 TRACKING->DEGRADED dt=5.015 reject=large_translation_correction rebuilt=1
127 DEGRADED->DEGRADED dt=2.785 reject=target_rebuild_large_correction rebuilt=0
128 DEGRADED->TRACKING dt=0.697 accepted rebuilt=0
```

结论：guard 成功阻止 frame 127 近阈值 correction 被直接接受，并让 15 秒片段更可控。

## 30 秒结果

target rebuild guard only：

- frames：301
- total distance：57.039m
- max step：3.174m
- `suspicious_divergence=false`
- target guard rejection：2
- GTSAM warning：0
- LOST/RELOCALIZING 仍很多：117/116

target rebuild guard + smoother guard：

- frames：301
- total distance：67.430m
- max step：2.842m
- `suspicious_divergence=false`
- target guard rejection：3
- smoother hold prior：84
- GTSAM warning：0

## 判断

target rebuild guard 明显改善 15 秒最小失败片段，并把 30 秒轨迹控制在几十米量级；但 30 秒仍不是稳定 localization baseline，因为后段仍长期 LOST/RELOCALIZING。
