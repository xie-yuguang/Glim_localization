# Recovery Sensitivity Sweep Report

## 1. Sweep 范围

本轮实际跑了以下组合：

| case | verify_raw_topk_distance | recovering_stable_frames |
| --- | ---: | ---: |
| `sweep_045_30s` | 0.45 | 3 |
| `sweep_050_30s` | 0.50 | 3 |
| `sweep_050_stable5_30s` | 0.50 | 5 |
| `verify_raw_topk_recovery_30s` | 0.55 | 3 |

没有穷举 `0.45/0.50/0.55 x 2/3/5`，原因是当前瓶颈已经清楚：恢复后 tracking 稳定性不足，而不是单纯候选数不足。

## 2. 结果表

| case | frames | total_distance_m | max_step_m | suspicious | LOST log | rejection | verify used | verify success | RECOVERING->TRACKING | GTSAM |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| sweep_045_30s | 301 | 240.234 | 57.747 | true | 112 | 41 | 9 | 9 | 1 | 0 |
| sweep_050_30s | 301 | 178.560 | 7.613 | false | 56 | 65 | 16 | 15 | 3 | 0 |
| sweep_050_stable5_30s | 301 | 178.573 | 7.613 | false | 56 | 65 | 16 | 15 | 3 | 0 |
| verify_raw_topk_recovery_30s | 301 | 391.431 | 9.188 | false | 64 | 87 | 28 | 22 | 3 | 0 |

## 3. 结论

- `0.45` 过窄，恢复次数不足，仍出现一次很大的 57.747m jump。
- `0.50` 是本轮更合理的实验点，既能恢复，又比 0.55 更少引入不稳定候选。
- `stable_frames=5` 相比 3 没有明显改善 max_step，只是增加 RECOVERING 停留。
- `0.55` 恢复更多，但 total distance 和 max_step 更差。

## 4. 推荐

下一轮如继续做正式 recovery，应优先使用：

```text
verify_raw_topk_distance = 0.50
recovering_stable_frames = 3
```

但必须增加：

- recovery continuity gate；
- active submap overlap；
- verification target 稳定性检查；
- recovery 后 target center jump 限制。
