# ScanContext Threshold Sweep Report

## 1. 输入

- CSV: `/tmp/glim_localization_phase0_9/relocalization_debug_verify_30s/debug/localization_debug.csv`
- Sweep 输出目录: `/tmp/glim_localization_phase0_9/relocalization_debug_verify_30s/scan_context_sweep`
- 分析窗口: frame 252-300
- 阈值: `0.30,0.35,0.40,0.45,0.50,0.55,0.60,0.70`

## 2. frame 252-300 top1 distance

- query rows: 12
- top1 distance min/mean/median/p90/max: `0.408 / 0.442 / 0.438 / 0.480 / 0.510`
- 常见 raw top-k submap: `221:11; 223:10; 228:10; 230:10; 222:6; 226:4`

## 3. 阈值 Sweep

| threshold | frames_with_candidates | top1_accepted_frames | mean_candidate_count | max_candidate_count |
| ---: | ---: | ---: | ---: | ---: |
| 0.30 | 0/12 | 0 | 0.000 | 0 |
| 0.35 | 0/12 | 0 | 0.000 | 0 |
| 0.40 | 0/12 | 0 | 0.000 | 0 |
| 0.45 | 8/12 | 8 | 5.167 | 10 |
| 0.50 | 11/12 | 11 | 8.417 | 10 |
| 0.55 | 12/12 | 12 | 10.000 | 10 |
| 0.60 | 12/12 | 12 | 10.000 | 10 |
| 0.70 | 12/12 | 12 | 10.000 | 10 |

## 4. 结论

`max_descriptor_distance=0.35` 对 frame 252-300 的 LOST query 明显过严：该窗口内所有实际 query 都被过滤。阈值到 `0.45` 才开始恢复多数候选，到 `0.50` 基本覆盖。

不建议直接把默认阈值改成 `0.50`。更安全的策略是：允许 raw top-k 进入 geometric verification，但正式恢复仍必须通过 registration score/inliers/residual/correction gating，并保留 LOST/RECOVERING 状态约束。

