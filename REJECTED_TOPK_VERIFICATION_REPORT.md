# Rejected Top-k Verification Report

## 1. 实现

本轮新增默认关闭的 debug-only verification：

```json
"relocalization_debug": {
  "verify_rejected_topk": false,
  "verify_rejected_topk_k": 5
}
```

运行时通过 `run_standard_experiment.sh --verify-rejected-topk --verify-rejected-topk-k 5` 开启。该模式只验证被 descriptor/translation filter 拒绝的 raw top-k，并只写入 Debug CSV：

- `debug_verification_only`
- `debug_verified_rejected_topk_count`
- `debug_verified_best_submap`
- `debug_verified_best_descriptor_distance`
- `debug_verified_best_score`
- `debug_verified_best_inliers`
- `debug_verified_best_residual`
- `debug_verified_success`

它不会更新 pose、smoother、trajectory、state，也不会把超阈值候选当成正式 relocalization success。

## 2. 30s 实验结果

- debug verification rows: 13
- success: 12
- failed: 1
- GTSAM warning: 0

成功示例：

| frame | best_submap | descriptor_distance | score | inliers | residual |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 252 | 226 | 0.427 | 0.611 | 2796 | 5758.364 |
| 253 | 221 | 0.419 | 0.656 | 3056 | 15203.807 |
| 259 | 221 | 0.408 | 0.654 | 3117 | 8732.639 |
| 269 | 221 | 0.453 | 0.653 | 3230 | 13490.666 |
| 284 | 225 | 0.450 | 0.643 | 3962 | 22144.414 |
| 299 | 228 | 0.456 | 0.685 | 4453 | 30180.407 |

frame 279 的 debug verification 未通过，best rejected candidate 为 submap 226，descriptor distance `0.515`，score `0.625`，inliers `3436`，但仍未满足 registration acceptance。

## 3. 解释

descriptor distance 与 geometric verification score 不是简单单调关系。多个 `0.42-0.46` 的超阈值候选可以通过几何验证，说明当前 `0.35` 阈值确实漏掉了可用候选。

这仍然不是线上成功：正式 relocalization 当前仍因 `candidates_after_filter=0` 不会调用 verification，也不会恢复 TRACKING。

## 4. 建议

下一轮建议实现“verification-first but state-safe”的候选策略：

- 保留 raw top-k；
- 将 descriptor 阈值作为排序/预筛，而不是唯一硬门；
- 只允许 top-k 中通过 geometric verification 的候选进入 RECOVERING；
- RECOVERING 连续 N 帧 scan-to-map accepted 后才回 TRACKING；
- 不改变默认配置，先用实验开关验证。

