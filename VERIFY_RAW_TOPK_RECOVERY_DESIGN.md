# Verify Raw Top-K Recovery Design

## 1. 目标

Phase 1.0 的 `verify_raw_topk` 是一个默认关闭的正式 recovery experiment，用于解决 Phase 0.8/0.9 发现的问题：LOST 后 raw top-k 存在，但主 descriptor 阈值 `0.35` 将候选全部过滤，导致 geometric verification 无法执行。

## 2. 安全边界

- 默认关闭：`localization.relocalization.verify_raw_topk.enable=false`。
- 主阈值不变：`localization.relocalization.max_descriptor_distance=0.35`。
- raw top-k 只在主候选为空时启用。
- raw top-k 不能绕过 geometric verification。
- verification 成功后只进入 `RECOVERING`。
- `RECOVERING -> TRACKING` 需要稳定帧确认。

## 3. 配置

```json
"verify_raw_topk": {
  "enable": false,
  "topk": 10,
  "max_descriptor_distance": 0.55,
  "require_geometric_verification": true,
  "allow_descriptor_rejected_candidates": true
}
```

说明：

- `max_descriptor_distance` 是实验路径阈值，不替代主阈值。
- `require_geometric_verification=false` 当前会被忽略，代码仍强制 geometric verification。
- translation filter 仍必须通过，避免明显远离当前预测位姿的候选进入验证。

## 4. 实现路径

涉及文件：

- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
- `include/glim_localization/core/localization_options.hpp`
- `src/glim_localization/core/localization_options.cpp`
- `include/glim_localization/relocalization/scan_context_relocalizer.hpp`
- `src/glim_localization/relocalization/scan_context_relocalizer.cpp`

关键函数：

- `ScanContextRelocalizer::query()`
  - 保存 raw top-k 到 `last_query_diagnostics()`。
- `OdometryEstimationLocalizationCPU::collect_raw_topk_recovery_candidates()`
  - 从 raw top-k 中筛选实验候选。
- `GeometricVerifier::verify()`
  - 对候选执行 scan-to-map 几何验证。
- `OdometryEstimationLocalizationCPU::accept_verified_relocalization()`
  - verification 成功后注入 pose prior，并进入 RECOVERING。

## 5. CSV 观察字段

- `verify_raw_topk_enable`
- `verify_raw_topk_used`
- `verify_raw_topk_candidate_count`
- `verify_raw_topk_best_submap`
- `verify_raw_topk_best_distance`
- `verify_raw_topk_best_score`
- `verify_raw_topk_best_inliers`
- `verify_raw_topk_best_residual`
- `verify_raw_topk_success`
- `descriptor_passed_main_threshold`
- `descriptor_passed_raw_topk_experiment`

## 6. 本轮实验结论

`distance=0.55`：

- raw top-k 使用 28 次。
- verification 成功 22 次。
- 触发 3 次 `RECOVERING -> TRACKING`。
- 没有 GTSAM warning。
- 但 max_step=9.188m，仍不是稳定 baseline。

`distance=0.50`：

- raw top-k 使用 16 次。
- verification 成功 15 次。
- 触发 3 次 `RECOVERING -> TRACKING`。
- max_step=7.613m，优于 0.55。

## 7. 长期建议

不建议直接把主阈值改为 0.50 或 0.55。更稳妥的正式路径：

1. 保持主 descriptor threshold = 0.35。
2. 在 LOST 中启用 raw top-k recovery experiment。
3. raw top-k 候选必须通过 geometric verification。
4. verification 后进入 RECOVERING。
5. RECOVERING 中增加 continuity gate 和 active submap 稳定检查。
