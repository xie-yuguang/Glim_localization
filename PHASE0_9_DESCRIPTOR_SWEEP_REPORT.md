# Phase 0.9 Descriptor Sweep Report

## 1. 执行摘要

本轮完成 Phase 0.9 的受控 ScanContext descriptor sweep 和 debug-only rejected top-k verification。没有修改上游 `glim`、`glim_ros2`、`glim_ext`，没有改变默认 GPU / localization 行为，也没有把超阈值候选作为正式 relocalization success。

关键结论：

- 10s smoke baseline 保持稳定：101 frames，LOST=0，rejection=0，max step=0.302m。
- 30s 仍不是稳定 baseline，但保持可控失败：301 CSV frames，251 trajectory frames，LOST=50，max step=1.977m，GTSAM warning=0。
- frame 252-300 的 top1 descriptor distance 为 `0.408-0.510`，均高于当前 `max_descriptor_distance=0.35`。
- 在 frame 252-300，阈值 `0.35` 产生 0/12 可用候选；`0.45` 可使 8/12 query 有候选；`0.50` 可使 11/12 query 有候选。
- debug-only verification 对 rejected top-k 进行几何验证，13 次中 12 次通过，说明当前硬 descriptor 阈值过严。
- 不建议直接把默认阈值改大。建议下一轮实现 verification-first 的安全候选策略，配合 RECOVERING 和连续帧确认。

## 2. 本轮修改列表

- `include/glim_localization/core/localization_options.hpp`
  - 增加 `relocalization_debug.verify_rejected_topk` 和 `verify_rejected_topk_k`。
- `src/glim_localization/core/localization_options.cpp`
  - 读取新增配置。
- `config/localization*.json`
  - 增加默认关闭的 debug-only verification 配置。
- `include/glim_localization/relocalization/scan_context_relocalizer.hpp`
  - 在 query diagnostics 中保留 raw top-k 对应的候选对象。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.*`
  - Debug CSV 新增 raw top-k 列和 debug verification 列。
  - 增加 `verify_rejected_topk_for_debug()`，只写诊断，不改变状态。
- `tools/run_offline_localization.sh`
  - 支持环境变量注入 debug-only verification 配置。
- `tools/run_standard_experiment.sh`
  - 增加 `--verify-rejected-topk` 和 `--verify-rejected-topk-k`。
- `tools/analyze_debug_csv.py`
  - 增加 `--threshold-sweep`、threshold sweep 输出和 debug verification 统计。
- `tools/sweep_scan_context_descriptors.py`
  - 新增离线 threshold sweep 工具。
- `README.md`、`DEBUG_CSV_SCHEMA.md`
  - 补充 Phase 0.9 说明。

## 3. 工作区状态

本轮开始前已有 Phase 0-0.8 大量未提交修改和未跟踪报告。本轮未自动 commit。

建议提交分组：

- Phase 0 cleanup：配置清理、`.gitignore`、LICENSE、package/launch。
- Phase 0.5 tooling：失败收尾、monitor、短片段。
- Phase 0.6 debug CSV and guards：Debug CSV、smoother guard、target rebuild guard。
- Phase 0.7 regression and analysis：CSV analyzer、confirmation guard、回归矩阵。
- Phase 0.8 relocalization diagnostics：relocalization debug 字段、self-query、lost recovery。
- Phase 0.9 descriptor sweep：本轮新增的 threshold sweep 和 debug-only verification。

## 4. ScanContext threshold sweep 结果

frame 252-300:

- query rows: 12
- top1 distance min/mean/median/p90/max: `0.408 / 0.442 / 0.438 / 0.480 / 0.510`
- raw top-k 常见 submap: `221, 223, 228, 230`

| threshold | frames_with_candidates | top1_accepted | mean_candidate_count |
| ---: | ---: | ---: | ---: |
| 0.35 | 0/12 | 0 | 0.000 |
| 0.45 | 8/12 | 8 | 5.167 |
| 0.50 | 11/12 | 11 | 8.417 |
| 0.55 | 12/12 | 12 | 10.000 |

## 5. rejected top-k debug verification 结果

- debug verification rows: 13
- success: 12
- failed: 1

典型通过帧：

- frame 252: best submap 226, descriptor distance 0.427, score 0.611, inliers 2796。
- frame 253: best submap 221, descriptor distance 0.419, score 0.656, inliers 3056。
- frame 299: best submap 228, descriptor distance 0.456, score 0.685, inliers 4453。

结论：超过 `0.35` 的 raw top-k 候选中存在几何可用候选，当前 descriptor hard gate 是恢复失败的主要阻塞之一。

## 6. 单帧 vs 累积 scan descriptor 对比

本轮未完成真实累积 scan sweep。当前 CSV 不包含 scan 点云或 descriptor matrix，不能离线重建 3/5 帧累积 descriptor。建议下一轮做 debug-only scan ring buffer，再比较 single-frame 和 accumulated-frame query。

## 7. descriptor preprocessing sweep 结果

本轮未调整 descriptor preprocessing。当前不建议优先改 `num_rings / num_sectors / max_radius`。更优先的是验证 raw top-k + geometric verification 的安全候选策略。

## 8. 10s smoke baseline 结果

- output: `/tmp/glim_localization_phase0_9/smoke_10s`
- exit code: 0
- frames: 101
- total distance: 7.026 m
- max step: 0.302 m
- suspicious divergence: false

## 9. 30s debug verification run 结果

- output: `/tmp/glim_localization_phase0_9/relocalization_debug_verify_30s`
- exit code: 0
- resource report / trajectory / benchmark / Debug CSV 均生成。
- Debug CSV analysis: `/tmp/glim_localization_phase0_9/relocalization_debug_verify_30s/debug_analysis`
- ScanContext sweep: `/tmp/glim_localization_phase0_9/relocalization_debug_verify_30s/scan_context_sweep`

## 10. max_descriptor_distance=0.35 是否过严

是。对 frame 252-300 的长期 LOST 段而言，`0.35` 过滤掉所有实际 query；而 debug-only verification 显示多数 `0.42-0.46` 的 raw top-k 候选可通过几何验证。

但不建议直接把默认值改成 `0.45/0.50`。推荐策略是：

- 保留 descriptor distance 作为排序和宽松预筛；
- 允许 raw top-k 进入 geometric verification；
- 只有 verification 通过才进入 RECOVERING；
- RECOVERING 需要连续稳定 scan-to-map 才回 TRACKING。

## 11. 推荐 relocalization 参数或算法改动

短期实验建议：

- 新增正式开关：`relocalization.verify_raw_topk.enable=false`。
- `raw_topk_k=5`。
- 宽松 descriptor guard 可先试 `0.50`，但必须配合 geometric verification。
- 不把 debug-only verification success 当作正式 success。

中期建议：

- 做 3/5 帧 query accumulation；
- 在 LOST 恢复后进入 RECOVERING，连续 N 帧 accepted 后再 TRACKING；
- 加入 candidate spatial continuity / target-map consistency 检查。

## 12. 是否获得 30s 稳定 baseline

没有。30s 仍是可控失败，不是稳定 localization baseline。

## 13. 当前仍然阻塞的问题

- 正式 relocalization 当前仍使用 hard descriptor gate，因此 frame 252 后无法进入 verification。
- debug-only verification 证明候选可能可用，但还没有正式 recovery policy 接入。
- query accumulation 和 descriptor preprocessing 还没有定量结果。

## 14. 下一轮建议

下一轮建议继续 Phase 0.x，而不是进入 Phase 1：

1. 实现默认关闭的正式 `verify_raw_topk` recovery experiment。
2. 在 LOST 中对 raw top-k 做 geometric verification，成功后进入 RECOVERING，不直接 TRACKING。
3. 加入 3/5 帧 query accumulation debug-only 对比。
4. 用 30s 片段验证是否能从 frame 252 后恢复。

## 15. 构建与测试结果

- `colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+`：通过。
- `colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure`：通过，15/15。
- `python3 -m py_compile tools/analyze_debug_csv.py tools/sweep_scan_context_descriptors.py`：通过。
- `bash -n tools/run_standard_experiment.sh tools/run_offline_localization.sh tools/run_phase0_regression_matrix.sh`：通过。
- `git diff --check`：通过。
- `check_scan_context_relocalizer`：exit code 4，仅因 submap 215 一个 self-query failure，作为 diagnostic warning 记录。

## 16. Git diff 摘要

当前 `git diff --stat` 仍包含 Phase 0-0.9 累积改动。Phase 0.9 相关新增文件：

- `tools/sweep_scan_context_descriptors.py`
- `PHASE0_9_DESCRIPTOR_SWEEP_REPORT.md`
- `SCAN_CONTEXT_THRESHOLD_SWEEP_REPORT.md`
- `REJECTED_TOPK_VERIFICATION_REPORT.md`
- `QUERY_ACCUMULATION_SWEEP_REPORT.md`
- `DESCRIPTOR_PREPROCESSING_SWEEP_REPORT.md`
- `PHASE0_9_30S_EXPERIMENT_REPORT.md`

