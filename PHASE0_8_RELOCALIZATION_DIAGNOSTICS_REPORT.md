# Phase 0.8 Relocalization Diagnostics Report

## 1. 执行摘要

本轮完成 relocalization candidate 诊断、ScanContext self-query 工具、CSV 自动分析扩展和最小 LOST recovery policy。默认配置仍保持安全：debug CSV、relocalization debug、smoother guard、target rebuild guard、lost recovery 均默认关闭。

核心结论：

- 10s smoke baseline 不退化：101 frames，全部 TRACKING，0 rejection，0 LOST，max step 0.302m。
- ScanContext map database 基本健康：232 个 submap 中 top1 self=218，top5 self=228，仅 1 个 self-query 失败。
- frame 252-300 的长期 LOST 主因已明确：query cloud 和 descriptor 有效，raw top-k 存在，但 top1/top3 descriptor distance 高于 `max_descriptor_distance=0.35`，所有候选被 descriptor filter 拦截。
- geometric verification 在长期 LOST 段没有被调用，因为 `candidates_after_filter=0`。
- 30s 仍不是稳定 baseline，但保持可控失败：trajectory 251 行，CSV 301 行，LOST 50，GTSAM warning 0，suspicious_divergence=false。

## 2. 本轮修改列表

代码：

- `include/glim_localization/core/localization_options.hpp`
- `src/glim_localization/core/localization_options.cpp`
- `include/glim_localization/relocalization/scan_context_relocalizer.hpp`
- `src/glim_localization/relocalization/scan_context_relocalizer.cpp`
- `src/glim_localization/relocalization/geometric_verifier.cpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`

工具和脚本：

- `tools/analyze_debug_csv.py`
- `tools/check_scan_context_relocalizer.cpp`
- `tools/run_offline_localization.sh`
- `tools/run_standard_experiment.sh`
- `tools/run_phase0_regression_matrix.sh`
- `CMakeLists.txt`

配置/文档：

- `config/localization*.json`
- `README.md`
- `DEBUG_CSV_SCHEMA.md`

新增报告：

- `PHASE0_8_RELOCALIZATION_DIAGNOSTICS_REPORT.md`
- `PHASE0_8_RELOCALIZATION_FAILURE_ANALYSIS.md`
- `SCAN_CONTEXT_SELF_QUERY_REPORT.md`
- `LOST_RECOVERY_POLICY_DESIGN.md`
- `PHASE0_8_30S_EXPERIMENT_REPORT.md`

## 3. 工作区状态

本轮开始前已有 Phase 0/0.5/0.6/0.7 未提交改动和报告文件。本轮未 commit。

建议提交分组：

- Phase 0 cleanup：`.gitignore`、LICENSE、config CPU/GPU split、package/launch。
- Phase 0.5 tooling：failure artifacts、monitor process tree、short segment support。
- Phase 0.6 debug CSV and guards：debug CSV、smoother guard、target rebuild guard。
- Phase 0.7 regression and analysis：CSV analyzer、phase0 regression matrix、target confirmation guard refinement。
- Phase 0.8 relocalization diagnostics：本轮 relocalization diagnostics、self-query、lost recovery policy。

## 4. Relocalization debug CSV 字段

新增字段覆盖：

- query 是否启用、原因、点数、descriptor valid/nonempty bins。
- database size、raw top-k top1/top2/top3。
- descriptor / translation / other filter 计数。
- filter 前后 candidate 数。
- verification 是否尝试、最佳 verification score/inliers/residual。
- failure reason：`all_filtered_by_descriptor_distance`、`not_requested`、`success` 等。

CSV schema 已写入 `DEBUG_CSV_SCHEMA.md`。

## 5. ScanContext self-query 诊断工具结果

工具：

```bash
ros2 run glim_localization check_scan_context_relocalizer \
  /home/xie/Glim/data/map_data/ceshichang_128lidar 10
```

结果：

```text
database_size: 232
query_count: 232
top1_self: 218
top5_self: 228
failed_self_query: 1
top1_distance_mean: 0.108655
failed_submap_ids: 215
```

判断：map database 基本健康。

## 6. 30s relocalization failure 分析

30s debug run：

```text
CSV: /tmp/glim_localization_phase0_8/relocalization_debug_30s/debug/localization_debug.csv
analysis: /tmp/glim_localization_phase0_8/relocalization_debug_30s/debug_csv_analysis
```

关键统计：

```text
first_degraded: 126
first_lost: 144
longest_lost_segment: 252..300
requested/debug rows: 52
success_frames: 2
no_candidate_frames: 50
topk_returned_min/max/mean: 10/10/10
top1_distance_min/mean/max: 0.286/0.419/0.510
```

## 7. candidates=0 的真实原因

frame 252 后实际 query 的帧均满足：

- query cloud 有效。
- descriptor 有效。
- database size = 232。
- raw top-k 返回 10。
- candidates_after_filter = 0。
- failure_reason = `all_filtered_by_descriptor_distance`。

典型帧：

```text
252 top1=223 distance=0.423758 candidates=0
253 top1=221 distance=0.418755 candidates=0
259 top1=221 distance=0.407722 candidates=0
279 top1=221 distance=0.510240 candidates=0
299 top1=223 distance=0.452786 candidates=0
```

因此 candidates=0 不是 no top-k，也不是 translation filter，也不是 verification failed，而是 descriptor distance 阈值过滤。

## 8. LOST recovery policy 设计与实现

本轮实现默认关闭的最小 LOST recovery policy：

- LOST 后周期性尝试 relocalization。
- 周期等待帧标记为 `lost_recovery_period_wait`。
- 无候选时 smoother guard action 标记为 `lost_pause_no_candidate_hold_prior`。
- 默认不写 LOST/hold 的正常 trajectory。

限制：

- 仍未真正 skip frame。
- 仍未 reset smoother。
- 在线 publisher 的 pose/odom suppression 尚未正式实现。

详见 `LOST_RECOVERY_POLICY_DESIGN.md`。

## 9. 10s smoke baseline 结果

```text
output: /tmp/glim_localization_phase0_8/smoke_10s
frames: 101
state: TRACKING 101
rejection: 0
LOST: 0
max_step_m: 0.302
suspicious_divergence: false
```

## 10. 30s debug run 结果

```text
output: /tmp/glim_localization_phase0_8/relocalization_debug_30s
exit_code: 0
csv_frames: 301
trajectory_frames: 251
LOST: 50
max_step_m: 1.977
GTSAM warning: 0
suspicious_divergence: false
```

## 11. 是否获得 30s 稳定 baseline

没有。当前 30s 是可控失败，不是稳定 localization baseline。

## 12. 当前仍然阻塞的问题

- ScanContext query scan 与 map submap descriptor 的距离偏大，frame 252 后 top1 通常在 0.41-0.51，高于 0.35。
- `0 0 0 0 0 0` 仍只是 smoke baseline 初值，不是可靠 30s baseline 初值。
- LOST recovery 仍是 hold-prior 保护策略，没有正式 skip frame / smoother reset。

## 13. 下一轮建议

1. 对 relocalization descriptor 做离线 sweep：只统计 candidate count 和 verification，不把阈值直接改成默认。
2. 调查 query descriptor 域差异：单帧 scan vs submap merged cloud、z 高度、去地面、局部累积 scan。
3. 实现 relocalization top10 明细 JSON/CSV，便于看同一段 top-k 是否稳定集中在 221/223/228。
4. 设计正式 smoother reset / frame insertion policy。

## 14. 构建与测试结果

```bash
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：通过。

```bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：15/15 passed。

额外检查：

```bash
python3 -m py_compile tools/analyze_debug_csv.py
bash -n tools/run_phase0_regression_matrix.sh
bash -n tools/run_standard_experiment.sh
bash -n tools/run_offline_localization.sh
git diff --check
```

结果：通过。

## 15. Git diff 摘要

`git diff --stat` 摘要：

```text
19 files changed, 1855 insertions(+), 73 deletions(-)
```

另有新增文件和 Phase 0 以来的未跟踪报告文件；pycache 删除仍在 staged 状态。
