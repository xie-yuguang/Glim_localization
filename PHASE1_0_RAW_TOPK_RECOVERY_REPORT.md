# Phase 1.0 Raw Top-K Recovery Report

## 1. 执行摘要

本轮实现了默认关闭的 `verify_raw_topk` recovery experiment：当 LOST/RELOCALIZING 中主 `max_descriptor_distance=0.35` 过滤后没有候选时，可以把 raw top-k 中低于实验阈值的候选送入 geometric verification。raw top-k 不能绕过几何验证，验证成功后进入 `RECOVERING`，需要连续稳定帧后才回 `TRACKING`。

结论：

- 10s smoke baseline 不退化：101 frames，max_step=0.302m，LOST=0，GTSAM warning=0。
- 默认主 `localization.relocalization.max_descriptor_distance` 仍为 `0.35`，没有直接放宽默认阈值。
- 30s recovery on 能从 LOST 反复进入 RECOVERING，并出现 `RECOVERING -> TRACKING`，且 GTSAM warning=0。
- 但 30s 仍不是稳定 baseline：recovery 后仍有大量 registration rejection，最佳组合仍有 9m 级 max step。

## 2. 本轮修改列表

代码：

- `include/glim_localization/core/localization_options.hpp`
  - 新增 `VerifyRawTopkOptions` 和 `RecoveringOptions`。
- `src/glim_localization/core/localization_options.cpp`
  - 读取 `localization.relocalization.verify_raw_topk.*` 和 `localization.relocalization.recovering.*`。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp`
  - 新增 raw top-k recovery CSV 字段、RECOVERING 稳定字段和 helper 声明。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
  - 新增 `collect_raw_topk_recovery_candidates()`。
  - 新增 `accept_verified_relocalization()`。
  - raw top-k 候选必须通过 `GeometricVerifier::verify()`。
  - verification 成功后进入 `RECOVERING`，不直接 `TRACKING`。
  - RECOVERING 中按稳定帧计数回 TRACKING。
- `tools/analyze_debug_csv.py`
  - 新增 verify_raw_topk / RECOVERING 统计。
- `tools/run_standard_experiment.sh`
  - 新增 `--enable-verify-raw-topk`、`--verify-raw-topk-distance`、`--verify-raw-topk-k`、`--enable-recovering`、`--recovering-stable-frames`。
- `tools/run_offline_localization.sh`
  - 通过环境变量注入实验配置。
- `tools/run_phase0_regression_matrix.sh`
  - 增加 raw top-k recovery 实验 case。
- `config/localization*.json`
  - 增加默认关闭的配置模板。
- `DEBUG_CSV_SCHEMA.md`
  - 增加 Phase 1.0 字段说明。

报告：

- `VERIFY_RAW_TOPK_RECOVERY_DESIGN.md`
- `RECOVERING_STABILIZATION_REPORT.md`
- `PHASE1_0_30S_EXPERIMENT_REPORT.md`
- `RECOVERY_SENSITIVITY_SWEEP_REPORT.md`

## 3. 工作区状态

本轮开始前工作区已包含 Phase 0 到 Phase 0.9 的大量未提交改动和报告文件。本轮没有自动 commit。

建议提交分组：

- Phase 0 cleanup：配置清理、`.gitignore`、LICENSE、launch。
- Phase 0.5 tooling：失败 artifacts、monitor、短片段实验。
- Phase 0.6 debug CSV and guards：debug CSV、smoother guard、target rebuild guard。
- Phase 0.7 regression and analysis：CSV analyzer、confirmation guard、回归矩阵。
- Phase 0.8 relocalization diagnostics：relocalization CSV 字段、self-query 工具、LOST recovery 设计。
- Phase 0.9 descriptor sweep：threshold sweep、debug-only verification。
- Phase 1.0 recovery experiment：本轮 raw top-k recovery 和 RECOVERING 稳定期。

## 4. verify_raw_topk recovery 实现

新增配置：

```json
"verify_raw_topk": {
  "enable": false,
  "topk": 10,
  "max_descriptor_distance": 0.55,
  "require_geometric_verification": true,
  "allow_descriptor_rejected_candidates": true
}
```

行为：

- 默认关闭。
- 主 descriptor 阈值仍是 `0.35`。
- 仅当主候选为空时，才从 `ScanContextRelocalizer::last_query_diagnostics().topk` 中收集 raw top-k。
- raw top-k 仍受实验阈值 `verify_raw_topk.max_descriptor_distance` 限制。
- translation filter 仍必须通过。
- raw top-k 必须经过 `GeometricVerifier::verify()`。
- verification 成功后进入 `RECOVERING`。

CSV 字段：

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

## 5. RECOVERING 稳定期实现

新增配置：

```json
"recovering": {
  "enable": true,
  "stable_frames": 3,
  "max_recovery_correction_translation": 2.0,
  "max_recovery_correction_angle": 0.5,
  "require_no_rejection": true,
  "write_trajectory_during_recovering": true
}
```

RECOVERING 判据：

- registration accepted。
- `delta_t <= max_recovery_correction_translation`。
- `delta_r <= max_recovery_correction_angle`。
- `reject_reason` 为空。

RECOVERING 中发生 rejection：

- `recovering_stable_count` 归零。
- `recovery_frames_remaining` 重置为所需稳定帧数。
- 状态保持 RECOVERING，连续 rejection 达到阈值后仍按现有逻辑进入 LOST。

## 6. debug CSV 和分析工具更新

`tools/analyze_debug_csv.py` 现在输出：

- verify_raw_topk 使用次数。
- verify_raw_topk 成功次数。
- `RECOVERING -> TRACKING` 次数。
- recovery 中 rejection 次数。
- `recovering_stable_count` 最大值。

30s recovery on 示例：

- `verify_raw_topk_used=28`
- `verify_raw_topk_success=22`
- `RECOVERING -> TRACKING=3`
- `recovering_stable_count_max=3`

## 7. 10s smoke baseline 结果

命令：

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 10 \
  --timeout-sec 90 \
  --output-dir /tmp/glim_localization_phase1_0/smoke_10s
```

结果：

- exit code：0
- frames：101
- total_distance_m：6.962
- max_step_m：0.302
- suspicious_divergence：false
- LOST：0
- registration rejection：0
- GTSAM warning：0

## 8. 30s recovery off vs recovery on 对比

| case | frames | total_distance_m | max_step_m | suspicious | LOST log count | rejection count | GTSAM warning |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| recovery_off_30s | 301 | 1799.622 | 21.059 | true | 303 | 17 | 0 |
| verify_raw_topk_recovery_30s, d=0.55 | 301 | 391.431 | 9.188 | false | 64 | 87 | 0 |
| best_combo_050_30s | 294 trajectory / 301 CSV | 145.450 | 9.452 | false | 37 | 55 | 0 |

`verify_raw_topk_recovery_30s` 状态统计：

- TRACKING：168
- RECOVERING：120
- DEGRADED：7
- LOST：6
- `verify_raw_topk_used`：28
- `verify_raw_topk_success`：22
- `RECOVERING -> TRACKING`：3

## 9. sensitivity sweep 结果

| case | distance | stable_frames | total_distance_m | max_step_m | suspicious | verify used | verify success | RECOVERING->TRACKING | GTSAM |
| --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| sweep_045_30s | 0.45 | 3 | 240.234 | 57.747 | true | 9 | 9 | 1 | 0 |
| sweep_050_30s | 0.50 | 3 | 178.560 | 7.613 | false | 16 | 15 | 3 | 0 |
| sweep_050_stable5_30s | 0.50 | 5 | 178.573 | 7.613 | false | 16 | 15 | 3 | 0 |
| verify_raw_topk_recovery_30s | 0.55 | 3 | 391.431 | 9.188 | false | 28 | 22 | 3 | 0 |

当前最优实验点更接近 `distance=0.50, stable_frames=3/5`，但仍不能称为稳定 baseline。

## 10. 是否获得 30s 稳定 baseline

没有。

30s recovery on 明显优于 recovery off：

- 没有 GTSAM warning。
- 没有千米级轨迹飞散。
- LOST 片段被多次恢复到 RECOVERING。
- 出现了 `RECOVERING -> TRACKING`。

但仍存在：

- max_step 7m 到 9m。
- recovery 中 rejection 较多。
- target map / verification target 仍会在恢复后引入较大 correction。

## 11. 新增风险和副作用

- raw top-k verification 能恢复，但可能选择几何上局部可用、全局连续性不足的候选。
- `distance=0.55` 过宽时恢复次数更多，但 total distance 和 max step 更差。
- RECOVERING 中写 trajectory 虽带状态字段，但仍可能让简单轨迹图看起来“连续”；分析时必须结合 status。
- `hold_last_pose_prior` 仍只是防 GTSAM 欠约束的保护，不代表定位成功。

## 12. 当前仍然阻塞的问题

- recovery 后 target map active submaps 有时跳到不连续区域。
- RECOVERING 稳定判据只检查 correction 大小，还没有检查 active submap 连续性 / target center 连续性。
- raw top-k verification target 可能过局部，容易通过几何验证但后续 tracking 不稳定。
- 仍未实现局部 scan accumulation query。

## 13. 下一轮建议

P0/P1：

1. 给 raw top-k recovery 增加 map continuity gate：
   - 候选 submap 与 last accepted target center 距离；
   - active submap set 连续性；
   - T_map_odom jump 限制。
2. 给 geometric verification 加 recovery 专用 target：
   - 不只用单 submap；
   - 固定验证范围；
   - 输出验证 target active_submaps。
3. 实现 debug-only 3/5 帧 scan accumulation query。
4. RECOVERING 中连续 rejected 超阈值后暂停 trajectory 输出，而不是继续写所有恢复帧。

## 14. 构建与测试结果

构建：

```bash
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：通过。

测试：

```bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：15/15 passed。

静态检查：

```bash
python3 -m py_compile src/Glim_localization/tools/analyze_debug_csv.py
python3 -m py_compile src/Glim_localization/tools/sweep_scan_context_descriptors.py
bash -n src/Glim_localization/tools/run_phase0_regression_matrix.sh
bash -n src/Glim_localization/tools/run_standard_experiment.sh
bash -n src/Glim_localization/tools/run_offline_localization.sh
git diff --check
```

结果：通过。

## 15. Git diff 摘要

本轮是在 Phase 0 到 Phase 0.9 累积未提交工作区上继续修改，因此 `git diff --stat` 包含前序阶段改动。

摘要：

```text
19 files changed, 2496 insertions(+), 127 deletions(-)
```

新增/更新的 Phase 1.0 关键文件：

- `include/glim_localization/core/localization_options.hpp`
- `src/glim_localization/core/localization_options.cpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
- `tools/run_standard_experiment.sh`
- `tools/run_offline_localization.sh`
- `tools/run_phase0_regression_matrix.sh`
- `tools/analyze_debug_csv.py`
- `DEBUG_CSV_SCHEMA.md`
- `config/localization*.json`
