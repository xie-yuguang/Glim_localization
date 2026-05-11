# Phase 0.6 Guard and Debug Report

## 1. 执行摘要

本轮完成 Debug CSV、smoother guard、target rebuild guard 实验和 10 秒 smoke baseline 固化。所有新策略默认关闭，不改变默认 CPU localization 行为。

结论：

- build 通过。
- test 通过，15/15 passed。
- Debug CSV 可用，已生成 `/tmp/glim_localization_phase0_6/debug_csv_failure_15s/debug/localization_debug.csv`。
- smoother guard 采用方案 B：`hold_last_pose_prior`，原因是不改上游 GLIM 时无法安全跳过 frame insertion。
- target rebuild guard 已实现默认关闭的 confirmation window，成功阻止 frame 127 的近阈值 correction。
- 10 秒 smoke baseline 未退化。
- 15 秒最小失败片段明显更可控：total distance 从 43.808m 降到 28.271m，max step 从 5.553m 降到 3.174m。
- 30 秒探索不再飞到数百米/上千米，GTSAM warning 为 0，但后段仍长期 LOST/RELOCALIZING，因此还没有稳定 30 秒 localization baseline。

## 2. 本轮修改列表

代码与配置：

- `include/glim_localization/core/localization_options.hpp`
- `src/glim_localization/core/localization_options.cpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
- `config/localization.json`
- `config/localization.cpu.json`
- `config/localization.gpu.json`
- `config/localization.template.json`

脚本与文档：

- `tools/run_offline_localization.sh`
- `tools/run_standard_experiment.sh`
- `tools/run_smoke_baseline_10s.sh`
- `CMakeLists.txt`
- `README.md`
- `docs/engineering_playbook.md`

新增报告：

- `DEBUG_CSV_SCHEMA.md`
- `SMOOTHER_GUARD_DESIGN.md`
- `TARGET_REBUILD_GUARD_EXPERIMENT.md`
- `SMOKE_BASELINE_10S.md`
- `PHASE0_6_GUARD_AND_DEBUG_REPORT.md`

## 3. debug CSV 实现与字段说明

新增配置：

```json
"debug": {
  "csv_enable": false,
  "csv_path": "/tmp/glim_localization_debug.csv"
}
```

标准实验脚本新增 `--debug-csv`，并把 CSV 注入到：

```text
output_dir/debug/localization_debug.csv
```

CSV 字段覆盖 frame id、状态前后、backend、target rebuild/reuse、target center、active submaps、source/target points、predicted/corrected pose、delta、score、residual、inliers、reject reason、relocalization、smoother guard action 和耗时。

详情见 `DEBUG_CSV_SCHEMA.md`。

## 4. smoother guard 实现与安全边界

采用方案 B：`hold_last_pose_prior`。

配置默认关闭：

```json
"smoother_guard": {
  "enable": false,
  "mode": "hold_last_pose_prior",
  "max_lost_frames_without_prior": 0
}
```

触发条件是 LOST/RELOCALIZING relocalization 无候选，且存在 last accepted pose。触发后状态仍为 LOST，不伪装为 TRACKING，CSV 写入 `smoother_guard_action=hold_last_pose_prior`。

方案 A 未实现，因为不改上游 GLIM 时安全跳过 frame insertion 会破坏下一帧对 smoother key 的假设。

## 5. target rebuild guard 实现与安全边界

配置默认关闭：

```json
"target_rebuild_guard": {
  "enable": false,
  "confirmation_frames": 2,
  "near_threshold_ratio": 0.8,
  "force_degraded_on_large_correction": true
}
```

当前最小实现：target map rebuild 后的 confirmation window 内，如果 accepted registration 的 `delta_t > max_pose_correction_translation * near_threshold_ratio`，则改为 rejected，原因 `target_rebuild_large_correction`。

## 6. 10s smoke baseline 固化结果

新增脚本：

```bash
bash tools/run_smoke_baseline_10s.sh --bag BAG --map MAP --output-dir /tmp/glim_localization_smoke_10s
```

本地验证：

| 指标 | 结果 |
| --- | ---: |
| frames | 101 |
| duration | 10.000s |
| LOST | 0 |
| rejection | 0 |
| GTSAM warning | 0 |
| total distance | 6.962m |
| max step | 0.302m |

## 7. 15s 最小失败片段对比

| 实验 | frames | LOST | RELOCALIZING | rejection | total distance | max step | GTSAM |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| guard off + CSV | 151 | 8 | 8 | 13 | 43.808m | 5.553m | 0 |
| smoother guard on | 151 | 8 | 8 | 13 | 44.836m | 5.553m | 0 |
| target rebuild guard on | 151 | 6 | 6 | 10 | 28.271m | 3.174m | 0 |

结论：smoother guard 对 15 秒早期错误匹配无明显影响；target rebuild guard 明显改善 frame 126 后的可控性。

## 8. 30s 实验结果

| 实验 | frames | LOST | RELOCALIZING | rejection | total distance | max step | target guard | smoother hold | GTSAM |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| target guard only | 301 | 117 | 116 | 19 | 57.039m | 3.174m | 2 | 0 | 0 |
| target + smoother guard | 301 | 175 | 174 | 21 | 67.430m | 2.842m | 3 | 84 | 0 |

30 秒不再是数百米/上千米发散，但状态质量不足，不能算稳定 baseline。

## 9. frame 120-140 CSV 摘要

guard off：

```text
126 TRACKING->DEGRADED dt=5.015 score=0.935 reject=large_translation_correction rebuilt=1
127 DEGRADED->TRACKING dt=2.785 score=0.958 accepted rebuilt=0
128 TRACKING->TRACKING dt=2.815 score=0.940 accepted rebuilt=0
```

target rebuild guard on：

```text
126 TRACKING->DEGRADED dt=5.015 score=0.935 reject=large_translation_correction rebuilt=1
127 DEGRADED->DEGRADED dt=2.785 score=0.958 reject=target_rebuild_large_correction rebuilt=0
128 DEGRADED->TRACKING dt=0.697 score=0.939 accepted rebuilt=0
```

CSV 能覆盖 frame 126 诊断所需的 predicted/corrected pose、target center、active submaps、delta_t 和 reject reason。

## 10. GTSAM warning / crash 是否改善

本轮所有 10s、15s、30s 短片段实验均为 exit code 0，GTSAM warning 计数为 0。

注意：Phase 0.5 的 GTSAM warning 出现在更长期 LOST/RELOCALIZING 后；30 秒组合实验中 smoother guard 触发 84 次，但仍需更长片段验证能否覆盖 `x1445/x2968` 这类问题。

## 11. 构建与测试结果

构建命令：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：成功，`Summary: 1 package finished`。

测试命令：

```bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：成功，`100% tests passed, 0 tests failed out of 15`。

## 12. 当前是否已有 30s baseline

没有稳定 30 秒 localization baseline。

当前只有：

- 稳定 10 秒 smoke baseline。
- 30 秒“受控失败/探索”片段：轨迹未巨大飞散、GTSAM warning 为 0，但后段 LOST/RELOCALIZING 过多。

## 13. 当前仍然阻塞的问题

- 30 秒内仍会长期 LOST/RELOCALIZING。
- target rebuild guard 是拒绝策略，不是确认策略；还没有验证多帧一致性。
- smoother guard 是 hold prior 实验保护，不是正式 LOST smoother reset 策略。
- 初始位姿和地图/点云匹配语义仍需继续确认。

## 14. 下一轮建议

建议继续 Phase 0.x，不建议直接进入 Phase 1：

1. 用 Debug CSV 分析 30 秒中第一次进入长期 LOST 的前 20 帧。
2. 把 target rebuild guard 从单帧 reject 升级为真正的 two-frame confirmation。
3. 为 relocalization candidates=0 调查 ScanContext 覆盖和 query 输入。
4. 用更合理 initial pose 或人工 RViz 对齐重跑 30 秒。
5. 设计正式 smoother reset / state freeze 策略，替代实验性 hold prior。

## 15. Git diff 摘要

工作区在 Phase 0 / 0.5 未提交改动基础上继续叠加。报告文件仍散落在仓库根目录，建议下一轮或提交前整理到 `docs/analysis/`。

`git diff --stat` 摘要：

```text
16 files changed, 1164 insertions(+), 67 deletions(-)
```

`git diff --cached --stat` 仍包含 Phase 0 已 staged 的 `tools/monitor/__pycache__` 删除。
