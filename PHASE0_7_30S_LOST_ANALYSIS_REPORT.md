# Phase 0.7 30s LOST Analysis Report

## 1. 执行摘要

本轮完成 Phase 0.7：新增 Debug CSV 自动分析工具，升级 target rebuild 后的 confirmation guard，量化 smoother guard 副作用，并固化 10s / 15s / 30s 回归矩阵。

结论：

- build 通过。
- test 通过，15/15 passed。
- `tools/analyze_debug_csv.py` 可用，并已自动分析 30s Debug CSV。
- 10s smoke baseline 不退化：101 frames，LOST=0，rejection=0，max_step=0.302m。
- 15s confirmation guard 继续改善 frame 126/127 链路：max_step 从 5.553m 降到 3.174m。
- 30s best 仍不是稳定 localization baseline，但已经是可控失败：CSV 301 frames，trajectory 251 frames，max_step=1.977m，GTSAM warning=0。
- 30s 长期 LOST 从 frame 252 开始，持续到 frame 300，共 49 frames。
- 长期 LOST 的直接原因是 relocalization candidates 基本为 0；target confirmation guard 只能缓解 rebuild 后 correction 注入，不能解决候选缺失。

## 2. 本轮修改列表

新增：

- `tools/analyze_debug_csv.py`
- `tools/run_phase0_regression_matrix.sh`
- `docs/phase0_regression_matrix.md`
- `PHASE0_7_30S_CSV_ANALYSIS.md`
- `TARGET_CONFIRMATION_GUARD_REPORT.md`
- `SMOOTHER_GUARD_SIDE_EFFECTS.md`
- `PHASE0_REGRESSION_MATRIX.md`
- `PHASE0_7_30S_LOST_ANALYSIS_REPORT.md`

修改：

- `CMakeLists.txt`
  - 安装 `analyze_debug_csv.py` 和 `run_phase0_regression_matrix.sh`。
- `include/glim_localization/core/localization_options.hpp`
- `src/glim_localization/core/localization_options.cpp`
  - 增加 smoother guard 连续 hold 边界配置。
  - 增加 target confirmation guard consistency/cooldown 配置。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp`
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
  - 增加 CSV 字段。
  - 升级 target rebuild confirmation guard。
  - 增加 smoother hold 连续帧计数和 trajectory 写入边界。
- `config/localization*.json`
  - 增加默认关闭的新 guard 配置项。
- `tools/run_offline_localization.sh`
- `tools/run_standard_experiment.sh`
  - 增加新配置注入和 CLI 参数。
- `DEBUG_CSV_SCHEMA.md`
  - 补充新 CSV 字段和自动分析工具说明。

## 3. 工作区状态

本轮开始时工作区已有 Phase 0、0.5、0.6 未提交改动和未跟踪报告文件。本轮没有自动 commit，也没有覆盖用户改动。

建议提交分组：

- Phase 0 cleanup：配置清理、LICENSE、package.xml、launch、CPU/GPU template、pycache 删除。
- Phase 0.5 tooling：失败 artifact、短片段参数、进程树 monitor、候选初值和 frame/GTSAM 分析报告。
- Phase 0.6 debug CSV and guards：Debug CSV、smoother guard、target rebuild guard、10s smoke。
- Phase 0.7 analysis and guard refinement：CSV 分析工具、target confirmation guard、smoother 副作用边界、回归矩阵。

报告文件目前仍散落在仓库根目录。建议提交前统一迁移到 `docs/analysis/`，本轮未强制移动。

## 4. CSV 自动分析工具

新增：

```bash
python3 tools/analyze_debug_csv.py /path/to/localization_debug.csv --out /path/to/report_dir
```

输出：

- `debug_csv_summary.md`
- `state_timeline.csv`
- `target_rebuild_events.csv`
- `correction_statistics.csv`
- `lost_segments.csv`
- `guard_actions.csv`

特点：

- 只依赖 Python 标准库。
- 对空 CSV、格式错误和缺失字段容错。
- 可自动统计状态、reject reason、rebuild 前后 active submaps、high score + large delta、LOST 段、smoother/target guard action 和 relocalization candidates。

本轮验证：

```bash
python3 -m py_compile tools/analyze_debug_csv.py
```

通过。

## 5. 30s CSV 分析结果

Phase 0.7 best 30s：

```text
/tmp/glim_localization_phase0_7/regression_matrix_v2/experimental_30s_best/debug/localization_debug.csv
```

自动分析输出：

```text
/tmp/glim_localization_phase0_7/regression_matrix_v2/experimental_30s_best/debug_csv_analysis
```

关键结果：

| 指标 | 结果 |
| --- | ---: |
| CSV frames | 301 |
| trajectory frames | 251 |
| first DEGRADED | 126 |
| first LOST | 144 |
| longest LOST segment | 252..300 |
| longest LOST frames | 49 |
| LOST total | 50 |
| TRACKING total | 236 |
| smoother hold prior | 50 |
| relocalization requested | 49 |
| relocalization success | 2 |
| relocalization no candidates | 48 |
| GTSAM warning | 0 |

30s 失败链路：

```text
frame 126 target rebuild
  -> frame 126 large_translation_correction rejected
  -> frame 127 target_rebuild_large_correction rejected by confirmation guard
  -> frame 128/129 小 correction 通过 confirmation
  -> frame 132/133/138/139 多次 high score + large delta
  -> frame 144 短暂 LOST，随后恢复
  -> frame 248 target rebuild
  -> frame 250/251 large correction
  -> frame 252 进入长期 LOST
  -> frame 252..300 relocalization candidates 基本为 0
```

## 6. target confirmation guard 实现与结果

新增配置默认关闭：

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

当前策略：

- rebuild 后进入 confirmation window；
- window 内大 correction 不写入 smoother，不更新 last accepted pose；
- 连续小 correction 达到 `confirmation_frames` 才退出 window；
- 连续大 correction 只记录为 pending，一致性写入 CSV，不直接接受。

15s 对比：

| case | frames | LOST | rejected | total distance | max step |
| --- | ---: | ---: | ---: | ---: | ---: |
| guard off | 151 | 1 | 13 | 43.776m | 5.553m |
| confirmation guard | 151 | 1 | 10 | 28.271m | 3.174m |

frame 126/127/128：

```text
126 TRACKING->DEGRADED delta_t=5.015 reject=large_translation_correction action=confirmation_window_started
127 DEGRADED->DEGRADED delta_t=2.785 reject=target_rebuild_large_correction action=pending_large_correction
128 DEGRADED->TRACKING delta_t=0.697 action=confirmation_small_correction
129 TRACKING->TRACKING action=confirmed_small_corrections
```

## 7. smoother guard 副作用分析

当前 smoother guard 是安全保护，不是定位成功。

30s best 中：

- hold prior 50 次。
- 最长连续 hold 区间为 frame 252..300。
- CSV 仍记录全部 301 frames。
- trajectory 默认跳过 hold frame，因此只有 251 frames。
- `suspicious_divergence=false`，max_step=1.977m。
- GTSAM warning=0。

主要副作用：

- 如果持续发布 pose，可能误导下游认为定位还有效。
- 如果持续写 trajectory，会把冻结位姿当普通定位结果保存。
- hold prior 不能修复 relocalization candidates=0，只是避免 fixed-lag smoother 被无约束状态污染。

本轮边界：

- 默认关闭。
- `write_trajectory_during_hold=false`，hold frame 不写入 trajectory。
- LOST 状态仍通过 CSV 和 ROS custom data 保留，不伪装 TRACKING。
- `max_consecutive_hold_priors` 已配置化。

正式方案建议见 `SMOOTHER_GUARD_SIDE_EFFECTS.md`。

## 8. 10s / 15s / 30s 回归矩阵结果

新增脚本：

```bash
bash tools/run_phase0_regression_matrix.sh --bag BAG --map MAP --output-dir DIR
```

本轮本地输出：

```text
/tmp/glim_localization_phase0_7/regression_matrix_v2
```

结果：

| case | exit | trajectory frames | CSV frames | LOST | rejected | max step | suspicious |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| smoke_10s | 0 | 101 | 101 | 0 | 0 | 0.302m | false |
| failure_15s_guard_off | 0 | 151 | 151 | 1 | 13 | 5.553m | false |
| failure_15s_confirmation_guard | 0 | 151 | 151 | 1 | 10 | 3.174m | false |
| experimental_30s_best | 0 | 251 | 301 | 50 | 14 | 1.977m | false |

## 9. 是否获得 30s 稳定 baseline

没有。

当前获得的是 30s 可控失败 baseline：

- 轨迹不再巨大漂移。
- GTSAM warning 为 0。
- 状态明确显示 frame 252 后长期 LOST。
- 不能把 hold prior 冻结段解释成稳定定位。

## 10. 如果没有，当前最可能阻塞原因

当前最主要阻塞不是 GICP 单帧 correction 阈值，而是 LOST 后 relocalization 候选缺失：

- frame 252..300 长期 LOST。
- relocalization requested 49 frames。
- no-candidate 48 frames。
- candidates 只有 0/1，mean 0.020。

需要继续调查 ScanContext query 输入、descriptor 有效性、候选过滤阈值和地图/初值语义。

## 11. 构建和测试结果

构建：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：成功，`Summary: 1 package finished`。

测试：

```bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：成功，`100% tests passed, 0 tests failed out of 15`。

额外检查：

```bash
python3 -m py_compile tools/analyze_debug_csv.py
bash -n tools/run_phase0_regression_matrix.sh
bash -n tools/run_standard_experiment.sh
bash -n tools/run_offline_localization.sh
python3 -m json.tool config/localization.json
```

均通过。

## 12. 新增风险和副作用

- target confirmation guard 默认关闭，仍是实验策略；过严可能导致更早 LOST。
- smoother guard 默认关闭；开启时可以防止 smoother 污染，但可能冻结 pose。
- `publish_pose_during_hold` 当前作为配置占位，尚未做到 publisher 层 status/pose 分离；下一轮应在 publisher 中正式处理。
- 回归矩阵脚本使用本地 bag/map 时仍依赖显式参数或环境变量，不写默认配置。

## 13. 下一轮建议

建议继续 Phase 0.x，不建议直接进入 Phase 1。

优先任务：

1. 调查 frame 252 后 `relocalization_no_candidates`：
   - 输出 query descriptor health；
   - 输出 top-k descriptor distance；
   - 区分候选为空和候选被过滤。
2. 设计正式 LOST tracking pause / smoother reset：
   - 替代实验性 hold prior；
   - 支持 status 继续发布，但 pose/odom 降级。
3. 分析 frame 132/133/138/139 的 high score + large delta：
   - 判断是否为 target set 切换、IMU prediction 漂移或地图坐标语义问题。
4. 尝试更可靠 initial pose：
   - 当前 `0 0 0 0 0 0` 只能支撑 10s smoke。
5. 为 relocalization 几何验证补充 per-candidate debug CSV 或 JSON。

## 14. Git diff 摘要

本轮报告生成前的工作区仍包含 Phase 0/0.5/0.6 未提交内容。

`git diff --stat` 摘要：

```text
CMakeLists.txt                                     |   7 +
README.md                                          | 117 +++++-
config/config.json                                 |   2 +-
config/localization.json                           |  25 +-
docs/engineering_playbook.md                       |  35 ++
include/glim_localization/core/localization_options.hpp | 27 ++
modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp | 456 ++++++++++++++++++++-
modules/odometry/localization_cpu/odometry_estimation_localization_cpu.hpp | 72 ++++
package.xml                                        |   9 +-
src/glim_localization/core/localization_options.cpp | 29 ++
tools/run_offline_localization.sh                  | 130 +++++-
tools/run_standard_experiment.sh                   | 378 +++++++++++++++--
```

新增未跟踪文件包括 Phase 0.7 报告、`tools/analyze_debug_csv.py`、`tools/run_phase0_regression_matrix.sh` 和 `docs/phase0_regression_matrix.md`。
