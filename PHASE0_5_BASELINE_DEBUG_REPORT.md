# Phase 0.5 Baseline Debug Report

## 1. 执行摘要

本轮完成 Phase 0.5：Baseline Debug & Failure Artifact Hardening。未修改 `koide3/glim`、`glim_ros2`、`glim_ext` 上游源码，未重写 localization pipeline，未默认启用 GPU。

结论：

- `run_standard_experiment.sh` 已改为主 localization 命令失败后仍继续生成 artifacts。
- 失败或 timeout 后总是生成 `experiment_record.md`、`failure_summary.md`、stdout/stderr tail、状态迁移摘要、registration rejection 摘要。
- `monitor_ps.sh` 已从单 PID 采样改为进程树聚合采样，summary 中能看到 `bash, timeout, ros2, glim_rosbag`。
- `run_offline_localization.sh` / `run_standard_experiment.sh` 已支持 `--start-offset`、`--duration`、`--timeout-sec`；`--max-frames` 暂时只记录请求，因为当前 `glim_rosbag` 没有逐帧停止参数。
- 找到一个临时 10 秒 CPU smoke baseline：`initial_pose="0 0 0 0 0 0"`、`duration=10`，101 帧、无 LOST、无 rejection、无 GTSAM warning。
- 没有找到可稳定通过 30 秒的 initial pose；所有 30 秒候选均发散或崩溃。
- frame 126 的最可能原因是 target map rebuild 后 scan-to-map 匹配给出高 score 但大平移修正，随后 target map 快速跳变并进入 LOST/RELOCALIZING 循环。
- GTSAM underconstrained 最可能与长期 LOST/RELOCALIZING 无有效 pose prior 时仍持续向 fixed-lag smoother 添加新 pose 变量相关。

## 2. 本轮修改列表

修改文件：

- `tools/run_standard_experiment.sh`
  - 主命令失败后继续收尾。
  - 新增 `logs/`、`failure_summary.md`、`stdout_tail.txt`、`stderr_tail.txt`、状态迁移和 rejection 摘要。
  - 新增 `--start-offset`、`--duration`、`--timeout-sec`、`--max-frames` 参数。
- `tools/run_offline_localization.sh`
  - 透传 `start_offset` 和 `playback_duration` 给 `glim_rosbag`。
  - `timeout-sec` 使用 GNU `timeout` 包裹 `ros2 run glim_ros glim_rosbag`。
  - 将整数参数规范化为 ROS2 double 参数，例如 `0 -> 0.0`。
- `tools/monitor/monitor_ps.sh`
  - 从 root PID 递归采样子进程，聚合 CPU/RSS/threads/IO。
- `tools/monitor/run_with_time.sh`
  - 记录初始和结束进程树快照。
- `tools/monitor/summarize_usage.py`
  - 输出 `ps_sampling_scope=process_tree`、process count、observed commands。
- `tools/plot_trajectory.py`
  - 输出 `max_step_m`、`span_xyz`、`suspicious_divergence`。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
  - 增强 accepted/rejected scan-to-map 日志字段：state、source/target points、active submaps。
- `README.md`、`docs/engineering_playbook.md`
  - 增加短片段 baseline 和失败 artifact 文档。

新增报告：

- `candidate_initial_poses.md`
- `SHORT_BASELINE_SWEEP_REPORT.md`
- `FRAME_126_FAILURE_ANALYSIS.md`
- `GTSAM_UNDERCONSTRAINED_INVESTIGATION.md`
- `PHASE0_5_BASELINE_DEBUG_REPORT.md`

## 3. 工作区状态

本轮开始时工作区已有上一轮未提交改动和未跟踪分析文档，因此没有创建分支、没有自动 commit。

建议提交清单：

- Phase 0 工程清理：`.gitignore`、`LICENSE`、`config/localization.*.json`、`launch/localization.launch.py`、`package.xml`、`CMakeLists.txt`、`README.md`。
- Phase 0.5 脚本与诊断：`tools/run_standard_experiment.sh`、`tools/run_offline_localization.sh`、`tools/monitor/*`、`tools/plot_trajectory.py`、`modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`、`docs/engineering_playbook.md`。
- 分析报告：建议后续移动到 `docs/analysis/` 后提交，包括 `GLIM_LOCALIZATION_ANALYSIS.md`、`PHASE0_BASELINE_HARDENING_REPORT.md`、本轮 5 个报告等。
- `tools/monitor/__pycache__/*.pyc` 删除应提交，避免继续跟踪生成物。

## 4. 实验脚本失败收尾增强

验收结果：

- `run_standard_experiment.sh` 在主命令 exit code 非 0 时仍继续生成：
  - `experiment_record.md`
  - `failure_summary.md`
  - `logs/stdout_tail.txt`
  - `logs/stderr_tail.txt`
  - `logs/state_transition_summary.txt`
  - `logs/registration_rejection_summary.txt`
- `c3_submap70` exit code `250` 且无 trajectory，也成功生成上述失败 artifacts。

exit code 解释已写入 `failure_summary.md`：

- `124`：GNU timeout 达到时间限制。
- `143`：进程收到 SIGTERM，通常来自 timeout 或父进程终止。
- 其他非零：程序错误或异常退出，需要看 stderr tail。

## 5. 资源监控 PID 修正

修正前：

- `avg_cpu_pct_ps` 只代表 wrapper bash PID，不能反映实际 `glim_rosbag`。

修正后：

- `monitor_ps.sh` 每次采样递归收集 root PID 的 children。
- `summary.json` / `summary.txt` 包含：
  - `ps_sampling_scope: process_tree`
  - `avg_process_count_ps`
  - `peak_process_count_ps`
  - `observed_commands`

实测 30 秒片段 observed commands：

```text
bash, glim_rosbag, ros2, timeout
```

## 6. 短片段 baseline 支持

确认 `glim_rosbag` 已有 ROS 参数：

- `start_offset`
- `playback_duration`
- `playback_until`
- `end_time`

本轮透传：

- `--start-offset SEC` -> `-p start_offset:=SEC.0`
- `--duration SEC` -> `-p playback_duration:=SEC.0`
- `--timeout-sec SEC` -> GNU `timeout`

`--max-frames`：

- 当前只记录到 manifest 和 stdout/stderr。
- 原因：`glim_rosbag` 当前没有 max frames 参数；本轮不改上游。

## 7. initial pose 候选来源和结论

候选详见 `candidate_initial_poses.md`。

最重要结论：

- bag 起始时间 `1743058060.351172527` 和 map 轨迹起始时间 `1744772001.560494184` 不一致，无法按时间戳直接对齐。
- `c0_origin` 与 `c1_map_first` 都可以撑过 10 秒，但不能撑过 30 秒。
- `c3_submap70` 明显不是 bag 起点，早期就进入 LOST 并触发 GTSAM exception。

## 8. 短片段 baseline sweep 结果

详见 `SHORT_BASELINE_SWEEP_REPORT.md`。

当前可用：

- 10 秒 smoke baseline：
  - output：`/tmp/glim_localization_phase0_5_debug/c0_origin_10s_offset0_dur10`
  - frames：`101`
  - total_distance：`6.962m`
  - max_step：`0.302m`
  - LOST：`0`
  - registration rejection：`0`
  - GTSAM warning：`0`

当前不可用：

- 所有 30 秒候选均不能作为稳定 baseline。
- `c0_origin` 30 秒输出 301 帧，但 end xyz 到 `(-714.804, -121.010, -1524.424)`，严重发散。

## 9. frame 126 失败分析

详见 `FRAME_126_FAILURE_ANALYSIS.md`。

核心证据：

```text
frame=126 reason=large_translation_correction
score=0.935/0.350 residual=26233.526020/1000000.000000
inliers=3730/30 source_points=3988 target_points=400000
delta_t=5.015/3.000 delta_r=0.142/0.700
active_submaps=[3,2,216,4,215,1,0,217]
```

frame 126 前 target map 从 `[0,1,217,218,2,219,216,220]` rebuild 到 `[3,2,216,4,215,1,0,217]`。之后 frame 127/128 以接近阈值的 `2.8m` 修正被接受，随后 target center 快速跳变并进入连续 rejection。

判断：更像是 target map 切换后 scan-to-map 错误匹配被局部高 score 掩盖，而不是单纯初值错误。

## 10. GTSAM underconstrained 调查

详见 `GTSAM_UNDERCONSTRAINED_INVESTIGATION.md`。

核心判断：

- `x48`、`x1445`、`x2968` 都出现在 LOST/RELOCALIZING 长时间持续之后。
- 在 `attempt_relocalization()` 无 candidates 或 registration rejected 返回空 localization factors 时，上游 GLIM 仍会创建新的 `X/V/B` 状态并更新 smoother。
- 如果此时 IMU/速度/bias 约束不足，fixed-lag smoother 内的 pose 变量可能欠约束。

建议下一轮只做小 guard：

- LOST/RELOCALIZING 且 candidates=0 时跳过 smoother update 或停止向 smoother 添加新 pose 状态。
- 先加日志/CSV 验证，不直接大改 smoother。

## 11. 构建和测试结果

构建命令：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：

- 通过。
- `Summary: 1 package finished`

测试命令：

```bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：

- 通过。
- `100% tests passed, 0 tests failed out of 15`

## 12. 当前是否已有可用 baseline

有一个很短的 smoke baseline：

- `c0_origin 0-10s`
- 可用于验证构建、配置、map loading、短 rosbag 回放、trajectory 输出、资源报告和状态基本链路。

没有可用的 30 秒或完整 bag baseline。

## 13. 仍然阻塞的问题

- P0：30 秒定位仍会发散，不能作为稳定 baseline。
- P0：frame 126 后的 target map rebuild / scan-to-map correction 组合会快速推高 delta_t。
- P0：LOST/RELOCALIZING 长时间无候选时可能继续污染 smoother，触发 GTSAM underconstrained。
- P1：缺少默认关闭的 debug CSV，当前只能从日志抽取诊断字段。

## 14. 建议下一轮 Codex 任务

1. P0：实现默认关闭的 debug CSV，字段至少包含 frame、state、predicted/corrected pose、score、residual、inliers、source/target points、active submaps、delta_t/r、reject reason。
2. P0：实现 LOST/RELOCALIZING 无候选时的 smoother guard proposal，并用 0-15s 最小失败片段验证。
3. P1：增加 target map rebuild 后的保护策略实验：例如 target set 改变且 correction 接近阈值时要求二次确认。
4. P1：把 10s smoke baseline 纳入脚本文档和回归命令。

## 15. Git diff 摘要

当前 `git diff --stat` 摘要：

```text
CMakeLists.txt                                     |   4 +
README.md                                          |  58 +++-
config/config.json                                 |   2 +-
config/localization.json                           |   4 +-
docs/engineering_playbook.md                       |  16 ++
modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp | 13 +-
package.xml                                        |   9 +-
tools/monitor/monitor_ps.sh                        |  90 +++++--
tools/monitor/run_with_time.sh                     |  12 +
tools/monitor/summarize_usage.py                   |  27 ++
tools/plot_trajectory.py                           |  26 +-
tools/run_offline_localization.sh                  |  79 +++++-
tools/run_standard_experiment.sh                   | 293 +++++++++++++++++++--
```

另外仍有上一轮 staged pycache 删除和多个未跟踪报告/模板文件，详见 `git status --short`。
