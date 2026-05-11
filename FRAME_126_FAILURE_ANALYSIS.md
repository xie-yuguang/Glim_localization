# Frame 126 Failure Analysis

数据来源：

- `/tmp/glim_localization_phase0_5_debug/c0_origin_15s_offset0_dur15/monitor/command.stdout.log`
- 本轮在 `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp` 中增强了 accepted/rejected registration 日志字段。

## 1. frame 100-126 摘要

frame 100-125 均为 `TRACKING` 下的 accepted scan-to-map：

- backend：`cpu_gicp`
- active_submaps：`[0,1,217,218,2,219,216,220]`
- source_points：约 `3359-3862`
- target_points：`400000`
- score：约 `0.861-0.944`
- delta_t：多数小于 `1.0m`，但 frame 125 已到 `1.014m`
- residual 在 frame 122/125 出现约 `4e4` 的尖峰

frame 126 前发生 target map rebuild：

```text
rebuilt localization target map: center=(2.072, 0.691, -1.333)
active_submaps=[3,2,216,4,215,1,0,217]
```

frame 126 rejection：

```text
scan-to-map registration rejected frame=126 state=TRACKING backend=cpu_gicp
reason=large_translation_correction score=0.935/0.350
residual=26233.526020/1000000.000000
inliers=3730/30 source_points=3988 target_points=400000
delta_t=5.015/3.000 delta_r=0.142/0.700
active_submaps=[3,2,216,4,215,1,0,217]
```

## 2. frame 127-140 摘要

- frame 127/128 又被 accepted，但 `delta_t` 已接近阈值：`2.785m`、`2.815m`。
- frame 129-134 多次 target map rebuild，中心快速移动：
  - `(5.063, 0.149, -1.817)`
  - `(7.008, 0.636, -1.996)`
  - `(9.113, 1.333, -1.892)`
- frame 135-137 连续三次 `large_translation_correction`：
  - frame 135：`delta_t=5.752/3.000`
  - frame 136：`delta_t=6.507/3.000`
  - frame 137：`delta_t=8.944/3.000`
- frame 137 触发 `DEGRADED -> LOST -> RELOCALIZING`，随后 relocalization verified 到 `submap=220`。
- frame 140 再次 `large_translation_correction`，`delta_t=3.304/3.000`。

## 3. 判断

最可能原因：**scan-to-map 错误匹配和 target map 切换共同触发发散**。

证据：

- frame 126 的 score/inliers 都很高，只有 pose correction translation 超阈值，说明 GICP 找到了“看起来质量高”的匹配，但匹配结果要求一次性修正 `5.015m`。
- frame 126 正好紧邻一次 target map rebuild，active submap 集合从 `[0,1,217,218,2,219,216,220]` 切到 `[3,2,216,4,215,1,0,217]`。
- frame 127/128 被接受但 delta_t 已接近阈值，随后 target center 快速跳变，进入连续 rebuild/rejection/relocalization 循环。
- 10 秒片段无 rejection，15 秒片段已出现 13 次 rejection，失败窗口非常集中。

次要可能：

- IMU prediction 漂移可能参与放大，但当前日志没有直接输出 predicted pose 与 corrected pose 的绝对位置序列，只能通过 delta_t 间接判断。
- gating 阈值本身不是主要证据点；阈值阻止了 frame 126 的 5m correction，但后续 frame 127/128 的 2.8m correction 被接受，可能已经足以把状态推到不稳定区域。

证据不足：

- 没有每帧 predicted pose、corrected pose、target center 的 CSV。
- 没有每帧 registration timing。
- 没有 IMU integrated count / prediction covariance 的同一行日志。

## 4. 建议

- P0：保留 0-15s 作为最小失败复现。
- P1：下一轮增加默认关闭的 debug CSV，记录 predicted/corrected pose、target center、active_submaps、registration_ms。
- P1：只做 guard proposal，不立即调阈值：考虑 target map rebuild 后，如果 correction 接近阈值且 target set 改变，进入 DEGRADED 前要求连续确认或触发 relocalization。
