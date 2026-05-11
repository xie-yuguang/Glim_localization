# GTSAM Underconstrained Investigation

## 1. 现象

Phase 0 完整 bag stderr：

```text
Indeterminant linear system detected while working near variable
8646911284551353765 (Symbol: x1445).

Indeterminant linear system detected while working near variable
8646911284551355288 (Symbol: x2968).
```

Phase 0.5 中 `c3_submap70 0-30s` 更快复现：

```text
Indeterminant linear system detected while working near variable
8646911284551352368 (Symbol: x48).
```

`c3_submap70` 随后 `ros2run` abort，run exit code 为 `250`。

## 2. 对应状态

`c3_submap70` 在 frame 18 后进入 LOST/RELOCALIZING：

- frame 11：`large_translation_correction`
- frame 16/17/18：连续 rejection，frame 18 进入 `LOST -> RELOCALIZING`
- frame 18：relocalization 成功到 `submap=0`
- frame 19/20/21：再次 rejection，frame 21 后持续 `RELOCALIZING -> LOST reason=relocalization_no_candidates`
- GTSAM warning 对应 `Symbol: x48`，大致发生在持续 LOST/RELOCALIZING 且无候选阶段

Phase 0 完整 bag 中 `x1445` / `x2968` 也出现在长期 LOST/RELOCALIZING 后，说明它不是 frame 126 首次 rejection 的直接结果，而是失稳后长期继续往 smoother 插入新状态时触发。

## 3. 代码路径

涉及文件：

- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
- 上游 GLIM：`glim/src/glim/odometry/odometry_estimation_imu.cpp`

关键路径：

- `OdometryEstimationIMU::insert_frame()` 每帧都会创建 `X(current)`、`V(current)`、`B(current)`。
- localization 覆写 `create_factors()`：
  - 正常 scan-to-map accepted：`add_graph_injection_factors(..., include_between_factor=true)`，添加 pose prior 和 pose between。
  - relocalization success：`add_graph_injection_factors(..., include_between_factor=false)`，只添加当前 pose prior。
  - registration rejected 但未超过连续阈值：返回空 localization factors。
  - LOST/RELOCALIZING 且 candidates=0：返回空 localization factors。
- 上游仍会添加 IMU / bias / velocity 相关因子；如果 IMU 约束、速度约束和 bias 约束不足以稳定 pose，fixed-lag smoother 可能在某个 `X(current)` 变成欠约束。

## 4. 可能根因

最可能根因：**LOST/RELOCALIZING 长时间无有效 scan-to-map pose prior 时，系统仍持续向 fixed-lag smoother 添加新 pose 变量，部分窗口内 pose 约束不足。**

次要因素：

- 错误初值或错误匹配导致状态快速远离地图，relocalizer 后续 candidates=0。
- rejected registration 后没有向 smoother 注入 pose prior/between；这是合理的安全选择，但需要配合 LOST 状态下的 smoother guard。
- relocalization success 使用 `include_between_factor=false`，可以避免错误 continuity 约束，但在频繁 LOST/RECOVERING 时可能让局部窗口更依赖单帧 prior。

## 5. 建议修复方案

本轮不实现大改，仅建议下一轮小型 guard：

1. P0：在 LOST/RELOCALIZING 且 relocalization candidates=0 时，不再把该帧推入 smoother，或跳过 smoother update，仅发布 LOST 状态。
2. P0：对 registration rejected 且即将进入 LOST 的 frame，避免继续使用未被 scan-to-map 修正的 predicted pose 作为下一帧强初值。
3. P1：在 `attempt_relocalization()` 返回空 factors 时记录 frame id、state、num_imu_integrated、是否新增 X/V/B、new_factors size。
4. P1：增加 debug CSV，便于把 GTSAM symbol `xN` 映射回 frame `N` 的状态和 registration 结果。

## 6. 是否建议下一轮实现

建议下一轮实现第 1 条或第 3 条中的最小版本。当前已有证据足够说明 warning 与长期 LOST/RELOCALIZING 后继续 smoother update 有强相关，但还不足以直接大改 smoother 结构。
