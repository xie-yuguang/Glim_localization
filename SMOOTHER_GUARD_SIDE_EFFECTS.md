# Smoother Guard Side Effects

## 当前实现

当前 smoother guard 仍采用 Phase 0.6 的方案 B：`hold_last_pose_prior`。

Phase 0.7 增加了安全边界配置：

```json
"smoother_guard": {
  "enable": false,
  "mode": "hold_last_pose_prior",
  "max_lost_frames_without_prior": 0,
  "max_consecutive_hold_priors": 30,
  "write_trajectory_during_hold": false,
  "publish_pose_during_hold": false
}
```

说明：

- 默认关闭。
- hold prior 触发时状态仍是 LOST/RELOCALIZING，不会伪装成 TRACKING。
- `write_trajectory_during_hold=false` 时，hold frame 不写入轨迹文件。
- ROS publisher 仍通过 custom data 获得 LOST 状态；`publish_pose_during_hold` 当前作为配置占位，尚未做到 status 与 pose 分离发布。

## Phase 0.7 30s 观测

30s best 实验：

- CSV frames：301
- trajectory frames：251
- smoother hold prior：50
- 最长 hold/LOST 区间：252..300，49 frames
- GTSAM warning：0
- trajectory max step：1.977m
- suspicious divergence：false

对比只开 target confirmation guard 的 30s：

- trajectory frames：301
- LOST：129
- trajectory max step：17.559m
- suspicious divergence：true

## 副作用判断

`hold_last_pose_prior` 是安全保护，不是定位成功。

它的好处：

- 避免 LOST/RELOCALIZING 且 no candidates 时持续污染 fixed-lag smoother。
- 防止轨迹继续跟随无约束预测漂移。
- 本轮短片段 GTSAM warning 为 0。

它的风险：

- 如果继续发布 pose，可能让下游误解为系统仍在正常定位。
- 如果持续写 trajectory，会制造“冻结但看似连续”的轨迹。
- hold prior 本质是在优化器里固定当前 X 到 last accepted pose，不是 sensor-based correction。
- 长期依赖 hold prior 会掩盖 relocalization candidates=0 的根因。

Phase 0.7 已通过 `write_trajectory_during_hold=false` 降低轨迹误导风险：CSV 仍记录每个 frame，trajectory 不再把 hold pose 当作普通定位结果写出。

## 是否建议长期保留

建议短期保留为默认关闭的实验安全阀，不建议作为长期正式策略。

长期正式策略应改为：

1. LOST/RELOCALIZING 无有效 prior 时，不插入 localization pose prior。
2. 如果上游 GLIM 允许，应支持跳过当前 frame 的 smoother update，或进入明确的 tracking-paused frame insertion 路径。
3. 重定位成功后执行小窗口 reset/reseed，而不是在旧窗口上长时间 hold。
4. ROS 输出分离 status 和 pose：LOST 状态继续发布，pose/odom 可选择停止、降级或带 covariance/status flag 发布。

## 如果要做 skip frame，需要改哪里

不改上游时，方案 A 不安全。原因是 `glim::OdometryEstimationIMU::insert_frame()` 已经创建 `EstimationFrame`、插入 `new_values`，并假设后续 frame 仍能引用前序 `X/V/B` key。

正式 skip frame 需要改造：

- `glim/src/glim/odometry/odometry_estimation_imu.cpp`
  - 在 subclass hook 返回前支持“不 update smoother / 不推进 fixed-lag key”的语义；
  - 或增加 `FrameInsertionDecision`。
- `modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp`
  - LOST no-candidate 时返回 skip decision；
  - 重定位成功后重建局部窗口或 reset smoother anchor。
- `modules/extension/localization_publisher/localization_publisher.cpp`
  - 明确发布 LOST status，但停止发布普通 pose/odom 或标记 degraded covariance。

## 下一阶段推荐方案

下一轮不应继续扩大 hold prior，而应实现正式的 “LOST tracking pause / relocalization reset” 设计草案和最小原型。hold prior 继续作为实验 guard，用于避免长期 bag 调试时 GTSAM underconstrained crash。
