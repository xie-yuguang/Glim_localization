# ROS 接口说明

本文档固定 `glim_localization` 当前对外 ROS 接口语义，作为部署、调试和长期维护的参考说明。

本文职责边界：

- 负责：topic、TF、frame、diagnostics 的稳定接口说明
- 不负责：算法细节、参数全表、首次快速上手

对应文档：

- 基线与契约：`docs/baseline_and_contract.md`
- 参数参考：`docs/config_reference.md`
- 工程化与回归：`docs/engineering_playbook.md`

## 1. 主要输出

标准输出 topic：

- `/localization/status`
- `/localization/diagnostics`
- `/localization/pose`
- `/localization/odom`
- `/localization/trajectory`
- `/localization/debug/input_scan`
- `/localization/debug/current_scan`
- `/localization/debug/local_target_map`
- `/localization/debug/active_submaps`

标准输入接口：

- `/initialpose`
- `/localization/relocalize`

TF：

- `map -> odom`
- `odom -> base_link`

## 2. frame 语义

- `map`
  - localization 固定参考系
  - `/localization/pose`
  - `/localization/trajectory`
  - debug target map

- `odom`
  - 连续性层
  - P2 起，relocalization 成功后允许调整 `T_map_odom`

- `base_link`
  - `/localization/odom.child_frame_id`
  - `odom -> base_link` 的 child

- `lidar`
  - 输入 scan 的传感器 frame
  - 用于 `/localization/debug/input_scan`

## 3. topic 语义

`/localization/status`

- 类型：`std_msgs/msg/String`
- 作用：兼容式简洁状态输出
- 内容：状态名、状态原因、score、reject reason 的简短文本

`/localization/diagnostics`

- 类型：`diagnostic_msgs/msg/DiagnosticArray`
- 作用：结构化 runtime 观测
- 适合：
  - 调试失锁
  - 观察 relocalization
  - 观察 recovery 稳定期
  - 记录 continuity 调整

关键字段包括：

- `status`
- `reason`
- `matching_score`
- `backend`
- `score_type`
- `reject_reason`
- `consecutive_rejections`
- `relocalization_message`
- `relocalization_attempts`
- `relocalization_candidate_count`
- `relocalization_verified_rank`
- `recovery_frames_remaining`
- `stable_tracking_successes`
- `descriptor_distance`
- `pose_delta_translation`
- `pose_delta_angle`
- `continuity_adjusted`
- `continuity_translation`
- `continuity_angle`
- `active_submaps`

`/localization/pose`

- 类型：`geometry_msgs/msg/PoseStamped`
- frame：`map`
- 语义：当前 `T_map_imu`

`/localization/odom`

- 类型：`nav_msgs/msg/Odometry`
- frame：`odom`
- child：`base_link`
- 语义：当前 `T_odom_base`

`/localization/trajectory`

- 类型：`nav_msgs/msg/Path`
- frame：`map`
- 语义：累计定位轨迹

`/localization/debug/input_scan`

- 类型：`sensor_msgs/msg/PointCloud2`
- frame：`sensor_frame`
- 语义：输入 scan 原始视角

`/localization/debug/current_scan`

- 类型：`sensor_msgs/msg/PointCloud2`
- frame：`map`
- 语义：当前用于定位的 scan，已对齐到 map

`/localization/debug/local_target_map`

- 类型：`sensor_msgs/msg/PointCloud2`
- frame：`map`
- 语义：active submaps 合并后的局部目标地图

`/localization/debug/active_submaps`

- 类型：`visualization_msgs/msg/Marker`
- frame：`map`
- 语义：文本 marker，显示状态、reason、score、backend、恢复倒计时和 active submap ids

## 4. 状态语义

当前稳定状态集：

- `WAIT_MAP`
- `WAIT_INITIAL_POSE`
- `INITIALIZING`
- `DEGRADED`
- `TRACKING`
- `LOST`
- `RELOCALIZING`
- `RECOVERING`

推荐理解方式：

- `INITIALIZING`
  - 拿到初始位姿后，但尚未建立稳定定位

- `DEGRADED`
  - 已经 tracking 过，但当前帧 registration 被拒绝，尚未 LOST

- `RELOCALIZING`
  - 正在做候选检索与几何验证

- `RECOVERING`
  - relocalization 已成功，但仍在稳定期

## 5. 调试建议

排查时优先组合观察：

- `/localization/status`
- `/localization/diagnostics`
- `/localization/odom`
- `/localization/trajectory`
- `/localization/debug/current_scan`
- `/localization/debug/local_target_map`
- `/localization/debug/active_submaps`

常见判断方式：

- `DEGRADED` 增多：看 `reject_reason`
- `LOST -> RELOCALIZING` 频繁：看 `relocalization_candidate_count`
- `RECOVERING` 长时间不结束：看 `recovery_frames_remaining`
- 轨迹有全局跳变但 odom 较平滑：看 `continuity_adjusted`
