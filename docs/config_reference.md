# glim_localization 配置参数参考

本文档基于当前工作区实际代码整理，只记录当前已经存在并被解析的配置项。主要依据：

- `glim_localization/src/glim_localization/core/localization_options.cpp`
- `glim_localization/include/glim_localization/core/localization_options.hpp`
- `glim_localization/include/glim_localization/initialization/initial_pose_options.hpp`
- `glim/src/glim/odometry/odometry_estimation_imu.cpp`
- `glim_ros2/src/glim_ros/glim_ros.cpp`
- `glim_ros2/src/glim_rosbag.cpp`
- `glim_localization/config/localization.json`

说明：

- 路径使用 JSON 路径表示，例如 `localization.matching.method`。
- “默认值”来自当前 C++ 代码的默认值，而不是只看示例 JSON。
- “是否必填”表示代码是否没有默认值就无法正常运行。很多参数代码有默认值，但为了 localization-only 正常工作，仍建议显式配置。
- 如果你的旧配置里还保留顶层 `enable_local_mapping`、`enable_global_mapping`，当前 localization 代码不会读取它们；真正被 `glim_ros2` 读取的是 `glim_ros.enable_local_mapping` 和 `glim_ros.enable_global_mapping`。

本文职责边界：

- 负责：当前真实代码已解析参数的权威参考
- 不负责：完整部署教程、最短闭环步骤、FAQ 式排障

对应文档：

- 基线与契约：`docs/baseline_and_contract.md`
- 最小闭环：`docs/quick_start.md`
- 完整部署：`docs/deployment_and_run.md`
- 常见问题：`docs/faq.md`

## 1. odometry_estimation

这些参数位于 `odometry_estimation` 节点。`so_name` 由 `glim_ros2` 读取并用于加载 odometry 动态库；其余参数由 GLIM 的 `OdometryEstimationIMUParams` 基类读取。`OdometryEstimationLocalizationCPUParams` 会在检测到 localization 初始位姿后覆盖一部分初始化行为。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `odometry_estimation.so_name` | string | `libodometry_estimation_cpu.so` | 是，localization 必填 | GLIM odometry 动态模块库名。由 `GlimROS` 调用 `OdometryEstimationBase::load_module()` 加载。 | `libodometry_estimation_localization_cpu.so` | 如果不设置为 localization 模块，会加载 GLIM 原始 odometry，无法执行 fixed-map localization。 |
| `odometry_estimation.fix_imu_bias` | bool | `false` | 否 | 是否固定 IMU bias。 | `false` | 影响 IMU smoother 中 bias 是否参与估计。 |
| `odometry_estimation.initialization_mode` | string | `LOOSE` | 否 | GLIM IMU odometry 初始状态估计模式。 | 当前示例为 `LOOSE`；localization 初始位姿有效时会被模块改为 `NAIVE` | 当 `localization.initial_pose.source` 为 `config` 或 `topic` 时，localization 模块会设置 `initialization_mode = "NAIVE"` 并使用给定初始位姿。 |
| `odometry_estimation.init_T_world_imu` | Isometry3d，通常为 `[x,y,z,qx,qy,qz,qw]` | identity | 否 | GLIM 原始 odometry 的初始 IMU 位姿。 | localization 流程中不建议使用，改用 `localization.initial_pose` | 当前 localization 模块通常会用 `localization.initial_pose` 覆盖初始位姿；只有异常配置路径下才可能体现基类默认行为。 |
| `odometry_estimation.init_v_world_imu` | vector<double>[3] | `[0,0,0]` | 否 | 初始 IMU 速度。 | `[0,0,0]` | localization 模块设置初始位姿时会把初始速度置零。 |
| `odometry_estimation.init_pose_damping_scale` | double | `1e10` | 否 | GLIM 初始位姿 damping scale。 | 保持默认 | 仅影响 GLIM 基类初始化阶段的 damping 行为。 |
| `odometry_estimation.smoother_lag` | double | `5.0` | 否 | fixed-lag smoother 时间窗口长度。 | `5.0` | 值越大，优化窗口越长，计算量越大，轨迹平滑性可能更好。 |
| `odometry_estimation.use_isam2_dogleg` | bool | `false` | 否 | 是否使用 ISAM2 Dogleg 优化参数。 | `false` | 改变 smoother 的非线性优化策略。 |
| `odometry_estimation.isam2_relinearize_skip` | int | `1` | 否 | ISAM2 重新线性化跳步。 | `1` | 值越大可能降低计算量，但响应变慢。 |
| `odometry_estimation.isam2_relinearize_thresh` | double | `0.1` | 否 | ISAM2 重新线性化阈值。 | `0.1` | 影响 smoother 重新线性化频率。 |
| `odometry_estimation.validate_imu` | bool | `true` | 否 | 是否启用 IMU 数据检查。 | `true` | 可过滤或警告异常 IMU 数据。 |
| `odometry_estimation.save_imu_rate_trajectory` | bool | `false` | 否 | 是否保存 IMU rate trajectory。 | `false` | 当前 localization 最小闭环不依赖该输出。 |
| `odometry_estimation.num_threads` | int | `4` | 否 | GLIM 前端协方差估计等处理线程数。 | 依据 CPU 核数设置，常用 `4` | 影响预处理和 per-factor 并行计算性能。 |

## 2. localization

这些参数由 `LocalizationOptions::load()` 从 `config_localization` 指向的 JSON 中读取。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.map_path` | string | `""` | 是 | GLIM dump 地图目录。 | 使用绝对路径，例如 `/data/maps/glim_dump` | 为空时地图不会加载，状态停留在 `WAIT_MAP`，无法完成 localization。 |
| `localization.map.load_voxelmaps` | bool | `true` | 否 | 加载完成后是否保留 submap voxelmaps。 | `true` | 当前 runtime 主流程不直接依赖 voxelmaps；设为 `false` 可减少内存占用。 |
| `localization.map.load_raw_frames` | bool | `false` | 否 | 加载完成后是否保留 submap 内部 `frames/odom_frames`。 | `false` | localization-only 运行通常只需要 merged submap frame；设为 `false` 可减少内存占用。 |
| `localization.map.strict` | bool | `true` | 否 | 地图格式检查和 submap 加载失败时是否严格失败。 | `true` | 为 `true` 时任何关键错误直接返回加载失败；为 `false` 时可跳过坏 submap 继续尝试加载。 |
| `localization.trajectory_path` | string | `/tmp/glim_localization_traj.txt` | 否 | localization 轨迹输出文件。 | `/tmp/glim_localization_traj.txt` 或项目指定路径 | 非空且可写时，每帧输出 `stamp x y z qx qy qz qw status score`。 |
| `localization.map_frame` | string | `map` | 否 | ROS pose 输出和 debug target map 的 map frame 名。 | `map` | 影响 `/localization/pose`、`/localization/target_map` 和 `map -> odom` TF frame id。 |
| `localization.odom_frame` | string | `odom` | 否 | ROS odometry frame 名。 | `odom` | 影响 `/localization/odom` 和 TF。 |
| `localization.base_frame` | string | `base_link` | 否 | ROS odometry child frame 名。 | `base_link` | 影响 `/localization/odom.child_frame_id` 和 `odom -> base_link` TF。当前实现实际使用 IMU 位姿填充该 transform。 |
| `localization.sensor_frame` | string | `lidar` | 否 | 传感器 frame 名预留配置。 | `lidar` | 当前主要作为配置项保留，核心输出更多使用 `map_frame/odom_frame/base_frame`。 |

当前坐标系约定：

- `world == map`
- `frame->T_world_imu == T_map_imu`
- `frame->T_world_lidar == T_map_lidar`
- `T_map_odom` 当前默认为 identity，并为后续 reset/relocalization 策略预留。

P1 起，`glim_localization_map_info` 和运行时地图加载日志会额外输出：

- `detected_format`
- `compatibility`
- `loaded/requested/skipped submaps`
- `merged_submap_points`
- 地图 origin bounds

这部分信息用于基础兼容性检查和地图健康诊断，不代表已经引入完整的地图版本迁移机制。

## 3. initial_pose

这些参数位于 `localization.initial_pose`。由 `LocalizationOptions::load()` 读取，再由 `ConfigInitialPoseProvider` 和 `OdometryEstimationLocalizationCPU` 使用。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.initial_pose.source` | string | `config` | 否，但建议显式配置 | 初始位姿来源。当前代码识别 `config` 和 `topic`。 | 离线 rosbag 用 `config`；在线交互可用 `topic` | `config` 时直接使用 `xyz/rpy` 初始化；`topic` 时等待 `/initialpose`，未收到前不插入 frame。 |
| `localization.initial_pose.xyz` | vector<double>[3] | `[0,0,0]` | source 为 `config` 时建议必填 | 初始 `T_map_imu` 平移，单位米。 | 根据地图坐标系填真实初值 | 初值偏差大时，nearby submap 查询和 scan-to-map registration 容易失败。 |
| `localization.initial_pose.rpy` | vector<double>[3] | `[0,0,0]` | source 为 `config` 时建议必填 | 初始 `T_map_imu` 欧拉角，顺序 roll、pitch、yaw，单位弧度。 | 根据地图坐标系填真实初值 | 旋转初值偏差大时，registration 容易被 gating 拒绝。 |

行为细节：

- `source == "config"` 时，`ConfigInitialPoseProvider::ready()` 返回 true。
- `source == "topic"` 时，模块会等待 runtime initial pose。topic 名来自 `localization.ros.initial_pose_topic`。
- 其他字符串当前不会被特殊处理，会导致 config provider 不 ready；如果也不是 `topic`，模块会持续等待初始位姿。
- P1 起，config / runtime 两种初始位姿在进入 `INITIALIZING` 前都会做一次附近 submap 诊断，并输出候选 submap 数量、最近距离和候选 ID。

## 4. target_map

这些参数位于 `localization.target_map`，用于从固定 `LocalizationMap` 中选择当前帧附近的 active submaps，并决定是否复用上一帧 target map。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.target_map.max_num_submaps` | int | `8` | 否 | 每次 target map 最多包含的 submap 数量。 | `4` 到 `12`，默认 `8` | 太小可能目标地图不足；太大 registration 变慢。 |
| `localization.target_map.max_distance` | double | `40.0` | 否 | nearby submap 查询最大距离，单位米。 | 中小场景 `30-60` | 太小可能查不到地图；太大可能引入无关 submaps。 |
| `localization.target_map.update_distance` | double | `2.0` | 否 | 平移超过该阈值才重建 target map，单位米。 | `1.0-3.0` | 值小会频繁重建；值大可能 target map 滞后。 |
| `localization.target_map.update_angle` | double | `0.2` | 否 | 旋转超过该阈值才重建 target map，单位弧度。 | `0.1-0.3` | 值小会频繁重建；值大在转弯时可能目标地图更新慢。 |
| `localization.target_map.use_submap_index` | bool | `true` | 否 | 是否构建并使用 `SubmapIndex`。 | 大多数场景保持 `true` | 大地图下可加速 nearby query；关闭后使用线性搜索。 |
| `localization.target_map.index_resolution` | double | `20.0` | 否 | submap index 栅格分辨率，单位米。 | 接近 submap 间距或查询半径的一半，默认 `20.0` | 太小 cell 多、索引开销变大；太大退化为大 cell 查询。 |

P1 起，target map 运行日志会区分三类行为：

- `reusing localization target map`
- `recentered localization target map`
- `rebuilt localization target map`

其中 `recentered` 表示本帧已经越过更新阈值，但新查询到的 active submap 集合与当前集合一致，此时只更新 target center，不重新替换 target map 对象。

## 5. matching

这些参数位于 `localization.matching`，用于 CPU GICP / 可选 GPU VGICP scan-to-map registration，以及 registration result gating。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.matching.method` | string | `cpu_gicp` | 否 | registration 后端选择。当前识别 `cpu_gicp` 和 `gpu_vgicp`。 | 默认 `cpu_gicp` | `gpu_vgicp` 仅在构建时启用 CUDA 且 gtsam_points 支持 CUDA 时可用，否则回退 CPU。 |
| `localization.matching.max_iterations` | int | `20` | 否 | registration 优化最大迭代次数。 | `10-30` | 过小可能不收敛；过大增加耗时。 |
| `localization.matching.min_score` | double | `0.35` | 否 | 接受匹配的最低 normalized acceptance score。CPU GICP 当前使用 inlier fraction，GPU VGICP 当前使用 residual 派生的置信度分数。 | `0.3-0.6` | 过高容易拒绝，过低可能接受错误匹配。 |
| `localization.matching.min_inliers` | int | `30` | 否 | 接受匹配的最低 inlier 数。 | 根据点云密度设置，默认 `30` | 点云很稀疏时可适当降低。 |
| `localization.matching.max_residual` | double | `1000000.0` | 否 | 接受匹配的最大 residual。`<=0` 时相当于不启用上限检查。 | 初期可保持默认 | 过小会导致大量 `high_residual` rejection。 |
| `localization.matching.max_pose_correction_translation` | double | `3.0` | 否 | 单次 registration 允许的最大平移修正，单位米。 | `1.0-5.0` | 初值不准时太小会拒绝；太大可能接受跳变。 |
| `localization.matching.max_pose_correction_angle` | double | `0.7` | 否 | 单次 registration 允许的最大旋转修正，单位弧度。 | `0.3-0.8` | 初始 yaw 偏差大时可临时放宽；太大可能接受错误匹配。 |
| `localization.matching.max_consecutive_rejections` | int | `3` | 否 | 连续匹配拒绝次数阈值。 | `3` | 达到阈值后状态进入 `LOST`，并尝试重定位。 |
| `localization.matching.max_correspondence_distance` | double | `2.0` | 否 | CPU GICP correspondence 最大距离。 | `1.0-3.0` | 太小不易匹配；太大容易引入错误对应。 |
| `localization.matching.pose_prior_precision` | double | `1000.0` | 否 | 将 accepted registration 转成 GTSAM prior/between factor 时使用的 isotropic precision。 | `100-3000` | 越大越信任 scan-to-map 结果；过大可能压制 IMU 预测。 |
| `localization.matching.num_threads` | int | `4` | 否 | CPU GICP registration 线程数。 | 依据 CPU 核数设置，常用 `4` | 影响 CPU registration 性能。 |
| `localization.matching.vgicp_resolution` | double | `0.5` | 否 | GPU VGICP voxel map 基础分辨率。 | `0.3-1.0` | 仅 GPU VGICP 路径使用。 |
| `localization.matching.vgicp_voxelmap_levels` | int | `2` | 否 | GPU VGICP 多层 voxel map 层数。 | `2` | 仅 GPU VGICP 路径使用；层数增加会增加计算量。 |
| `localization.matching.vgicp_voxelmap_scaling_factor` | double | `2.0` | 否 | GPU VGICP 多层分辨率缩放因子。 | `2.0` | 仅 GPU VGICP 路径使用。 |

registration 接受条件来自当前代码：

- optimizer converged
- residual 有限且不超过 `max_residual`
- score 不低于 `min_score`
- inlier 数不低于 `min_inliers`
- pose correction translation 不超过 `max_pose_correction_translation`
- pose correction angle 不超过 `max_pose_correction_angle`

P1 起，CPU / GPU backend 共用同一套 gating 顺序和 reject reason 命名：

- `not_converged`
- `high_residual`
- `low_score`
- `few_inliers`
- `large_translation_correction`
- `large_rotation_correction`

## 6. ros output

这些参数位于 `localization.ros`，由 `LocalizationOptions::load()` 读取，并由 `modules/extension/localization_publisher/localization_publisher.cpp` 使用。注意：这些输出只有在 `glim_ros.extension_modules` 包含 `liblocalization_publisher.so` 且该库成功加载时才生效。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.ros.publish_tf` | bool | `true` | 否 | 是否发布 TF。 | 在线调试保持 `true` | 发布 `map -> odom` 和 `odom -> base_link`。 |
| `localization.ros.publish_debug_target_map` | bool | `true` | 否 | 是否发布 debug target map 点云。 | 调试时 `true`，性能敏感时可设 `false` | 为 true 且 topic 有订阅者时，会合并 active submap 点云并发布，可能增加计算量。 |
| `localization.ros.publish_diagnostics` | bool | `true` | 否 | 是否发布结构化 diagnostics。 | `true` | 发布 `diagnostic_msgs/msg/DiagnosticArray`，便于观察失锁、恢复、重定位和 continuity 调整。 |
| `localization.ros.initial_pose_topic` | string | `/initialpose` | 否 | runtime 初始位姿订阅 topic。 | `/initialpose` | 当 `initial_pose.source == "topic"` 时，模块等待该 topic。 |
| `localization.ros.relocalization_service` | string | `/localization/relocalize` | 否 | 手动触发重定位服务名。 | `/localization/relocalize` | service callback 会设置 runtime relocalization request。 |
| `localization.ros.status_topic` | string | `/localization/status` | 否 | localization 状态输出 topic。 | `/localization/status` | 发布 `std_msgs/msg/String`，内容为状态名和 score。 |
| `localization.ros.diagnostic_topic` | string | `/localization/diagnostics` | 否 | 结构化 diagnostics 输出 topic。 | `/localization/diagnostics` | 发布状态原因、reject reason、恢复计数、候选数和 continuity 信息。 |
| `localization.ros.odom_topic` | string | `/localization/odom` | 否 | odometry 输出 topic。 | `/localization/odom` | 发布 `nav_msgs/msg/Odometry`，frame 为 `odom_frame`，child 为 `base_frame`。 |
| `localization.ros.pose_topic` | string | `/localization/pose` | 否 | pose 输出 topic。 | `/localization/pose` | 发布 `geometry_msgs/msg/PoseStamped`，frame 为 `map_frame`。 |
| `localization.ros.trajectory_topic` | string | `/localization/trajectory` | 否 | 轨迹可视化 topic。 | `/localization/trajectory` | 发布 `nav_msgs/msg/Path`，内容为当前累计 `T_map_imu` 轨迹。 |
| `localization.ros.input_scan_topic` | string | `/localization/debug/input_scan` | 否 | 当前输入 scan 可视化 topic。 | `/localization/debug/input_scan` | 发布预处理后的原始输入点云，frame 为 `sensor_frame`。 |
| `localization.ros.current_scan_topic` | string | `/localization/debug/current_scan` | 否 | 当前 deskewed scan 可视化 topic。 | `/localization/debug/current_scan` | 发布当前用于定位的 deskewed scan，并已变换到 `map_frame`。 |
| `localization.ros.target_map_topic` | string | `/localization/debug/local_target_map` | 否 | debug local target map 点云 topic。 | `/localization/debug/local_target_map` | 发布 active submaps 合并后的 `sensor_msgs/msg/PointCloud2`。 |
| `localization.ros.active_submaps_topic` | string | `/localization/debug/active_submaps` | 否 | active submap ids debug marker topic。 | `/localization/debug/active_submaps` | 发布 `visualization_msgs/msg/Marker`，内容包含状态、score 和 active submap ids。 |

## 7. ROS runtime / 输入相关配置

这些参数位于 `glim_ros`，不是 `LocalizationOptions::load()` 解析，而是由 `glim_ros2` 的 `GlimROS` 和 `glim_rosbag` 解析。它们决定输入 topic、动态扩展加载和是否关闭 mapping。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `glim_ros.imu_topic` | string | 在线节点默认 `""`；`glim_rosbag` 默认 `/imu` | 是 | IMU 输入 topic。 | 与 bag/在线传感器一致，例如 `/imu` | topic 不一致时没有 IMU 输入，odometry 无法正常运行。 |
| `glim_ros.points_topic` | string | 在线节点默认 `""`；`glim_rosbag` 默认 `/points` | 是 | 点云输入 topic。 | 与 bag/在线传感器一致，例如 `/points` | topic 不一致时没有点云输入，无法产生 localization frame。 |
| `glim_ros.image_topic` | string | `glim_rosbag` 默认 `/image`；示例配置为 `""` | 否 | 图像输入 topic。 | localization 当前设为 `""` | 当前 localization 主流程不依赖图像。 |
| `glim_ros.keep_raw_points` | bool | `false` | 否 | 是否保留 raw points。 | `false` | 影响 GLIM ROS 点云数据保存和扩展模块可用数据。 |
| `glim_ros.imu_time_offset` | double | `0.0` | 否 | IMU 时间戳偏移，单位秒。 | `0.0`，有标定结果时填写 | 直接加到 IMU stamp，影响 IMU/点云同步。 |
| `glim_ros.points_time_offset` | double | `0.0` | 否 | 点云时间戳偏移，单位秒。 | `0.0`，有标定结果时填写 | 直接加到点云 stamp，影响同步和 deskew。 |
| `glim_ros.acc_scale` | double | `0.0` | 否 | 加速度缩放。`0.0` 时 GLIM ROS 会根据 IMU 加速度范数自动判断。 | 普通 m/s² IMU 可用 `0.0` 或 `1.0`；Livox 等 g 单位可用 `9.80665` | 设置错误会影响 IMU 预测和初始化。 |
| `glim_ros.enable_local_mapping` | bool | `true` | 是，localization 必须显式关闭 | 是否加载 local mapping module。 | `false` | localization-only 必须关闭，否则会尝试运行建图模块。 |
| `glim_ros.enable_global_mapping` | bool | `true` | 是，localization 必须显式关闭 | 是否加载 global mapping module。 | `false` | localization-only 必须关闭，否则会尝试运行全局建图模块。 |
| `glim_ros.extension_modules` | vector<string> | 未配置时为空 | ROS 输出需要 | GLIM extension 动态库列表。 | `["liblocalization_publisher.so"]` | 未加载该扩展时，不会发布 localization status/pose/odom/TF/debug target map，也不会订阅 `/initialpose`。 |

## 8. sensors

这些参数位于 `sensors`，由 GLIM 的 `OdometryEstimationIMUParams` 基类读取。虽然它们不在 `localization` 节点下，但会直接影响 localization 结果。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `sensors.T_lidar_imu` | Isometry3d，通常为 `[x,y,z,qx,qy,qz,qw]` | identity | 强烈建议显式配置 | LiDAR 到 IMU 的外参。 | 使用建图时一致的外参 | 错误外参会导致 deskew、`T_map_lidar` 和 scan-to-map registration 不一致。 |
| `sensors.imu_bias_noise` | double | `1e-3` | 否 | IMU bias noise。 | `0.001` | 影响 IMU bias 相关优化约束。 |
| `sensors.imu_bias` | vector<double>[6] | `[0,0,0,0,0,0]` | 否 | 初始 IMU bias，6 维。 | 不确定时用零 | 影响 IMU 预积分初始 bias。 |

当前 `localization.json` 中还包含 `sensors.intensity_field` 和 `sensors.ring_field`。这两个字段不在上述 `OdometryEstimationIMUParams` 中解析；它们可能由 GLIM 其他传感器/预处理代码使用。本文聚焦 localization 模块及其直接运行链路，因此不展开说明。

## 9. relocalization

这些参数位于 `localization.relocalization`，当前代码已经解析，并由 `ScanContextRelocalizer` 和 `OdometryEstimationLocalizationCPU` 使用。

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.relocalization.enable` | bool | `true` | 否 | 是否启用重定位。 | `true` | 为 false 时，LOST 后不会构建/使用 relocalizer。 |
| `localization.relocalization.max_candidates` | int | `5` | 否 | relocalizer 查询返回的最大候选数。 | `3-10` | 候选越多，几何验证开销越大，但成功率可能提高。 |
| `localization.relocalization.num_rings` | int | `20` | 否 | ScanContext 风格描述子的 ring 数。 | `20` | 影响描述子分辨率和检索鲁棒性。 |
| `localization.relocalization.num_sectors` | int | `60` | 否 | ScanContext 风格描述子的 sector 数。 | `60` | 影响 yaw 方向分辨率和计算量。 |
| `localization.relocalization.min_radius` | double | `1.0` | 否 | 描述子使用点的最小半径。 | `1.0` | 过滤近距离点，影响描述子稳定性。 |
| `localization.relocalization.max_radius` | double | `80.0` | 否 | 描述子使用点的最大半径。 | 与雷达有效范围一致，如 `60-100` | 太小会丢失全局结构；太大可能加入噪声/稀疏远点。 |
| `localization.relocalization.max_descriptor_distance` | double | `0.35` | 否 | 候选描述子最大距离。 | `0.3-0.5` | 越小候选越严格；越大候选更多但误检增加。 |
| `localization.relocalization.candidate_translation_weight` | double | `0.01` | 否 | 候选重排时，对候选初值与当前预测位姿之间平移差的软惩罚权重。 | `0.0-0.05` | 值越大，越偏向当前预测附近的候选。 |
| `localization.relocalization.max_candidate_translation_delta` | double | `80.0` | 否 | 候选过滤允许的最大平移差，单位米。`<=0` 时可视为关闭该过滤。 | `40-120` | 值太小会漏掉远距离重定位候选。 |
| `localization.relocalization.verification_target_max_submaps` | int | `4` | 否 | 几何验证阶段允许构建的局部 target map 最大 submap 数。 | `2-6` | 值越大，验证更稳，但耗时更高。 |
| `localization.relocalization.verification_target_max_distance` | double | `20.0` | 否 | 几何验证阶段局部 target map 的查询半径，单位米。 | `10-30` | 太小可能 target 不足；太大则验证变慢。 |
| `localization.relocalization.recovery_stable_frames` | int | `2` | 否 | relocalization 成功后，需要再连续接受多少帧 scan-to-map 才从 `RECOVERING` 切回 `TRACKING`。 | `1-5` | 值越大，恢复判定越保守。 |

## 10. debug

当前代码没有解析独立的 `debug` JSON 节点。现有 debug/诊断相关能力由以下真实参数控制：

| 参数路径 | 类型 | 默认值 | 是否必填 | 含义 | 推荐值 | 对运行行为的影响 |
|---|---:|---|---|---|---|---|
| `localization.trajectory_path` | string | `/tmp/glim_localization_traj.txt` | 否 | 文件级定位结果记录。 | 调试时保持非空 | 用于离线检查每帧 pose、状态和 matching score。 |
| `localization.ros.trajectory_topic` | string | `/localization/trajectory` | 否 | 在线轨迹可视化输出。 | `/localization/trajectory` | RViz 中用 `Path` 显示累计定位轨迹。 |
| `localization.ros.input_scan_topic` | string | `/localization/debug/input_scan` | 否 | 输入 scan 输出。 | `/localization/debug/input_scan` | RViz 中保留传感器坐标系视角，检查输入点云本身是否正常。 |
| `localization.ros.current_scan_topic` | string | `/localization/debug/current_scan` | 否 | 当前 deskewed scan 输出。 | `/localization/debug/current_scan` | RViz 中直接叠加到 local target map 检查配准效果。 |
| `localization.ros.publish_debug_target_map` | bool | `true` | 否 | 是否允许发布 debug target map。 | 调试时 `true`，性能测试时 `false` | 为 true 且有订阅者时才构建并发布 target cloud。 |
| `localization.ros.target_map_topic` | string | `/localization/debug/local_target_map` | 否 | debug target map 输出 topic。 | `/localization/debug/local_target_map` | 用于 RViz 检查当前 active submaps 是否合理。 |
| `localization.ros.active_submaps_topic` | string | `/localization/debug/active_submaps` | 否 | active submaps marker 输出 topic。 | `/localization/debug/active_submaps` | 在 RViz 中显示状态、score 和当前 active submap ids。 |
| `odometry_estimation.validate_imu` | bool | `true` | 否 | IMU 输入诊断。 | `true` | 可帮助发现异常 IMU 数据。 |

此外，registration rejection 日志中会输出：

- active submap ids
- target reused/rebuilt
- matching accepted/rejected
- score、residual、inliers
- pose correction translation/angle
- reject reason

这些日志行为当前没有独立 JSON 开关。

## 11. 当前未列入的配置项

以下字段在当前示例配置或结构体中出现，但本文不作为“已解析参数”列入主表：

- 顶层 `enable_local_mapping`
- 顶层 `enable_global_mapping`

原因：当前 GLIM ROS2 读取的是 `glim_ros.enable_local_mapping` 和 `glim_ros.enable_global_mapping`。

- `sub_mapping.so_name`
- `global_mapping.so_name`

原因：在 localization 配置中 `glim_ros.enable_local_mapping=false` 且 `glim_ros.enable_global_mapping=false`，因此当前 localization-only 运行路径不会加载这些 mapping module。

## 12. 完整配置示例

下面示例保留当前代码实际会读取并影响 localization 运行的主要参数。路径需按实际部署调整。

```json
{
  "glim_ros": {
    "imu_topic": "/imu",
    "points_topic": "/points",
    "image_topic": "",
    "keep_raw_points": false,
    "imu_time_offset": 0.0,
    "points_time_offset": 0.0,
    "acc_scale": 0.0,
    "enable_local_mapping": false,
    "enable_global_mapping": false,
    "extension_modules": ["liblocalization_publisher.so"]
  },
  "odometry_estimation": {
    "so_name": "libodometry_estimation_localization_cpu.so",
    "initialization_mode": "LOOSE",
    "smoother_lag": 5.0,
    "fix_imu_bias": false,
    "validate_imu": true,
    "save_imu_rate_trajectory": false,
    "num_threads": 4,
    "use_isam2_dogleg": false,
    "isam2_relinearize_skip": 1,
    "isam2_relinearize_thresh": 0.1,
    "init_pose_damping_scale": 10000000000.0
  },
  "localization": {
    "map_path": "/path/to/glim_dump",
    "map": {
      "load_voxelmaps": true,
      "load_raw_frames": false,
      "strict": true
    },
    "trajectory_path": "/tmp/glim_localization_traj.txt",
    "map_frame": "map",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "sensor_frame": "lidar",
    "initial_pose": {
      "source": "config",
      "xyz": [0.0, 0.0, 0.0],
      "rpy": [0.0, 0.0, 0.0]
    },
    "target_map": {
      "max_num_submaps": 8,
      "max_distance": 40.0,
      "update_distance": 2.0,
      "update_angle": 0.2,
      "use_submap_index": true,
      "index_resolution": 20.0
    },
    "matching": {
      "method": "cpu_gicp",
      "max_iterations": 20,
      "min_score": 0.35,
      "min_inliers": 30,
      "max_residual": 1000000.0,
      "max_pose_correction_translation": 3.0,
      "max_pose_correction_angle": 0.7,
      "max_consecutive_rejections": 3,
      "max_correspondence_distance": 2.0,
      "pose_prior_precision": 1000.0,
      "num_threads": 4,
      "vgicp_resolution": 0.5,
      "vgicp_voxelmap_levels": 2,
      "vgicp_voxelmap_scaling_factor": 2.0
    },
    "relocalization": {
      "enable": true,
      "max_candidates": 5,
      "num_rings": 20,
      "num_sectors": 60,
      "min_radius": 1.0,
      "max_radius": 80.0,
      "max_descriptor_distance": 0.35
    },
    "ros": {
      "publish_tf": true,
      "publish_debug_target_map": true,
      "initial_pose_topic": "/initialpose",
      "relocalization_service": "/localization/relocalize",
      "status_topic": "/localization/status",
      "odom_topic": "/localization/odom",
      "pose_topic": "/localization/pose",
      "trajectory_topic": "/localization/trajectory",
      "input_scan_topic": "/localization/debug/input_scan",
      "current_scan_topic": "/localization/debug/current_scan",
      "target_map_topic": "/localization/debug/local_target_map",
      "active_submaps_topic": "/localization/debug/active_submaps"
    }
  },
  "sensors": {
    "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "imu_bias_noise": 0.001,
    "imu_bias": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "intensity_field": "intensity",
    "ring_field": ""
  }
}
```
