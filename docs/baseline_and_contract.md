# P0 基线与契约

本文档用于收束 `glim_localization` 在 P0 阶段的边界、基线、接口语义与回归口径。目标不是新增功能，而是把当前工作区中已经存在并可运行的能力固定成后续 P1 / P2 / P3 的稳定基础。

适用范围：

- `glim_localization` 当前已经实现的 fixed-map localization 主流程
- `glim` / `glim_ros2` 当前真实接口上可直接支撑的离线与在线运行方式
- 当前已存在的轨迹输出、benchmark、资源监测与 HTML 报告工具

不在本文档收束范围内：

- 新的 scan-to-map backend
- 更复杂的 relocalization / continuity 机制
- 新的 ROS 消息类型或状态机实现
- 地图格式扩展与版本迁移机制

## 1. P0 的目标边界

P0 只固定以下内容：

- 标准实验基线
- 坐标系与 topic 对外契约
- 状态机对外语义
- trajectory / benchmark / resource monitoring 的标准用法
- 最小测试集与回归步骤

P0 不以“更强功能”为目标，而以“后续优化不漂移”为目标。

## 2. 标准实验基线

### 2.1 命名规则

P0 推荐统一使用两套离线基线。仓库中不直接内置 bag / map 大文件，因此基线以“命名组合 + 固定约束”方式定义：

- `baseline_offline_cpu`
- `baseline_offline_gpu`

建议团队在仓库外统一维护一份基线资产目录，例如：

```text
<baseline_root>/
  bags/
    localization_baseline_01/
  maps/
    localization_baseline_01_dump/
  notes/
    localization_baseline_01.md
```

### 2.2 `baseline_offline_cpu`

推荐组合：

- bag：固定一段包含 `/imu` 和点云 topic 的 rosbag
- map：与该 bag 同场景、同坐标系的 GLIM dump 地图
- config：基于 `Glim_localization/config/localization.json`
- matching backend：`cpu_gicp`
- initial pose：固定一组 `x y z roll pitch yaw`

强约束：

- `glim_ros.enable_local_mapping = false`
- `glim_ros.enable_global_mapping = false`
- `odometry_estimation.so_name = libodometry_estimation_localization_cpu.so`
- `glim_ros.extension_modules` 包含 `liblocalization_publisher.so`
- `localization.initial_pose.source = config`
- `localization.matching.method = cpu_gicp`

推荐用途：

- CPU-only 回归
- 最小闭环验收
- scan-to-map 与状态机的基础稳定性验证

### 2.3 `baseline_offline_gpu`

推荐组合与 CPU 基线保持同一份 bag、map 和初始位姿，唯一差异是：

- 构建方式：`BUILD_WITH_CUDA=ON`
- matching backend：`gpu_vgicp`

强约束：

- 必须确认 `glim_localization` 实际编译了 GPU VGICP 路径
- 必须确认运行时没有回退到 CPU GICP
- 除 GPU 相关参数外，其余 target map / gating / initial pose 条件尽量与 CPU 基线一致

推荐用途：

- GPU-enabled 回归
- CPU vs GPU 性能与资源对照
- GPU 路径 warning / hook /资源占用专项分析

### 2.4 最小闭环验证路径

P0 固定的最小闭环路径是：

```text
GLIM dump map
  + rosbag
  + config initial pose
  + offline glim_rosbag
  + localization trajectory output
  + localization ROS debug output
```

也就是说：

- P0 的“标准回归”优先以离线 rosbag 为准
- 在线 ROS2 链路属于兼容性与部署验证，不作为唯一基线
- `/initialpose` 在线初始化和 relocalization 是已支持能力，但不作为 P0 唯一验收入口

## 3. 坐标系契约

### 3.1 内部定位语义

基于当前代码，`glim_localization` 内部固定使用：

- `world == map`
- `frame->T_world_imu == T_map_imu`
- `frame->T_world_lidar == T_map_lidar`

对应实现见：

- `OdometryEstimationLocalizationCPU` 注释与赋值逻辑
- `LocalizationResult`
- `LocalizationPublisher`

### 3.2 `map / odom / base_link / imu / lidar` 的当前语义

- `map_frame`
  - localization 的固定参考系
  - `/localization/pose`
  - `/localization/trajectory`
  - debug target map
  - `map -> odom` TF

- `odom_frame`
  - 当前主要用于兼容 ROS odom/TF 链路
  - 当前实现中 `T_map_odom` 基本保持 identity
  - 因此 `T_odom_base` 目前近似等价于 `T_map_imu`

- `base_frame`
  - 作为 `/localization/odom.child_frame_id`
  - 作为 `odom -> base_link` TF 的 child
  - 但当前填充的位姿本质上是 IMU 位姿，不是单独估计的车体几何中心位姿

- `imu`
  - 内部 smoother 和 scan-to-map 的主状态变量
  - `X(i)` 表示 IMU pose

- `lidar`
  - 输入点云坐标系
  - `sensor_frame` 当前主要用于 `/localization/debug/input_scan`
  - `current_scan` 已被变换到 `map_frame`

### 3.3 P0 对外口径

P0 统一对外说明为：

- `map` 是 localization 的固定参考系
- `odom` 当前是兼容层，不承诺非 identity 漂移管理
- `base_link` 当前承载的是 localization 输出位姿的 child frame id，不额外承诺车体几何中心语义
- 真正参与内部优化和 registration 的核心状态是 `T_map_imu`

## 4. ROS 输出契约

### 4.1 标准 topic 集合

P0 固定以下 topic 作为 `glim_localization` 的标准 ROS 输出：

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

### 4.2 topic 语义

- `/localization/status`
  - 当前是 `std_msgs/String`
  - 语义为“状态名 + matching score”
  - 这是 P0 的既有契约，不在本阶段更换消息类型

- `/localization/pose`
  - `map_frame` 下的当前 `T_map_imu`

- `/localization/odom`
  - `odom_frame -> base_frame`
  - 当前本质上由 `T_map_odom^-1 * T_map_imu` 生成

- `/localization/trajectory`
  - 当前累计定位轨迹

- `/localization/debug/input_scan`
  - 原始输入 scan 可视化
  - frame 使用 `sensor_frame`

- `/localization/debug/current_scan`
  - 当前用于定位的 scan，可视化时已在 `map_frame`

- `/localization/debug/local_target_map`
  - active submaps 合并出的局部目标地图
  - 只在 `publish_debug_target_map = true` 且有订阅者时发布

- `/localization/debug/active_submaps`
  - 当前状态、score、active submap ids 的 marker

### 4.3 TF 契约

P0 固定：

- 发布 `map -> odom`
- 发布 `odom -> base_link`

同时明确：

- `map -> odom` 保持现有命名契约不变
- P2 起，relocalization 成功后允许更新 `T_map_odom` 以维持 `odom -> base_link` 的连续性
- 这类 continuity 调整会通过 `/localization/diagnostics` 和 marker/status 文本暴露

## 5. 状态机对外语义

P0 固定对外状态名：

- `WAIT_MAP`
- `WAIT_INITIAL_POSE`
- `INITIALIZING`
- `TRACKING`
- `LOST`
- `RELOCALIZING`

P0 不改变当前状态机实现，但固定这些状态的可见语义：

- `WAIT_MAP`
  - 地图未加载完成或不可用

- `WAIT_INITIAL_POSE`
  - 地图已准备，但初始化位姿未准备好

- `INITIALIZING`
  - 已具备启动条件，系统正在进入正常定位过程
  - P1 起，系统在拿到初始位姿后不会立刻视为 `TRACKING`；会先保持 `INITIALIZING`，直到首次 scan-to-map 被接受或重定位恢复成功

- `TRACKING`
  - 系统正在尝试维持正常 fixed-map localization
  - 当系统已经建立过有效定位后，单次 registration reject 但未超过阈值时，状态仍可能保持 `TRACKING`

- `DEGRADED`
  - 已经建立过有效定位，但当前 registration 被拒绝，且尚未达到 LOST 阈值
  - 用于把“还能继续跑”和“当前帧质量明显变差”区分开

- `LOST`
  - 连续拒绝或恢复失败后，当前 tracking 已不可依赖

- `RELOCALIZING`
  - 正在执行候选检索与几何验证

- `RECOVERING`
  - relocalization 已经成功，但系统仍处于恢复稳定期
  - 在该阶段内，`map -> odom` 已做连续性修正，但仍需要若干帧 accepted scan-to-map 才回到 `TRACKING`

这一定义属于 P0 契约。后续阶段可以增强内部逻辑，但不应随意改变这些状态名的基本含义。

## 6. trajectory / benchmark / resource monitoring 的标准用法

### 6.1 trajectory

P0 固定轨迹文件格式为：

```text
stamp x y z qx qy qz qw status_int matching_score
```

标准工具：

- `TrajectoryWriter`
- `tools/plot_trajectory.py`

P0 用途：

- 最小闭环是否成立
- 轨迹连续性是否异常
- 修复前后对比

### 6.2 benchmark

P0 对 `benchmark_localization` 的口径固定为：

- 这是“submap query / index”的 benchmark
- 不是完整 localization 精度 benchmark
- 不能替代离线 rosbag 闭环回归

### 6.3 resource monitoring

P0 对资源监测工具的标准口径固定为：

- 它们是第三方观测工具
- 不集成到 `glim_localization` 主运行逻辑
- 用于 CPU / memory / thread / GPU / I/O 的实验记录与部署评估

标准入口：

- `tools/monitor/run_with_time.sh`
- `tools/monitor/plot_usage.py`
- `tools/monitor/generate_usage_report.py`

## 7. 文档职责分工

P0 统一文档职责如下：

- `README.md`
  - 项目全景、模块边界、主入口、目录总索引

- `docs/baseline_and_contract.md`
  - P0 基线、接口契约、状态语义、标准回归口径

- `docs/quick_start.md`
  - 第一次运行的最短离线闭环

- `docs/deployment_and_run.md`
  - 从依赖安装到离线/在线运行的完整操作手册

- `docs/config_reference.md`
  - 当前代码已解析配置项的权威参考

- `docs/faq.md`
  - 高频问题的快速答案，不重复展开教程

- `docs/resource_monitoring.md`
  - 外部资源监测、图表与 HTML 报告

P0 原则：

- 运行教程不复制配置全表
- FAQ 不重复长教程
- benchmark / monitoring 不宣称自己是精度验证工具
- “已实现 / 部分实现 / 后续计划” 在 README 中明确区分，不在其他文档中混写

## 8. 最小测试集与回归口径

### 8.1 必须保留的最小测试集

P0 最小测试集包括：

- `test_localization_options`
- `test_map_format_checker`
- `test_glim_map_loader`
- `test_submap_index`
- `test_localization_map`
- `test_cpu_map_registration`
- `test_scan_context_relocalizer`
- `test_plugin_module_loading_odometry`
- `test_plugin_module_loading_extension`
- `test_localization_contracts`
- `test_trajectory_writer`

其中：

- `test_localization_contracts`
  - 固化默认 frame/topic/status 名称契约

- `test_trajectory_writer`
  - 固化轨迹文件列格式契约

### 8.2 CPU-only 回归步骤

1. 编译并通过最小测试集。
2. 用 `baseline_offline_cpu` 跑一次离线 rosbag。
3. 检查：
   - 轨迹文件已生成
   - `/localization/status` 状态变化合理
   - RViz 中 pose/current_scan/target_map 可用
   - 无异常崩溃
4. 如有性能评估需求，追加运行资源监测。

### 8.3 GPU-enabled 回归步骤

1. 以 CUDA 方式重新编译。
2. 确认实际启用了 `gpu_vgicp`。
3. 用与 CPU 基线相同的 bag / map / initial pose 跑一次离线 rosbag。
4. 检查：
   - 轨迹文件已生成
   - 没有静默回退到 CPU
   - GPU 监测日志与图表可生成
   - 主要 warning 数量与资源占用在预期范围内

### 8.4 后续阶段“不退化”的统一判断口径

后续 P1 / P2 / P3 的修改，至少不应破坏以下 P0 口径：

- 最小测试集必须继续通过
- `baseline_offline_cpu` 必须继续闭环
- 如果启用 CUDA，`baseline_offline_gpu` 必须继续闭环
- 轨迹文件格式不能随意变更
- 标准 topic / frame / status 名称不能随意变更
- 资源监测脚本与报告生成链路不能失效

## 9. P0 验收标准

完成 P0 收束后，应满足：

- 有一份集中式基线与契约文档
- README 能明确指向各文档职责
- quick start / deployment / config / FAQ / monitoring 的职责边界清晰
- 默认 frame/topic/status/trajectory 契约被测试固定
- 最小测试集与 CPU/GPU 回归口径被写清楚

P0 不以算法增强为验收目标，而以“后续优化有稳定参照物”为验收目标。
