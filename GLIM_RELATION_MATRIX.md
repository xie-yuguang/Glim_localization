# Glim_localization 与 koide3/Glim 关系矩阵

对照基准：

- `glim`: `/home/xie/Glim/src/glim`，remote `https://github.com/koide3/glim`，commit `25ad190 v1.2.1`
- `glim_ros2`: `/home/xie/Glim/src/glim_ros2`，remote `https://github.com/koide3/glim_ros2`
- `glim_ext`: `/home/xie/Glim/src/glim_ext`，remote `https://github.com/koide3/glim_ext`
- `glim_localization`: `/home/xie/Glim/src/Glim_localization`，remote `git@github.com:xie-yuguang/Glim_localization.git`

## 总体结论

`glim_localization` 不是 `koide3/glim` 的直接 fork。它是一个独立 ROS2/ament 包，链接 `glim::glim`，复用 GLIM 的配置、预处理、IMU odometry、fixed-lag smoother、SubMap dump load 和动态模块机制，并新增 localization-only 的地图、registration、relocalization、ROS 输出和实验工具。

| 模块/文件 | Glim 原项目中是否存在 | Glim_localization 中状态 | 修改程度 | localization 相关性 | 风险 |
|---|---:|---:|---:|---:|---:|
| `CMakeLists.txt` | 存在同类文件 | 独立包 CMake，链接 `glim::glim` | 新增 | 高 | Medium：ROS1 分支未验证，CUDA 默认 OFF |
| `package.xml` | 存在同类文件 | 独立包，声明 `glim` 依赖 | 新增 | 高 | Medium：maintainer/author 仍是 koide |
| `glim::OdometryEstimationIMU` | 存在 | 被继承复用 | 轻微侵入式复用 | 高 | High：对 GLIM 内部语义耦合深 |
| `glim::OdometryEstimationCPU` | 存在 | 未直接继承；算法思想部分借鉴 | 中度参考 | 中 | Medium：prior/between 注入策略相似但缺完整协方差 |
| `odometry_estimation_localization_cpu.cpp` | 不存在 | 新动态 odometry module | 新增 | 高 | High：核心大类承担过多职责 |
| `odometry_estimation_localization_cpu_create.cpp` | GLIM 有 `_create.cpp` 模式 | 新 module factory | 新增/沿用模式 | 高 | Low |
| `glim_ros2/GlimROS` | 在 `glim_ros2` 存在 | 运行时直接复用 | 未修改 | 高 | Medium：没有独立 localization node |
| `localization_publisher.cpp` | GLIM 有 extension 机制，无此文件 | 新 ROS2 extension | 新增 | 高 | Medium：输出语义和线程边界需加强 |
| `glim::SubMap::load()` | 存在 | 地图加载直接调用 | 未修改复用 | 高 | Medium：受 GLIM dump 格式限制 |
| `GlimMapLoader` | 不存在 | fixed-map loader | 新增 | 高 | Medium：地图格式版本/元数据不足 |
| `MapFormatChecker` | 不存在 | 检查 GLIM dump | 新增 | 高 | Medium：仅检查基础文件 |
| `LocalizationMap` | 不存在 | fixed map 容器 | 新增 | 高 | Low/Medium |
| `LocalTargetMap` | 不存在 | active submaps + merged target cloud | 新增 | 高 | High：合并点云可能是瓶颈 |
| `SubmapIndex` | 不存在 | submap origin grid index | 新增 | 中 | Low：当前地图 query 很快 |
| `CpuGicpMapRegistration` | GLIM 有 CPU GICP/VGICP odometry | 新 fixed-map GICP backend | 中度参考/新增 | 高 | High：质量评价和退化检测不足 |
| `GpuVgicpMapRegistration` | GLIM 有 GPU odometry | 新 fixed-map GPU VGICP backend | 中度参考/新增 | 高 | High：每帧 clone/build voxelmap |
| `ScanContextRelocalizer` | `glim_ext` 有 ScanContext loop detector 思路 | 新 relocalizer | 中度参考/新增 | 高 | Medium：MVP，无多假设 |
| `GeometricVerifier` | 不存在 | registration 验证候选 | 新增 | 高 | Medium |
| `RuntimeInitialPose` | 不存在 | 全局缓存 `/initialpose` | 新增 | 高 | Medium：多实例隔离不足 |
| `RuntimeRelocalizationRequest` | 不存在 | 全局 relocalize flag | 新增 | 高 | Medium：多实例隔离不足 |
| `TrajectoryWriter` | GLIM 有 trajectory manager | 新 localization trajectory writer | 新增 | 中 | Low |
| `config/localization.json` | GLIM 有多个 config JSON | 合并 GLIM+localization 配置 | 新增/裁剪 | 高 | High：绝对路径、GPU 默认冲突 |
| `config/config.json` | GLIM 有 global config 索引 | 指向 localization.json | 新增/适配 | 高 | High：绝对路径 |
| `tools/run_offline_localization.sh` | GLIM 无此脚本 | 包装 `glim_rosbag` | 新增 | 高 | Medium：依赖环境 |
| `tools/run_standard_experiment.sh` | GLIM 无此脚本 | 标准实验 orchestrator | 新增 | 高 | Medium：真实 baseline 待固化 |
| `tools/monitor/*` | GLIM 无此工具 | 外部资源监控 | 新增 | 中 | Low：pycache 被跟踪 |
| `rviz/localization.rviz` | GLIM ROS2 有 rviz | localization RViz 配置 | 新增 | 中 | Low |
| local mapping | GLIM 存在 | 配置关闭 | 删除运行路径 | 低/负相关 | Medium：若配置错会加载 mapping |
| global mapping / pose graph | GLIM 存在 | 配置关闭 | 删除运行路径 | 低/负相关 | Medium |
| viewer / map editor | GLIM 存在 | 不在本包复用 | 删除运行路径 | 低 | Low |
| GNSS / extensions | `glim_ext` 存在 | 未直接链接 | 删除/未接入 | 中 | Medium：localization 场景可能需要 |

## 关系分类

| 分类 | 是否适用 | 说明 |
|---|---:|---|
| 直接 fork | 否 | 独立仓库、独立包名、没有复制整个 GLIM 源码树 |
| 代码裁剪版 | 部分 | 运行路径裁剪了 mapping/global mapping，但不是源码裁剪 GLIM |
| wrapper | 部分 | 通过 `glim_ros2` 和动态库包装运行 |
| 基于 GLIM 的二次开发 | 是 | 继承 GLIM IMU odometry，新增 fixed-map localization |
| SLAM pipeline 改造成 localization pipeline | 是 | fixed map scan-to-map 替换在线建图 target |
| 仅参考设计思想 | 否 | 实际链接并继承 GLIM 类型和模块 |

## localization 中建议逐步弱化的 GLIM 耦合

1. `world == map` 的隐式约定应封装为明确的 `FrameSemantics`。
2. `EstimationFrame::custom_data` 可保留，但建议增加类型化 callback/result adapter。
3. `glim_ros2` 入口短期保留，中期可新增 `glim_localization_ros` node。
4. 地图加载可继续支持 GLIM dump，同时增加 map metadata/schema。
5. GLIM fixed-lag smoother 短期继续复用；长期根据 localization 需求抽出更小的 state estimator。
