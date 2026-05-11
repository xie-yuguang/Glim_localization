# Phase 0 Baseline Hardening Report

## 1. 执行摘要

本轮完成 Phase 0 baseline hardening 和少量 Phase 1 工程清理，未修改 `koide3/glim`、`glim_ros2`、`glim_ext` 上游源码，未重写 localization pipeline。

结论：

- 默认配置已移除个人绝对路径：`config/config.json` 的 `global.config_path` 改为空字符串，`config/localization.json` 的 `localization.map_path` 改为空字符串。
- 默认 matching backend 已从 `gpu_vgicp` 改为 CPU-safe 的 `cpu_gicp`。
- 新增 CPU/GPU/template 配置模板：`config/localization.cpu.json`、`config/localization.gpu.json`、`config/localization.template.json`。
- 已通过 `git rm -r --cached tools/monitor/__pycache__` 从 Git 索引移除 pycache，并新增 `.gitignore`。
- 新增 MIT `LICENSE`，更新 `package.xml` 当前项目维护者/作者信息，并保留 `koide3/Glim` upstream attribution。
- 新增最小 ROS2 launch：`launch/localization.launch.py`，包装 `glim_ros` 的 `glim_rosnode`。
- `colcon build --packages-select glim_localization` 通过。
- `colcon test --packages-select glim_localization` 通过，15/15 passed。
- map info 和 benchmark 通过。
- 真实 bag CPU baseline 已尝试，但 600 秒 timeout，不能视为成功 baseline；日志显示轨迹严重发散、后段反复 `LOST -> RELOCALIZING` 且候选为 0，并出现 GTSAM fixed-lag smoother underconstrained warning。

## 2. 本轮改动列表

修改文件：

- `CMakeLists.txt`
  - 安装 `launch/` 目录。
  - 排除 `launch/__pycache__` 和 `*.pyc` 安装。
- `README.md`
  - 补充 CPU/GPU/template 配置说明。
  - 补充 GPU 启用条件。
  - 补充最小 launch 启动命令。
  - 清理 README 中残留的 `/home/xie/...` 示例路径。
- `config/config.json`
  - `global.config_path` 从个人绝对路径改为空字符串。
- `config/localization.json`
  - `localization.map_path` 从个人绝对路径改为空字符串。
  - `localization.matching.method` 从 `gpu_vgicp` 改为 `cpu_gicp`。
- `package.xml`
  - maintainer/author 改为当前项目维护者 `xie-yuguang`。
  - 保留 `Kenji Koide (upstream GLIM)` author attribution。
  - 描述中明确本项目基于 / 集成 `koide3/Glim`，但不是官方 GLIM 项目。
  - 增加 ROS2 launch runtime 依赖：`launch`、`launch_ros`。

新增文件：

- `.gitignore`
- `LICENSE`
- `config/localization.cpu.json`
- `config/localization.gpu.json`
- `config/localization.template.json`
- `launch/localization.launch.py`
- `PHASE0_BASELINE_HARDENING_REPORT.md`

Git 索引清理：

- `tools/monitor/__pycache__/generate_usage_report.cpython-312.pyc`
- `tools/monitor/__pycache__/plot_usage.cpython-312.pyc`
- `tools/monitor/__pycache__/summarize_usage.cpython-312.pyc`
- `tools/monitor/__pycache__/usage_report_lib.cpython-310.pyc`
- `tools/monitor/__pycache__/usage_report_lib.cpython-312.pyc`

工作区说明：

- 执行本轮任务前，仓库已存在上一轮未跟踪报告文件：
  - `GLIM_LOCALIZATION_ANALYSIS.md`
  - `GLIM_RELATION_MATRIX.md`
  - `IMMEDIATE_FIXES.md`
  - `OPTIMIZATION_ROADMAP.md`
  - `RUN_AND_BUILD_NOTES.md`
- 因工作区非干净状态，本轮未创建 `codex/phase0-baseline-hardening` 分支，继续在当前 `main` 分支工作。

## 3. 配置路径清理结果

确认过 GLIM 配置机制：

- `glim::GlobalConfig::instance(config_path)` 会加载 `${config_path}/config.json`。
- 初始化时会执行 `override_param("global", "config_path", config_path)`。
- 因此 `config/config.json` 中默认 `global.config_path` 可以安全设为空字符串，由运行时 ROS 参数 `config_path` 注入实际配置目录。

清理结果：

- `config/config.json`
  - before：`"/home/xie/Glim/src/Glim_localization/config"`
  - after：`""`
- `config/localization.json`
  - before：`"/home/xie/Glim/data/map_data/ceshichang_128lidar/"`
  - after：`""`

验证命令：

```bash
python3 -m json.tool config/config.json >/dev/null
python3 -m json.tool config/localization.json >/dev/null
rg -n '/home/xie' config .gitignore package.xml CMakeLists.txt launch README.md -S
```

结果：

- JSON 解析通过。
- 上述路径范围未发现 `/home/xie` 残留。

注意：

- 默认 `localization.map_path = ""` 会导致未注入地图路径时进入 `WAIT_MAP` 或地图加载失败，这是预期的安全模板行为。
- 离线脚本 `tools/run_offline_localization.sh` 会复制配置到临时目录，并注入 `map_path`、`trajectory_path`、`initial_pose` 和可选 matching method。

## 4. CPU/GPU 配置拆分结果

默认配置：

- `config/localization.json`
  - `localization.matching.method = "cpu_gicp"`
  - `localization.map_path = ""`

新增模板：

- `config/localization.cpu.json`
  - CPU baseline 模板。
  - matching backend：`cpu_gicp`。
- `config/localization.gpu.json`
  - GPU VGICP 模板。
  - matching backend：`gpu_vgicp`。
  - 使用前需要 CUDA 构建和 `gtsam_points` CUDA 支持。
- `config/localization.template.json`
  - 通用无个人绝对路径模板。
  - 当前与 CPU-safe 语义一致。

GPU 构建命令已写入 README：

```bash
colcon build --symlink-install --packages-select glim_localization --cmake-args -DBUILD_WITH_CUDA=ON
```

约束：

- 仅设置 `BUILD_WITH_CUDA=ON` 不足以保证 GPU backend 可用。
- 还需要 `gtsam_points` 检测到 CUDA 支持，即 CMake 侧 `GTSAM_POINTS_USE_CUDA` 为 true。
- 如果运行时请求 `gpu_vgicp` 但未编译 GPU 支持，现有代码会 warning 并回退 CPU GICP。

## 5. .gitignore / pycache / LICENSE / package.xml 处理结果

执行命令：

```bash
git rm -r --cached tools/monitor/__pycache__
```

结果：

- pycache 文件已从 Git 索引移除。
- 本地生成文件不作为源码提交。

新增 `.gitignore` 内容：

```gitignore
__pycache__/
*.py[cod]
build/
install/
log/
.cache/
.DS_Store
.vscode/
.idea/
```

新增 `LICENSE`：

- MIT License。
- Copyright 使用 `2026 xie-yuguang and contributors`。

`package.xml`：

- 当前项目 maintainer：
  - `xie-yuguang <xie-yuguang@users.noreply.github.com>`
- 当前项目 author：
  - `xie-yuguang <xie-yuguang@users.noreply.github.com>`
- upstream attribution：
  - `Kenji Koide (upstream GLIM) <k.koide@aist.go.jp>`
- description 明确：
  - `Localization-only runtime modules based on / integrating with koide3/Glim. This package is not the official GLIM project.`

## 6. launch 文件结果

新增文件：

- `launch/localization.launch.py`

功能：

- 包装 `glim_ros` 包的 `glim_rosnode`。
- 暴露 launch argument：`config_path`。
- 默认值使用当前包安装路径：
  - `FindPackageShare("glim_localization") / "config"`
- 不写死任何个人绝对路径。

启动方式：

```bash
ros2 launch glim_localization localization.launch.py config_path:=/path/to/config_dir
```

静态验证：

```bash
python3 -m py_compile launch/localization.launch.py
source /home/xie/Glim/install/setup.bash
ros2 launch glim_localization localization.launch.py --show-args
```

`--show-args` 输出摘要：

```text
Arguments (pass arguments as '<name>:=<value>'):

    'config_path':
        Directory containing config.json and localization.json.
```

注意：

- `ros2 launch --show-args` 会在 install share 目录生成 Python `__pycache__`，该生成物未进入 Git。
- `CMakeLists.txt` 已对 `launch/__pycache__` 和 `*.pyc` 加安装排除规则。

## 7. 构建结果

执行命令：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：

- 成功。
- `Summary: 1 package finished`。
- 安装确认包含：
  - `share/glim_localization/config/localization.cpu.json`
  - `share/glim_localization/config/localization.gpu.json`
  - `share/glim_localization/config/localization.template.json`
  - `share/glim_localization/launch/localization.launch.py`

构建环境摘要：

- ROS2 Humble。
- `ament_cmake 1.3.11`
- `rclcpp 16.0.11`
- 默认 RMW：`rmw_cyclonedds_cpp`
- 本轮未启用 CUDA 构建。

## 8. 测试结果

执行命令：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：

- 成功。
- `100% tests passed, 0 tests failed out of 15`
- `Summary: 1 package finished`

通过的测试：

- `test_localization_options`
- `test_map_format_checker`
- `test_glim_map_loader`
- `test_localization_map`
- `test_submap_index`
- `test_cpu_map_registration`
- `test_scan_context_relocalizer`
- `test_geometric_verifier`
- `test_localization_contracts`
- `test_trajectory_writer`
- `test_run_offline_localization_syntax`
- `test_run_offline_localization_runtime`
- `test_run_standard_experiment_syntax`
- `test_plugin_module_loading_odometry`
- `test_plugin_module_loading_extension`

测试备注：

- `test_glim_map_loader` 因未设置 `GLIM_LOCALIZATION_TEST_MAP`，跳过真实 GLIM dump loading test；这属于现有测试行为。

## 9. 地图检查结果

检测路径存在：

```text
/home/xie/Glim/data/map_data/ceshichang_128lidar
```

执行命令：

```bash
source /home/xie/Glim/install/setup.bash
ros2 run glim_localization glim_localization_map_info /home/xie/Glim/data/map_data/ceshichang_128lidar
```

结果摘要：

```text
valid: true
detected_format: glim_dump
compatibility: supported
num_submaps: 232
num_all_frames: 3471
num_matching_cost_factors: 18967
missing_submap_directories: 0
missing_submap_data_files: 0
loaded_submaps: 232
loaded_vs_requested: 232/232
skipped_submaps: 0
merged_submap_points: 11349833
origin_min: -59.1177 -26.8874 -2.07708
origin_max:  94.9779  42.2848 0.877271
index_resolution: 20
index_cells: 28
index_max_cell_size: 49
```

结论：

- 本地测试地图格式有效。
- 232 个 submap 均可加载。

## 10. benchmark 结果

执行命令：

```bash
source /home/xie/Glim/install/setup.bash
ros2 run glim_localization benchmark_localization /home/xie/Glim/data/map_data/ceshichang_128lidar 100 40 8 20
```

结果摘要：

```text
submaps: 232
load_ms: 580.264
queries: 100
max_distance: 40
max_num_submaps: 8
linear_total_ms: 0.215448
linear_avg_ms: 0.00215448
linear_hits: 800
index_resolution: 20
index_cells: 28
index_max_cell_size: 49
index_total_ms: 0.346318
index_avg_ms: 0.00346318
index_hits: 800
```

结论：

- 当前 232 submap 地图下，submap 查询本身不是主要瓶颈。
- 后续性能瓶颈仍应重点关注 target cloud merge、registration、debug cloud 发布和 smoother。

## 11. 真实 bag baseline 结果

检测到 bag 和 map 均存在：

```text
/home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39
/home/xie/Glim/data/map_data/ceshichang_128lidar
```

bag 信息：

```text
Bag size: 2.7 GiB
Duration: 383.779581605s
Messages: 80597
Topic: /imu              Count: 76759
Topic: /rslidar_points   Count: 3838
```

执行命令：

```bash
cd /home/xie/Glim
source install/setup.bash
timeout 600s bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --output-dir /tmp/glim_localization_baseline_cpu
```

结果：

- 外层命令 exit code：`124`，由 `timeout 600s` 触发。
- monitor 记录的子命令 exit code：`143`，表示被终止。
- 标准实验未完整完成，未生成：
  - `resource_report/resource_report.html`
  - `trajectory/trajectory_2d.png`
  - `trajectory/trajectory_3d.png`
  - `trajectory/trajectory_stats.txt`
  - `benchmark/benchmark.txt`
  - `experiment_record.md`
- 已生成 artifacts：
  - `/tmp/glim_localization_baseline_cpu/experiment_manifest.txt`
  - `/tmp/glim_localization_baseline_cpu/monitor/command.stdout.log`
  - `/tmp/glim_localization_baseline_cpu/monitor/command.stderr.log`
  - `/tmp/glim_localization_baseline_cpu/monitor/summary.txt`
  - `/tmp/glim_localization_baseline_cpu/monitor/summary.json`
  - `/tmp/glim_localization_baseline_cpu/trajectory/glim_localization_traj.txt`

轨迹文件：

```text
3838 /tmp/glim_localization_baseline_cpu/trajectory/glim_localization_traj.txt
```

首行和尾部显示轨迹严重发散：

```text
1743058060.240581512 -0.040334858 0.784889336 -0.990241074 ... 4 0.896812356
1743058443.938464642 77827.810865322 -121579.551239322 -740582.744115944 ... 5 0.644417958
```

关键日志：

```text
loaded localization map: format=glim_dump compatibility=supported loaded=232/232 skipped=0 points=11349833
WAIT_MAP -> WAIT_INITIAL_POSE reason=map_ready
WAIT_INITIAL_POSE -> INITIALIZING reason=config_initial_pose_ready
INITIALIZING -> TRACKING reason=tracking_registration_accepted
scan-to-map registration rejected frame=126 ... reason=large_translation_correction
TRACKING -> DEGRADED reason=registration_rejected
DEGRADED -> LOST reason=registration_rejections_exceeded
LOST -> RELOCALIZING reason=relocalization_query
relocalization query frame=3837 candidates=0
RELOCALIZING -> LOST reason=relocalization_no_candidates
```

stderr 中出现 GTSAM smoother warning：

```text
Indeterminant linear system detected while working near variable
8646911284551353765 (Symbol: x1445).

Indeterminant linear system detected while working near variable
8646911284551355288 (Symbol: x2968).
```

判断：

- baseline 尝试未成功，不能作为稳定 CPU baseline。
- bag、map、topic 基本匹配，失败不像是缺 topic 或 map 无效。
- 主要问题更像是定位配置/初始位姿/registration gating/状态恢复策略导致 tracking 早期失稳，随后重定位候选不足。
- 另外 `glim_rosbag` 在处理完 3838 个点云帧后未在 600 秒内退出，需要单独确认是 ROS bag 退出逻辑、GLIM 模块析构、后台线程、还是脚本等待问题。
- monitor 当前只采到 wrapper bash PID，资源统计没有覆盖实际 `glim_rosbag` 子进程，`avg_cpu_pct_ps` 等指标不可作为真实定位性能数据。

## 12. 当前仍然阻塞的问题

P0 / High：

- 真实 bag CPU baseline 未完成，外层 timeout。
- 轨迹严重发散，说明 `"0 0 0 0 0 0"` 初值不适合作为该 map/bag 的有效 baseline，或当前 scan-to-map/IMU smoother 组合在该配置下不稳定。
- 后段持续 `RELOCALIZING -> LOST` 且 `candidates=0`，重定位候选覆盖或触发策略不足。
- GTSAM fixed-lag smoother 出现 underconstrained warning，需要定位哪些因子/状态在 LOST/RELOCALIZING 阶段约束不足。

P1 / Medium：

- `run_standard_experiment.sh` 在主命令 timeout/失败时不会继续生成 trajectory plot、benchmark 和 experiment record；应增加失败后 artifacts 汇总。
- 资源监控目前按主 wrapper PID 采样，未正确覆盖实际 `glim_rosbag` 子进程。
- 默认 launch 可启动，但默认 `map_path=""`，因此在线运行仍需要用户传入配置目录或运行前注入 map。

## 13. 建议下一轮 Codex 处理的问题

建议按以下顺序继续：

1. P0：为真实 bag 找到正确 initial pose。
   - 从 map graph、bag 起点、历史实验记录或人工 RViz 对齐获取 `x y z roll pitch yaw`。
   - 用 `run_standard_experiment.sh --initial-pose "<correct pose>" --matching-method cpu_gicp` 重新跑 baseline。
2. P0：给 `run_standard_experiment.sh` 增加失败后收尾逻辑。
   - 即使命令失败，也生成 `experiment_record.md`。
   - 保存 stdout/stderr 摘要。
   - 若 trajectory 存在，仍尝试生成 trajectory stats/plot。
3. P1：修复资源监控 PID 采样口径。
   - 采样进程树，而不是只采 wrapper bash。
   - 或让 `run_with_time.sh` 记录实际 exec 后进程。
4. P1：增加 offline run 的 `playback_duration` / `start_offset` 支持。
   - 先用 30 秒短片段建立快速 regression baseline。
5. P1：调查 GTSAM underconstrained warning。
   - 重点看 LOST/RELOCALIZING 时是否仍向 smoother 写入欠约束状态。
   - 检查 rejected registration 后 factor 注入逻辑。
6. P1：为 launch 增加可选参数文档。
   - `config_path`
   - `use_sim_time`
   - 是否需要 remap `/imu`、`/rslidar_points`

## 14. Git diff 摘要

`git status --short`：

```text
 M CMakeLists.txt
 M README.md
 M config/config.json
 M config/localization.json
 M package.xml
D  tools/monitor/__pycache__/generate_usage_report.cpython-312.pyc
D  tools/monitor/__pycache__/plot_usage.cpython-312.pyc
D  tools/monitor/__pycache__/summarize_usage.cpython-312.pyc
D  tools/monitor/__pycache__/usage_report_lib.cpython-310.pyc
D  tools/monitor/__pycache__/usage_report_lib.cpython-312.pyc
?? .gitignore
?? GLIM_LOCALIZATION_ANALYSIS.md
?? GLIM_RELATION_MATRIX.md
?? IMMEDIATE_FIXES.md
?? LICENSE
?? OPTIMIZATION_ROADMAP.md
?? PHASE0_BASELINE_HARDENING_REPORT.md
?? RUN_AND_BUILD_NOTES.md
?? config/localization.cpu.json
?? config/localization.gpu.json
?? config/localization.template.json
?? launch/
```

`git diff --stat`：

```text
 CMakeLists.txt           |  4 ++++
 README.md                | 37 +++++++++++++++++++++++++++++++++----
 config/config.json       |  2 +-
 config/localization.json |  4 ++--
 package.xml              |  9 ++++++---
 5 files changed, 46 insertions(+), 10 deletions(-)
```

`git diff --cached --stat`：

```text
 .../__pycache__/generate_usage_report.cpython-312.pyc   | Bin 2617 -> 0 bytes
 tools/monitor/__pycache__/plot_usage.cpython-312.pyc    | Bin 2665 -> 0 bytes
 .../monitor/__pycache__/summarize_usage.cpython-312.pyc | Bin 12647 -> 0 bytes
 .../__pycache__/usage_report_lib.cpython-310.pyc        | Bin 28565 -> 0 bytes
 .../__pycache__/usage_report_lib.cpython-312.pyc        | Bin 44711 -> 0 bytes
 5 files changed, 0 insertions(+), 0 deletions(-)
```

`git diff -- config/config.json config/localization.json package.xml .gitignore`：

```diff
diff --git a/config/config.json b/config/config.json
index 1ff4540..9677f22 100644
--- a/config/config.json
+++ b/config/config.json
@@ -1,6 +1,6 @@
 {
   "global": {
-    "config_path": "/home/xie/Glim/src/Glim_localization/config",
+    "config_path": "",
     "config_ros": "localization.json",
     "config_logging": "localization.json",
     "config_viewer": "localization.json",
diff --git a/config/localization.json b/config/localization.json
index 5e7cfba..a838716 100644
--- a/config/localization.json
+++ b/config/localization.json
@@ -21,7 +21,7 @@
     "num_threads": 4
   },
   "localization": {
-    "map_path": "/home/xie/Glim/data/map_data/ceshichang_128lidar/",
+    "map_path": "",
     "map": {
       "load_voxelmaps": true,
       "load_raw_frames": false,
@@ -46,7 +46,7 @@
       "index_resolution": 20.0
     },
     "matching": {
-      "method": "gpu_vgicp",  
+      "method": "cpu_gicp",
       "max_iterations": 20,
       "min_score": 0.35,
       "min_inliers": 30,
diff --git a/package.xml b/package.xml
index c601bac..846ed01 100644
--- a/package.xml
+++ b/package.xml
@@ -3,9 +3,10 @@
 <package format="3">
   <name>glim_localization</name>
   <version>0.1.0</version>
-  <description>Localization-only runtime modules based on GLIM</description>
-  <maintainer email="k.koide@aist.go.jp">k.koide</maintainer>
-  <author email="k.koide@aist.go.jp">k.koide</author>
+  <description>Localization-only runtime modules based on / integrating with koide3/Glim. This package is not the official GLIM project.</description>
+  <maintainer email="xie-yuguang@users.noreply.github.com">xie-yuguang</maintainer>
+  <author email="xie-yuguang@users.noreply.github.com">xie-yuguang</author>
+  <author email="k.koide@aist.go.jp">Kenji Koide (upstream GLIM)</author>
   <license>MIT</license>
 
   <buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
@@ -23,6 +24,8 @@
   <depend condition="$ROS_VERSION == 2">geometry_msgs</depend>
   <depend condition="$ROS_VERSION == 2">tf2_ros</depend>
   <depend condition="$ROS_VERSION == 2">visualization_msgs</depend>
+  <exec_depend condition="$ROS_VERSION == 2">launch</exec_depend>
+  <exec_depend condition="$ROS_VERSION == 2">launch_ros</exec_depend>
 
   <export>
     <build_type condition="$ROS_VERSION == 2">ament_cmake</build_type>
```

说明：

- `.gitignore` 是新增未跟踪文件，普通 `git diff -- .gitignore` 不显示内容；其内容已在第 5 节列出。
- `git diff --stat` 不包含新增未跟踪文件和已 staged 的 pycache 删除；因此本节额外列出 `git diff --cached --stat` 和新增文件清单。
