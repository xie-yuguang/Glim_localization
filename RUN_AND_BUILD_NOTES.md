# Glim_localization 构建与运行记录

## 环境

| 项 | 值 |
|---|---|
| 日期 | 2026-05-10 |
| workspace | `/home/xie/Glim` |
| package | `/home/xie/Glim/src/Glim_localization` |
| ROS2 | Humble，命令输出中 `rclcpp 16.0.11`、`ament_cmake 1.3.11` |
| RMW | `rmw_cyclonedds_cpp` |

## 包列表

```bash
colcon list
```

输出：

```text
glim              src/glim              (ros.ament_cmake)
glim_ext          src/glim_ext          (ros.ament_cmake)
glim_localization src/Glim_localization (ros.ament_cmake)
glim_ros          src/glim_ros2         (ros.ament_cmake)
```

## 推荐构建命令

全依赖构建：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to glim_ros glim_localization
```

单包增量构建：

```bash
cd /home/xie/Glim
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select glim_localization
```

GPU 构建需要确认 `gtsam_points` 已启用 CUDA：

```bash
colcon build --symlink-install --packages-select glim_localization --cmake-args -DBUILD_WITH_CUDA=ON
```

## 实际构建结果

执行：

```bash
source /opt/ros/*/setup.bash 2>/dev/null || true
source install/setup.bash
colcon build --symlink-install --packages-select glim_localization --event-handlers console_direct+
```

结果：成功。

关键生成物：

```text
/home/xie/Glim/build/glim_localization/libglim_localization.so
/home/xie/Glim/build/glim_localization/libodometry_estimation_localization_cpu.so
/home/xie/Glim/build/glim_localization/liblocalization_publisher.so
/home/xie/Glim/build/glim_localization/glim_localization_map_info
/home/xie/Glim/build/glim_localization/benchmark_localization
```

安装树中可执行入口：

```text
/home/xie/Glim/install/glim_ros/lib/glim_ros/glim_rosbag
/home/xie/Glim/install/glim_ros/lib/glim_ros/glim_rosnode
```

## 实际测试结果

执行：

```bash
source /opt/ros/*/setup.bash 2>/dev/null || true
source install/setup.bash
colcon test --packages-select glim_localization --event-handlers console_direct+ --ctest-args --output-on-failure
```

结果：15/15 passed。

测试列表：

```text
test_localization_options
test_map_format_checker
test_glim_map_loader
test_localization_map
test_submap_index
test_cpu_map_registration
test_scan_context_relocalizer
test_geometric_verifier
test_localization_contracts
test_trajectory_writer
test_run_offline_localization_syntax
test_run_offline_localization_runtime
test_run_standard_experiment_syntax
test_plugin_module_loading_odometry
test_plugin_module_loading_extension
```

备注：`test_glim_map_loader` 在未设置 `GLIM_LOCALIZATION_TEST_MAP` 时跳过真实地图加载。

## 真实地图检查

执行：

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
loaded_submaps: 232
merged_submap_points: 11349833
index_resolution: 20
index_cells: 28
index_max_cell_size: 49
```

## 地图 query benchmark

执行：

```bash
ros2 run glim_localization benchmark_localization /home/xie/Glim/data/map_data/ceshichang_128lidar 100 40 8 20
```

结果摘要：

```text
load_ms: 549.705
linear_avg_ms: 0.00240266
index_avg_ms: 0.00348129
linear_hits: 800
index_hits: 800
```

结论：232 submap 规模下 nearby query 很快；优先 profile target cloud merge 和 registration。

## 离线运行命令

脚本方式：

```bash
cd /home/xie/Glim
source install/setup.bash
bash src/Glim_localization/tools/run_offline_localization.sh \
  /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  /home/xie/Glim/data/map_data/ceshichang_128lidar \
  0 0 0 0 0 0 \
  /tmp/glim_localization_traj.txt
```

标准实验方式：

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --output-dir /tmp/glim_localization_baseline_cpu
```

注意：本次分析未跑完整真实 bag localization，因为该操作可能耗时较长；报告中只记录了构建、测试、真实地图加载和 query benchmark。

## 在线运行命令

```bash
cd /home/xie/Glim
source install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/path/to/config_dir
```

当前没有独立 `glim_localization` ROS node，也没有 launch 文件。

## 常见风险

- `config/config.json` 和 `config/localization.json` 当前含 `/home/xie/...` 绝对路径。
- `localization.matching.method` 当前默认 `gpu_vgicp`，但 CMake 默认 `BUILD_WITH_CUDA=OFF`。
- `ros2 run glim_localization glim_localization_map_info` 已可用，但主在线/离线节点来自 `glim_ros` 包。
