# glim_localization 快速上手

这是一份面向第一次运行 `glim_localization` 的最小闭环说明。目标是在最短路径内完成一次离线 localization-only 验证：

```text
GLIM dump 地图 + ROS2 rosbag + 初始位姿 -> 离线定位 -> 输出轨迹文件
```

本文只覆盖当前代码中已经落地的离线 rosbag 流程，不展开在线 ROS2、重定位调参和完整架构细节。

本文职责边界：

- 负责：第一次跑通离线最小闭环
- 不负责：完整部署、在线运行、参数全表、FAQ、资源监测

对应文档：

- 基线与契约：`docs/baseline_and_contract.md`
- 完整部署：`docs/deployment_and_run.md`
- 参数参考：`docs/config_reference.md`
- ROS 接口：`docs/ros_interface.md`
- 工程化与回归：`docs/engineering_playbook.md`
- 常见问题：`docs/faq.md`
- 资源监测：`docs/resource_monitoring.md`

## 1. 这是什么

`glim_localization` 是基于 `glim` 的 localization-only 模块。它不在线建图，而是加载已有 GLIM dump 地图，把当前 LiDAR/IMU 数据匹配到固定地图上，并输出定位轨迹。

当前最小闭环使用：

- `glim_ros2` 的 `glim_rosbag` 回放 rosbag。
- `glim_localization` 的 `libodometry_estimation_localization_cpu.so` 作为 GLIM odometry 动态模块。
- `glim_localization/tools/run_offline_localization.sh` 生成临时配置并启动离线定位。
- `TrajectoryWriter` 输出轨迹文件，默认 `/tmp/glim_localization_traj.txt`。

## 2. 需要准备什么

你需要准备：

1. 已编译可用的 ROS2 + GLIM workspace。

   workspace 中至少包含：

   ```text
   glim/
   glim_ros2/
   glim_localization/
   ```

2. 一个 GLIM dump 地图目录。

   最小结构应类似：

   ```text
   <map_path>/
     graph.txt
     000000/
       data.txt
       ...
     000001/
       data.txt
       ...
   ```

   `graph.bin` 和 `values.bin` 缺失时当前 checker 会给 warning，但 submap loader 仍可尝试加载。

3. 一个 ROS2 rosbag。

   bag 中至少应包含：

   - `/imu`
   - `/points`

   如果你的 topic 名不同，需要修改 `glim_localization/config/localization.json` 中：

   ```json
   {
     "glim_ros": {
       "imu_topic": "/imu",
       "points_topic": "/points"
     }
   }
   ```

4. 一个初始位姿。

   当前离线最小闭环推荐直接用脚本参数传入：

   ```text
   x y z roll pitch yaw
   ```

   单位：

   - `x y z`：米
   - `roll pitch yaw`：弧度

5. 一个用于观察结果的 RViz2。

   当前最小实时可视化直接复用 `liblocalization_publisher.so` 的 ROS2 topic 输出，不需要额外启动新的 viewer 进程。

## 3. 3~5 步快速跑通流程

### Step 1：编译

在 colcon workspace 根目录执行：

```bash
colcon build --symlink-install --packages-up-to glim_ros glim_localization
source install/setup.bash
```

确认关键动态库存在：

```bash
ls install/glim_localization/lib/libodometry_estimation_localization_cpu.so
ls install/glim_localization/lib/liblocalization_publisher.so
```

### Step 2：检查 rosbag topic

```bash
ros2 bag info /path/to/your_bag
```

确认 bag 内 topic 与 `localization.json` 一致。默认需要：

```text
/imu
/points
```

### Step 3：检查地图

推荐先运行 map info 工具：

```bash
ros2 run glim_localization glim_localization_map_info /path/to/glim_dump
```

成功时应看到：

```text
valid: true
loaded_submaps: <N>
```

如果该工具路径不存在，说明 `glim_localization` 还没有成功编译/安装，先回到 Step 1。

### Step 4：运行离线 localization

使用源码树中的脚本：

```bash
bash glim_localization/tools/run_offline_localization.sh \
  /path/to/your_bag \
  /path/to/glim_dump \
  0.0 0.0 0.0 0.0 0.0 0.0 \
  /tmp/glim_localization_traj.txt
```

参数含义：

```text
run_offline_localization.sh \
  <rosbag_path> \
  <map_path> \
  <x> <y> <z> <roll> <pitch> <yaw> \
  [trajectory_path]
```

脚本现在同时支持源码树和安装后的 ROS2 布局。它会从可用的 `config/` 模板目录复制配置到 `/tmp/glim_localization_config.XXXXXX`，再调用：

```bash
ros2 run glim_ros glim_rosbag "<rosbag_path>" --ros-args -p "config_path:=<临时配置目录>"
```

### Step 5：检查轨迹输出

```bash
wc -l /tmp/glim_localization_traj.txt
head /tmp/glim_localization_traj.txt
tail /tmp/glim_localization_traj.txt
```

只要文件存在且持续写入多行，就说明离线闭环已经跑通。

### Step 6：在 RViz 中检查实时可视化

运行 bag 时打开另一个终端：

```bash
source install/setup.bash
rviz2 -d install/glim_localization/share/glim_localization/rviz/localization.rviz
```

在 RViz 中建议：

- `Fixed Frame` 设为 `map`
- 添加 `TF`
- 添加 `Odometry`，topic 设为 `/localization/odom`
- 添加 `Pose`，topic 设为 `/localization/pose`
- 添加 `Path`，topic 设为 `/localization/trajectory`
- 添加 `PointCloud2`，topic 设为 `/localization/debug/input_scan`
- 添加 `PointCloud2`，topic 设为 `/localization/debug/current_scan`
- 添加 `PointCloud2`，topic 设为 `/localization/debug/local_target_map`
- 添加 `Marker`，topic 设为 `/localization/debug/active_submaps`

这些 topic 中：

- `/localization/debug/input_scan` 是当前输入 scan，保留 `sensor_frame` 下的传感器视角
- `/localization/debug/current_scan` 是已经 deskew 并对齐到 `map` frame 的当前 scan
- `/localization/debug/local_target_map` 是当前 active submaps 合并出的局部目标地图
- `/localization/debug/active_submaps` 显示当前状态、matching score 和 active submap ids

## 4. 最小配置示例

脚本会自动生成临时配置并覆盖最关键字段。你也可以手动参考下面的最小配置片段：

```json
{
  "glim_ros": {
    "imu_topic": "/imu",
    "points_topic": "/points",
    "image_topic": "",
    "enable_local_mapping": false,
    "enable_global_mapping": false,
    "extension_modules": ["liblocalization_publisher.so"]
  },
  "odometry_estimation": {
    "so_name": "libodometry_estimation_localization_cpu.so"
  },
  "localization": {
    "map_path": "/path/to/glim_dump",
    "trajectory_path": "/tmp/glim_localization_traj.txt",
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
      "max_consecutive_rejections": 3,
      "max_correspondence_distance": 2.0,
      "pose_prior_precision": 1000.0,
      "num_threads": 4
    },
    "ros": {
      "pose_topic": "/localization/pose",
      "odom_topic": "/localization/odom",
      "trajectory_topic": "/localization/trajectory",
      "input_scan_topic": "/localization/debug/input_scan",
      "current_scan_topic": "/localization/debug/current_scan",
      "target_map_topic": "/localization/debug/local_target_map",
      "active_submaps_topic": "/localization/debug/active_submaps"
    }
  },
  "sensors": {
    "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "intensity_field": "intensity",
    "ring_field": ""
  }
}
```

最小闭环里最重要的是：

- `odometry_estimation.so_name = "libodometry_estimation_localization_cpu.so"`
- `glim_ros.enable_local_mapping = false`
- `glim_ros.enable_global_mapping = false`
- `localization.map_path` 指向已有 GLIM dump。
- `localization.initial_pose.source = "config"`。

## 5. 最小运行命令

把下面三个路径换成你的实际路径：

```bash
source install/setup.bash

bash glim_localization/tools/run_offline_localization.sh \
  /data/bags/demo_bag \
  /data/maps/glim_dump \
  0.0 0.0 0.0 0.0 0.0 0.0 \
  /tmp/glim_localization_traj.txt
```

如果初始位姿已知，例如车辆在地图坐标系下大约位于 `(12.3, -4.5, 0.2)`，yaw 约 `1.57`：

```bash
bash glim_localization/tools/run_offline_localization.sh \
  /data/bags/demo_bag \
  /data/maps/glim_dump \
  12.3 -4.5 0.2 0.0 0.0 1.57 \
  /tmp/glim_localization_traj.txt
```

## 6. 成功运行后的预期输出

### 日志中应看到

典型成功日志会包含类似内容：

```text
localization trajectory output: /tmp/glim_localization_traj.txt
valid GLIM map dump: <N> submaps in <map_path>
loaded localization map: <N> submaps
localization map loaded: <N> submaps
localization submap index built: ...
using config initial pose for localization (world == map)
rebuilt localization target map: ... active_submaps=[...]
scan-to-map registration accepted ...
```

如果启用了 ROS2 extension，也可能看到：

```text
localization publisher extension initialized
```

并且可以在另一个终端看到这些 topic：

```bash
ros2 topic list | grep /localization
```

最少应包含：

```text
/localization/status
/localization/pose
/localization/odom
/localization/trajectory
/localization/debug/input_scan
/localization/debug/current_scan
/localization/debug/local_target_map
/localization/debug/active_submaps
```

### 轨迹文件

默认轨迹文件：

```text
/tmp/glim_localization_traj.txt
```

每行格式：

```text
stamp x y z qx qy qz qw status_int matching_score
```

其中 `status_int` 来自当前状态枚举：

```text
0 WAIT_MAP
1 WAIT_INITIAL_POSE
2 INITIALIZING
3 TRACKING
4 LOST
5 RELOCALIZING
```

最小检查：

```bash
tail /tmp/glim_localization_traj.txt
```

如果最后几行的 `status_int` 多数是 `3`，通常表示 tracking 正常。

### 离线轨迹出图

只要 `localization.trajectory_path` 非空，定位运行过程中就会生成 `glim_localization_traj.txt`。

如果环境里还没有 `matplotlib`：

```bash
python3 -m pip install matplotlib
```

生成 2D 和 3D 轨迹图：

```bash
python3 tools/plot_trajectory.py /tmp/glim_localization_traj.txt
```

只生成 2D 俯视图，并显示方向箭头和时间着色：

```bash
python3 tools/plot_trajectory.py /tmp/glim_localization_traj.txt \
  --plot-2d \
  --show-arrows \
  --time-color \
  --output /tmp/glim_localization_topdown.png
```

脚本会输出：

- 帧数
- 总路程
- 起点/终点坐标
- xyz bounds

生成图片的含义：

- 2D 图主要看平面路线是否连续、是否突然跳变
- 3D 图主要看 z 方向是否稳定
- 绿色圆点是起点，红色 X 是终点
- 开启 `--time-color` 后，颜色变化表示轨迹推进顺序

## 7. 最常见的 5 个报错和解决办法

### 1. `localization.map_path is empty`

原因：

- `localization.map_path` 没有设置。
- 没有使用脚本，或手动配置目录不正确。

解决：

- 使用 `run_offline_localization.sh`，第二个参数传入地图目录。
- 或手动修改 `localization.json`：

```json
"localization": {
  "map_path": "/path/to/glim_dump"
}
```

### 2. `missing graph.txt` 或 `invalid GLIM map dump`

原因：

- 地图目录不是 GLIM dump。
- `graph.txt` 不存在。
- `graph.txt` 前三行格式不符合当前 checker 解析逻辑。

解决：

```bash
ls /path/to/glim_dump/graph.txt
head -n 3 /path/to/glim_dump/graph.txt
ros2 run glim_localization glim_localization_map_info /path/to/glim_dump
```

`graph.txt` 应包含类似：

```text
num_submaps: 10
num_all_frames: 1234
num_matching_cost_factors: 0
```

### 3. 找不到 `libodometry_estimation_localization_cpu.so`

原因：

- `glim_localization` 没有编译成功。
- 没有 `source install/setup.bash`。
- 当前终端环境找不到安装后的库。

解决：

```bash
colcon build --symlink-install --packages-select glim_localization
source install/setup.bash
ls install/glim_localization/lib/libodometry_estimation_localization_cpu.so
```

同时确认配置中：

```json
"odometry_estimation": {
  "so_name": "libodometry_estimation_localization_cpu.so"
}
```

### 4. rosbag 里没有 `/imu` 或 `/points`

原因：

- bag 的 topic 名和 `localization.json` 默认配置不一致。

解决：

先看 bag：

```bash
ros2 bag info /path/to/your_bag
```

然后修改：

```json
"glim_ros": {
  "imu_topic": "/your/imu_topic",
  "points_topic": "/your/points_topic"
}
```

再重新运行脚本或手动运行 `glim_rosbag`。

### 5. 一直 `scan-to-map registration rejected` 或进入 `LOST`

原因通常是：

- 初始位姿偏差过大。
- `T_lidar_imu` 与建图时不一致。
- 地图和 rosbag 不是同一场景。
- `target_map.max_distance` 太小，查不到合适 submap。
- `matching.min_score`、`matching.min_inliers` 或 pose correction gating 太严格。

解决建议：

1. 先给更接近真实位置的初始位姿。
2. 检查 `sensors.T_lidar_imu` 是否和建图配置一致。
3. 增大 `target_map.max_distance`，例如从 `40.0` 调到 `60.0`。
4. 排查阶段可临时降低 `matching.min_score` 或 `matching.min_inliers`。
5. 查看日志里的 reject reason，例如：

```text
low_score
few_inliers
large_translation_correction
large_rotation_correction
```
