# glim_localization 部署与运行指南

本文档面向从干净环境开始部署 `glim_localization` 的用户，覆盖依赖安装、workspace 布局、编译、配置、地图准备、离线运行、在线运行、验证和故障排查。

文档依据当前工作区实际代码编写：

- `glim/CMakeLists.txt`
- `glim/package.xml`
- `glim_ros2/CMakeLists.txt`
- `glim_ros2/package.xml`
- `glim_localization/CMakeLists.txt`
- `glim_localization/package.xml`
- `glim_localization/config/localization.json`
- `glim_localization/tools/run_offline_localization.sh`

注意：

- 当前文档以 ROS2 + colcon 工作流为主。
- 第三方库安装方式会随系统、ROS2 发行版、CUDA 环境变化。无法从仓库完全确定的安装命令均标注为“示例命令，需按本机环境调整”。
- 当前 `glim_localization` 没有独立 `glim_localization_ros` 包，在线/离线运行入口来自 `glim_ros2` 包，ROS2 包名是 `glim_ros`。

## 1. 系统要求

推荐环境：

- Ubuntu 22.04 + ROS2 Humble，或其他已验证的 ROS2 发行版。
- CMake >= 3.16。
- C++17 编译器。
- colcon。
- rosdep。
- 已安装或可编译的：
  - GTSAM 4.2
  - gtsam_points >= 1.2.0
  - Eigen3
  - Boost
  - OpenMP
  - spdlog
  - fmt
- 可选：
  - CUDA：用于 GLIM/GTSAM_POINTS GPU 路径和 `gpu_vgicp`。
  - OpenCV：用于 GLIM/GLIM ROS 图像相关功能；localization 最小闭环不依赖图像。
  - Iridescence：用于 GLIM viewer；localization 最小闭环可关闭 viewer。

最小离线 localization-only 验证需要：

- `glim`
- `glim_ros2`
- `glim_localization`
- 一个 GLIM dump 地图
- 一个包含 IMU 和点云 topic 的 ROS2 rosbag

## 2. 依赖安装

### 2.1 安装 ROS2 基础环境

示例命令，需按本机环境调整：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

安装并 source ROS2。以下以 Humble 为例：

```bash
source /opt/ros/humble/setup.bash
```

如果你的 ROS2 发行版不是 Humble，请替换为实际路径，例如：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

如果系统已经初始化过 rosdep，`sudo rosdep init` 可能报已存在，可忽略。

### 2.2 安装 ROS2 包依赖

当前 `glim_ros2/package.xml` 依赖：

- `rclcpp`
- `rclcpp_components`
- `rosbag2_cpp`
- `rosbag2_compression`
- `rosbag2_storage`
- `cv_bridge`
- `image_transport`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`

当前 `glim_localization/package.xml` 依赖：

- `glim`
- `rclcpp`
- `std_msgs`
- `std_srvs`
- `nav_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`

在 workspace 准备好源码后，推荐用 rosdep 安装：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

如果当前不是标准 `src/` 布局，而是直接在仓库根目录放置多个包，可在该根目录执行：

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### 2.3 安装 GLIM 核心第三方依赖

`glim/CMakeLists.txt` 显式查找：

- `Boost REQUIRED serialization`
- `Eigen3 REQUIRED`
- `GTSAM 4.2 REQUIRED`
- `gtsam_points 1.2.0 REQUIRED`
- `OpenMP REQUIRED`
- `spdlog REQUIRED`
- `Iridescence REQUIRED`，仅当 `BUILD_WITH_VIEWER=ON`
- `OpenCV REQUIRED COMPONENTS core`，仅当 `BUILD_WITH_OPENCV=ON`

示例命令，需按本机环境调整：

```bash
sudo apt update
sudo apt install -y \
  libboost-all-dev \
  libeigen3-dev \
  libomp-dev \
  libspdlog-dev \
  libfmt-dev \
  libopencv-dev
```

GTSAM 和 gtsam_points 的安装方式无法仅从当前仓库完全确定。请按你的系统环境安装或从源码编译，使 CMake 能找到：

```text
find_package(GTSAM 4.2 REQUIRED)
find_package(gtsam_points 1.2.0 REQUIRED)
```

如果只是跑 CPU localization 最小闭环，建议先关闭 CUDA、viewer 和 OpenCV 相关路径，降低依赖复杂度。

## 3. 获取源码与 workspace 布局

推荐标准 colcon workspace：

```bash
mkdir -p ~/glim_ws/src
cd ~/glim_ws/src
```

将源码放成如下结构：

```text
~/glim_ws/src/
  glim/
  glim_ros2/
  glim_ext/
  glim_localization/
```

其中最小运行需要：

- `glim/`
- `glim_ros2/`
- `glim_localization/`

`glim_ext/` 当前不是 `glim_localization` 的直接编译依赖，但工作区已有时可以一起保留。

如果你当前使用的是本工作区这种平铺结构：

```text
<workspace_root>/
  glim/
  glim_ros2/
  glim_ext/
  glim_localization/
```

也可以直接在 `<workspace_root>` 执行 `colcon build`。

## 4. 编译步骤

### 4.1 CPU 最小闭环推荐编译

为了先跑通 localization-only，推荐关闭 CUDA、viewer 和不必要的 OpenCV 支持。示例命令，需按本机环境调整：

```bash
cd ~/glim_ws
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install \
  --packages-up-to glim_ros glim_localization \
  --cmake-args \
    -DBUILD_WITH_CUDA=OFF \
    -DBUILD_WITH_VIEWER=OFF \
    -DBUILD_WITH_OPENCV=OFF \
    -DBUILD_WITH_CV_BRIDGE=OFF
```

说明：

- `glim` 使用 `BUILD_WITH_CUDA`、`BUILD_WITH_VIEWER`、`BUILD_WITH_OPENCV`。
- `glim_ros2` 使用 `BUILD_WITH_CUDA`、`BUILD_WITH_VIEWER`、`BUILD_WITH_CV_BRIDGE`。
- `glim_localization` 使用 `BUILD_WITH_CUDA`。
- 某些 CMake 变量对部分包无效时通常只会被忽略。

### 4.2 启用默认功能编译

如果你的系统已经具备 CUDA、OpenCV 和 viewer 依赖，可直接：

```bash
cd ~/glim_ws
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install --packages-up-to glim_ros glim_localization
```

### 4.3 只重新编译 glim_localization

当 `glim` 和 `glim_ros` 已经编译成功，只改了 localization 代码：

```bash
colcon build --symlink-install --packages-select glim_localization
```

### 4.4 编译 GPU registration

当前 `glim_localization` 提供 `gpu_vgicp_map_registration.cpp`，但只有在 `BUILD_WITH_CUDA=ON` 且 `gtsam_points` 检测到 CUDA 支持时才会编译：

```bash
colcon build --symlink-install \
  --packages-select glim_localization \
  --cmake-args -DBUILD_WITH_CUDA=ON
```

如果 CMake 输出：

```text
BUILD_WITH_CUDA is ON, but gtsam_points CUDA support was not detected.
```

说明 GPU registration 未启用，运行时请求 `gpu_vgicp` 会回退 CPU GICP。

### 4.5 确认编译产物

编译完成后：

```bash
source install/setup.bash

ls install/glim_localization/lib/libglim_localization.so
ls install/glim_localization/lib/libodometry_estimation_localization_cpu.so
ls install/glim_localization/lib/liblocalization_publisher.so
ls install/glim_localization/lib/glim_localization/glim_localization_map_info
ls install/glim_localization/lib/glim_localization/benchmark_localization
```

关键产物：

- `libodometry_estimation_localization_cpu.so`：GLIM odometry 动态模块。
- `liblocalization_publisher.so`：ROS2 localization 输出扩展。
- `glim_localization_map_info`：地图检查工具。
- `benchmark_localization`：submap query benchmark。

## 5. 环境变量或 source 步骤

每个新终端都需要：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/glim_ws/install/setup.bash
```

确认 ROS2 包可见：

```bash
ros2 pkg list | grep glim
```

应至少看到：

```text
glim
glim_ros
glim_localization
```

确认动态库搜索路径包含 workspace：

```bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep glim
```

如果运行时提示找不到 `libodometry_estimation_localization_cpu.so` 或 `liblocalization_publisher.so`，通常是没有 source `install/setup.bash`。

## 6. 地图准备步骤

当前支持的地图输入是 GLIM dump 目录。

地图目录至少应包含：

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

可选文件：

```text
graph.bin
values.bin
```

当前 checker 行为：

- `graph.txt` 必须存在。
- `graph.txt` 前三项必须是：

```text
num_submaps: <non-negative int>
num_all_frames: <non-negative int>
num_matching_cost_factors: <non-negative int>
```

- 每个 submap 目录必须存在 `data.txt`。
- `graph.bin` 和 `values.bin` 缺失只会 warning，不会直接让 submap loader 失败。

检查地图：

```bash
ros2 run glim_localization glim_localization_map_info /path/to/glim_dump
```

成功输出应包含：

```text
valid: true
loaded_submaps: <N>
merged_submap_points: <M>
```

如果该命令失败，先不要跑 rosbag，先修地图路径或 dump 内容。

## 7. 配置文件准备步骤

当前配置目录：

```text
glim_localization/config/
  config.json
  localization.json
```

`config.json` 把 GLIM 需要的多个配置入口都指向 `localization.json`。运行时需要将 `config_path` 指向这个目录。

### 7.1 推荐：用离线脚本生成临时配置

离线 rosbag 最小闭环推荐使用：

```bash
glim_localization/tools/run_offline_localization.sh
```

脚本会：

- 复制 `glim_localization/config/` 到 `/tmp/glim_localization_config.XXXXXX`
- 写入地图路径
- 写入初始位姿
- 写入轨迹输出路径
- 确保：
  - `glim_ros.enable_local_mapping = false`
  - `glim_ros.enable_global_mapping = false`
  - `odometry_estimation.so_name = "libodometry_estimation_localization_cpu.so"`

### 7.2 手动准备配置

如果手动运行，请复制配置目录：

```bash
cp -r glim_localization/config /tmp/glim_localization_config
```

编辑：

```bash
nano /tmp/glim_localization_config/localization.json
```

最重要的字段：

```json
{
  "glim_ros": {
    "imu_topic": "/imu",
    "points_topic": "/points",
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
    }
  }
}
```

注意：

- `config_path` 建议传绝对路径。
- `glim_ros2` 如果收到相对 `config_path`，会按 `share/glim/<config_path>` 解析，不一定是你当前目录。

## 8. 离线运行步骤

### 8.1 检查 rosbag topic

```bash
ros2 bag info /path/to/your_bag
```

确认 bag 中有配置对应 topic：

```text
/imu
/points
```

如果 topic 不同，修改 `glim_ros.imu_topic` 和 `glim_ros.points_topic`。

### 8.2 使用脚本运行

推荐命令：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/glim_ws/install/setup.bash

bash glim_localization/tools/run_offline_localization.sh \
  /path/to/your_bag \
  /path/to/glim_dump \
  0.0 0.0 0.0 0.0 0.0 0.0 \
  /tmp/glim_localization_traj.txt
```

参数：

```text
run_offline_localization.sh \
  <rosbag_path> \
  <map_path> \
  <x> <y> <z> <roll> <pitch> <yaw> \
  [trajectory_path]
```

单位：

- `x y z`：米
- `roll pitch yaw`：弧度

### 8.3 手动运行 glim_rosbag

如果已经准备好配置目录：

```bash
ros2 run glim_ros glim_rosbag /path/to/your_bag \
  --ros-args -p config_path:=/tmp/glim_localization_config
```

`glim_rosbag` 当前源码还支持这些 ROS 参数：

```bash
ros2 run glim_ros glim_rosbag /path/to/your_bag \
  --ros-args \
  -p config_path:=/tmp/glim_localization_config \
  -p start_offset:=5.0 \
  -p playback_duration:=30.0
```

可用参数包括：

- `delay`
- `start_offset`
- `playback_duration`
- `playback_until`
- `end_time`

## 9. 在线运行步骤

在线模式使用 `glim_ros2` 的 `glim_rosnode`。当前没有独立 `glim_localization_ros` 节点。

### 9.1 准备在线配置

编辑配置：

```bash
nano /tmp/glim_localization_config/localization.json
```

确认：

```json
{
  "glim_ros": {
    "imu_topic": "/imu",
    "points_topic": "/points",
    "enable_local_mapping": false,
    "enable_global_mapping": false,
    "extension_modules": ["liblocalization_publisher.so"]
  },
  "odometry_estimation": {
    "so_name": "libodometry_estimation_localization_cpu.so"
  },
  "localization": {
    "map_path": "/path/to/glim_dump",
    "initial_pose": {
      "source": "config",
      "xyz": [0.0, 0.0, 0.0],
      "rpy": [0.0, 0.0, 0.0]
    }
  }
}
```

如果希望通过 RViz 或命令行发送初始位姿：

```json
{
  "localization": {
    "initial_pose": {
      "source": "topic"
    }
  }
}
```

### 9.2 启动在线节点

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/glim_ws/install/setup.bash

ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:=/tmp/glim_localization_config
```

### 9.3 如果使用 `/initialpose`

当前默认 topic：

```text
/initialpose
```

命令行发送示例：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

也可以在 RViz 中用 `2D Pose Estimate` 发布。

### 9.4 在线输出

加载 `liblocalization_publisher.so` 后，当前代码会发布：

- `/localization/status`，`std_msgs/msg/String`
- `/localization/odom`，`nav_msgs/msg/Odometry`
- `/localization/pose`，`geometry_msgs/msg/PoseStamped`
- `/localization/target_map`，`sensor_msgs/msg/PointCloud2`
- TF：
  - `map -> odom`
  - `odom -> base_link`
- 服务：
  - `/localization/relocalize`，`std_srvs/srv/Trigger`

手动触发重定位：

```bash
ros2 service call /localization/relocalize std_srvs/srv/Trigger {}
```

## 10. 验证步骤

### 10.1 验证包和可执行文件

```bash
ros2 pkg list | grep glim
```

检查 `glim_ros` 包导出的可执行入口：

```bash
ros2 pkg executables glim_ros
```

应能看到至少：

```text
glim_ros glim_rosbag
glim_ros glim_rosnode
```

`glim_rosbag` 当前源码没有实现专用 `--help` 参数。需要检查 usage 时，可不带 bag 路径运行：

```bash
ros2 run glim_ros glim_rosbag
```

它会输出：

```text
usage: glim_rosbag input_rosbag_path
```

### 10.2 验证地图

```bash
ros2 run glim_localization glim_localization_map_info /path/to/glim_dump
```

应看到：

```text
valid: true
loaded_submaps: <N>
```

### 10.3 验证离线轨迹文件

运行离线脚本后：

```bash
ls -lh /tmp/glim_localization_traj.txt
wc -l /tmp/glim_localization_traj.txt
tail /tmp/glim_localization_traj.txt
```

轨迹格式：

```text
stamp x y z qx qy qz qw status_int matching_score
```

状态值：

```text
0 WAIT_MAP
1 WAIT_INITIAL_POSE
2 INITIALIZING
3 TRACKING
4 LOST
5 RELOCALIZING
```

如果多数输出为 `status_int = 3`，说明 tracking 基本正常。

### 10.4 验证在线 topic

```bash
ros2 topic list | grep localization
ros2 topic echo /localization/status
ros2 topic echo /localization/pose
ros2 topic echo /localization/odom
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

检查 debug target map：

```bash
ros2 topic echo /localization/target_map --once
```

注意：当前 debug target map 只有在 `publish_debug_target_map=true` 且存在订阅者时才构建并发布。

### 10.5 运行单元测试

```bash
colcon test --packages-select glim_localization
colcon test-result --verbose
```

真实 GLIM dump loader 测试可通过环境变量指定地图：

```bash
GLIM_LOCALIZATION_TEST_MAP=/path/to/glim_dump \
  colcon test --packages-select glim_localization --ctest-args -R test_glim_map_loader
```

## 11. 故障排查

### 11.1 `find_package(GTSAM 4.2 REQUIRED)` 失败

原因：

- GTSAM 未安装。
- GTSAM 版本不匹配。
- CMake 找不到 GTSAM config。

处理：

- 安装 GTSAM 4.2。
- 或设置 `CMAKE_PREFIX_PATH` 指向 GTSAM 安装路径。

示例命令，需按本机环境调整：

```bash
export CMAKE_PREFIX_PATH=/path/to/gtsam/install:$CMAKE_PREFIX_PATH
```

### 11.2 `find_package(gtsam_points 1.2.0 REQUIRED)` 失败

原因：

- gtsam_points 未安装。
- 版本低于 1.2.0。
- 没有 source 或 CMake 路径不包含安装位置。

处理：

```bash
export CMAKE_PREFIX_PATH=/path/to/gtsam_points/install:$CMAKE_PREFIX_PATH
```

然后重新编译。

### 11.3 `Iridescence` 或 `OpenCV` 找不到

如果只跑 localization 最小闭环，可以关闭 viewer/OpenCV：

```bash
colcon build --symlink-install \
  --packages-up-to glim_ros glim_localization \
  --cmake-args \
    -DBUILD_WITH_VIEWER=OFF \
    -DBUILD_WITH_OPENCV=OFF \
    -DBUILD_WITH_CV_BRIDGE=OFF
```

### 11.4 找不到 `libodometry_estimation_localization_cpu.so`

检查：

```bash
ls install/glim_localization/lib/libodometry_estimation_localization_cpu.so
source install/setup.bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep glim_localization
```

确认配置：

```json
{
  "odometry_estimation": {
    "so_name": "libodometry_estimation_localization_cpu.so"
  }
}
```

### 11.5 找不到 `liblocalization_publisher.so`

检查：

```bash
ls install/glim_localization/lib/liblocalization_publisher.so
```

确认是在 ROS2 环境下编译：

```bash
echo $ROS_VERSION
```

应为：

```text
2
```

确认配置：

```json
{
  "glim_ros": {
    "extension_modules": ["liblocalization_publisher.so"]
  }
}
```

### 11.6 `config_path` 不生效

`glim_ros2` 中相对 `config_path` 会按 `share/glim/<config_path>` 解析。建议始终传绝对路径：

```bash
ros2 run glim_ros glim_rosbag /path/to/bag \
  --ros-args -p config_path:=/absolute/path/to/config_dir
```

配置目录内必须有：

```text
config.json
localization.json
```

### 11.7 地图加载失败

先运行：

```bash
ros2 run glim_localization glim_localization_map_info /path/to/glim_dump
```

常见原因：

- `localization.map_path` 为空。
- 地图路径不是目录。
- 缺少 `graph.txt`。
- `graph.txt` 前三行格式不对。
- 缺少 `000000/data.txt` 等 submap 文件。

### 11.8 rosbag 没有数据输入

检查 bag：

```bash
ros2 bag info /path/to/bag
```

检查配置：

```json
{
  "glim_ros": {
    "imu_topic": "/imu",
    "points_topic": "/points"
  }
}
```

topic 名必须与 bag 完全一致。

### 11.9 一直等待 `/initialpose`

原因：

- `localization.initial_pose.source = "topic"`。
- 还没有发布 `/initialpose`。

处理：

- 发布 `/initialpose`。
- 或改成配置初始位姿：

```json
{
  "localization": {
    "initial_pose": {
      "source": "config",
      "xyz": [0.0, 0.0, 0.0],
      "rpy": [0.0, 0.0, 0.0]
    }
  }
}
```

### 11.10 registration 大量 rejected 或进入 LOST

常见原因：

- 初始位姿偏差过大。
- `sensors.T_lidar_imu` 与建图时不同。
- rosbag 与地图不是同一场景。
- `target_map.max_distance` 太小。
- `matching.min_score` 或 `matching.min_inliers` 太严格。
- 点云时间同步或 IMU 加速度单位错误。

排查建议：

1. 先使用更准确的初始位姿。
2. 检查 `T_lidar_imu`。
3. 确认 bag 和地图来自同一环境。
4. 临时增大：

```json
{
  "localization": {
    "target_map": {
      "max_distance": 60.0
    }
  }
}
```

5. 临时放宽：

```json
{
  "localization": {
    "matching": {
      "min_score": 0.25,
      "min_inliers": 10,
      "max_pose_correction_translation": 5.0,
      "max_pose_correction_angle": 1.0
    }
  }
}
```

这些放宽参数用于排查，稳定运行时应根据数据质量重新收紧。

### 11.11 安装后的 `run_offline_localization.sh` 如何定位配置

当前脚本已经支持源码树和安装后的 ROS2 布局。源码树和安装后的常用入口分别是：

```bash
bash glim_localization/tools/run_offline_localization.sh ...
```

```bash
install/glim_localization/lib/glim_localization/run_offline_localization.sh ...
```

脚本会依次检查：

- `GLIM_LOCALIZATION_CONFIG_DIR`
- 源码树相对路径 `../config`
- 安装树相对路径 `../../share/glim_localization/config`
- `ros2 pkg prefix glim_localization` 推导出的 share 配置目录

如果部署环境是非标准布局，可通过环境变量显式指定：

```bash
export GLIM_LOCALIZATION_CONFIG_DIR=/path/to/glim_localization/config
```
