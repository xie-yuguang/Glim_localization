# FAQ

本文职责边界：

- 负责：回答高频问题，给出最短检查路径
- 不负责：重复展开完整教程或配置全表

建议使用顺序：

1. 先看 `docs/baseline_and_contract.md` 确认当前基线与接口契约。
2. 再看 `docs/quick_start.md` 或 `docs/deployment_and_run.md` 对照操作步骤。
3. 最后用本文快速定位高频问题。

## Q1：为什么 RViz 里看不到 `/localization/debug/local_target_map`？

`localization_publisher` 只有在同时满足下面两个条件时才会发布这个 topic：

- `localization.ros.publish_debug_target_map = true`
- 该 topic 当前存在订阅者

最直接的检查方式：

```bash
ros2 topic echo /localization/debug/local_target_map --once
```

## Q2：为什么只能看到 pose/trajectory，看不到 scan？

先检查是否存在这两个 topic：

```bash
ros2 topic list | grep /localization/debug
```

正常应至少包含：

- `/localization/debug/input_scan`
- `/localization/debug/current_scan`

说明：

- `/localization/debug/input_scan` 是输入 scan，frame 默认为 `sensor_frame`
- `/localization/debug/current_scan` 是 deskew 后并变换到 `map` frame 的 scan

## Q3：`/initialpose` 发了，但定位还是没启动？

先确认配置里使用的是 topic 初始位姿模式：

```json
"initial_pose": {
  "source": "topic"
}
```

然后确认消息 frame 是 `map`，并且 `glim_ros.extension_modules` 里包含：

```json
["liblocalization_publisher.so"]
```

## Q4：`map -> odom` 为什么看起来始终像 identity？

这是当前实现的真实行为。`glim_localization` 里：

- `world == map`
- `T_map_odom` 当前默认保持 identity
- publisher 仍然会发布 `map -> odom`，是为了保持和标准 ROS/RViz 可视化链路兼容

后续如果引入 reset / relocalization 连续性管理，这里才会变成非 identity 的动态变换。

## Q5：`input_scan`、`current_scan`、`local_target_map` 应该怎么一起看？

推荐：

- 以 `map` 作为 RViz `Fixed Frame`
- 默认打开 `/localization/debug/current_scan` 和 `/localization/debug/local_target_map`
- 需要排查输入点云本身时，再打开 `/localization/debug/input_scan`

其中：

- `current_scan + local_target_map` 主要用于看配准效果
- `input_scan` 主要用于看输入点云方向、坐标系和字段是否正常

## Q6：有没有现成 RViz 配置？

有。安装后可以直接运行：

```bash
rviz2 -d install/glim_localization/share/glim_localization/rviz/localization.rviz
```

## Q7：`glim_localization_traj.txt` 是怎么生成的？

由 `TrajectoryWriter` 在运行时自动写出。只要配置里：

```json
"localization": {
  "trajectory_path": "/tmp/glim_localization_traj.txt"
}
```

并且该路径可写，定位每更新一帧就会追加一行：

```text
stamp x y z qx qy qz qw status_int matching_score
```

## Q8：怎么把轨迹文件画成图片？

使用项目自带脚本：

```bash
python3 tools/plot_trajectory.py /tmp/glim_localization_traj.txt
```

默认会生成 2D 和 3D 两张图，并在终端打印轨迹统计信息。

## Q9：没有 `matplotlib` 怎么办？

安装即可：

```bash
python3 -m pip install matplotlib
```

脚本只依赖：

- Python 标准库
- `matplotlib`

不依赖 `numpy`。

## Q10：怎么看轨迹图是否正常？

优先看这几项：

- 2D 俯视轨迹是否连续，是否出现明显跳点
- 起点和终点是否在预期区域
- 3D 图里 z 方向是否异常抖动
- 开启 `--time-color` 后，轨迹颜色顺序是否连贯
- 如果姿态箭头方向明显乱跳，通常要继续检查初始位姿、外参或定位状态切换
