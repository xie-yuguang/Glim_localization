# 资源监测

本文档提供一套从第三方视角监测 `glim_localization` 运行资源的方法。所有方案都在进程外部执行，不需要把监测逻辑集成进 `glim_localization` 主代码。

适用场景：

- 离线 rosbag 运行：`ros2 run glim_ros glim_rosbag ...`
- 在线 ROS2 运行：`ros2 run glim_ros glim_rosnode ...`
- CPU-only 构建
- GPU-enabled 构建，尤其是 `localization.matching.method = "gpu_vgicp"` 的实验对比

## 1. 当前项目运行方式与监测重点

基于当前工作区代码：

- 离线入口：`glim_ros2/src/glim_rosbag.cpp`
- 在线入口：`glim_ros2/src/glim_rosnode.cpp`
- localization odometry 动态模块：`libodometry_estimation_localization_cpu.so`
- ROS2 输出扩展：`liblocalization_publisher.so`
- 可选 GPU registration：`gpu_vgicp`
- `glim_localization` 只有在 `BUILD_WITH_CUDA=ON` 且 `gtsam_points` 检测到 CUDA 支持时才会编译 GPU 路径

对于资源监测，最有价值的指标是：

1. CPU 使用率
2. 最大内存 / 峰值 RSS
3. 线程数量
4. 运行时长
5. 读写 I/O
6. GPU 利用率与显存占用

## 2. 工具选择

当前方案优先使用 Linux 常见标准工具：

- `/usr/bin/time -v`
- `ps`
- `/proc/<pid>/io`
- `pidstat`
- `iostat`
- `vmstat`
- `nvidia-smi`

可选增强但不作为默认依赖：

- `perf stat`

当前环境检查结果：

- 可用：`/usr/bin/time`、`pidstat`、`perf`、`iostat`、`vmstat`、`nvidia-smi`
- 未检测到：`psrecord`

## 3. 依赖安装

Ubuntu / Debian 常见安装方式：

```bash
sudo apt update
sudo apt install time procps sysstat python3
sudo apt install python3-matplotlib
```

说明：

- `time` 提供 `/usr/bin/time -v`
- `procps` 提供 `ps`、`top`、`vmstat`
- `sysstat` 提供 `pidstat`、`iostat`
- `nvidia-smi` 一般随 NVIDIA 驱动提供
- `python3-matplotlib` 用于生成 PNG 图表和 HTML 报告

`perf` 常见安装方式：

```bash
sudo apt install linux-tools-common linux-tools-generic
sudo apt install "linux-tools-$(uname -r)"
```

## 4. 最小方案

最小方案尽量兼容绝大多数 Linux 环境，核心思路是：

- 用 `/usr/bin/time -v` 记录 wall time、user time、sys time、最大 RSS
- 用 `ps + /proc/<pid>/io` 每秒采样目标进程
- 输出为纯文本和 CSV，便于实验记录

脚本位置：

- `tools/monitor/run_with_time.sh`
- `tools/monitor/monitor_ps.sh`
- `tools/monitor/summarize_usage.py`

### 4.1 最小方案输出内容

最小方案至少记录：

- wall time
- user time
- sys time
- `/usr/bin/time` 统计的最大 RSS
- 采样平均 CPU
- 采样峰值 CPU
- 采样峰值 RSS
- 峰值线程数
- 基于 `/proc/<pid>/io` 的读写字节增量

### 4.2 离线 rosbag 监测示例

```bash
bash Glim_localization/tools/monitor/run_with_time.sh \
  -o /tmp/glim_monitor_offline \
  -i 1 \
  -- \
  ros2 run glim_ros glim_rosbag /path/to/your.bag \
  --ros-args -p "config_path:=/home/xie/Glim/src/Glim_localization/config"
```

### 4.3 在线 ROS2 监测示例

```bash
bash Glim_localization/tools/monitor/run_with_time.sh \
  -o /tmp/glim_monitor_online \
  -i 1 \
  -- \
  ros2 run glim_ros glim_rosnode \
  --ros-args -p "config_path:=/home/xie/Glim/src/Glim_localization/config"
```

### 4.4 最小方案产物

输出目录下主要文件：

- `command.txt`：完整命令
- `run_meta.txt`：运行元信息
- `time.txt`：`/usr/bin/time -v` 原始输出
- `command.stdout.log`：标准输出
- `command.stderr.log`：标准错误
- `ps_samples.csv`：每秒采样数据
- `summary.txt`：汇总摘要
- `summary.json`：结构化摘要

### 4.5 `ps_samples.csv` 字段解释

- `pcpu`：采样时刻进程 CPU 使用率
- `pmem`：采样时刻进程内存占比
- `rss_kb`：常驻内存大小，单位 KB
- `vsz_kb`：虚拟内存大小，单位 KB
- `nlwp`：线程数量
- `etimes_s`：进程已运行时长，单位秒
- `read_bytes` / `write_bytes`：内核统计的实际读写字节
- `rchar` / `wchar`：逻辑读写字符数

### 4.6 `summary.txt` 解释

典型字段：

- `wall_time_s`：真实运行时长
- `user_time_s`：用户态 CPU 时间
- `sys_time_s`：内核态 CPU 时间
- `cpu_percent_time`：GNU time 给出的整体 CPU 百分比
- `avg_cpu_pct_ps`：按采样点计算的平均 CPU
- `peak_cpu_pct_ps`：按采样点计算的峰值 CPU
- `max_rss_mb_time`：GNU time 记录的最大 RSS
- `peak_rss_mb_ps`：采样观察到的峰值 RSS
- `peak_threads`：采样期间观测到的最大线程数
- `read_bytes_delta` / `write_bytes_delta`：运行期间 I/O 增量

## 5. 增强方案

增强方案适合做实验记录和对比分析，建议在最小方案基础上开启：

- `pidstat`
- `iostat`
- `nvidia-smi`

脚本位置：

- `tools/monitor/monitor_pidstat.sh`
- `tools/monitor/monitor_iostat.sh`
- `tools/monitor/monitor_gpu.sh`

推荐直接用总控脚本：

```bash
bash Glim_localization/tools/monitor/run_with_time.sh \
  -o /tmp/glim_monitor_enhanced \
  -i 1 \
  --enhanced \
  --gpu \
  -- \
  ros2 run glim_ros glim_rosbag /path/to/your.bag \
  --ros-args -p "config_path:=/home/xie/Glim/src/Glim_localization/config"
```

### 5.1 增强方案额外产物

- `pidstat_cpu_mem_io.log`
- `pidstat_threads.log`
- `iostat.log`
- `gpu_gpus.csv`
- `gpu_apps.csv`

### 5.2 `pidstat` 的作用

- `pidstat_cpu_mem_io.log`：PID 级 CPU / 内存 / I/O
- `pidstat_threads.log`：线程级 CPU / 内存 / I/O

适合回答：

- 某段时间 CPU 抖动是否由主线程还是工作线程引起
- I/O 峰值是否与目标地图加载或日志写出相关
- 线程数是否异常增长

### 5.3 `iostat` 的作用

`iostat -x -y -t` 适合观察整机磁盘层面指标，例如：

- `%util`
- `r/s`
- `w/s`
- `rkB/s`
- `wkB/s`
- `await`

当你怀疑瓶颈来自地图文件读取、bag 解码或磁盘写日志时很有用。

### 5.4 `gpu_gpus.csv` / `gpu_apps.csv` 的作用

`gpu_gpus.csv` 记录 GPU 级指标：

- `utilization_gpu_pct`
- `utilization_mem_pct`
- `memory_used_mb`
- `memory_total_mb`
- `power_draw_w`

`gpu_apps.csv` 记录进程级显存：

- `pid`
- `process_name`
- `used_gpu_memory_mb`

这对 `gpu_vgicp` 实验非常有用，可以区分：

- GPU 是否真的被用起来
- GPU 总利用率高不高
- 当前 localization 进程本身用了多少显存

## 6. CPU-only 与 GPU-enabled 的区别

### 6.1 CPU-only

建议至少记录：

- `time.txt`
- `ps_samples.csv`
- `pidstat_cpu_mem_io.log`

重点看：

- CPU 是否接近瓶颈
- RSS 是否随着 target map 变化明显抖动
- I/O 是否异常

### 6.2 GPU-enabled

建议在 CPU-only 方案上再加：

- `gpu_gpus.csv`
- `gpu_apps.csv`

重点看：

- GPU 利用率是否持续有效
- 显存占用是否稳定
- CPU 是否仍然异常高，说明可能存在同步点或 CPU 侧瓶颈

## 7. 针对已在运行的进程单独采样

如果 `glim_localization` 已经在另一个终端启动，可以直接对 PID 采样。

先找 PID：

```bash
ps -ef | grep -E "glim_rosbag|glim_rosnode" | grep -v grep
```

最小采样：

```bash
bash Glim_localization/tools/monitor/monitor_ps.sh <pid> /tmp/glim_ps.csv 1
```

增强采样：

```bash
bash Glim_localization/tools/monitor/monitor_pidstat.sh <pid> /tmp/glim_pidstat 1
bash Glim_localization/tools/monitor/monitor_gpu.sh /tmp/glim_gpu 1 <pid>
```

## 8. 推荐的实验记录方式

推荐每次实验都固定保存一个目录，例如：

```text
/tmp/exp_20260414_gpu_vgicp_case01/
  command.txt
  run_meta.txt
  time.txt
  ps_samples.csv
  pidstat_cpu_mem_io.log
  pidstat_threads.log
  iostat.log
  gpu_gpus.csv
  gpu_apps.csv
  summary.txt
  summary.json
```

建议命名中包含：

- 数据集 / bag 名称
- 配置版本
- 是否 GPU
- 关键参数，例如 `levels1`、`submaps8`

## 9. 结果解释建议

如果你要对比修复前后、参数前后或 CPU/GPU 两种模式，建议至少记录下列指标：

- 总 wall time
- 平均 CPU / 峰值 CPU
- 峰值 RSS
- 峰值线程数
- I/O 字节增量
- 峰值 GPU 利用率
- 峰值显存占用

推荐将 `summary.json` 收集后统一汇总到表格中，再结合：

- 轨迹精度
- warning 数量
- 回放速度

一起比较。

## 10. 可选补充工具

### 10.1 `vmstat`

系统级快速观察：

```bash
vmstat 1
```

适合看整体 CPU、内存、上下文切换。

### 10.2 `perf stat`

适合做一次性性能计数：

```bash
perf stat -d -- ros2 run glim_ros glim_rosbag /path/to/your.bag \
  --ros-args -p "config_path:=/home/xie/Glim/src/Glim_localization/config"
```

注意：

- `perf` 更适合对照实验，不建议作为默认常驻采样器
- 某些系统需要 root 权限或额外 perf 权限配置

## 11. 何时优先用哪套方案

最小方案适合：

- 快速确认是否变慢
- 做基础实验记录
- CPU-only 机器
- 没有 `sysstat` 或 NVIDIA 驱动的环境

增强方案适合：

- GPU 路径分析
- 想看线程级行为
- 想做较正式的实验对照
- 需要保留完整运行期资源曲线

## 12. 资源图表与 HTML 报告

在已有监测日志基础上，可以继续生成 PNG 图表、CSV 汇总表和一页式 HTML 报告。

新增脚本：

- `tools/monitor/plot_usage.py`
- `tools/monitor/generate_usage_report.py`
- `tools/monitor/usage_report_lib.py`

### 12.1 支持的输入日志

当前图表和报告工具优先复用这些现有输出：

- `ps_samples.csv`
- `time.txt`
- `gpu_gpus.csv`
- `gpu_apps.csv`
- `command.txt`
- `exit_code.txt`

其中：

- CPU / RSS / Threads / I/O 曲线来自 `ps_samples.csv`
- wall time / user time / sys time / max RSS 来自 `time.txt`
- GPU 利用率 / 显存曲线来自 `gpu_gpus.csv` 和 `gpu_apps.csv`

如果某些日志不存在，工具会优雅降级：

- 缺少 `gpu_*.csv`：只生成 CPU / 内存 / 线程 / I/O 图
- 缺少连续 I/O 计数：跳过 I/O 曲线
- 缺少 `ps_samples.csv`：无法生成 CPU / RSS / Threads 曲线，但仍可导出已有统计项

### 12.2 只生成图表和汇总表

```bash
python3 Glim_localization/tools/monitor/plot_usage.py \
  --input-dir /tmp/glim_monitor_offline \
  --output-dir /tmp/glim_report_offline \
  --title "glim_localization rosbag offline run"
```

默认输出：

- `cpu_usage.png`
- `memory_usage.png`
- `thread_usage.png`
- `io_usage.png`
- `gpu_usage.png`
- `gpu_memory_usage.png`
- `resource_summary.csv`
- `resource_summary.json`
- `plot_manifest.json`

### 12.3 生成完整 HTML 报告

```bash
python3 Glim_localization/tools/monitor/generate_usage_report.py \
  --input-dir /tmp/glim_monitor_offline \
  --output-dir /tmp/glim_report_offline \
  --title "glim_localization rosbag offline run"
```

生成结果：

- `resource_report.html`
- 全部 PNG 图表
- `resource_summary.csv`
- `resource_summary.json`

查看方式：

```bash
xdg-open /tmp/glim_report_offline/resource_report.html
```

如果当前机器不方便用浏览器，也可以直接查看输出目录中的 PNG 和 CSV。

### 12.4 图表内容与解释

当前至少支持这些图：

- CPU 使用率随时间变化曲线
- 内存 RSS 随时间变化曲线
- 线程数量随时间变化曲线
- I/O 读写速率曲线
- GPU 利用率曲线
- GPU 显存占用曲线

图中默认会标注：

- 峰值点
- 平均值
- P95

建议解释方式：

- CPU 曲线高且长期贴近峰值：说明处理主要受 CPU 限制
- 内存曲线在 target map 更新时明显抬升：说明地图规模和局部地图重建对内存敏感
- 线程数持续偏高：说明并行工作较多，移植时要关注线程调度能力
- GPU 利用率峰值集中在 matching 时段：说明 GPU 确实承担了主要配准计算
- GPU 显存持续爬升：要关注地图规模和 batch 行为是否会逼近显存上限
- I/O 曲线在启动或地图重建阶段冲高：说明存储速度可能影响整体时延

### 12.5 统计汇总项

报告会汇总这些关键指标：

- 总运行时长
- 平均 CPU
- 峰值 CPU
- P95 CPU
- 平均内存
- 峰值内存 / 峰值 RSS
- 平均线程数
- 峰值线程数
- GPU 平均利用率 / 峰值利用率
- GPU 平均显存 / 峰值显存
- 磁盘平均读写速率 / 峰值读写速率

### 12.6 如何据此做移植前硬件评估

建议先看三个量：

1. `peak_cpu_pct`
2. `peak_rss_mb`
3. `peak_process_gpu_mem_mb` 或 `peak_gpu_total_mem_mb`

经验上可以这样估算：

- CPU-only：至少按峰值 CPU 对应的逻辑核数，再加 20%~30% 余量
- 内存：至少按峰值 RSS 的 1.5 倍留余量
- GPU：至少按峰值进程显存的 1.5 倍留余量

同时要结合：

- 是否还会叠加其他 ROS2 节点
- bag 或地图是否会继续变大
- 是否需要更高回放倍速或实时性余量

### 12.7 多次实验对比建议

推荐每次实验都独立输出一个目录：

```text
/tmp/monitor_logs/run_001/
/tmp/monitor_logs/run_002/
/tmp/monitor_logs/run_003/
```

再分别生成：

```text
/tmp/monitor_reports/run_001/
/tmp/monitor_reports/run_002/
/tmp/monitor_reports/run_003/
```

做对比时重点看：

- `resource_summary.csv`
- `resource_summary.json`
- `cpu_usage.png`
- `memory_usage.png`
- `gpu_usage.png`

最适合横向比较的实验包括：

- CPU-only vs GPU-enabled
- 不同 `target_map.max_num_submaps`
- 不同 `target_map.max_distance`
- 不同 `max_iterations`
- 修复前 vs 修复后
