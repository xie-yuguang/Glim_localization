# 工程化与回归手册

本文档面向 `glim_localization` 的长期维护者，目标是把部署、实验、benchmark、trajectory、资源监测和长期回归路径收成一套稳定的工程化流程。

本文职责边界：

- 负责：标准实验链路、使用矩阵、实验记录模板、长期回归建议、子项目边界
- 不负责：参数逐项解释、第一次快速跑通、FAQ 式排障

对应文档：

- 项目全景：`README.md`
- 基线与契约：`docs/baseline_and_contract.md`
- ROS 接口：`docs/ros_interface.md`
- 部署运行：`docs/deployment_and_run.md`
- 资源监测：`docs/resource_monitoring.md`

## 1. 标准实验链路

P3 推荐把离线标准实验统一成下面这条链路：

```text
offline rosbag run
  -> trajectory output
  -> benchmark_localization
  -> external resource monitoring
  -> PNG + HTML resource report
  -> experiment record
```

推荐入口：

```bash
bash tools/run_standard_experiment.sh \
  --bag /path/to/bag \
  --map /path/to/map \
  --initial-pose "0 0 0 0 0 0" \
  --output-dir /tmp/glim_exp_cpu \
  --matching-method cpu_gicp \
  --enhanced
```

GPU 版本：

```bash
bash tools/run_standard_experiment.sh \
  --bag /path/to/bag \
  --map /path/to/map \
  --initial-pose "0 0 0 0 0 0" \
  --output-dir /tmp/glim_exp_gpu \
  --matching-method gpu_vgicp \
  --enhanced \
  --gpu-monitor
```

脚本会统一产出：

- `experiment_manifest.txt`
- `experiment_record.md`
- `monitor/summary.txt`
- `resource_report/resource_report.html`
- `trajectory/trajectory_stats.txt`
- `trajectory/trajectory_2d.png`
- `trajectory/trajectory_3d.png`
- `benchmark/benchmark.txt`

## 2. 使用矩阵

建议维护以下四种标准组合：

| 模式 | 入口 | 匹配后端 | 主要用途 |
|---|---|---|---|
| `offline_cpu` | `glim_rosbag` | `cpu_gicp` | 最小闭环、CPU-only 回归 |
| `offline_gpu` | `glim_rosbag` | `gpu_vgicp` | GPU 性能对比、warning/资源分析 |
| `online_cpu` | `glim_rosnode` | `cpu_gicp` | 在线接口与 TF 行为验证 |
| `online_gpu` | `glim_rosnode` | `gpu_vgicp` | 在线 GPU 路径验证 |

长期维护中，推荐：

- 每次主干变更至少回归 `offline_cpu`
- 影响 GPU 或 hook 行为时同时回归 `offline_gpu`
- 改 ROS 接口或 TF 行为时补跑 `online_cpu`

## 3. 实验记录模板

标准实验记录至少包含：

- 实验名称
- bag / map / config 版本
- 匹配后端
- 初始位姿
- 输出目录
- 轨迹结论
- 状态机结论
- 资源结论
- benchmark 结论
- 是否通过回归

推荐直接复用 `tools/run_standard_experiment.sh` 自动生成的 `experiment_record.md`，再在其中补充人工结论。

## 4. benchmark、trajectory、monitor 的统一口径

`benchmark_localization`

- 用于测 submap query / index 性能
- 不代表完整 localization 总时延
- 适合比较地图规模、索引分辨率、query 参数变化

`trajectory`

- 用于检查是否有明显跳变、塌陷、漂移失控
- 当前默认输出不带真值，不承担精度评估全部职责
- 适合做“相对比较”与异常排查

`resource monitoring`

- 负责 CPU / 内存 / 线程 / I/O / GPU 的第三方观测
- 不替代轨迹与状态机回归
- 推荐配合 HTML 报告归档

三者建议一起看：

- benchmark 看静态地图查询成本
- trajectory 看定位结果是否异常
- monitor/report 看运行资源是否超预算

## 5. 长期回归建议

建议按层次做回归：

1. 单元/模块层
- `ctest --test-dir build/glim_localization --output-on-failure`

2. 离线最小基线
- `baseline_offline_cpu`

3. GPU 基线
- `baseline_offline_gpu`

4. 接口层
- RViz / TF / diagnostics topic 行为检查

5. 工程层
- `run_standard_experiment.sh` 产物是否齐全
- HTML 报告是否成功生成

## 6. 继续留在 glim_localization 的能力

长期建议继续留在 `glim_localization` 中演进：

- fixed-map localization 状态机
- target map 管理
- localization-specific registration policy
- relocalization
- localization ROS publisher
- localization-oriented tools、trajectory、experiment workflow

## 7. 未来适合回抽的能力

如果后续稳定下来，以下能力适合评估是否回抽：

- 回抽到 `glim`
  - 更通用的 fixed-map metadata / dump metadata
  - 可能通用的 registration measurement 抽象

- 回抽到 `glim_ros2`
  - 若多个子方向都需要，可复用的 diagnostics publisher 骨架

当前不建议回抽到 `glim_ext`：

- `glim_ext` 更适合作为实验扩展和参考实现，不适合作为长期主干 localization 归宿

## 8. 正式子方向候选判断口径

P3 之后，建议用下面四条判断是否可以视为“正式子方向候选”：

- 有稳定的基线、契约、文档和回归路径
- 有明确的 ROS 接口与状态/diagnostics 语义
- 有统一的实验和资源分析链路
- 有明确的留存边界与未来回抽边界

如果以上四条都满足，可以认为 `glim_localization` 已经达到“正式子方向候选”。
