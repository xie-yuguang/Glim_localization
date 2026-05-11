# Glim_localization 优化路线图

## Phase 0：确认和基线建立

目标：

- 稳定构建。
- 跑通最小 demo。
- 记录输入输出。
- 建立 baseline。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| 固定 workspace 构建命令 | `README.md`、`docs/quick_start.md` | `colcon build --packages-up-to glim_ros glim_localization` 通过 | 低 | 新人可复现 |
| 固定 CPU baseline | `tools/run_standard_experiment.sh`、`docs/baseline_and_contract.md` | 生成 trajectory/resource/benchmark artifact | 中 | 后续优化有对照 |
| 固定 GPU baseline | `config`、CMake 文档 | 确认未回退 CPU | 中 | 性能对照真实 |
| 清理绝对路径 | `config/config.json`、`config/localization.json` | 配置可复制到其它机器 | 低 | 部署风险大幅下降 |
| 记录真实 map/bag 结果 | `docs/experiment_record_template.md` | 有至少一次真实 bag 运行记录 | 中 | 成熟度可判断 |

## Phase 1：清理和文档化

目标：

- 清理无用模块和生成物。
- 补充 README。
- 明确参数、frame/tf、topic、地图格式。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| 移除 tracked pycache | `tools/monitor/__pycache__`、`.gitignore` | `git ls-files` 不含 pycache | 低 | 仓库干净 |
| 添加 LICENSE | `LICENSE`、`package.xml` | 许可证清晰 | 低 | 合规 |
| 更新 maintainer/author | `package.xml` | 不再误用 koide 维护者身份 | 低 | 项目身份清楚 |
| 明确 frame 语义 | `docs/ros_interface.md`、`localization_publisher.cpp` | pose/odom/tf 与文档一致 | 中 | 下游集成安全 |
| 补 launch 示例 | `launch/*.launch.py` | 一条命令启动在线 localization | 低 | 易用性提升 |
| 参数 schema/模板 | `config/*.template.json` | 缺参/非法参有明确提示 | 中 | 减少误配置 |

## Phase 2：localization pipeline 稳定化

目标：

- 初始位姿、地图加载、scan-to-map、状态输出、失败检测和 localization status 稳定。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| 抽出状态机 | `odometry_estimation_localization_cpu.cpp` | 状态转换可单测 | 中 | 降低核心类复杂度 |
| 初值有效性硬检查 | `validate_initial_pose()` | 初值远离地图时有明确状态码 | 低 | 减少错误起跑 |
| 结构化质量输出 | `LocalizationResult`、publisher | 下游可解析 score/residual/inliers | 中 | 可观测性提升 |
| 修正 base pose 输出 | publisher、config | 支持 IMU/LiDAR/base 三种输出语义 | 中 | TF 可靠 |
| registration covariance/degeneracy | registration backend | 退化帧可识别 | 高 | 定位安全性提升 |

## Phase 3：性能优化

目标：

- profile。
- 减少拷贝。
- 降采样。
- 地图索引。
- 异步队列。
- debug 开关。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| 运行时分段计时 | odometry、registration、publisher | 每帧输出处理耗时 | 低 | 定位瓶颈明确 |
| target cloud 缓存 | `LocalTargetMap` | rebuild 耗时下降 | 中 | CPU/memory 降低 |
| GPU voxelmap 缓存 | `GpuVgicpMapRegistration` | GPU 每帧耗时下降 | 中/高 | GPU 性能释放 |
| debug 发布限频 | publisher | 订阅 debug 不拖垮主流程 | 低 | 在线调试安全 |
| localization 专用 downsample | config + registration | 大点云实时性提升 | 中 | 处理耗时下降 |

## Phase 4：鲁棒性增强

目标：

- 退化检测。
- 重定位。
- 动态物体处理。
- 质量评估。
- fallback 策略。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| 多候选重定位确认 | relocalization | 错误候选不立即跳变 | 高 | 失锁恢复更安全 |
| smoother reset 策略 | odometry module | 重定位后历史状态一致 | 高 | 减少跳变/不稳定 |
| 动态物体过滤 | preprocessing/registration | 动态区域 score 更稳定 | 中 | 城市场景鲁棒 |
| 时间戳异常处理 | ROS/input adapter | bag 跳变可恢复 | 中 | 离线/在线稳定 |
| 外部定位 prior | 新接口 | GNSS/wheel 可辅助初始化 | 中 | 大场景可用性提升 |

## Phase 5：测试和工程化

目标：

- 单元测试。
- bag 回归测试。
- CI。
- Docker。
- benchmark。
- release 配置。

具体任务：

| 任务 | 涉及文件 | 验收标准 | 风险 | 收益 |
|---|---|---|---|---|
| CI | `.github/workflows` | PR 自动构建和测试 | 中 | 回归防护 |
| Docker | `docker/` | 干净环境能构建运行 | 中 | 部署可复现 |
| bag regression | `tests`、`tools` | 固定 bag 有阈值验收 | 高 | 真实质量可控 |
| benchmark 阈值 | CTest/脚本 | query/registration 性能漂移可见 | 中 | 性能回归可控 |
| release config | `config/release` | CPU/GPU/online/offline 配置分离 | 低 | 上线更安全 |

## 下一轮可直接执行任务

1. 清理配置绝对路径，生成 `localization.cpu.json` 和 `localization.gpu.json`。
2. 移除 `tools/monitor/__pycache__`，补 `.gitignore`。
3. 添加 `LICENSE`，更新 `package.xml` maintainer/author。
4. 新增 `launch/localization.launch.py`，包装 `glim_rosnode`。
5. 给主 pipeline 加 per-stage timing 并写入 diagnostics。
