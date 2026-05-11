# Glim_localization 建议立即修复项

## P0

| 问题 | 严重程度 | 证据 | 建议修复 |
|---|---|---|---|
| 默认配置含个人绝对路径 | High | `config/config.json:3`、`config/localization.json:24` | 改为空模板或运行脚本注入；不要把用户机器路径作为默认 |
| 默认 matching 是 GPU，但默认不编译 CUDA | High | `config/localization.json:49`、`CMakeLists.txt:15` | 默认改 `cpu_gicp`；GPU 单独配置和文档 |
| 没有真实 bag baseline 结果 | High | 当前 CTest 主要 synthetic，真实地图只验证加载 | 用现有 bag/map 跑 `run_standard_experiment.sh`，保存结果 |
| `__pycache__` 被 Git 跟踪 | Low | `git ls-files` 包含 `tools/monitor/__pycache__/*` | `git rm -r --cached tools/monitor/__pycache__`，添加 `.gitignore` |
| 缺 LICENSE 文件 | Medium | 仓库根目录无 LICENSE，`package.xml` 声明 MIT | 添加 LICENSE，并说明基于 GLIM 的许可证关系 |
| maintainer/author 仍是 koide | Medium | `package.xml:7-8` | 改为当前项目维护者，保留 upstream attribution |

## P1

| 问题 | 严重程度 | 证据 | 建议修复 |
|---|---|---|---|
| `base_frame` 语义可能错误 | High | publisher 用 `T_map_imu` 直接生成 `odom -> base_link` | 增加 `T_base_imu` 或输出 child 改为 IMU frame |
| 状态输出是字符串 | Medium | `/localization/status` 使用 `std_msgs/String` | 新增 typed status/quality topic，保留旧 topic 兼容 |
| 缺运行时耗时统计 | Medium | 无 per-stage timing | 给 map query、target merge、registration、publish 加计时 |
| 初值远离地图只 warning | Medium | `validate_initial_pose()` 不阻断 | 增加 `INITIAL_POSE_OUT_OF_MAP` 状态或启动失败策略 |

## 最小 patch proposal

```text
1. config/localization.json:
   - localization.map_path 改为空字符串或占位 `/path/to/glim_dump`
   - localization.matching.method 改为 `cpu_gicp`
   - localization.map.load_voxelmaps 默认改 false（确认不依赖后）

2. config/config.json:
   - 移除 `/home/xie/...` 绝对路径，依赖运行时 config_path

3. 仓库根目录:
   - 新增 `.gitignore`
   - 新增 `LICENSE`
   - 移除 pycache 跟踪

4. docs/baseline_and_contract.md:
   - 追加本地真实 bag/map baseline 命令和 artifact 路径
```
