# Phase 0 Regression Matrix

Phase 0.x 调试期间，推荐使用统一脚本运行 10s / 15s / 30s 回归矩阵：

```bash
bash tools/run_phase0_regression_matrix.sh \
  --bag /path/to/bag \
  --map /path/to/glim_dump \
  --output-dir /tmp/glim_localization_phase0_regression \
  --initial-pose "0 0 0 0 0 0"
```

矩阵：

| case | duration | guard config | 目标 |
| --- | ---: | --- | --- |
| `smoke_10s` | 10s | experimental guards off, debug CSV on | 基础链路不退化 |
| `failure_15s_guard_off` | 15s | guards off, debug CSV on | 复现 frame 126 |
| `failure_15s_confirmation_guard` | 15s | target confirmation guard on | 验证 frame 126/127 改善 |
| `experimental_30s_best` | 30s | target confirmation guard + smoother guard | 检查是否可控失败或稳定 |

脚本不会修改默认配置文件。bag/map 路径必须通过参数或环境变量显式传入。

详细 Phase 0.7 结果见仓库根目录：

- `PHASE0_REGRESSION_MATRIX.md`
- `PHASE0_7_30S_LOST_ANALYSIS_REPORT.md`
