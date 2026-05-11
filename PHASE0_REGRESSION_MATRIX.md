# Phase 0 Regression Matrix

## 入口

```bash
bash tools/run_phase0_regression_matrix.sh \
  --bag /path/to/bag \
  --map /path/to/glim_dump \
  --output-dir /tmp/glim_localization_phase0_regression \
  --initial-pose "0 0 0 0 0 0"
```

也可以通过环境变量传入：

```bash
export GLIM_LOCALIZATION_REGRESSION_BAG=/path/to/bag
export GLIM_LOCALIZATION_REGRESSION_MAP=/path/to/glim_dump
bash tools/run_phase0_regression_matrix.sh
```

脚本不修改默认配置文件；每个 case 通过临时配置注入 bag/map、initial pose、debug CSV 和 guard 设置。

## 矩阵

| case | duration | guard config | 目标 |
| --- | ---: | --- | --- |
| `smoke_10s` | 10s | experimental guards off, debug CSV on | 基础链路不退化 |
| `failure_15s_guard_off` | 15s | guards off, debug CSV on | 复现 frame 126 问题 |
| `failure_15s_confirmation_guard` | 15s | target confirmation guard on | 验证 frame 126/127 改善 |
| `experimental_30s_best` | 30s | target confirmation guard + smoother guard | 检查是否可控失败或稳定 |

## 本轮本地结果

输出目录：

```text
/tmp/glim_localization_phase0_7/regression_matrix_v2
```

| case | exit | trajectory frames | CSV frames | LOST | rejected | max step | suspicious |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `smoke_10s` | 0 | 101 | 101 | 0 | 0 | 0.302m | false |
| `failure_15s_guard_off` | 0 | 151 | 151 | 1 | 13 | 5.553m | false |
| `failure_15s_confirmation_guard` | 0 | 151 | 151 | 1 | 10 | 3.174m | false |
| `experimental_30s_best` | 0 | 251 | 301 | 50 | 14 | 1.977m | false |

`experimental_30s_best` 的 trajectory frames 少于 CSV frames 是预期行为：smoother guard hold frame 仍写入 CSV，但默认不写 trajectory。

## 验收解释

- 10s smoke 是稳定 smoke baseline，不是完整定位 baseline。
- 15s confirmation guard 能稳定拦住 frame 127 的近阈值 correction。
- 30s best 是“可控失败”：没有巨大轨迹发散、GTSAM warning 为 0，但后段 252..300 仍长期 LOST，因此不能声明为稳定 localization baseline。

## 自动 CSV 分析

每个 case 完成后会自动生成：

```text
<case>/debug_csv_analysis/debug_csv_summary.md
<case>/debug_csv_analysis/state_timeline.csv
<case>/debug_csv_analysis/target_rebuild_events.csv
<case>/debug_csv_analysis/correction_statistics.csv
<case>/debug_csv_analysis/lost_segments.csv
<case>/debug_csv_analysis/guard_actions.csv
```

## 建议提交位置

本文件可保留在仓库根目录作为 Phase 0 交付摘要；长期建议迁移到 `docs/analysis/PHASE0_REGRESSION_MATRIX.md`，并在 `docs/phase0_regression_matrix.md` 保留用户入口。
