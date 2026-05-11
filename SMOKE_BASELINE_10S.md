# 10s Smoke Baseline

## 定位

这是 Phase 0.x 的短片段 smoke baseline，只用于快速确认构建、配置、rosbag 回放、地图加载、scan-to-map 前 10 秒主链路没有退化。它不是完整 localization baseline。

## 运行方式

```bash
bash tools/run_smoke_baseline_10s.sh \
  --bag /path/to/bag \
  --map /path/to/glim_dump \
  --output-dir /tmp/glim_localization_smoke_10s
```

也可使用环境变量：

```bash
export GLIM_LOCALIZATION_SMOKE_BAG=/path/to/bag
export GLIM_LOCALIZATION_SMOKE_MAP=/path/to/glim_dump
bash tools/run_smoke_baseline_10s.sh
```

## 本地 Phase 0.6 验证

输出目录：

```text
/tmp/glim_localization_phase0_6/smoke_10s_guard_off
```

结果：

| 指标 | 结果 |
| --- | ---: |
| exit code | 0 |
| frames | 101 |
| duration | 10.000s |
| LOST | 0 |
| RELOCALIZING | 0 |
| registration rejection | 0 |
| GTSAM warning | 0 |
| total distance | 6.962m |
| max step | 0.302m |
| suspicious divergence | false |

## 验收标准

- frames 约 101。
- LOST = 0。
- registration rejection = 0。
- GTSAM warning = 0。
- max step 不应明显超过当前参考值 0.302m。
