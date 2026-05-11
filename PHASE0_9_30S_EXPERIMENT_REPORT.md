# Phase 0.9 30s Experiment Report

## 1. 命令

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 30 \
  --timeout-sec 120 \
  --debug-csv \
  --relocalization-debug \
  --relocalization-debug-topk 10 \
  --verify-rejected-topk \
  --verify-rejected-topk-k 5 \
  --target-rebuild-guard \
  --target-rebuild-guard-confirmation-frames 2 \
  --smoother-guard \
  --smoother-guard-max-holds 200 \
  --lost-recovery \
  --lost-recovery-period-frames 5 \
  --output-dir /tmp/glim_localization_phase0_9/relocalization_debug_verify_30s
```

## 2. 结果

- exit code: 0
- CSV frames: 301
- trajectory frames: 251
- first DEGRADED: 126
- first LOST: 144
- longest LOST segment: 252-300, 49 frames
- GTSAM warning: 0
- trajectory total distance: 42.751 m
- max step: 1.977 m
- suspicious divergence: false

状态统计：

- TRACKING: 236
- DEGRADED: 11
- LOST: 50
- RECOVERING: 4

relocalization：

- requested/debug rows: 52
- success frames: 2
- no-candidate frames: 50
- actual descriptor-filter failures after frame 252: 12 query rows
- debug-only rejected top-k verification: 12/13 success

## 3. 结论

30s 仍不是稳定 localization baseline。它是可控失败：轨迹没有巨大飞散，GTSAM warning 为 0，但 frame 252 后长期 LOST 仍未恢复。

本轮最重要的新证据是：frame 252-300 的 raw top-k 并非无效，多个被 `0.35` 阈值过滤掉的候选可以通过 geometric verification。因此当前恢复失败主要是候选进入 verification 前的 descriptor hard gate，而不是 query cloud 空、database 空或 verification 普遍失败。

