# Phase 0.8 30s Experiment Report

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
  --target-rebuild-guard \
  --target-rebuild-guard-confirmation-frames 2 \
  --smoother-guard \
  --smoother-guard-max-holds 200 \
  --lost-recovery \
  --lost-recovery-period-frames 5 \
  --output-dir /tmp/glim_localization_phase0_8/relocalization_debug_30s
```

## 2. 结果

```text
exit_code: 0
csv_frames: 301
trajectory_frames: 251
first_degraded: 126
first_lost: 144
longest_lost_segment: 252..300
LOST frames: 50
GTSAM warning: 0
suspicious_divergence: false
max_step_m: 1.977
total_distance_m: 42.751
```

## 3. Relocalization

```text
requested/debug rows: 52
success_frames: 2
no_candidate_frames: 50
topk_returned: 10 when query attempted
top1_distance_min/mean/max: 0.286 / 0.419 / 0.510
failure reasons:
  not_requested: 37
  all_filtered_by_descriptor_distance: 13
  success: 2
```

说明：`not_requested` 来自 `lost_recovery_period_wait`，表示 LOST recovery 周期等待帧，不是 query 失败。实际 query 失败主因是 `all_filtered_by_descriptor_distance`。

## 4. 10s Smoke

```text
frames: 101
state: all TRACKING
rejection: 0
LOST: 0
max_step_m: 0.302
suspicious_divergence: false
```

## 5. 判断

30s 仍不是稳定 localization baseline，但它是可控失败：

- 无 GTSAM underconstrained warning。
- LOST 后不再把 hold pose 写成正常 trajectory。
- failure chain 可以解释到 descriptor distance filter。

下一轮不应继续盲调 scan-to-map correction 阈值，而应针对 ScanContext query descriptor 做离线/短片段实验。
