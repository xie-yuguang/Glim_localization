# Debug CSV Schema

## 目的

`localization.debug.csv_enable` 默认关闭。开启后，`OdometryEstimationLocalizationCPU` 会按 frame 输出结构化诊断，用于复盘 target map rebuild、scan-to-map correction、relocalization 和 smoother guard 行为。

标准实验脚本使用：

```bash
bash tools/run_standard_experiment.sh ... --debug-csv --output-dir /tmp/run
```

输出路径：

```text
/tmp/run/debug/localization_debug.csv
```

## 配置

```json
"debug": {
  "csv_enable": false,
  "csv_path": "/tmp/glim_localization_debug.csv"
},
"relocalization_debug": {
  "enable": false,
  "dump_topk": 10,
  "dump_rejected_candidates": true
},
"lost_recovery": {
  "enable": false,
  "relocalization_period_frames": 5,
  "recovery_stable_frames": 3,
  "max_lost_frames_before_pause": 3,
  "write_trajectory_while_lost": false
}
```

## 字段

| 字段 | 含义 |
| --- | --- |
| `frame_id` | GLIM odometry frame id |
| `timestamp` | 输入点云时间戳 |
| `state_before` / `state_after` | `create_factors()` 前后的 localization 状态 |
| `registration_backend` | `cpu_gicp` / `gpu_vgicp` / 空 |
| `target_rebuilt` / `target_reused` | 本帧 target map 是否 rebuild/reuse |
| `target_center_x/y/z` | target map center |
| `active_submaps` | `;` 分隔的 active submap ids |
| `source_points` / `target_points` | registration 输入点数 |
| `predicted_x/y/z` | scan-to-map 前预测位姿平移 |
| `corrected_x/y/z` | registration/guard 后写入当前 frame 的平移 |
| `delta_t` / `delta_r` | registration correction 平移/旋转 |
| `score` / `residual` | registration 质量指标 |
| `inliers` / `inlier_fraction` | inlier 数和比例 |
| `reject_reason` | 拒绝原因，accepted 时为空 |
| `localization_factors` | 本包向 smoother 注入的 localization factors 数量 |
| `relocalization_requested` | 本帧是否进入 relocalization 路径 |
| `relocalization_candidates` | relocalization 候选数 |
| `relocalization_success` | relocalization 是否验证成功 |
| `relocalization_query_enable` | relocalization 查询是否启用 |
| `relocalization_query_reason` | 查询触发原因，例如 `registration_rejections_exceeded`、`lost_state`、`lost_recovery_period_wait` |
| `relocalization_query_points` | 查询 scan 点数 |
| `relocalization_descriptor_valid` | 查询 descriptor 是否有效 |
| `relocalization_descriptor_nonempty_bins` | 查询 descriptor 非空 bin 数 |
| `relocalization_database_size` | ScanContext database 条目数 |
| `relocalization_topk_requested` / `relocalization_topk_returned` | debug top-k 请求/返回数量，仅用于诊断 |
| `relocalization_top1_submap` / `relocalization_top1_distance` / `relocalization_top1_yaw` | 原始 top1 结果，不代表通过 gating |
| `relocalization_top2_submap` / `relocalization_top2_distance` | 原始 top2 结果 |
| `relocalization_top3_submap` / `relocalization_top3_distance` | 原始 top3 结果 |
| `relocalization_topk_submaps` / `relocalization_topk_distances` / `relocalization_topk_yaws` | Phase 0.9 新增的 raw top-k 列表，使用 `;` 分隔，用于离线 threshold sweep |
| `relocalization_filtered_by_descriptor` | 被 descriptor distance 阈值过滤的原始候选数 |
| `relocalization_filtered_by_translation` | 被 translation filter 过滤的候选数 |
| `relocalization_filtered_by_other` | 其他原因过滤数 |
| `relocalization_candidates_before_filter` / `relocalization_candidates_after_filter` | filter 前后候选数量 |
| `relocalization_verification_attempted` / `relocalization_verification_success` | 几何验证是否执行/成功 |
| `relocalization_verification_best_submap` | verification 中最佳候选 submap id |
| `relocalization_verification_best_score` / `relocalization_verification_best_inliers` / `relocalization_verification_best_residual` | verification 最佳 registration 质量 |
| `relocalization_failure_reason` | `disabled`、`not_requested`、`empty_query_cloud`、`descriptor_invalid`、`database_empty`、`no_topk`、`all_filtered_by_descriptor_distance`、`all_filtered_by_translation`、`verification_failed`、`success` 等 |
| `debug_verification_only` | Phase 0.9 debug-only rejected top-k verification 是否启用并执行；不改变正式定位状态 |
| `debug_verified_rejected_topk_count` | 被送入 debug-only verification 的 rejected raw top-k 数量 |
| `debug_verified_best_submap` / `debug_verified_best_descriptor_distance` | debug-only verification 中最佳候选 |
| `debug_verified_best_score` / `debug_verified_best_inliers` / `debug_verified_best_residual` | debug-only verification 最佳 registration 质量 |
| `debug_verified_success` | debug-only verification 是否几何通过；只表示阈值研究结果，不表示正式 relocalization success |
| `verify_raw_topk_enable` | Phase 1.0 正式 recovery experiment 是否启用；默认关闭 |
| `verify_raw_topk_used` | 本帧是否使用 raw top-k 候选进入正式 geometric verification |
| `verify_raw_topk_candidate_count` | 进入正式 verification 的 raw top-k 候选数 |
| `verify_raw_topk_best_submap` / `verify_raw_topk_best_distance` | raw top-k verification 中最佳候选 |
| `verify_raw_topk_best_score` / `verify_raw_topk_best_inliers` / `verify_raw_topk_best_residual` | raw top-k verification 的最佳几何验证质量 |
| `verify_raw_topk_success` | raw top-k 候选是否通过 geometric verification；通过后只进入 `RECOVERING`，不直接 `TRACKING` |
| `recovery_state` | 本帧最终 recovery/status 状态 |
| `recovering_stable_count` / `recovering_required_stable_frames` | `RECOVERING` 稳定计数和所需稳定帧数 |
| `recovering_transition_reason` | 最近一次状态迁移或 recovery 判据原因 |
| `descriptor_passed_main_threshold` | 最佳 relocalization 候选是否通过主 `max_descriptor_distance` |
| `descriptor_passed_raw_topk_experiment` | 最佳 relocalization 候选是否通过实验 raw top-k descriptor 阈值 |
| `smoother_guard_action` | 例如 `hold_last_pose_prior` |
| `target_rebuild_guard_action` | target confirmation guard 动作，例如 `confirmation_window_started`、`pending_large_correction`、`confirmed_small_corrections` |
| `confirmation_window_remaining` | target rebuild 后 confirmation window 剩余帧数 |
| `pending_correction_delta_t` | confirmation guard 缓存的可疑 correction 平移量 |
| `pending_correction_consistent` | 当前 large correction 是否与 pending correction 一致；当前只记录，不直接接受 |
| `registration_ms` | registration / geometric verification 耗时 |
| `target_build_ms` | target map 查询或 rebuild 耗时 |
| `total_create_factors_ms` | 本包 `create_factors()` 总耗时 |

## 缺失值

没有 registration 的 LOST/RELOCALIZING 帧会将 `delta_t`、`score` 等字段写为 `nan` 或空字段。`active_submaps` 使用可解析的 `0;1;217` 形式。

## Phase 0.6 验证

已生成：

```text
/tmp/glim_localization_phase0_6/debug_csv_failure_15s/debug/localization_debug.csv
```

该 CSV 覆盖 frame 120-140，并能定位 frame 126 的 target rebuild、大 correction 和 frame 127/128 的后续 correction。

## Phase 0.7 自动分析工具

Phase 0.7 新增：

```bash
python3 tools/analyze_debug_csv.py /path/to/localization_debug.csv --out /tmp/debug_csv_report
```

输出：

- `debug_csv_summary.md`
- `state_timeline.csv`
- `target_rebuild_events.csv`
- `correction_statistics.csv`
- `lost_segments.csv`
- `guard_actions.csv`
- `relocalization_summary.csv`
- `relocalization_topk_summary.csv`
- `relocalization_failure_reasons.csv`

该工具只依赖 Python 标准库。CSV 为空、格式错误或字段缺失时，会生成明确错误摘要，不影响 localization 运行。

## Phase 0.8 Relocalization 诊断

Phase 0.8 起，`--relocalization-debug` 会在不改变候选 gating 的情况下记录原始 top-k 和过滤原因。典型命令：

```bash
bash tools/run_standard_experiment.sh \
  --bag /path/to/bag \
  --map /path/to/glim_dump \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --duration 30 \
  --debug-csv \
  --relocalization-debug \
  --target-rebuild-guard \
  --smoother-guard \
  --lost-recovery \
  --output-dir /tmp/glim_localization_phase0_8_debug
```

注意：`relocalization_top1_*` 是 raw top-k 诊断结果，即使超过 `max_descriptor_distance` 也会被记录；只有 `relocalization_candidates_after_filter > 0` 的候选才会进入 geometric verification。

## Phase 1.0 Raw Top-K Recovery 实验

Phase 1.0 起，`verify_raw_topk` 是一个默认关闭的正式 recovery experiment。它不会放宽主 `localization.relocalization.max_descriptor_distance=0.35`，而是在 LOST/RELOCALIZING 且主候选为空时，把 raw top-k 中低于实验阈值的候选送入 geometric verification：

```json
"verify_raw_topk": {
  "enable": false,
  "topk": 10,
  "max_descriptor_distance": 0.55,
  "require_geometric_verification": true,
  "allow_descriptor_rejected_candidates": true
},
"recovering": {
  "enable": true,
  "stable_frames": 3,
  "max_recovery_correction_translation": 2.0,
  "max_recovery_correction_angle": 0.5,
  "require_no_rejection": true,
  "write_trajectory_during_recovering": true
}
```

安全边界：

- raw top-k 超过主阈值不能绕过 geometric verification。
- verification 成功后进入 `RECOVERING`，连续稳定帧满足要求后才回 `TRACKING`。
- `verify_raw_topk_success=1` 表示恢复实验验证通过，不等价于直接稳定定位成功；应结合后续 `RECOVERING -> TRACKING` 和 trajectory stats 判断。

## Phase 0.9 Descriptor Sweep

Phase 0.9 起，`analyze_debug_csv.py` 支持 threshold sweep：

```bash
python3 tools/analyze_debug_csv.py /path/to/localization_debug.csv \
  --out /tmp/debug_csv_report \
  --threshold-sweep 0.30,0.35,0.40,0.45,0.50,0.55,0.60,0.70
```

也可以单独运行：

```bash
python3 tools/sweep_scan_context_descriptors.py /path/to/localization_debug.csv \
  --out /tmp/scan_context_sweep \
  --frame-start 252 \
  --frame-end 300
```

`--verify-rejected-topk` 会对超过 descriptor 阈值的 raw top-k 做 debug-only geometric verification。该结果只写入 CSV / 报告，不会更新 pose、smoother、trajectory 或状态。
