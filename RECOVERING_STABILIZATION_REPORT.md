# RECOVERING Stabilization Report

## 1. 实现内容

Phase 1.0 将 relocalization 成功后的恢复路径固化为：

```text
RELOCALIZING -> RECOVERING -> TRACKING
```

而不是：

```text
RELOCALIZING -> TRACKING
```

## 2. 稳定帧配置

```json
"recovering": {
  "enable": true,
  "stable_frames": 3,
  "max_recovery_correction_translation": 2.0,
  "max_recovery_correction_angle": 0.5,
  "require_no_rejection": true,
  "write_trajectory_during_recovering": true
}
```

## 3. 稳定判据

每一帧 RECOVERING 中的 scan-to-map registration 需要满足：

- accepted；
- `delta_t <= 2.0m`；
- `delta_r <= 0.5rad`；
- `reject_reason` 为空。

满足则 `recovering_stable_count++`。达到 `stable_frames` 后进入 TRACKING。

## 4. rejection 行为

RECOVERING 中发生 rejection：

- `recovering_stable_count=0`；
- `recovery_frames_remaining` 重置为所需稳定帧数；
- 状态保持 RECOVERING；
- 若连续 rejection 达到 `matching.max_consecutive_rejections`，按原逻辑进入 LOST。

## 5. 实验观察

30s `verify_raw_topk_recovery_30s`：

- RECOVERING：120 frames；
- `RECOVERING -> TRACKING`：3 次；
- recovery rejection frames：57；
- `recovering_stable_count_max=3`；
- GTSAM warning：0。

30s `best_combo_050_30s`：

- RECOVERING：61 frames；
- `RECOVERING -> TRACKING`：1 次；
- LOST：7 frames；
- GTSAM warning：0；
- trajectory frames：294/301。

## 6. 当前不足

RECOVERING 稳定判据只检查单帧 correction，仍缺：

- active submap set 连续性；
- target center jump 限制；
- relocalization candidate 与 last accepted pose 的 continuity gate；
- verification target 是否足够稳定的检查。

## 7. 下一步建议

推荐下一轮添加：

- `recovery_continuity.max_translation_from_last_accepted`；
- `recovery_continuity.max_target_center_jump`；
- `recovery_continuity.require_active_submap_overlap`；
- RECOVERING 中 `write_trajectory_during_recovering=false` 的对比实验。
