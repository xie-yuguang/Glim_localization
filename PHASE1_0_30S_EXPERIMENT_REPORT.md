# Phase 1.0 30s Experiment Report

## 1. 实验矩阵

| case | 说明 |
| --- | --- |
| `smoke_10s` | 10s smoke，所有 recovery 实验关闭 |
| `recovery_off_30s` | 30s 对照，recovery 关闭 |
| `verify_raw_topk_recovery_30s` | 30s raw top-k recovery，distance=0.55，stable_frames=3 |
| `sweep_045_30s` | sensitivity，distance=0.45，stable_frames=3 |
| `sweep_050_30s` | sensitivity，distance=0.50，stable_frames=3 |
| `sweep_050_stable5_30s` | sensitivity，distance=0.50，stable_frames=5 |
| `best_combo_050_30s` | Phase 0.7 guards + raw top-k recovery，distance=0.50 |

## 2. 10s smoke

- output：`/tmp/glim_localization_phase1_0/smoke_10s`
- exit code：0
- frames：101
- max_step_m：0.302
- suspicious_divergence：false
- LOST：0
- rejection：0
- GTSAM warning：0

## 3. 30s recovery off

- output：`/tmp/glim_localization_phase1_0/recovery_off_30s`
- exit code：0
- frames：301
- total_distance_m：1799.622
- max_step_m：21.059
- suspicious_divergence：true
- LOST log count：303
- relocalizing log count：302
- rejection count：17
- GTSAM warning：0

结论：不加 Phase 0.7/1.0 保护时，30s 仍会明显发散。

## 4. 30s recovery on

- output：`/tmp/glim_localization_phase1_0/verify_raw_topk_recovery_30s`
- exit code：0
- frames：301
- total_distance_m：391.431
- max_step_m：9.188
- suspicious_divergence：false
- LOST log count：64
- relocalizing log count：64
- rejection count：87
- GTSAM warning：0

CSV 状态：

- TRACKING：168
- RECOVERING：120
- DEGRADED：7
- LOST：6
- verify_raw_topk_used：28
- verify_raw_topk_success：22
- `RECOVERING -> TRACKING`：3

结论：raw top-k recovery 能恢复状态，但 recovery 后仍不够稳定。

## 5. best combo 30s

- output：`/tmp/glim_localization_phase1_0/best_combo_050_30s`
- exit code：0
- CSV frames：301
- trajectory frames：294
- total_distance_m：145.450
- max_step_m：9.452
- suspicious_divergence：false
- LOST log count：37
- rejection count：55
- GTSAM warning：0

CSV 状态：

- TRACKING：225
- RECOVERING：61
- DEGRADED：8
- LOST：7
- verify_raw_topk_used：17
- verify_raw_topk_success：14
- smoother hold prior：7
- target rebuild guard rejected：5
- `RECOVERING -> TRACKING`：1

结论：best combo 比 recovery off 稳定很多，但 max_step 仍过大，不能称为稳定 30s baseline。

## 6. 是否获得 30s baseline

没有。

当前最佳结果是“可控失败”：

- GTSAM warning=0；
- suspicious_divergence=false；
- LOST 可恢复；
- 但仍存在 7m 到 9m 单步跳变和多次 recovery rejection。

## 7. 失败原因

主要不是 candidates=0 了，而是：

- raw top-k verification 后的局部几何验证不等于长期稳定 tracking；
- recovery target map / active submaps 仍可能不连续；
- RECOVERING 判据缺少 target center 和 active submap continuity；
- verification target 可能太局部，恢复后的下一帧容易出现大 correction。
