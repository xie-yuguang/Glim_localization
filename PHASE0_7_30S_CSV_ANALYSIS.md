# Phase 0.7 30s CSV Analysis

## 数据来源

- Phase 0.6 组合实验 CSV：`/tmp/glim_localization_phase0_6/best_30s_experimental/debug/localization_debug.csv`
- Phase 0.7 best 30s CSV：`/tmp/glim_localization_phase0_7/regression_matrix_v2/experimental_30s_best/debug/localization_debug.csv`
- 自动分析输出：`/tmp/glim_localization_phase0_7/regression_matrix_v2/experimental_30s_best/debug_csv_analysis/`

## 30s best 关键结论

| 指标 | 结果 |
| --- | ---: |
| CSV frames | 301 |
| trajectory frames | 251 |
| first DEGRADED | 126 |
| first LOST | 144 |
| longest LOST segment | 252..300, 49 frames |
| LOST frames | 50 |
| smoother hold prior | 50 |
| relocalization requested | 49 |
| relocalization success | 2 |
| relocalization no candidates | 48 |
| GTSAM warning | 0 |

说明：CSV 保留了全部 301 帧；trajectory 只有 251 行，因为 Phase 0.7 默认 `write_trajectory_during_hold=false`，从 frame 252 开始的 hold frame 没有写入轨迹文件。

## 失败链路

最可能链路：

```text
frame 126 target rebuild
  -> frame 126 large_translation_correction rejected
  -> frame 127 target_rebuild_large_correction rejected by confirmation guard
  -> frame 128/129 小 correction 通过 confirmation
  -> frame 132/133/138/139 多次 high score + large delta
  -> frame 144 短暂 LOST 后 relocalization 恢复
  -> frame 248 target rebuild
  -> frame 250/251 large translation/rotation correction
  -> frame 252 进入长期 LOST
  -> frame 252..300 relocalization candidates 基本为 0
  -> smoother guard hold_last_pose_prior 防止 smoother 欠约束，但不代表定位成功
```

## target rebuild 相关性

target rebuild 与 DEGRADED/LOST 有明显相关性：

- frame 126 rebuild 后立即出现 `delta_t=5.015m`。
- frame 127 的 `delta_t=2.785m` 被 confirmation guard 拦截为 `target_rebuild_large_correction`。
- frame 248 rebuild 后，frame 250/251 继续出现 high delta；frame 252 进入长期 LOST。

但 30s 后段的长期 LOST 不只是 target rebuild guard 能解决的问题。更关键的是 LOST 后 relocalization 候选长期为 0。

## high score + large delta

30s best 中主要异常帧：

- frame 126：score=0.935，delta_t=5.015，`large_translation_correction`
- frame 127：score=0.958，delta_t=2.785，`target_rebuild_large_correction`
- frame 132：score=0.932，delta_t=4.826，`large_translation_correction`
- frame 133：score=0.929，delta_t=4.986，`large_translation_correction`
- frame 138：score=0.894，delta_t=3.951，`large_translation_correction`
- frame 139：score=0.897，delta_t=3.539，`large_translation_correction`
- frame 140：score=0.886，delta_t=2.942，被 accepted

## relocalization

30s best 中 relocalization requested 49 帧，success 2 帧，no candidates 48 帧。长期 LOST 的核心阻塞是：进入 LOST 后查询不到有效 ScanContext 候选，无法进入几何验证恢复链路。

## 还缺的字段

当前 CSV 已足够定位 frame、状态、target rebuild、correction 和 guard 动作。下一步若要解释 no candidates，需要补充：

- relocalization query descriptor distance top-k；
- query scan point 数和 ring/sector 有效占用；
- 候选被 `max_candidate_translation_delta` 或 descriptor 阈值过滤前后的数量；
- verification 失败时每个候选的 score/residual/inlier。
