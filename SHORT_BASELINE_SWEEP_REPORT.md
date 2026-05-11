# Short Baseline Sweep Report

## 1. 实验设置

命令模板：

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "<candidate>" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 30 \
  --timeout-sec 90 \
  --benchmark-queries 100 \
  --output-dir /tmp/glim_localization_phase0_5_sweep/<run>
```

补充调试片段：

- `/tmp/glim_localization_phase0_5_debug/c0_origin_10s_offset0_dur10`
- `/tmp/glim_localization_phase0_5_debug/c0_origin_15s_offset0_dur15`

## 2. 结果表

| run | exit | frames | duration_s | total_distance_m | max_step_m | end_xyz | LOST/RELOCALIZING | rejections | GTSAM warnings | 是否可用 |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| c0_origin 0-10s | 0 | 101 | 10.000 | 6.962 | 0.302 | `(0.823, 0.656, -1.214)` | 0 / 0 | 0 | 0 | 是，临时 smoke baseline |
| c0_origin 0-15s | 0 | 151 | 14.999 | 43.808 | 5.553 | `(-4.032, 0.408, -4.751)` | 8 / 8 | 13 | 0 | 否，frame 126 后失稳 |
| c0_origin 0-30s | 0 | 301 | 29.999 | 1799.622 | 21.059 | `(-714.804, -121.010, -1524.424)` | 303 / 302 | 17 | 0 | 否，严重发散 |
| c1_map_first 0-30s | 0 | 301 | 29.999 | 924.062 | 14.074 | `(573.472, -433.529, -344.724)` | 245 / 244 | 25 | 0 | 否，发散 |
| c2_prev_first 0-30s | 0 | 301 | 29.999 | 1312.776 | 17.461 | `(938.311, 419.235, -640.698)` | 277 / 276 | 22 | 0 | 否，发散 |
| c3_submap70 0-30s | 250 | n/a | n/a | n/a | n/a | n/a | 61 / 60 | 7 | 4 | 否，GTSAM exception |

## 3. 关键观察

- `--duration` 已成功透传到 `glim_rosbag` 的 `playback_duration`，30 秒实验实际处理约 301 个点云帧。
- `c0_origin` 的 0-10s 片段无 LOST、无 registration rejection、无 GTSAM warning，可作为当前最小 smoke baseline。
- 所有 30 秒候选都失败：虽然部分 run 的 exit code 是 0，但 trajectory 和状态机显示定位已经 LOST/RELOCALIZING 并明显发散。
- 0-15s 已能复现 frame 126 触发后的不稳定，适合作为下一轮最小失败复现。

## 4. 最小复现

稳定 smoke baseline：

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 10 \
  --timeout-sec 90 \
  --output-dir /tmp/glim_localization_smoke_10s
```

最小失败复现：

```bash
bash src/Glim_localization/tools/run_standard_experiment.sh \
  --bag /home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39 \
  --map /home/xie/Glim/data/map_data/ceshichang_128lidar \
  --initial-pose "0 0 0 0 0 0" \
  --matching-method cpu_gicp \
  --start-offset 0 \
  --duration 15 \
  --timeout-sec 90 \
  --output-dir /tmp/glim_localization_failure_15s
```
