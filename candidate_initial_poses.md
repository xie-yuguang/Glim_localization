# Candidate Initial Poses

本文件记录 Phase 0.5 中用于真实 bag 短片段 CPU baseline 的初始位姿候选。位姿格式为 `x y z roll pitch yaw`，角度单位为 rad。

bag:

- `/home/xie/Glim/data/bag_data/ceshichang_data_rosbag2_2025_03_27-14_47_39`
- 起始时间：`1743058060.351172527`
- 点云 topic：`/rslidar_points`
- IMU topic：`/imu`

map:

- `/home/xie/Glim/data/map_data/ceshichang_128lidar`
- GLIM dump，有 `232` 个 submap、`3471` 个地图轨迹帧。
- 地图轨迹起始时间：`1744772001.560494184`，与 bag 起始时间不一致，因此不能直接用时间戳关联 bag 起点和 map frame。

| candidate_id | x | y | z | roll | pitch | yaw | 来源 | 可信度 | 备注 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| c0_origin | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | 0.000000 | Phase 0 baseline 原始初值 | 中 | 必测对照项；0-10s 稳定，0-30s 发散。 |
| c1_map_first | 0.003799 | -0.000595 | -0.000308 | 0.003148 | 0.015065 | -0.000454 | `traj_imu.txt` 第一行 | 中 | map 第一帧 IMU pose；与 c0 接近，0-30s 仍发散。 |
| c2_prev_first | -0.040335 | 0.784889 | -0.990241 | 0.287509 | 0.080870 | -0.083388 | Phase 0 失败 baseline 轨迹第一行 | 低 | 这是 scan-to-map 修正后的输出，不是独立外部初值；0-30s 仍发散。 |
| c3_submap70 | 7.546420 | 7.432070 | -0.231980 | 0.000438 | 0.009397 | -1.060904 | `000070/data.txt` 的 `T_world_origin` | 低 | 地图中部候选；0-30s 早期 LOST 并触发 GTSAM underconstrained。 |

其他检查：

- `000000/data.txt` 的 `T_world_origin` 为 `(0.026179, -0.002189, -0.000207)`，与 c0/c1 接近。
- `traj_imu.txt` 中 submap 126/144/231 对应地图中后段，但没有证据表明 bag 起点从这些位置开始。
- 本地未发现可直接证明真实 bag 起点在某个 submap 的历史实验记录或人工 RViz 标注。

结论：

- 当前最可用的短片段初值仍是 `c0_origin` / `c1_map_first` 附近。
- 0-10 秒片段可作为临时 smoke baseline；30 秒片段尚不能作为稳定 localization baseline。
