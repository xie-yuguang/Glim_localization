# ScanContext Self Query Report

## 1. 命令

```bash
source /home/xie/Glim/install/setup.bash
ros2 run glim_localization check_scan_context_relocalizer \
  /home/xie/Glim/data/map_data/ceshichang_128lidar 10
```

## 2. 结果摘要

```text
database_size: 232
query_count: 232
top_k: 10
top1_self: 218
top5_self: 228
failed_self_query: 1
top1_distance_min: 0.00338467
top1_distance_mean: 0.108655
top1_distance_max: 0.460177
top5_self_distance_mean: 0.110492
failed_submap_ids: 215
exit_code: 4
```

## 3. 判断

ScanContext map database 基本健康：232 个 submap 中 218 个 top1 是自身，228 个 top5 包含自身，只有 submap 215 self-query 未通过当前过滤。

因此 Phase 0.8 的 30s run 中 `candidates=0` 不像是 database 没建起来或 map index 全局损坏，更可能是在线 query scan descriptor 与 map submap descriptor 的距离超过当前 `max_descriptor_distance=0.35`。

## 4. 注意

工具返回 4 是因为存在 1 个 failed self-query。这个结果应视作诊断 warning，不代表工具不可用。
