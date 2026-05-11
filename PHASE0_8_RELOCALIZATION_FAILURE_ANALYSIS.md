# Phase 0.8 Relocalization Failure Analysis

## 1. 数据来源

- CSV: `/tmp/glim_localization_phase0_8/relocalization_debug_30s/debug/localization_debug.csv`
- 分析目录: `/tmp/glim_localization_phase0_8/relocalization_debug_30s/debug_csv_analysis`
- 配置：CPU GICP，target confirmation guard on，smoother guard on，lost recovery on，relocalization debug on。

## 2. frame 252 前后链路

30s run 的长期 LOST 从 frame 252 开始，持续到 frame 300。

关键链路：

```text
frame 248 target rebuild
frame 250/251 large correction / rejection
frame 252 registration_rejections_exceeded -> LOST
frame 252 relocalization query 有效，但 candidates_after_filter=0
frame 253/254 继续 query，有效，但 candidates_after_filter=0
frame 255-258 lost_recovery_period_wait，不发起 query
之后每 5 帧尝试一次 query，均被 descriptor distance 阈值过滤
```

## 3. Query 是否有效

frame 252 后实际发起的 relocalization query 中：

- query cloud points 有效：4577 到 6502 点。
- descriptor 有效：`relocalization_descriptor_valid=1`。
- descriptor nonempty bins 有效：127 到 167。
- database size 正常：232。
- raw top-k 正常返回：`topk_returned=10`。

这排除了 `empty_query_cloud`、`descriptor_invalid`、`database_empty`、`no_topk` 作为主因。

## 4. candidates=0 的真实原因

实际发起 query 的失败原因全部是：

```text
all_filtered_by_descriptor_distance
```

frame 252-300 的典型 top1 distance：

```text
252 top1=223 distance=0.423758
253 top1=221 distance=0.418755
259 top1=221 distance=0.407722
269 top1=221 distance=0.453055
279 top1=221 distance=0.510240
294 top1=223 distance=0.425545
299 top1=223 distance=0.452786
```

当前阈值是 `localization.relocalization.max_descriptor_distance = 0.35`。上述 top1 均高于 0.35，因此所有 raw top-k 都被 descriptor filter 拦截，`relocalization_candidates_after_filter=0`。

## 5. Verification 是否调用

frame 252-300 的长期 LOST 段中：

- `relocalization_verification_attempted=0`
- `relocalization_verification_success=0`

原因不是 verification 失败，而是没有候选能通过 descriptor filter 进入 verification。

## 6. 最可能根因

最可能的失败链路是：

```text
target rebuild / large correction 造成 tracking 不稳定
-> frame 252 进入 LOST
-> query scan descriptor 有效，map database 正常
-> raw top-k 存在，但 top1/top3 distance 高于 0.35
-> 所有候选被 descriptor distance 过滤
-> geometric verification 没有机会执行
-> LOST recovery 只能周期性等待 + hold prior 防止 smoother 欠约束
```

## 7. 下一步证据需求

不建议直接放宽阈值伪造成功。下一轮建议做受控实验：

- 记录 top10 全量距离分布到单独 JSON/CSV。
- 对 `max_descriptor_distance` 做离线 sweep，只统计 candidate count 和 verification 结果，不直接作为默认配置。
- 对比 query scan 使用当前 scan、局部累积 scan、去地面/高度归一化 scan 的 descriptor distance。
- 检查 ScanContext descriptor 的 z 高度定义是否对 live scan 与 submap merged cloud 存在域差异。
