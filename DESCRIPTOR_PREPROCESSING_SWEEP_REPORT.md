# Descriptor Preprocessing Sweep Report

## 1. 本轮状态

本轮没有实现 descriptor 输入变体 sweep。没有修改 ScanContext descriptor 构造、`num_rings`、`num_sectors`、`min_radius`、`max_radius`，也没有加入地面去除或高度归一化。

原因：

- Phase 0.9 的主要问题已经通过 threshold sweep 和 debug-only verification 定位：`0.35` descriptor 硬阈值过滤掉了可几何验证的 raw top-k。
- descriptor preprocessing 会改变候选分布，必须和 false positive 风险一起评估，不能在没有离线 descriptor dump 的情况下盲改。

## 2. 当前建议

短期不建议调整 `num_rings / num_sectors / max_radius` 作为首选方案。优先级更高的是：

1. 将 raw top-k + geometric verification 作为安全候选策略；
2. 做 query accumulation 的受控实验；
3. 再比较 descriptor preprocessing 是否进一步降低 top1 distance。

## 3. 后续可测变体

- 单帧原始 scan；
- 3 帧 / 5 帧累积 scan；
- 去除极低 z 或近似地面点；
- 高度归一化；
- `max_radius = 60 / 80 / 100`；
- `num_rings = 20 / 30`；
- `num_sectors = 60 / 90`。

