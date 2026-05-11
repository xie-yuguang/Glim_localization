# Query Accumulation Sweep Report

## 1. 本轮状态

本轮没有实现 runtime query accumulation，也没有完成单帧 / 3 帧 / 5 帧累积 scan 的真实 descriptor 对比。

原因：

- 当前 Debug CSV 记录的是 frame 级统计和 raw top-k 距离，不包含可重建 descriptor 的 scan 点云或 descriptor matrix。
- 在 runtime 内直接累积 query scan 会影响 relocalization 输入语义，风险高于本轮 Phase 0.9 的“离线 sweep + debug-only verification”目标。
- 现有 `ScanContextRelocalizer::make_descriptor()` 是 private，外部离线工具不能直接对任意历史 scan 做 descriptor 重建。

## 2. 已获得的间接证据

frame 252-300 的 query cloud 有效，点数约 `4577-6502`，descriptor nonempty bins 约 `127-167`，database size 正常。raw top-k 稳定集中在 submap `221/223/228/230` 附近，但 top1 distance 多在 `0.41-0.51`。

这符合“单帧 query scan 与 map submap descriptor 信息量不对称”的假设，但本轮还不能定量证明 3/5 帧累积会降低 distance。

## 3. 下一轮建议

建议下一轮做一个 debug-only ring buffer：

- 缓存最近 3/5 帧 scan；
- 仅在 LOST/RELOCALIZING 时构建累积 query descriptor；
- 限制累计帧之间相对位移和旋转；
- 同时输出 single-frame 和 accumulated-frame top-k distance；
- 仍不把累积结果直接当成正式 recovery。

