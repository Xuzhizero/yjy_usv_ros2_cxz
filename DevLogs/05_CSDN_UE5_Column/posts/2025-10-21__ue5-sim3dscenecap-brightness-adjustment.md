---
title: UE5中的sim3dSceneCap中视角亮度怎么调节？
date: 2025-10-21
source: https://blog.csdn.net/a2213086589/article/details/153676202
tags: [UE5, Simulink, Sim3dSceneCap, Camera, Exposure, Brightness]
category: UE5
---

# UE5中的Sim3dSceneCap视角亮度调节指南

## 文章信息
- **标题**: UE5中的sim3dSceneCap中视角亮度怎么调节？
- **发布日期**: 2025-10-21
- **作者**: 氩氪氙氡_xuzhi
- **浏览量**: 1.2k | 点赞: 27 | 收藏: 11

---

## 核心解决方案

### 快速调整方法

用户遇到的问题是通过 Simulation 3D Camera Get 获取的画面过暗。解决方案涉及两个主要位置的参数调整：

**1. 捕获源设置**
- 选中 `Sim3dSceneCap` → `SceneCaptureComponent2D`
- 将 **Capture Source** 改为 **Final Color (LDR) in RGB**
- 这样可以应用色调映射和伽马校正，立即改善画面亮度

**2. 后期曝光参数**
在 Post Process Settings 中：
- **Metering Mode**: 选择 Auto Exposure Basic 或 Manual
- **Min/Max EV100**: 设置为相同数值以锁定亮度
- **Exposure Compensation**: 调整 +1~+2 EV 进行微调

---

## 具体调整步骤

### 方案一：固定亮度（最稳定）
- 勾选 Min EV100 和 Max EV100，设为相同值
- 从 **EV = 1.0~2.0** 开始调整
- 如仍过暗，向负数方向调（如 -1.0、-2.0）

### 方案二：保留自动曝光
- 启用 Exposure Compensation，设为 **+1~+2**
- 允许根据场景自动调整，但亮度总体提升
- 适合画面需要自然变化的场景

---

## 关键经验总结

> "EV每增加+1等于亮度减半；每减少-1等于亮度翻倍"

用户最初设置 EV=8.5 导致全黑的原因，是不了解 EV 值的反向逻辑。正确做法是从 EV=0 开始，根据需要向负值调整以增加亮度。

**其他检查项**：
- 确认 Capture Source 使用 LDR Final Color
- 验证场景光源强度（日光/天光）
- 检查 Post Process Volume 的 Infinite Extent (Unbound) 是否启用

---

**结果**: 用户按照此指南调整后成功解决了问题。
