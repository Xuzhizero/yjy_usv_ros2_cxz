---
title: UE5提升分辨率和帧率的方法
date: 2025-08-28
source: https://blog.csdn.net/a2213086589/article/details/150955505
tags: [UE5, Resolution, Framerate, TSR, Performance, Optimization]
category: UE5
---

# UE5提升分辨率和帧率的方法

**发布日期**: 2025-08-28 16:32:11

**作者**: 氩氪氙氡_xuzhi

**浏览量**: 2.4k | **点赞**: 18 | **收藏**: 23

---

## 核心概念

### 分辨率与帧率的区别

**分辨率** 主要影响"看起来糊不糊"；**帧率与帧时间稳定性** 主要影响"顺不顺"。

---

## 快速提升UE5分辨率的三个方法

### 1. 使用超分技术"提清晰"

在 **Project Settings → Rendering → Anti-Aliasing Method** 中将设置改为 **TSR (Temporal Super Resolution)**。该技术能在同等或更低内部分辨率下输出更清晰的图像，与 **Screen Percentage** 配合使用效果最佳。

参考资源：[Epic Games Developers - Temporal Super Resolution](https://dev.epicgames.com/documentation/en-us/unreal-engine/temporal-super-resolution-in-unreal-engine)

### 2. 调整渲染分辨率

在视口或控制台中调整 **`r.ScreenPercentage`** 参数：
- **设为100**：原生分辨率
- **设为110-130**：轻度超采样，画质更锐但GPU消耗更高

### 3. 启用动态分辨率

使用动态分辨率自动上下调整：

```
r.DynamicRes.OperationMode=1
r.DynamicRes.MinScreenP
```

这提供了一个"画质↔性能"的动态平衡旋钮。

---

## 文章标签

`#ue5` `#人工智能`

---

**相关推荐文章**：
- UE5实现项目基本设置（分辨率、音量、渲染等）
- UE5使用DLSS超级采样提升FPS优化方案
- UE5游戏性能优化指南
