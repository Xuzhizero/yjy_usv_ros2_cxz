---
title: UE5导入的CAD文件零件如何被Merge?
date: 2025-11-14
source: https://blog.csdn.net/a2213086589/article/details/154821050
tags: [UE5, CAD, Merge, Datasmith, Nanite]
category: UE5
---

# UE5导入的CAD文件零件如何被Merge?

**发布日期:** 2025-11-14 09:01:41

---

## 问题描述

UE5中导入的CAD文件各个零件未被合并，点击外壳会选中具体的组成部分而非整体。用户希望将所有组件合并成一个整体。

---

## 解决方案

### 方法1：Merge Actors（推荐）

**步骤：**

1. 在World Outliner中多选所有要合并的零件（Ctrl+点击）
2. 编辑器左上角工具栏 → **Merge Actors**
3. 在弹出窗口中：
   - 确认选中的Actors列表
   - 配置Mesh Settings（材质合并方式）
   - 点击 **"Merge Actors"**
4. 保存合并后的新Static Mesh资产

**优点：** 生成新的Static Mesh资产，性能最优
**缺点：** 合并后无法单独编辑各零件

---

## Replace Source Actors 说明

勾选此选项后的效果：

- ✅ **仅影响当前Level中的Actor实例**
- ✅ 删除原单独零件Actors，创建新合并Actor替换
- ✅ 生成新Static Mesh资产保存到Content Browser
- ❌ **不修改DataSmith Scene源文件**
- ❌ **不影响原始资产**

**验证：** 再次从Content Browser导入同一DataSmith Scene仍为未合并状态。

---

## Nanite Settings 指南

### 是否启用Nanite？

**启用条件（✅建议）：**
- 模型多边形数量 > 50,000
- 复杂CAD/建筑细节
- 需在场景中放置多实例
- 目标平台为PC/主机

**不启用条件（❌不建议）：**
- 模型面数 < 5,000
- 需要骨骼动画或变形
- 需要World Position Offset材质效果
- 目标平台为移动设备

### Nanite代价：

| 代价 | 说明 |
|------|------|
| **性能开销** | 简单模型反而性能更差 |
| **硬件要求** | 需支持Mesh Shader的显卡 |
| **构建时间** | 初次导入更耗时（几秒至几分钟） |
| **存储占用** | 通常为原模型1.5-3倍 |
| **功能限制** | 不支持骨骼、Morph、半透明等 |

### 经验法则：

| 模型类型 | 三角面数 | 启用Nanite |
|---------|---------|----------|
| 简单道具 | <5K | ❌ |
| 普通资产 | 5K-50K | 🤔可选 |
| 复杂资产 | 50K-500K | ✅推荐 |
| 超高精度 | >500K | ✅✅强烈推荐 |

---

## 最佳实践

**策略性启用Nanite：** 针对高精度CAD模型启用，对简单几何体不必启用。工具应匹配使用场景。

对于复杂机械零件或建筑模型，建议启用Nanite以显著提升性能。
