---
title: UE5 的 Waterline Pro 6的浮力作用机制解析
date: 2025-11-12
source: https://blog.csdn.net/a2213086589/article/details/154730746
tags: [UE5, Waterline Pro, Buoyancy, Physics, Water]
category: UE5
---

# UE5 Waterline Pro 6 浮力作用机制解析

**发布日期:** 2025-11-12

## 核心内容总结

这篇文章通过Q&A形式深入讲解UE5中Waterline Pro的浮力系统架构。

### 主要话题

**1. BoatModel与Float点的关系**

船体采用典型的"一艘船+4个浮筒点"结构。BoatModel是启用物理模拟的刚体，Float1-4则充当"浮力采样点"，不参与物理计算，仅用于采样水面高度。

**2. Simulate Physics的作用**

该选项让组件成为物理引擎管理的刚体。启用后，物体会受重力、浮力、碰撞等力的影响；禁用则只能通过蓝图或动画驱动。

**3. 浮力计算的数据流**

流程为：水面采样 → 入水深度计算 → 浮力大小计算 → 力施加到刚体 → 物理积分。文章指出每帧执行这一循环，使船体能自然浮动、起伏与倾斜。

### BP_BuoyancyData参数

文章详解了关键参数：
- **C_Coefficient**: 控制浮力随深度增长的强度
- **C_DampingFactor**: 线性阻尼，控制运动平稳性
- **C_MaxForce**: 浮力上限限制
- **C_DragCoefficient**: 流体拖拽阻力

### Water Surface Mask功能

该组件为浮力系统提供实时水面数据（高度、法线、流向），是"视觉层水面与物理层浮力系统的桥梁"。它从波场采样局部高度贴图，确保浮力与波浪视觉同步。

**标签:** #ue5
