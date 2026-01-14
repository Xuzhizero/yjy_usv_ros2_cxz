---
title: UE5中，导入的CAD的Actor为什么Simulate Physics无法被勾选？
date: 2025-11-14
source: https://blog.csdn.net/a2213086589/article/details/154825682
tags: [UE5, CAD, Physics, Datasmith, Collision]
category: UE5
---

# UE5中导入的CAD的Actor为什么Simulate Physics无法被勾选？

**发布日期:** 2025-11-14 09:45:28

**作者:** 氩氪氙氡_xuzhi

---

## 问题描述

用户在UE5中导入CAD文件后，发现StaticMeshComponent的 **Simulate Physics** 选项无法被勾选。

## 解决方案

### 主要原因：Mobility设置不正确

**关键步骤：**
1. 选中 StaticMeshComponent
2. 在Details面板中找到 **Transform** 部分的 **Mobility** 设置
3. 将其改为 **"Movable"**（静态和固定模式都无法启用物理模拟）

### 其他常见原因

#### 1. 碰撞设置问题
- 确保 **Collision Enabled** = "Query and Physics"
- 检查 **Object Type** 和碰撞响应配置

#### 2. **CAD模型缺少碰撞体**（最可能！）

打开Static Mesh编辑器：
- 在顶部工具栏找到 **Collision** 菜单
- 选择 **Auto Convex Collision**（推荐用于复杂模型）
  - Hull Count: 4-8
  - Hull Precision: 100000
- 或选择简单碰撞（Box/Capsule/Sphere）
- 点击 **Apply** 并保存

#### 3. 组件层级问题
- 确保该组件是根组件或其父组件允许物理模拟

### 快速检查清单
- ✅ Mobility = Movable
- ✅ Collision Enabled = Query and Physics
- ✅ 静态网格有碰撞体（绿色线框可见）
- ✅ 保存后返回Level Editor

**结论：** 导入的CAD模型通常缺少碰撞体。使用 **Auto Convex Collision** 自动生成碰撞体后，"Simulate Physics"选项即可启用。
