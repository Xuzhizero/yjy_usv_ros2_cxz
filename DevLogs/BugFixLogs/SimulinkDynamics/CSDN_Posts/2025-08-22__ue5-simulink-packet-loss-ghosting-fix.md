---
title: 修复Simulink到UE5丢包时被控船体的残影问题
date: 2025-08-22
source: https://blog.csdn.net/a2213086589/article/details/150578134
tags: [UE5, Simulink, Blueprint, Message, Packet Loss, Debugging]
category: UE5
---

# 修复Simulink到UE5丢包时被控船体的残影问题

**发布日期:** 2025-08-22 17:23:35
**最后修改:** 2025-08-22 17:24:58
**作者:** 氩氪氙氡_xuzhi
**标签:** #ue5 #matlab

## 问题描述

使用Simulink的"Simulation 3D Scene Configuration"和"Simulation 3D Message"模块与UE5协同时，出现被控Actor位置坐标中混杂0值的现象。虽然模块优先级和采样时间(0.02s)配置正确，但运行后期稳定出现"2个正常接收(x=100)中间夹杂3个x=0"的模式，导致Actor在屏幕中产生残影效果。

## 问题原因分析

关键蓝图图中直接将"Read Vector From Simulation 3D Message"的Data输出连接到"SetActorLocationAndRotation"。问题在于未使用Status/Found状态检查。当消息未命中时，Data会默认回退为(0,0,0)坐标，该零值被直接写入Actor位置，导致闪烁。

## 解决方案

**创建缓存机制防止无效数据写入:**

1. **添加缓存变量:**
   - LastLocation (Vector类型)
   - LastRotation (Rotator类型)
   - bLocValid (布尔值)
   - bRotValid (布尔值)

2. **修改位置处理流程:**
   - 连接Read Vector节点的红色**Status**引脚至Branch节点
   - True分支：更新LastLocation，设置bLocValid=true
   - False分支：跳过Data写入（禁止零值回写）

3. **对旋转数据做同样处理:**
   - Status→Branch逻辑分离有效/无效数据

这样确保Actor仅在收到有效数据时更新位置，未命中消息时保持上一帧的有效状态，消除残影问题。
