---
title: 为UE5的Actor添加能够读写姿态的功能
date: 2025-06-04
source: https://blog.csdn.net/a2213086589/article/details/148429215
tags: [UE5, C++, Actor, Pose, Simulink, Location, Rotation]
category: UE5
---

# 为UE5的Actor添加能够读写姿态的功能

**发布日期**: 2025-06-04 17:08:07
**作者**: 氩氪氙氡_xuzhi
**标签**: #ue5 #机器人 #c++

## 文章概述

本文介绍如何为Unreal Engine 5中的Actor类添加读写三维位置和旋转信息的功能，支持与Simulink数据交互。

## 核心代码解析

### SetGetActorLocation.cpp 主要功能

该类通过Sim3D消息通道实现：

**输入功能**：
> "从SignalReader中读取float[3]数组，解析为X、Y、Z坐标，使用SetActorLocation设置Actor位置"

**输出功能**：
> "将当前坐标写入数组，用SignalWriter输出给外部系统"

```cpp
// 关键函数调用
SetActorLocation(NewLocation);
WriteSimulation3DMessage(SignalWriter, sizeof(float)*3, fvector);
```

## 扩展方案：合并位置与旋转功能

用户需求是创建**单个Actor实例**同时具有：
- 三维位置读写（Location）
- 三维旋转读写（Rotation）
- 静态网格体组件绑定

### 建议的合并设计

**类名**: `ASetGetActorLocationRotation`

**文件名**: `SetGetActorLocation_Rotation.cpp`

**核心特性**：
- 使用`FRotator`处理Roll、Pitch、Yaw三个旋转角
- `SetActorRotation()`函数设置旋转（继承自AActor）
- 分别创建"SetPos"/"GetPos"和"SetRot"/"GetRot"四个通道
- 在构造函数中添加`UStaticMeshComponent`进行网格绑定

此方案避免了两个实例的冗余，更符合游戏对象设计原则。
