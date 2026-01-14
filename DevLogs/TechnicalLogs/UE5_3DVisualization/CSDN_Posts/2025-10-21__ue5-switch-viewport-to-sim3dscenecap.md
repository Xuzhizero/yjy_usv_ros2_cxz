---
title: UE5中右下角摄像头视角怎么切换为Sim3dSceneCap的视角？
date: 2025-10-21
source: https://blog.csdn.net/a2213086589/article/details/153675775
tags: [UE5, Simulink, Sim3dSceneCap, Camera, Viewport, Blueprint]
category: UE5
---

# UE5中右下角的摄像头视角怎么切换为Sim3dSceneCap的视角？

**发布日期:** 2025-10-21 08:43:48
**修改日期:** 2025-10-21 08:44:52
**浏览:** 761 | **点赞:** 21 | **收藏:** 15

---

## 问题描述

UE5中右下角现在有摄像头的视角窗口，不过摄像头的视角是默认的车辆后面的视角，不是`sim3dSceneCap`的视角。怎么切换为sim3dSceneCap的视角？

---

## 解决方案

### 核心要点

游戏视角默认跟随**被占有（Possess）的Pawn的CameraComponent**，而**Sim3dSceneCap是SceneCapture2D，不会自动成为游戏视角**。

### 快速方法（编辑器内预览）

1. **创建相机对象**：在关卡里新增CameraActor或在车辆上加CameraComponent，命名为`PreviewCam`

2. **对齐参数**：将`PreviewCam`的位置、朝向和FOV与Sim3dSceneCap保持一致

3. **切换视角**：在视口中选中`PreviewCam`，点击视口左上角的"Lock Viewport to Actor"选项

### 稳定方法（运行时自动切换）

在**Level Blueprint**的`Event BeginPlay`中添加：
- `Get Player Controller` → `Set View Target with Blend`
- **New View Target** 指向`PreviewCam`
- Blend Time设为0或自定义过渡时间

### 常见注意事项

- "Pilot"一个SceneCapture2D不会真正切换到其投影参数
- 车辆Pawn设置`Auto Possess Player = Player 0`不影响，BeginPlay中的Set View Target会覆盖默认视角
- 切换到`PreviewCam`后，Simulink回传仍来自**Sim3dSceneCap**（Tag配对正确时）

### 操作清单

✓ `PreviewCam`已与`Sim3dSceneCap`同位置/同朝向
✓ `PreviewCam`的FOV = `Sim3dSceneCap`的FOVAngle
✓ 编辑器中Pilot/Lock到`PreviewCam`，或通过Level Blueprint在BeginPlay调用`Set View Target with Blend`
✓ 保持`Sim3dSceneCap`的Tag = `IdealCamera1`

---

**标签:** #ue5 #3d #matlab
