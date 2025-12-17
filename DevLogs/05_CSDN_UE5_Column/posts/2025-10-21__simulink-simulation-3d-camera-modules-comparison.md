---
title: simulink中的Simulation 3D Camera模块区别
date: 2025-10-21
source: https://blog.csdn.net/a2213086589/article/details/153676094
tags: [Simulink, Simulation 3D, Camera, UE5, FOV]
category: UE5
---

# Simulink中的Simulation 3D Camera模块和Simulation 3D Camera Get模块的区别

**发布日期：** 2025-10-21 08:46:37

---

## 文章内容

### 问题1：FOV参数位置

**Q:** 为什么Simulink中的Simulation 3D Camera模块没有看到视场角Field of Angle参数项？

**A:** 在R2024b中，**Simulation 3D Camera Get**的视场角参数位于对话框的**Parameters**小节，名称为**Horizontal field of view（水平视场角，度）**，默认值为60度。需要展开Parameters小节才能看到该参数。

若无法找到，可能的原因：
- 点开的不是Camera Get块
- Parameters小节处于折叠状态
- 需要在UE5侧修改SceneCapture的FOV Angle

---

### 问题2：两个模块的核心区别

#### **Simulation 3D Camera Get**
- **功能：** 读取UE场景中已存在的理想相机（Sim3dSceneCap/SceneCapture2D）
- **工作流：** 在UE5中放置相机，标记为`IdealCameraN`，在块中配置Sensor identifier配对
- **输出：** 仅RGB图像，可设分辨率与水平FOV
- **特点：** 无畸变、内参或地面真值端口

#### **Simulation 3D Camera**（非Get）
- **功能：** 由Simulink侧生成/管理的带镜头模型的相机（针孔+镜头畸变）
- **参数：** 焦距、主点、像幅尺寸、径向/切向畸变、轴偏斜等完整内参
- **输出：** Image、Depth、Labels、Translation/Rotation等Ground Truth数据
- **用途：** 适合感知算法训练、标注与几何评测

---

### 使用建议

**选择Camera Get的场景：**
> "不想在Simulink里放车辆块，只想让UE5里的相机回传画面...最轻、最稳"

流程：UE5中Attach相机到Pawn并标记 → Simulink中Camera Get配置Sensor ID → 连接Video Display

**选择Simulation 3D Camera的场景：**
需要相机畸变、标定内参、深度图、语义分割或相机位姿等地面真值数据时使用。

---

## 关键配置

两个模块共同依赖**Simulation 3D Scene Configuration**，执行顺序需设置为：Scene Config（优先级0）→ 相机块（优先级1）
