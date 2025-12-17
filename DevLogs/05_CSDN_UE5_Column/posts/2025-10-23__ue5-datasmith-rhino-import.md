---
title: 使用 Datasmith 将 Rhino 模型导入 Unreal Engine 5
date: 2025-10-23
source: https://blog.csdn.net/a2213086589/article/details/153772736
tags: [UE5, Datasmith, Rhino, CAD, Import]
category: UE5
---

# 使用 Datasmith 将 Rhino 模型导入 Unreal Engine 5（UE5）

**发布日期**: 2025-10-23 09:41:55
**浏览量**: 878 | **点赞**: 28 | **收藏**: 17

---

## 文章内容

### 为什么要用 Datasmith？

Datasmith 是 Epic Games 提供的 CAD 到 UE 数据传输工具链。它能够保留模型层级结构、材质与贴图、以及相机和光源信息。需要在 UE5 和 CAD 软件中分别安装插件。

### SolidWorks vs Rhino

虽然 SolidWorks 支持 Datasmith 导出，但在材质和渲染方面受限，导出模型多呈灰黑色调。Rhino 在材质管理上更灵活，支持直接调整模型外观颜色，因此该项目选择 Rhino 路线。

### 在 Rhino 端安装 Datasmith Exporter

1. 访问 [Epic Games 官方插件下载页](https://www.unrealengine.com/en-US/datasmith/plugins)
2. 确保 Rhino 版本与 UE5 版本匹配
3. 运行安装程序并保持默认设置
4. 重新启动 Rhino

### 从 Rhino 导出 `.udatasmith` 文件

**步骤**：
- 文件 → 导出选定对象
- 选择 Datasmith (.udatasmith) 格式
- 点击导出

**文件结构**：
```
ship_scene.udatasmith
ship_scene/
├─ Assets/
│  ├─ brep_1.udsmesh
│  └─ ...
├─ Materials/
└─ Textures/
```

**关键注意点**：
- ".udatasmith 与同名文件夹必须在同一目录下"
- 如未使用贴图，Materials/Textures 文件夹可能不生成
- 在导出时改名，而非导出后修改
- 建议使用英文命名避免路径编码问题

### `.udatasmith` 工作机制

该文件是场景描述文件，本身不含几何数据，而是引用同目录的 `.udsmesh` 文件。UE5 导入时会读取文件并将其转换为 Static Mesh 格式。

### 在 UE5 中启用 Datasmith 插件

1. 编辑 → 插件
2. 搜索并启用：
   - Datasmith CAD Importer
   - Datasmith Importer
   - Datasmith Content
3. 重启 UE5

### 在 UE5 中导入 `.udatasmith`

1. 创建 → Datasmith → 文件导入
2. 选择 `.udatasmith` 文件
3. 推荐导入选项：
   - ✅ Import Geometry
   - ✅ Import Materials
   - ✅ Import Textures
   - ✅ Convert to Static Mesh
   - ✅ Generate Lightmap UVs
   - ❌ Merge Actors（保持层级结构）

### 常见问题与排查

| 问题 | 原因 | 解决方法 |
|------|------|--------|
| 导入报错 | 路径不匹配 | 恢复原始文件夹名或重新导出 |
| 材质全黑 | 未带贴图或光照不足 | 添加光源或重新设置材质 |
| 无 Textures 文件夹 | 模型未使用贴图 | 正常现象 |
| 比例不对 | 单位换算问题 | Scale Factor 设置为 0.1 |

### 总结

1. 在 Rhino 中安装 Datasmith Exporter
2. 导出 `.udatasmith` 与资源文件夹
3. 在 UE5 中启用 Datasmith 插件并导入
4. 验证材质与网格正确加载

**作者经验**：永远不要在导出后改 `.udatasmith` 或文件夹名字，UE5 对路径引用极其严格。

---

## 标签

- ue5
- Datasmith
- 模型导入
- Rhino
