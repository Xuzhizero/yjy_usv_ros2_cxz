# 开发日志索引 (DevLogs Index)

**Tags:** `devlogs`, `index`, `development-history`, `engineering-notes`

**最后更新**: 2026-01-14

---

## 简介

本目录记录项目开发过程中遇到的问题、解决方案、技术决策以及经验总结。按**技术开发日志**和**Bug修复日志**两大类分类，每类下按功能模块细分，便于后续检索和知识传承。

---

## 目录结构

```
DevLogs/
├── README.md                           # 本文件：索引与导航
├── TechnicalLogs/                      # 技术开发日志
│   ├── MultiMachineComm/               # 多机通信
│   ├── UE5_3DVisualization/            # UE5三维场景可视化
│   ├── FlightControl/                  # 航控部分
│   ├── Perception/                     # 感知部分
│   ├── SimulinkDynamics/               # Simulink动力学模型
│   └── LinuxSystem/                    # Linux系统相关
└── BugFixLogs/                         # Bug修复日志
    ├── MultiMachineComm/               # 多机通信
    ├── UE5_3DVisualization/            # UE5三维场景可视化
    ├── FlightControl/                  # 航控部分
    ├── Perception/                     # 感知部分
    ├── SimulinkDynamics/               # Simulink动力学模型
    └── LinuxSystem/                    # Linux系统相关
```

---

## TechnicalLogs - 技术开发日志

### MultiMachineComm - 多机通信

| 文档 | 描述 | 状态 |
|------|------|------|
| [Network_Connection_Setup.md](TechnicalLogs/MultiMachineComm/Network_Connection_Setup.md) | 研华工控机与仿真计算机网络连接配置 | 已完成 |
| [Node_Integration.md](TechnicalLogs/MultiMachineComm/Node_Integration.md) | ROS2节点集成记录 | 模板 |

---

### UE5_3DVisualization - UE5三维场景可视化

| 文档 | 描述 | 状态 |
|------|------|------|
| [Level_Migration.md](TechnicalLogs/UE5_3DVisualization/Level_Migration.md) | UE关卡迁移与资产更新 | 已完成 |

**CSDN技术文章归档：**

| 日期 | 标题 | 主题 |
|------|------|------|
| 2025-11-14 | [UE5 CAD零件合并](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-11-14__ue5-cad-merge-parts.md) | DataSmith/CAD |
| 2025-11-14 | [DataSmith CAD模型迁移](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-11-14__ue5-datasmith-cad-move-to-another-level.md) | DataSmith/CAD |
| 2025-11-13 | [UDS官方光源和雾](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-11-13__ue5-uds-official-light-sources.md) | Ultra Dynamic Sky |
| 2025-11-12 | [Waterline Pro浮力机制](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-11-12__ue5-waterline-pro-buoyancy-mechanism.md) | 物理仿真 |
| 2025-11-03 | [UDS光照对齐真实时间](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-11-03__ue5-uds-align-real-time.md) | Ultra Dynamic Sky |
| 2025-10-27 | [Ultra Dynamic Sky教程](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-10-27__ue5-ultra-dynamic-sky-tutorial.md) | Ultra Dynamic Sky |
| 2025-10-27 | [关卡蓝图视图恢复](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-10-27__ue5-level-blueprint-view-recovery.md) | 蓝图系统 |
| 2025-10-23 | [Datasmith导入Rhino模型](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-10-23__ue5-datasmith-rhino-import.md) | DataSmith/CAD |
| 2025-10-21 | [Sim3dSceneCap亮度调节](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-10-21__ue5-sim3dscenecap-brightness-adjustment.md) | 视角与渲染 |
| 2025-10-21 | [视角切换到Sim3dSceneCap](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-10-21__ue5-switch-viewport-to-sim3dscenecap.md) | 视角与渲染 |
| 2025-08-28 | [分辨率和帧率优化](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-08-28__ue5-improve-resolution-and-framerate.md) | 性能优化 |
| 2025-08-22 | [关卡蓝图副本保存](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-08-22__ue5-level-blueprint-copy.md) | 蓝图系统 |
| 2025-06-05 | [蓝图中配置Actor资产](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-06-05__ue5-blueprint-configure-actor-type-asset.md) | 蓝图系统 |
| 2025-06-04 | [C++类集成到蓝图](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-06-04__ue5-cpp-class-to-blueprint-integration.md) | C++开发 |
| 2025-06-04 | [Actor姿态读写功能](TechnicalLogs/UE5_3DVisualization/CSDN_Posts/2025-06-04__ue5-actor-pose-read-write.md) | C++开发 |

---

### FlightControl - 航控部分

| 文档 | 描述 | 状态 |
|------|------|------|
| [differential_steering_verification_table.md](TechnicalLogs/FlightControl/differential_steering_verification_table.md) | 差速转向验证表 | 已完成 |

---

### Perception - 感知部分

*暂无文档*

---

### SimulinkDynamics - Simulink动力学模型

| 文档 | 描述 | 状态 |
|------|------|------|
| [UE5_Simulink_Interface.md](TechnicalLogs/SimulinkDynamics/UE5_Simulink_Interface.md) | UE5与Simulink通信接口 | 模板 |
| [Simulink_UE5_Performance_Analysis.md](TechnicalLogs/SimulinkDynamics/Simulink_UE5_Performance_Analysis.md) | 性能分析对比 | 已完成 |
| [USV_Simulation_Framework_QA.md](TechnicalLogs/SimulinkDynamics/USV_Simulation_Framework_QA.md) | 仿真框架深度问答 | 已完成 |
| [analysis_turning_radius_too_large.md](TechnicalLogs/SimulinkDynamics/analysis_turning_radius_too_large.md) | 转向半径分析 | 已完成 |

**CSDN技术文章归档：**

| 日期 | 标题 | 主题 |
|------|------|------|
| 2025-11-03 | [Simulink驱动UE5海况切换](TechnicalLogs/SimulinkDynamics/CSDN_Posts/2025-11-03__ue5-simulink-sea-state-switching.md) | 联合仿真 |
| 2025-10-21 | [Simulation 3D Camera模块对比](TechnicalLogs/SimulinkDynamics/CSDN_Posts/2025-10-21__simulink-simulation-3d-camera-modules-comparison.md) | 联合仿真 |

**图片资源：**
- [img/](TechnicalLogs/SimulinkDynamics/img/) - 仿真相关图片资源

---

### LinuxSystem - Linux系统相关

| 文档 | 描述 | 状态 |
|------|------|------|
| [GDM启动失败与磁盘空间不足故障排查.md](TechnicalLogs/LinuxSystem/GDM启动失败与磁盘空间不足故障排查.md) | GDM故障排查日志 | 已完成 |
| [向日葵远程控制与Wayland-Xorg切换指南.md](TechnicalLogs/LinuxSystem/向日葵远程控制与Wayland-Xorg切换指南.md) | 远程控制与显示协议 | 已完成 |

---

## BugFixLogs - Bug修复日志

### UE5_3DVisualization - UE5三维场景可视化

| 文档 | 描述 | 状态 |
|------|------|------|
| [BP_Compile_Errors.md](BugFixLogs/UE5_3DVisualization/BP_Compile_Errors.md) | 蓝图编译错误排查 | 模板 |
| [UE5_Slate_Array_Index_Crash.md](BugFixLogs/UE5_3DVisualization/UE5_Slate_Array_Index_Crash.md) | UI框架崩溃修复 | 已完成 |

**CSDN Bug修复文章归档：**

| 日期 | 标题 | 问题类型 |
|------|------|---------|
| 2025-11-12 | [蓝图实例引用问题](BugFixLogs/UE5_3DVisualization/CSDN_Posts/2025-11-12__ue5-blueprint-instance-reference-issue.md) | 蓝图引用 |

---

### SimulinkDynamics - Simulink动力学模型

| 文档 | 描述 | 状态 |
|------|------|------|
| [GM_Negative_Natural_Frequency_NaN_Issue.md](BugFixLogs/SimulinkDynamics/GM_Negative_Natural_Frequency_NaN_Issue.md) | 6DOF自然频率NaN问题 | 已完成 |
| [Ship_Propeller_Trajectory_Issue.md](BugFixLogs/SimulinkDynamics/Ship_Propeller_Trajectory_Issue.md) | 螺旋桨轨迹异常 | 已完成 |
| [fix_surge_damping_coefficient_Xu.md](BugFixLogs/SimulinkDynamics/fix_surge_damping_coefficient_Xu.md) | 纵荡阻尼系数修复 | 已完成 |

**CSDN Bug修复文章归档：**

| 日期 | 标题 | 问题类型 |
|------|------|---------|
| 2025-11-14 | [CAD Actor物理模拟问题](BugFixLogs/SimulinkDynamics/CSDN_Posts/2025-11-14__ue5-cad-actor-simulate-physics-issue.md) | 物理配置 |
| 2025-08-22 | [丢包残影修复](BugFixLogs/SimulinkDynamics/CSDN_Posts/2025-08-22__ue5-simulink-packet-loss-ghosting-fix.md) | 通信 |
| 2025-06-04 | [显存不足错误](BugFixLogs/SimulinkDynamics/CSDN_Posts/2025-06-04__ue5-simulink-video-memory-error.md) | 资源 |
| 2025-06-04 | [第二次仿真崩溃](BugFixLogs/SimulinkDynamics/CSDN_Posts/2025-06-04__ue5-simulink-second-run-crash.md) | 稳定性 |

---

## 按标签检索

### 控制算法
- `#PID` - PID控制相关
- `#MFAC` - 无模型自适应控制
- `#Splane` - S-plane控制

### 系统模块
- `#ROS2` - ROS2相关问题
- `#UE5` - Unreal Engine 5
- `#Simulink` - MATLAB/Simulink
- `#Navigation` - 导航与制导
- `#Network` - 网络通信

### 问题类型
- `#Bug` - Bug修复记录
- `#Performance` - 性能优化
- `#Refactor` - 重构记录
- `#Integration` - 集成问题

---

## 如何添加新日志

### 模板结构

每个开发日志应包含以下部分：

```markdown
# [标题]

**Tags:** `tag1`, `tag2`, `tag3`
**日期**: YYYY-MM-DD
**状态**: [已解决 / 进行中 / 待验证]

---

## 1. 背景 (Background)
简要说明问题出现的场景和上下文

## 2. 问题描述 (Problem)
详细描述遇到的问题，包括错误信息、异常现象等

## 3. 原因分析 (Root Cause)
分析问题的根本原因

## 4. 处理方案 (Solution)
详细说明采用的解决方案和实施步骤

## 5. 验证结果 (Verification)
说明如何验证问题已解决

## 6. 经验总结 (Lessons Learned)
提炼可复用的经验和最佳实践

## 7. 关联资源 (References)
- 相关 Commit: `<commit-hash>`
- 相关文档: [链接]
- 参考资料: [链接]
```

### 添加步骤

1. 判断是技术开发日志还是Bug修复日志
2. 选择对应的功能模块文件夹
3. 使用上述模板创建`.md`文件
4. 在本`README.md`中添加索引链接
5. 提交并关联相关commit

---

## CSDN技术专栏归档说明

本日志包含从CSDN博客归档的UE5技术专栏文章：

- **专栏名称**: UE5
- **作者**: 氩氪氙氡_xuzhi (a2213086589)
- **专栏地址**: https://blog.csdn.net/a2213086589/category_12980554.html
- **归档日期**: 2025-12-17
- **文章总数**: 22篇

所有文章已按技术开发/Bug修复分类，并归入对应的功能模块文件夹中的`CSDN_Posts/`子目录。

**版权声明**: 所有文章内容版权归原作者所有。本归档仅用于个人学习和备份目的。

---

## 图片资源说明

仿真相关图片存放于 `TechnicalLogs/SimulinkDynamics/img/` 目录：

| 文件名 | 描述 | 用途 |
|--------|------|------|
| simulink_6dof_overview.png | 6DOF模块总体结构图 | GM_Negative_Natural_Frequency_NaN_Issue.md |
| natural_frequency_module.png | 自然频率计算模块图 | GM_Negative_Natural_Frequency_NaN_Issue.md |
| simulink_pure_test_*.png | Simulink纯环境测试 | Simulink_UE5_Performance_Analysis.md |
| realtime_test_stopwatch_*.png | 实时测试计时 | Simulink_UE5_Performance_Analysis.md |

---

## 统计信息

| 分类 | 文档数量 | 最后更新 |
|------|---------|---------|
| TechnicalLogs/MultiMachineComm | 2 | 2026-01-14 |
| TechnicalLogs/UE5_3DVisualization | 16 | 2025-12-16 |
| TechnicalLogs/FlightControl | 1 | 2025-12-21 |
| TechnicalLogs/SimulinkDynamics | 7 | 2025-12-21 |
| TechnicalLogs/LinuxSystem | 2 | 2026-01-12 |
| BugFixLogs/UE5_3DVisualization | 3 | 2025-12-16 |
| BugFixLogs/SimulinkDynamics | 7 | 2026-01-04 |

---

## 相关文档

- [项目 README](../README.md)
- [系统架构文档](../docs/architecture.md)
- [使用指南](../docs/usage.md)

---

**维护者**: 项目团队
**更新频率**: 随开发进度更新
