# 开发日志索引 (DevLogs Index)

**Tags:** `devlogs`, `index`, `development-history`, `engineering-notes`

**最后更新**: 2025-12-16

---

## 📋 简介

本目录记录项目开发过程中遇到的问题、解决方案、技术决策以及经验总结。按功能模块分类，便于后续检索和知识传承。

---

## 📂 目录结构

```
DevLogs/
├── README.md                    # 本文件：索引与导航
├── 01_Integration/              # 系统集成相关
│   └── Level_Migration.md       # 关卡迁移与版本兼容
├── 02_Blueprint_Issues/         # UE 蓝图问题
│   └── BP_Compile_Errors.md     # 蓝图编译错误排查
├── 03_ROS2/                     # ROS2 集成与调试
│   └── Node_Integration.md      # 节点集成记录
├── 04_Simulation/               # 仿真系统开发
│   └── UE5_Simulink_Interface.md  # UE5 与 Simulink 接口
└── 99_Notes/                    # 其他技术笔记
```

---

## 🗂️ 分类说明

### 01_Integration - 系统集成

**适用场景**:
- 跨版本迁移问题
- 多模块集成调试
- 依赖库版本冲突
- 系统架构调整

**当前日志**:
- [Level_Migration.md](01_Integration/Level_Migration.md) - UE 关卡迁移与资产更新

---

### 02_Blueprint_Issues - 蓝图编译问题

**适用场景**:
- UE 蓝图编译错误
- 节点连接异常
- 蓝图性能优化
- 蓝图重构记录

**当前日志**:
- [BP_Compile_Errors.md](02_Blueprint_Issues/BP_Compile_Errors.md) - 常见蓝图编译错误与修复

---

### 03_ROS2 - ROS2 节点集成

**适用场景**:
- ROS2 节点开发
- 话题通信调试
- 服务与动作集成
- 参数配置问题

**当前日志**:
- [Node_Integration.md](03_ROS2/Node_Integration.md) - ROS2 节点集成过程与问题

---

### 04_Simulation - 仿真系统

**适用场景**:
- UE5 仿真环境搭建
- Simulink 接口开发
- 物理引擎调试
- 传感器仿真

**当前日志**:
- [UE5_Simulink_Interface.md](04_Simulation/UE5_Simulink_Interface.md) - UE5 与 Simulink 通信接口

---

### 99_Notes - 其他技术笔记

**适用场景**:
- 临时技术探索
- 工具使用技巧
- 性能测试记录
- 未分类的技术笔记

**当前日志**:
- 待添加

---

## 🔍 按标签检索

### 控制算法
- `#PID` - PID 控制相关
- `#MFAC` - 无模型自适应控制
- `#Splane` - S-plane 控制

### 系统模块
- `#ROS2` - ROS2 相关问题
- `#UE5` - Unreal Engine 5
- `#Simulink` - MATLAB/Simulink
- `#Navigation` - 导航与制导

### 问题类型
- `#Bug` - Bug 修复记录
- `#Performance` - 性能优化
- `#Refactor` - 重构记录
- `#Integration` - 集成问题

---

## 📝 如何添加新日志

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

1. 在对应分类目录下创建 `.md` 文件
2. 使用上述模板填写内容
3. 在本 `README.md` 中添加索引链接
4. 提交并关联相关 commit

---

## 📊 统计信息

| 分类 | 日志数量 | 最后更新 |
|------|---------|---------|
| 01_Integration | 1 | 2025-12-16 |
| 02_Blueprint_Issues | 1 | 2025-12-16 |
| 03_ROS2 | 1 | 2025-12-16 |
| 04_Simulation | 1 | 2025-12-16 |
| 99_Notes | 0 | - |

---

## 🔗 相关文档

- [项目 README](../README.md)
- [系统架构文档](../docs/architecture.md)
- [使用指南](../docs/usage.md)

---

**维护者**: 项目团队
**更新频率**: 随开发进度更新
