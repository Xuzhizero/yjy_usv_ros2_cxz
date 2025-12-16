# YJY USV ROS2 CXZ Project

**Tags:** `ros2`, `usv`, `unmanned-surface-vehicle`, `simulation`, `control`, `navigation`

---

## 项目简介

本项目是一个基于 ROS2 的无人水面艇（USV）控制与仿真系统，集成了多种控制算法、导航规划以及仿真接口。

### 主要特性

- **多种控制算法**：PID、MFAC（无模型自适应控制）、S-plane 控制等
- **导航与制导**：LOS 制导、路径跟踪、目标跟随
- **推力分配**：双推进器推力分配算法
- **仿真支持**：集成 Unreal Engine 5 和 Simulink 仿真接口
- **ROS2 架构**：基于 ROS2 Humble/Foxy 的分布式系统设计

---

## 快速开始

### 环境要求

- ROS2 Humble/Foxy
- Python 3.8+
- Ubuntu 20.04/22.04

### 构建项目

```bash
# 克隆仓库
git clone <repository-url>
cd yjy_usv_ros2_cxz

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建工作空间
colcon build

# 加载环境
source install/setup.bash
```

### 运行仿真

```bash
# 启动 USV 仿真
ros2 launch control_planner launch_usvSim.launch.py
```

---

## 文档导航

### 📚 核心文档

| 文档 | 描述 |
|------|------|
| [项目概览](docs/overview.md) | 项目整体架构和功能概述 |
| [系统架构](docs/architecture.md) | 系统架构设计与模块说明 |
| [使用指南](docs/usage.md) | 详细的使用说明和 API 参考 |

### 📝 开发日志

| 分类 | 描述 |
|------|------|
| [DevLogs 索引](DevLogs/README.md) | 所有开发日志的入口 |
| [01_Integration](DevLogs/01_Integration/) | 系统集成相关问题 |
| [02_Blueprint_Issues](DevLogs/02_Blueprint_Issues/) | UE 蓝图编译问题 |
| [03_ROS2](DevLogs/03_ROS2/) | ROS2 节点集成记录 |
| [04_Simulation](DevLogs/04_Simulation/) | 仿真接口开发日志 |
| [99_Notes](DevLogs/99_Notes/) | 其他技术笔记 |

---

## 项目结构

```
yjy_usv_ros2_cxz/
├── README.md              # 本文件
├── docs/                  # 稳定文档（面向评审/新人）
├── DevLogs/               # 开发日志（面向工程过程）
├── ReadMe/                # 早期信息文档
└── src/                   # ROS2 源代码
    ├── control_planner/   # 控制与规划包
    └── pid_interfaces/    # 自定义消息接口
```

---

## 贡献指南

欢迎贡献！请参考以下流程：

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

---

## 许可证

待定

---

## 联系方式

- 项目维护者：[待补充]
- 问题反馈：[GitHub Issues]

---

**最后更新**: 2025-12-16
