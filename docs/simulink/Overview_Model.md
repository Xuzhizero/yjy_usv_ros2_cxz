# ROS2_simulink_UE5_0821 模型概述

## 模型简介

本Simulink模型是一个集成了ROS2通信、虚幻引擎5(UE5)可视化的船舶控制仿真系统，用于自主无人船(ASV)的动力学仿真与控制验证。

## 关键统计数据

| 项目 | 数量 |
|------|------|
| 总系统/子系统数 | 58 |
| 总模块数 | 889 |
| 总连线数 | 824 |
| 顶层子系统数 | 7 |

## 层级架构

```
ROS2_simulink_UE5_0821 (Root)
│
├── ROS2simulink (SID:10)          # ROS2输入处理
│   └── 18个模块, 9条连线
│
├── ROS2simulink1 (SID:813)        # 辅助ROS2输入处理
│   └── 8个模块, 5条连线
│
├── Scenario (SID:779)             # 场景控制和环境设置
│   └── 33个模块, 21条连线
│
├── Simulink2ROS (SID:841)         # Simulink到ROS2输出
│   └── 13个模块, 9条连线
│
├── Subsystem (SID:748)            # 船舶平台主系统 ★核心模块★
│   ├── Environment                 # 环境模型
│   ├── Payload                     # 有效载荷模型
│   └── Vessel Platform             # 船舶平台动力学
│       ├── Input Processing
│       ├── Thrust
│       ├── Waves and Crossflow
│       ├── Hydrodynamics
│       ├── Hydrostatic
│       ├── Sum of Forces and Moments
│       ├── 6DOF
│       └── Output Processing
│
├── simulink2UE1 (SID:751)         # UE5通信接口(备用)
│   └── 46个模块, 29条连线
│
└── simulink2UE5 (SID:20)          # UE5通信接口(主)
    └── 23个模块, 13条连线
```

## 数据流总览

### 1. 输入阶段
- **ROS2接口**: 接收外部控制命令和状态数据
- **常数模块**: 提供螺旋桨转速设定值 (左120rpm, 右121rpm)

### 2. 处理阶段
- **Bus Creator**: 组合控制命令为总线信号
- **Subsystem**: 执行船舶动力学计算
- **Scenario**: 管理仿真场景参数

### 3. 输出阶段
- **Bus Selector**: 提取关键状态量
  - Yaw Angle (deg) - 偏航角
  - x (m) - X位置
  - y (m) - Y位置
  - speed (m/s) - 速度
  - Yaw Rate (rad/s) - 偏航率
- **Simulink2ROS**: 发布数据到ROS2
- **simulink2UE5**: 发送可视化数据到虚幻引擎5
- **Scope**: 实时监控关键参数

## 主要连线关系

```
ROS2simulink ─┬─ out:1 ──→ simulink2UE5 in:1 (psi角度)
              ├─ out:2 ──→ simulink2UE5 in:2 (转速n)
              └─ out:3 ──→ simulink2UE5 in:3

PropellerCommand_L_rpm ──→ Bus Creator2 ──→ Subsystem ──→ Scenario
PropellerCommand_R_rpm ──┘

Scenario ──→ Bus Selector ─┬─ Yaw Angle ──→ Simulink2ROS
                           ├─ x ──────────→ Simulink2ROS
                           ├─ y ──────────→ Simulink2ROS
                           ├─ speed ──────→ Scope3
                           └─ Yaw Rate ───→ Simulink2ROS
```

## 系统关键特性

| 特性 | 说明 |
|------|------|
| 双向ROS2通信 | 既能接收也能发送ROS2消息 |
| UE5集成 | 实时3D可视化仿真结果 |
| 模块化设计 | 清晰的功能分离和层级结构 |
| 备份接口 | 包含备用的UE通信接口(simulink2UE1) |
| 完整船舶模型 | 包含环境、载荷和平台动力学 |
| 实时监控 | 多个Scope和Display用于调试 |

## 文档结构导航

| 目录 | 说明 |
|------|------|
| [ros2_interface/](ros2_interface/) | ROS2通信接口模块文档 |
| [ue5_interface/](ue5_interface/) | UE5可视化接口模块文档 |
| [scenario/](scenario/) | 场景控制模块文档 |
| [environment/](environment/) | 环境模型文档 |
| [payload/](payload/) | 载荷模型文档 |
| [vessel_platform/](vessel_platform/) | 船舶平台动力学文档 |

## 相关文档

- [坐标系统定义](坐标系统定义.md) - 模型使用的坐标系定义
- [Simulink模型参数解析](Simulink模型参数解析.md) - 模型参数详解
- [船体规格参数](船体规格参数.md) - 船体尺寸、吨位和推力曲线
- [仿真变量参考](仿真变量参考.md) - 仿真模型变量及导出方法
- [船体水动力参数获取方法](船体水动力参数获取方法.md) - 水动力参数说明
- [Inertials变量来源说明](Inertials变量来源说明.md) - 惯性参数说明
