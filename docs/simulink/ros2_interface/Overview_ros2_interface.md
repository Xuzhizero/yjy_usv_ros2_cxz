# ROS2 通信接口模块

## 模块概述

ROS2通信接口负责Simulink模型与ROS2系统之间的双向数据交换，包括接收外部控制命令和发布仿真状态数据。

## 子模块列表

| 子模块 | SID | 功能 | 内部模块数 |
|--------|-----|------|-----------|
| [ROS2simulink](ROS2simulink.md) | 10 | ROS2输入处理（主） | 18 |
| [ROS2simulink1](ROS2simulink1.md) | 813 | ROS2输入处理（辅助） | 8 |
| [Simulink2ROS](Simulink2ROS.md) | 841 | Simulink到ROS2输出 | 13 |

## 数据流架构

```
                    ┌─────────────────┐
  ROS2 Topics ───→  │  ROS2simulink   │ ───→ psi, n, signal3 ───→ simulink2UE5
                    │    (SID:10)     │
                    └─────────────────┘

                    ┌─────────────────┐
  ROS2 Topics ───→  │  ROS2simulink1  │ ───→ 辅助数据 ───→ Bus Creator1
                    │    (SID:813)    │
                    └─────────────────┘

                    ┌─────────────────┐
  Yaw, x, y,    ───→│  Simulink2ROS   │ ───→ ROS2 Topics
  speed, YawRate    │    (SID:841)    │
                    └─────────────────┘
```

## 与其他模块的连接关系

### 输入接口
- 从ROS2系统接收控制命令和传感器数据
- ROS2simulink的输出连接到simulink2UE5进行可视化
- ROS2simulink1的输出连接到Bus Creator1进行总线组合

### 输出接口
- Simulink2ROS接收来自Bus Selector的5个状态信号：
  1. Yaw Angle (deg) - 偏航角
  2. x (m) - X位置
  3. y (m) - Y位置
  4. speed (m/s) - 速度
  5. Yaw Rate (rad/s) - 偏航率

## ROS2消息类型

> **待补充**: 需要提供ROS2simulink和Simulink2ROS使用的具体ROS2消息类型定义

## 文档状态

| 文档 | 状态 | 说明 |
|------|------|------|
| ROS2simulink.md | 🟡 待完善 | 需要内部模块截图和消息格式说明 |
| ROS2simulink1.md | 🟡 待完善 | 需要内部模块截图和消息格式说明 |
| Simulink2ROS.md | 🟡 待完善 | 需要内部模块截图和消息格式说明 |
