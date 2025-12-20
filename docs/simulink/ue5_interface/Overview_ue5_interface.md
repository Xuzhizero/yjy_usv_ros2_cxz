# UE5 可视化接口模块

## 模块概述

UE5接口模块负责将Simulink仿真数据实时发送到虚幻引擎5(Unreal Engine 5)进行3D可视化渲染，使用户能够直观地观察船舶运动状态。

## 子模块列表

| 子模块 | SID | 功能 | 状态 | 内部模块数 |
|--------|-----|------|------|-----------|
| [simulink2UE5](simulink2UE5.md) | 20 | 主UE5通信接口 | 主用 | 23 |
| [simulink2UE1](simulink2UE1.md) | 751 | 备用UE5通信接口 | 备用 | 46 |

## 数据流架构

```
                              ┌─────────────────┐
ROS2simulink ─┬─ psi ───────→│                 │
              │               │   simulink2UE5  │───→ UE5 (3D渲染)
              ├─ n ──────────→│     (SID:20)    │
              │               │                 │
              └─ signal3 ────→└─────────────────┘
                                  ↑
                           MATLAB Functions
                           (数据处理)


备用接口 (simulink2UE1) 结构类似但更复杂
```

## 通信机制

> **待补充**: 需要说明Simulink与UE5之间的通信方式：
> - UDP/TCP Socket通信？
> - 共享内存？
> - ROS2桥接？
> - 其他中间件？

## 与UE5的数据接口

### 发送的数据类型

| 数据 | 说明 | 用途 |
|------|------|------|
| psi | 姿态角/偏航角 | 船舶朝向渲染 |
| n | 转速信息 | 螺旋桨动画 |
| signal3 | 第三信号 | 待确认 |

### 数据格式

> **待补充**: 需要提供发送到UE5的数据格式定义

## MATLAB Function子系统

两个UE5接口模块都包含MATLAB Function子系统用于数据处理：

### simulink2UE5
- MATLAB Function
- MATLAB Function1

### simulink2UE1
- MATLAB Function
- MATLAB Function1

> **待补充**: 需要提供MATLAB Function的代码和功能说明

## 模块对比

| 特性 | simulink2UE5 (主) | simulink2UE1 (备) |
|------|-------------------|-------------------|
| 内部模块数 | 23 | 46 |
| 连线数 | 13 | 29 |
| Constant模块 | 4 | 14 |
| 复杂度 | 较低 | 较高 |
| 使用状态 | 主用 | 备用 |

## 文档状态

| 文档 | 状态 | 说明 |
|------|------|------|
| simulink2UE5.md | 🟡 待完善 | 需要内部模块截图和MATLAB代码 |
| simulink2UE1.md | 🟡 待完善 | 需要内部模块截图和MATLAB代码 |
