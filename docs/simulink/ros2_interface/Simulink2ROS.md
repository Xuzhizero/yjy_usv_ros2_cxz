# Simulink2ROS 模块 (SID:841)

## 模块概述

Simulink2ROS负责将Simulink仿真数据打包并发布到ROS2系统，实现仿真状态的实时输出。

## 基本信息

| 属性 | 值 |
|------|-----|
| SID | 841 |
| 内部模块数 | 13 |
| 内部连线数 | 9 |
| 输入端口数 | 5 |

## 内部模块构成

| 模块类型 | 数量 | 功能 |
|---------|------|------|
| Inport | 5 | 输入端口 |
| Reference | 4 | 库引用模块(ROS2发布器) |
| BusAssignment | 2 | 总线赋值 |
| Sin | 2 | 正弦信号生成 |

## 输入/输出

### 输入

| 端口 | 信号名 | 单位 | 来源 |
|------|--------|------|------|
| 1 | Yaw Angle | deg | Bus Selector out:1 |
| 2 | x | m | Bus Selector out:2 |
| 3 | y | m | Bus Selector out:3 |
| 4 | speed | m/s | (未连接或内部使用) |
| 5 | Yaw Rate | rad/s | Bus Selector out:5 |

### 输出
- 发布到ROS2 Topic的消息

## 数据流

```
Bus Selector ─┬─ [1] Yaw Angle (deg) ──→┐
              │                          │
              ├─ [2] x (m) ─────────────→├──→ Simulink2ROS ──→ ROS2 Topics
              │                          │
              ├─ [3] y (m) ─────────────→│
              │                          │
              └─ [5] Yaw Rate (rad/s) ──→┘
```

## Simulink构造

> **待补充**: 需要提供该模块的Simulink内部截图

## ROS2接口说明

> **待补充**: 需要提供以下信息：
> - 发布的ROS2 Topic名称
> - 消息类型定义（可能是geometry_msgs/Pose, nav_msgs/Odometry等）
> - 发布频率

## Sin模块说明

模块内部包含2个Sin模块，可能用于：
- 测试信号生成
- 周期性扰动添加
- 信号调制

> **待补充**: Sin模块的具体用途和参数设置
