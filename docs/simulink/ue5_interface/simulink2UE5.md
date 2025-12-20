# simulink2UE5 模块 (SID:20)

## 模块概述

simulink2UE5是主要的虚幻引擎5通信接口，负责将仿真数据实时发送到UE5进行3D可视化渲染。

## 基本信息

| 属性 | 值 |
|------|-----|
| SID | 20 |
| 内部模块数 | 23 |
| 内部连线数 | 13 |
| 输入端口数 | 3 |
| 嵌套子系统数 | 2 |

## 内部模块构成

| 模块类型 | 数量 | 功能 |
|---------|------|------|
| Display | 6 | 数值显示/调试 |
| Reference | 5 | 库引用模块 |
| Constant | 4 | 常数配置 |
| Inport | 3 | 输入端口 |
| DataTypeConversion | 3 | 数据类型转换 |
| SubSystem | 2 | MATLAB Function子系统 |

## 输入/输出

### 输入

| 端口 | 信号名 | 来源 | 说明 |
|------|--------|------|------|
| 1 | psi | ROS2simulink out:1 | 姿态角/偏航角 |
| 2 | n | ROS2simulink out:2 | 转速信息 |
| 3 | signal3 | ROS2simulink out:3 | 第三信号 |

### 输出
- 通过通信接口发送到UE5

## 二级子系统

### MATLAB Function
用于处理输入数据，进行必要的坐标变换或数据格式转换。

> **待补充**: MATLAB Function代码

### MATLAB Function1
辅助处理函数。

> **待补充**: MATLAB Function1代码

## Simulink构造

> **待补充**: 需要提供该模块的Simulink内部截图

## 数据处理流程

```
输入信号 → DataTypeConversion → MATLAB Function → Reference(通信) → UE5
    ↑              ↑                   ↑
    └── Inport     └── 类型转换        └── 数据处理
```

## 与UE5的接口

> **待补充**: 需要提供以下信息：
> - 通信协议（UDP/TCP/共享内存等）
> - 端口配置
> - 数据包格式
> - 刷新频率

## Constant配置

模块包含4个Constant模块，可能用于：
- 通信参数配置
- 坐标偏移设置
- 缩放因子
- 其他固定参数

> **待补充**: 各Constant的具体值和含义
