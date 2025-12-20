# Vessel Platform 船舶平台模块

## 模块概述

Vessel Platform是船舶动力学仿真的核心模块，包含完整的船舶运动学和动力学模型。该模块接收控制命令和环境参数，输出船舶的六自由度运动状态。

## 子模块导航

| 子模块 | 说明 | 文档链接 |
|--------|------|----------|
| Input Processing | 输入处理和参数预处理 | [input_processing/](input_processing/) |
| Thrust | 推进器推力计算 | [thrust/](thrust/) |
| Waves and Crossflow | 波浪和横流力计算 | [waves_and_crossflow/](waves_and_crossflow/) |
| Hydrodynamics | 水动力计算(阻尼、科氏力等) | [hydrodynamics/](hydrodynamics/) |
| Hydrostatic | 静水恢复力计算 | [hydrostatic/](hydrostatic/) |
| Sum of Forces and Moments | 力和力矩求和 | [sum_of_forces_and_moments/](sum_of_forces_and_moments/) |
| 6DOF | 六自由度运动积分 | [6dof/](6dof/) |
| Output Processing | 输出处理和状态整形 | [output_processing/](output_processing/) |

## 模块架构图

<div align="center">

![ASV Simulink Model](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752652888962-6ac08684-85fc-48bd-9961-e0d1a09f51fe.png)

</div>

## 数据流概览

```
                    ┌─────────────────┐
VesselCommand ─────→│                 │
Payload ───────────→│ Input Processing│──→ νr, Inertials, H, MA, G, ω, T
Environment ───────→│                 │
State [X] ─────────→└─────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ↓                     ↓                     ↓
   ┌─────────┐        ┌────────────────┐    ┌──────────────┐
   │ Thrust  │        │ Waves and      │    │ Hydrodynamics│
   │         │        │ Crossflow      │    │              │
   └────┬────┘        └───────┬────────┘    └──────┬───────┘
        │                     │                    │
        │                     │                    │
        │    ┌────────────────┼────────────────────┤
        │    │                │                    │
        │    │         ┌──────┴──────┐            │
        │    │         │ Hydrostatic │            │
        │    │         └──────┬──────┘            │
        │    │                │                   │
        ↓    ↓                ↓                   ↓
   ┌──────────────────────────────────────────────────┐
   │          Sum of Forces and Moments               │
   └─────────────────────────┬────────────────────────┘
                             ↓
                    ┌─────────────────┐
                    │   6DOF (In CO)  │
                    └────────┬────────┘
                             ↓
                    ┌─────────────────┐
                    │Output Processing│──→ VesselStates
                    └─────────────────┘
```

## 各模块输入输出说明

下面按信号流从左到右，逐块说明各模块的输入含义。

---

### 1. 输入处理（Input Processing）

| 输入端口 | 物理意义 |
| --- | --- |
| **Payload Mass (kg)** | 载荷质量，用来修正总体质量矩阵（惯性） |
| **Payload Location (m)** | 载荷位置（重心偏移），用来修正船体重心坐标 |
| **Environment.Earth.Gravity (m/s²)** | 重力加速度，用于浮力、静水恢复力等计算 |
| **Environment.Water.Water Density (kg/m³)** | 水密度，用于计算浮力、附加质量、阻尼等水动力参数 |
| **Environment.Current.Current speed (m/s)** | 环境水流速度，用于得到船相对水体的"相对速度" |
| **Environment.Current.Current direction (rad)** | 水流方向，用来分解流速向量 |
| **$[X]$** | 船体当前状态向量（位置×姿态×速度×角速度），用于内部参数预处理 |

> **输出**：
>
> + **$\nu_r$**：相对速度（船速度–水流速度）
> + **$g$**：重力向量
> + **Inertials**：船体质量和惯性矩阵
> + **$H$**：水动力阻尼矩阵
> + **$MA$**：附加质量矩阵
> + **$G$**：静水刚度矩阵
> + **$[\omega_3, \omega_4, \omega_5]$**：竖向、横摇、纵倾三自由度的固有频率
> + **$T$**：吃水深度

---

### 2. 推力（Thrust）

| 输入端口 | 物理意义 |
| --- | --- |
| **VesselCommand** | 推进器控制命令（螺旋桨转速或推力指令） |
| **Environment.Earth.Gravity (m/s²)** | 重力加速度，用于将推力向量投影到船体坐标 |

> **输出**：各螺旋桨／水喷射器产生的推力和相关力矩。

---

### 3. 波浪及横流（Waves and Crossflow）

| 输入端口 | 物理意义 |
| --- | --- |
| **$\nu_r$** | 相对速度（船体速度–水流速度） |
| **$T$** | 吃水深度 |
| **Environment.Water.Water Density (kg/m³)** | 水密度 |

> **输出**：波浪和横流作用下的激励力及力矩。

---

### 4. 水动力（Hydrodynamics）

| 输入端口 | 物理意义 |
| --- | --- |
| **$\nu_r$** | 相对速度 |
| **$H$** | 水动力阻尼矩阵 |
| **$g$** | 重力向量 |
| **$X$** | 重心到几何中心／浮心的偏移向量 |
| **$MA$** | 附加质量矩阵 |
| **Inertials** | 质量与惯性矩阵 |
| **$[\omega_3, \omega_4, \omega_5]$** | 第3、4、5自由度（heave、roll、pitch）的固有频率 |
| **Environment.Earth.Gravity (m/s²)** | 重力加速度 |

> **输出**：依据阻尼、附加质量、惯性等计算得到的水动力阻抗和附加力。

---

### 5. 静水（Hydrostatic）

| 输入端口 | 物理意义 |
| --- | --- |
| **$X$** | 重心到浮心的偏移向量 |
| **$G$** | 静水刚度矩阵（恢复力刚度） |

> **输出**：静水恢复力和恢复力矩（使船体复位的弹性力）。

---

### 6. 力和力矩求和（Sum of Forces and Moments）

| 输入端口 | 物理意义 |
| --- | --- |
| 推力、波浪横流力、水动力、静水力等各分量 | 各类力及力矩分量 |

> **输出**：合成后的总力和总力矩向量。

---

### 7. 6 自由度运动学（6DOF (In CO)）

| 输入端口 | 物理意义 |
| --- | --- |
| **Inertials** | 质量与惯性矩阵 |
| **Sum of Forces and Moments** | 总力和总力矩 |
| **Initial Conditions** | 初始位置、姿态、速度、角速度（用于积分起点） |

> **输出**：船体六自由度状态向量 **$[X]$**（位置、姿态、线速度、角速度）。

---

### 8. 输出处理（Output Processing）

| 输入端口 | 物理意义 |
| --- | --- |
| **$[X]$** | 来自 6DOF 的完整状态向量 |
| **Initial Condition1** | （可选）用于状态复位或后处理的初始条件 |

> **输出**：清洗／整形后的 **VesselStates**，供上层控制或显示使用。

---

## 总结

以上即是该 Simulink 子系统中，各模块输入信号的物理含义说明。如需进一步了解内部算法（例如 Thrust 模型如何根据命令计算推力、Hydrodynamics 模型的具体方程），可以针对单个模块再做深入分析。

---

**文档说明**：
- 本文档详细描述了ASV船体Simulink模型中各个模块的输入含义
- 信号流按从左到右的顺序进行说明
- 包含8个主要模块：输入处理、推力、波浪及横流、水动力、静水、力和力矩求和、6自由度运动学、输出处理
