# Input Processing 模块

## 提问

<div align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1752652674586-280f42c5-5232-4628-aa6d-1b4f471bb85c.png" alt="Input Processing 模块图">
</div>

进一步地，这个图是输入处理(Input Processing)模块内部的simulink构造，要求你分析一下图中每一个模块功能以及各自的输入输出的含义

## 回答

下面按功能模块分组，逐一说明各子模块的作用及其主要输入／输出含义。

---

### 1. 相对速度处理（Relative Velocity）

```
[5] Current Speed (m/s) ─┐
[6] Current Direction (rad) ─┐
[7] X (状态向量) ────────────┐
                         ┌───────────────┐
                         │ Relative      │───► [1] Relative Velocity (ν_r)
                         │  Velocity     │
                         └───────────────┘
```

+ **Current Speed、Current Direction**：环境水流的大小和方向。
+ **X**：船体当前六自由度状态向量（包括线速度 u,v,w）。
+ **Relative Velocity 子系统**：先将(水流速+方向)变换到船体坐标系，得到水体速度向量 $V_c$，再用船体速度 $\nu$ 减去 $V_c$，计算出相对水体的速度 $\nu_r$。
+ **输出** $\nu_r$：后续所有水动力、波浪、推力都基于它来计算。

---

### 2. 排水量与重心修正（Displacement & CG）

```
[1] Payload Mass (kg) ────┐      ┌─────────────┐
                          │      │ Volume      │───► [Volume (m³)]
[4] Water Density (kg/m³) ─┘      └─────────────┘

[1] Payload Mass (kg) ────┐      ┌───────────────────┐
                          │      │ CG corrected for  │───► [r_g] （修正后重心位置，m）
[2] Payload Location (m) ─┘      └───────────────────┘
```

1. **Volume**：
    - $m_p$（Payload Mass）＋模块内预设的船体质量，共同除以 $\rho$，得到排水体积 V。
    - **输出 V**：用于浮力、刚度、吃水计算。

2. **CG corrected for payload**：
    - 根据载荷质量与其在船体纵向（surge）方向上的位置 $t_p$，修正整体重心位置：

$$
r_g = \frac{m_h r_{g_h} + m_p t_p}{m_h + m_p}
$$

+ **输出** $r_g$：后续惯性矩阵和静水矩阵都要用到重心位置。

---

### 3. 从 CG 到参考原点（CO）的坐标变换

```
[r_g] ────────────┐
                  │      ┌───────────────────┐
[H]───────────────┘      │ Transform CG to CO │───► [5] r (CO 坐标系下的 CG)
                         └───────────────────┘
```

+ **Transform CG to CO**：
    - 输入修正后重心向量 $r_g$，以及内部预设的变换矩阵 **H**（从 CG 点到坐标原点 CO 的平移/旋转），
    - 输出在 CO 坐标系下的重心位置 **r**。

---

### 4. 刚体质量矩阵与水动力附加质量

```
[1] Payload Mass ─┐
                 │      ┌──────────────┐       ┌─────────────────┐
                 └───►  │   MRB_CG     │───►Aᵀ│ Matrix Multiply │──► [M]  ─► 再输出到 Inertials
                        └──────────────┘       └─────────────────┘
                           rigid‐body           (Aᵀ·MRB_CG·A)
                            inertia at CG              ↳ [4] total mass m_p

[2] Payload Location ─┐
[1] Payload Mass ─────┼────────────────────────────────────────────────►┐
                      │                                                 │
                      │   ┌───────────────────────────────────────────┐ │
                      └──►│ Inertial Matrix (hull+payload) in CO      │ │──► [3] Inertial Matrix (hull+payload) in CO
                          ├───────────────────────────────────────────┤ │
                          │ Added Mass (Hydrodynamics)                │ │──► [6] Added Mass Matrix MA
                          └───────────────────────────────────────────┘ │
                                                                    └─►► 供后续水动力、恢复力等使用
```

1. **MRB_CG**
    - 根据船体固有质心处的惯性矩（预设）与载荷质量 $m_p$，构建 6×6 刚体惯性矩阵 $M_{RB\_CG}$。

2. **Matrix Multiply** $(A^T \cdot M_{RB\_CG} \cdot A)$
    - 用第 3 步得到的平移／旋转矩阵 A，将惯性矩阵从 CG 点移到坐标原点 CO，得到刚体质量惯性矩阵 **M**。
    - 同时输出总质量 $m_p$ （供 Inertials 汇总）。

3. **Inertial Matrix (hull+payload) in CO & Added Mass**
    - 在同一子系统中，
        * **Inertial Matrix…**：也可选用简化公式直接根据载荷位置和质量算出的刚体惯性矩阵。
        * **Added Mass (Hydrodynamics)**：基于船体流线型、水密度及水动力特性，给出水动力附加质量矩阵 $M_A$。
    - **输出**：惯性矩阵（CO） [#3] 和附加质量矩阵 $M_A$ [#6]，供六自由度动力学方程使用。

---

### 5. 静水恢复力（Spring Stiffness）在 CO

```
[LCF]      ┐
[  0 ]     ├─► Transform CF to CO ──► [H2]
[r (CF)]   ┘                         └─► 与下游矩阵相乘

[4] Water Density (kg/m³) ┐
[3] Gravity (m/s²)        ├──►┐
[ T ] Draft (m)           ├──►│
[ V ] Volume (m³)         ├──►│   ┌────────┐   ┌───────────────┐
[ r_g ] CG (m)            └──►│   │  G_CF  │──▶│ Matrix Multiply│──► [7] G (in CO)
                                └──▶│ (in CF)│   └───────────────┘
                                   └────────┘
```

1. **Transform CF to CO**
    - 将浮力中心（CF）坐标系下的恢复矩阵变换到 CO 系。

2. $G_{CF}$ **(Spring Stiffness in CF)**
    - 基于 $\rho g V$ (浮力)、重心位置 $r_g$ 与吃水 T，计算船体在俯仰、横摇、纵荡的静水刚度：
        * $G_{33}$（heave）、$G_{44}$（roll）、$G_{55}$（pitch）分量。
    - 输出对角矩阵 $G_{CF}$。

3. **Matrix Multiply**
    - 用 H2 将 $G_{CF}$ 从 CF 转到 CO，得到最终的**静水刚度矩阵 G** [#7]。

---

### 6. 自振频率 & 吃水（Natural Frequencies & Draft）

```
[G33] ┐
[G44] ├─► Natural Frequencies ─► [8] ω₃, ω₄, ω₅
[G55] ├                        （heave/roll/pitch）
[ M ] ┘

[V] Volume (m³) ──► Draft ─► [9] T (m)
```

1. **Natural Frequencies**
    - 输入各维度的刚度（$G_{33}$/$G_{44}$/$G_{55}$）与对应惯性（M 或 $M_A$），
    - 计算三大模态的固有角频率 $\omega = \sqrt{G / M}$。

2. **Draft**
    - 根据排水体积 V 和已知水线面几何（模块内部），反算吃水深度 T。

---

这样，Input Processing 子系统就将环境参数、载荷质量／位置、船体几何与物理特性，依次转化为仿真所需的：

+ **相对速度** $\nu_r$
+ **排水体积** V
+ **修正重心** $r_g$
+ **刚体惯性矩阵** M、附加质量矩阵 $M_A$
+ **静水刚度矩阵** G
+ **固有频率** $\omega_3, \omega_4, \omega_5$
+ **吃水** T

——供后续 Thrust、Hydrodynamics、Hydrostatic、6DOF 等子系统直接使用。

---

## Simulink 变量映射表

以下是 Input Processing 模块中从 Simulink 模型参数 CSV 提取的关键变量映射：

### 输入端口映射

| 端口号 | 端口名称 | Simulink变量名 | 物理含义 |
|-------|---------|---------------|---------|
| 1 | Payload Mass (kg) | mp | 载荷质量 |
| 2 | Payload Location (m) | rp | 载荷位置向量 |
| 3 | Gravity (m/s²) | g | 重力加速度 |
| 4 | Water Density (kg/m³) | rho | 水密度 |
| 5 | Current Speed (m/s) | V_c | 水流速度 |
| 6 | Current Direction (rad) | beta_c | 水流方向 |
| 7 | X | X | 六自由度状态向量 |

### 附加质量系数（Added Mass Hydrodynamics）

| 增益模块 | 增益值 | 输出变量 | 物理含义 |
|---------|-------|---------|---------|
| Gain | -0.1 | Xudot | 纵荡附加质量系数 $X_{\dot{u}}$ |
| Gain2 | -1.5 | Yvdot | 横荡附加质量系数 $Y_{\dot{v}}$ |
| Gain3 | -1.0 | Zwdot | 垂荡附加质量系数 $Z_{\dot{w}}$ |
| Gain4 | -0.2 | Kpdot | 横摇附加惯量系数 $K_{\dot{p}}$ |
| Gain5 | -0.8 | Mqdot | 纵摇附加惯量系数 $M_{\dot{q}}$ |
| Gain6 | -1.7 | Nrdot | 艏摇附加惯量系数 $N_{\dot{r}}$ |

**说明**：附加质量矩阵 $M_A$ 为对角矩阵，各系数乘以船体质量 m 后构成对角元素：
$$M_A = \text{diag}(-X_{\dot{u}} \cdot m, -Y_{\dot{v}} \cdot m, -Z_{\dot{w}} \cdot m, -K_{\dot{p}} \cdot I_{xx}, -M_{\dot{q}} \cdot I_{yy}, -N_{\dot{r}} \cdot I_{zz})$$

### 内部常量参数

| 模块路径 | 变量名 | 说明 |
|---------|-------|------|
| mass (kg) | m | 船体质量（工作区变量） |
| CG hull only (m) | rg_hull | 船体重心位置 |
| Inertial Matrix in CG | I_CG | 船体在CG处的惯性张量 |

### 输出信号映射

| 输出端口 | 信号名 | 物理含义 |
|---------|-------|---------|
| Add5 | M in CO | 总惯性矩阵（CO坐标系） |
| Added Mass (Hydrodynamics) | MA | 附加质量矩阵 |
| G_CF | G_CF | 浮心处弹簧刚度矩阵 |
| Transform CG to CO | H | CG到CO变换矩阵 |
| Transform CF to CO | H2 | CF到CO变换矩阵 |

---

## Simulink 模型导出文件

为了完整保存 Input Processing 模块的结构信息，已将该模块解析为两个 CSV 表格文件，可用于文档化、版本控制和模型重建。

### 1. 参数表（Parameter Table）

**文件**: [`input_processing_parameters.csv`](./input_processing_parameters.csv)

**用途**: 记录模块内所有块（Block）的完整配置参数

**表结构**:

| 列名 | 说明 | 示例 |
|-----|------|------|
| BlockPath | 块的完整 Simulink 路径 | `ROS2_simulink_UE5_0821/Subsystem/Vessel Platform/Input Processing/Payload Mass (kg)` |
| BlockType | 块类型 | `Inport`, `Constant`, `Gain`, `Subsystem` 等 |
| ParamContext | 参数上下文路径 | `Block`, `PortProperties/Port[out:1]` |
| ParamName | 参数名称 | `Position`, `Port`, `Value`, `Gain` 等 |
| ParamValue | 参数值 | `"[110, 123, 140, 137]"`, `mp`, `0.1` 等 |

**典型应用**:
- 查找特定块的所有配置参数（如增益值、常量值、位置坐标）
- 提取模型中使用的工作区变量名称
- 重建块的属性设置（端口名、信号传播名等）
- 比对不同版本模型的参数变化

**使用示例**:
```bash
# 查找所有 Gain 块的增益值
grep ",Gain," input_processing_parameters.csv | grep "ParamName,Gain"

# 查找特定块的所有参数
grep "Added Mass (Hydrodynamics)/Gain," input_processing_parameters.csv
```

### 2. 连接表（Connection Table）

**文件**: [`input_processing_connections.csv`](./input_processing_connections.csv)

**用途**: 记录模块内所有信号线的连接关系

**表结构**:

| 列名 | 说明 | 示例 |
|-----|------|------|
| SourceBlockPath | 源块的完整路径 | `ROS2_simulink_UE5_0821/Subsystem/Vessel Platform/Input Processing/Added Mass (Hydrodynamics)/Gain1` |
| SourcePortType | 源端口类型 | `out` |
| SourcePortIndex | 源端口编号 | `1`, `2`, `3` 等 |
| DestBlockPath | 目标块的完整路径 | `ROS2_simulink_UE5_0821/Subsystem/Vessel Platform/Input Processing/Added Mass (Hydrodynamics)/Added Mass (Hydrodynamcis)` |
| DestPortType | 目标端口类型 | `in` |
| DestPortIndex | 目标端口编号 | `1`, `2`, `3` 等 |

**典型应用**:
- 追踪信号流向（某个块的输出连接到哪些块）
- 反向追溯信号来源（某个块的输入来自哪里）
- 绘制数据流图
- 分析模块间的依赖关系
- 验证模型连线的完整性

**使用示例**:
```bash
# 查找特定块的所有输出连接
grep "^ROS2_simulink_UE5_0821/Subsystem/Vessel Platform/Input Processing/Draft (m)/Divide7," input_processing_connections.csv

# 查找特定块的所有输入来源
grep "ROS2_simulink_UE5_0821/Subsystem/Vessel Platform/Input Processing/GM/Sum1,in," input_processing_connections.csv
```

### 3. 文件生成说明

这两个 CSV 文件是通过 MATLAB/Simulink API 自动解析生成的，包含了以下完整信息：

- **所有层级的子系统**：递归解析了 Input Processing 及其所有子模块
- **完整的块参数**：包括块类型、位置、端口属性、计算参数等
- **完整的连接关系**：记录了所有块之间的信号连线

**注意事项**:
- BlockPath 使用完整的 Simulink 路径，便于定位
- 端口索引从 1 开始计数（符合 MATLAB 习惯）
- 参数值中的数组用双引号包裹（如位置坐标）
- 部分参数可能为空（ParamContext 为空表示块级参数）

### 4. 与本文档的关系

本文档中的"Simulink 变量映射表"是从上述 CSV 文件中提取的关键信息摘要，方便快速查阅。完整的参数和连接信息请参考原始 CSV 文件。

---

