# Input Processing 模块与弹簧刚度矩阵分析

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

## 继续追问

<div align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1755158950651-c9ce3024-2dc6-4058-a877-2cf7588a8591.png" alt="Simulink接线图">
</div>

这是Simulink的接线，它是无人水面艇的动力学Simulink模型的一部分。分析一下框出来的这个输出的表达式

## 回答

根据这个Simulink模型图，框出来的输出"Spring Stiffness Matrix in CO"（标记为7）的表达式可以分析如下：

### 表达式推导

$$G_{CO} = A^T \times G_{CF}$$

其中：

#### 1. 坐标变换矩阵 A

+ 由"Transform CF to CO"模块生成
+ 输入：LCF（局部坐标系参数）和0
+ 输出：变换矩阵A，然后取转置 $A^T$

#### 2. CF坐标系下的弹簧刚度矩阵 $G_{CF}$

$G_{CF}$ 由以下参数构成：

+ **rho**: 水密度 (kg/m³)
+ **g**: 重力加速度 (m/s²)
+ **nabla**: 排水体积 (m³)
+ **GM_T**: 横向稳心高度 (m)
+ **GM_L**: 纵向稳心高度 (m)
+ **G33, G44, G55**: 特定的刚度系数

#### 3. 物理意义

这个矩阵表示无人水面艇在CO坐标系（可能是原点坐标系或船体固定坐标系）下的**静水回复力刚度矩阵**，用于描述：

+ 垂荡（heave）回复力
+ 横摇（roll）回复力矩
+ 纵摇（pitch）回复力矩

#### 4. 数学形式

最终的6×6刚度矩阵大致形式为：

$$G_{CO} = A^T \times \text{diag}[0, 0, \rho g \nabla, \rho g \nabla GM_T, \rho g \nabla GM_L, 0] \times A$$

这种坐标变换确保了刚度矩阵能正确反映船体在目标坐标系下的静水回复特性。

---

## 继续提问

<div align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1755158987176-306ad79e-1e5e-4421-ab62-8bdbc9b37150.png" alt="G_CF构建图">
</div>

再通过我上传的新图，分析一下 $G_{CF}$ 的表达式，以及如何通过Simulink实现的

## 回答

根据这个新的Simulink图，可以清楚地看到 $G_{CF}$（船体坐标系下的弹簧刚度矩阵）的构建过程：

### $G_{CF}$ 表达式分析

#### 1. 计算过程

从图中可以看到以下计算步骤：

**基础浮力项：**

$$\rho g \nabla = \rho \times g \times \nabla$$

**刚度矩阵对角元素：**

+ $G_{33} = \rho g \nabla \times 2 \times A_{w\_pont}$ (垂荡刚度)
+ $G_{44} = \rho g \nabla \times GM_T$ (横摇刚度)
+ $G_{55} = \rho g \nabla \times GM_L$ (纵摇刚度)

#### 2. 矩阵构建

通过Mux模块将计算得到的刚度系数组合，然后通过**diag**模块构建6×6对角矩阵：

$$G_{CF} = \text{diag}([0, 0, G_{33}, G_{44}, G_{55}, 0])$$

即：

$$G_{CF} = \text{diag}([0, 0, \rho g \nabla \times 2 \times A_{w\_pont}, \rho g \nabla \times GM_T, \rho g \nabla \times GM_L, 0])$$

#### 3. Simulink实现方式

1. **输入处理**：
    - $\rho$、$g$、$\nabla$ 通过乘法器串联
    - $GM_T$、$GM_L$ 分别与 $\rho g \nabla$ 相乘

2. **增益模块**：
    - "$2 \times A_{w\_pont}$"增益用于垂荡刚度计算
    - $A_{w\_pont}$ 可能表示水线面面积

3. **矩阵组装**：
    - Mux模块收集所有刚度系数
    - diag模块生成最终的6×6对角刚度矩阵

#### 4. 物理意义

+ $G_{33}$：垂荡方向的静水回复刚度
+ $G_{44}$：横摇方向的静水回复刚度
+ $G_{55}$：纵摇方向的静水回复刚度
+ 纵荡(surge)、横荡(sway)、艏摇(yaw)方向刚度为0

这种实现方式清晰地展现了船舶静水回复力矩阵的物理构成和数值计算过程。

---

## 继续提问

弹簧刚度矩阵是什么意思？还有simulink这里面nabla是不是指排开水的体积？

## 回答

### 弹簧刚度矩阵的含义

**"弹簧刚度矩阵"在船舶动力学中是一个比喻性的称呼，实际上指的是静水回复力矩阵或恢复力矩阵**。

#### 1. 物理机制

当船舶偏离平衡位置时：

+ **垂荡**：船体下沉时，浮力增大，向上推；上浮时，浮力减小，向下拉
+ **横摇**：船体倾斜时，浮心偏移产生回复力矩
+ **纵摇**：船体俯仰时，同样产生回复力矩

这种"偏离→回复力"的关系类似于弹簧的**胡克定律**：

$$F = -kx \quad \text{(力与位移成正比)}$$

#### 2. 数学表达

$$F_{\text{restoring}} = -G_{CF} \times \eta$$

其中：

+ $F_{\text{restoring}}$：静水回复力/力矩向量
+ $G_{CF}$：刚度矩阵
+ $\eta$：位置/姿态偏移向量

#### 3. 矩阵结构

$$G_{CF} = \text{diag}([0, 0, \rho g \nabla \times 2 \times A_w, \rho g \nabla \times GM_T, \rho g \nabla \times GM_L, 0])$$

+ 前两个0：纵荡、横荡无静水回复力
+ $G_{33}$：垂荡回复刚度
+ $G_{44}$：横摇回复刚度
+ $G_{55}$：纵摇回复刚度
+ 最后0：艏摇无静水回复力矩

### 关于nabla（$\nabla$）

**是的！nabla确实指排开水的体积**

#### 船舶术语中的 $\nabla$：

+ $\nabla$ = **排水体积**（Displacement Volume）
+ 单位：立方米 (m³)
+ 物理意义：船舶排开的水的体积

#### 相关概念：

+ **排水量** = $\rho \times \nabla$（质量，单位：吨）
+ **浮力** = $\rho \times g \times \nabla$（力，单位：牛顿）

#### 阿基米德原理：

$$\text{浮力} = \text{排开液体的重量} = \rho_{\text{water}} \times g \times \nabla$$

所以在Simulink模型中：

+ $\rho g \nabla$ = 船舶受到的总浮力
+ 这是计算所有静水回复力的基础

这就是为什么所有刚度系数（$G_{33}$、$G_{44}$、$G_{55}$）都包含 $\rho g \nabla$ 这一项的原因。
