# G_CF 浮心坐标系静水恢复刚度矩阵模块分析

## 模块功能说明

**G_CF (spring stiffness matrix in the CF)** 模块计算在浮心坐标系（CF）下的静水恢复刚度矩阵。该矩阵描述了船体偏离平衡位置时产生的恢复力/力矩。

## 输入输出端口

### 输入

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $\rho$ | rho (来自Water Density) | 水密度 | kg/m³ |
| 2 | $g$ | g (来自Gravity) | 重力加速度 | m/s² |
| 3 | $\nabla$ | nabla (来自Volume模块) | 排水体积 | m³ |
| 4 | $GM_T$ | GM_T (来自GM模块) | 横向稳心高度 | m |
| 5 | $GM_L$ | GM_L (来自GM模块) | 纵向稳心高度 | m |

内部使用的常量：

| 常量名 | Simulink变量名 | 值 | 说明 |
|--------|----------------|-----|------|
| 0 | Constant | 0 | 零刚度（用于surge, sway, yaw） |
| $2 A_{w,pont}$ | Gain2增益 | 2*A_wp | 双浮筒水线面积系数 |

### 输出

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $G_{CF}$ | G_CF | 6×6静水恢复刚度矩阵 | N/m, N·m/rad |
| 2 | $G_{33}$ | G33 | 垂荡刚度 | N/m |
| 3 | $G_{44}$ | G44 | 横摇刚度 | N·m/rad |
| 4 | $G_{55}$ | G55 | 纵摇刚度 | N·m/rad |

## Simulink变量对应关系表

| 物理量符号 | Simulink变量名 | 计算公式/来源 | 说明 |
|------------|----------------|---------------|------|
| $\rho$ | rho | 输入端口1 | 水密度 |
| $g$ | g | 输入端口2 | 重力加速度 |
| $\nabla$ | nabla | 输入端口3 | 排水体积 |
| $GM_T$ | GM_T | 输入端口4 | 横向稳心高度 |
| $GM_L$ | GM_L | 输入端口5 | 纵向稳心高度 |
| $\rho g$ | Product6输出 | $\rho \times g$ | 中间变量 |
| $\rho g \nabla$ | Product7输出 | $\rho g \times \nabla$ | 浮力 |
| $G_{33}$ | Gain2输出 / G33 | $\rho g \times 2 A_{w,pont}$ | 垂荡刚度 |
| $G_{44}$ | Product9输出 / G44 | $\rho g \nabla \times GM_T$ | 横摇刚度 |
| $G_{55}$ | Product10输出 / G55 | $\rho g \nabla \times GM_L$ | 纵摇刚度 |
| 对角向量 | Mux输出 | $[0,0,G_{33},G_{44},G_{55},0]$ | 刚度向量 |
| $G_{CF}$ | Create Diagonal Matrix输出 | diag(对角向量) | 最终矩阵 |

## 核心计算公式

$$G_{33} = \rho g \times 2 A_{w,pont}$$
$$G_{44} = \rho g \nabla \times GM_T$$
$$G_{55} = \rho g \nabla \times GM_L$$

$$G_{CF} = \text{diag}([0, 0, G_{33}, G_{44}, G_{55}, 0])$$

---

## 详细说明

<img width="1581" height="490" alt="image" src="https://github.com/user-attachments/assets/24d99a21-bd15-4517-a52c-4a392687c151" />
<img width="1864" height="568" alt="image" src="https://github.com/user-attachments/assets/4b871224-4be4-41b3-b3e0-b79e3cc4f3d9" />

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
