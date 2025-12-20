# MRB_CG 刚体惯性矩阵（重心系）模块分析

## 模块功能说明

**MRB_CG（Rigid-Body Mass Matrix at CG）**模块计算以重心（Center of Gravity, CG）为参考点的刚体惯性矩阵。这是船体（含载荷）的基本惯性描述，为后续转换到坐标原点CO提供基础。

### 惯性矩阵的意义

刚体惯性矩阵$M_{RB}$描述了刚体对各方向加速度的惯性阻抗。在6自由度系统中，它是一个6×6矩阵，将加速度向量$\dot{\nu}$映射到所需的力/力矩向量。

当参考点选在重心CG时，惯性矩阵具有最简形式——平移和转动解耦：

$$M_{RB,CG} = \begin{pmatrix}
m I_3 & 0_{3\times3} \\
0_{3\times3} & I_G
\end{pmatrix}$$

其中：
- $m$：总质量
- $I_3$：3×3单位矩阵
- $I_G$：关于重心的转动惯量张量（3×3）

## 输入输出端口

### 输入

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $m_p$ | mp | 载荷质量 | kg |
| 2 | $I_g$ | Ig | 船体+载荷在CO坐标系下的转动惯量矩阵（来自Inertial Matrix模块） | kg·m² |

内部使用的常量：

| 常量名 | Simulink变量名 | 值 | 说明 |
|--------|----------------|-----|------|
| $m$ | mass (kg) | m | 船体质量 |
| $0_{3\times3}$ | Constant1 | zeros(3,3) | 3×3零矩阵 |
| $I_3$ | Constant2 | eye(3) | 3×3单位矩阵 |

### 输出

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $M_{RB,CG}$ | MRB_CG | 6×6刚体惯性矩阵（在重心系） | kg, kg·m² |

## Simulink变量对应关系表

| 物理量符号 | Simulink变量名 | 计算公式/来源 | 说明 |
|------------|----------------|---------------|------|
| $m$ | mass (kg) | Constant块 | 船体质量 |
| $m_p$ | mp | 输入端口1 | 载荷质量 |
| $m_{total}$ | Add4输出 | $m + m_p$ | 总质量 |
| $I_g$ | Ig | 输入端口2 | 转动惯量矩阵 |
| $m \cdot I_3$ | Product4输出 | $(m+m_p) \times eye(3)$ | 质量矩阵块 |
| 上半部分 | Matrix Concatenate输出 | $[m \cdot I_3; 0_{3\times3}]$ | 左边两行 |
| 下半部分 | Matrix Concatenate1输出 | $[0_{3\times3}; I_g]$ | 右边两行 |
| $M_{RB,CG}$ | Matrix Concatenate2输出 | 见下文公式 | 最终6×6矩阵 |

转动惯量张量的一般形式：
$$I_G = \begin{pmatrix}
I_{xx} & -I_{xy} & -I_{xz} \\
-I_{xy} & I_{yy} & -I_{yz} \\
-I_{xz} & -I_{yz} & I_{zz}
\end{pmatrix}$$

对于关于xz平面对称的船体，$I_{xy} = I_{yz} = 0$。

## 核心计算公式

### 对角线形式（重心系）

在重心系下，惯性矩阵为块对角形式：

$$M_{RB,CG} = \begin{pmatrix}
m & 0 & 0 & 0 & 0 & 0 \\
0 & m & 0 & 0 & 0 & 0 \\
0 & 0 & m & 0 & 0 & 0 \\
0 & 0 & 0 & I_{xx} & -I_{xy} & -I_{xz} \\
0 & 0 & 0 & -I_{xy} & I_{yy} & -I_{yz} \\
0 & 0 & 0 & -I_{xz} & -I_{yz} & I_{zz}
\end{pmatrix}$$

### 组合转动惯量

当船体加上载荷时，总转动惯量需要使用平行轴定理组合：

$$I_{G,total} = I_{G,hull} + I_{G,payload} + m_p S(\mathbf{r}_{p/G})^T S(\mathbf{r}_{p/G})$$

其中$\mathbf{r}_{p/G}$是载荷相对于新重心的位置向量，$S(\cdot)$是反对称矩阵算子。

### 反对称矩阵

对于向量$\mathbf{r} = (x, y, z)^T$：

$$S(\mathbf{r}) = \begin{pmatrix}
0 & -z & y \\
z & 0 & -x \\
-y & x & 0
\end{pmatrix}$$

### 平行轴定理

将转动惯量从一点转移到另一点：

$$I_{O} = I_{G} + m \cdot S(\mathbf{r}_{G/O})^T S(\mathbf{r}_{G/O})$$

展开后：
$$I_{O} = I_G + m \begin{pmatrix}
y^2+z^2 & -xy & -xz \\
-xy & x^2+z^2 & -yz \\
-xz & -yz & x^2+y^2
\end{pmatrix}$$

## 转动惯量的物理意义

| 分量 | 物理含义 |
|------|----------|
| $I_{xx}$ | 绕x轴（纵轴）的横摇惯量 |
| $I_{yy}$ | 绕y轴（横轴）的纵摇惯量 |
| $I_{zz}$ | 绕z轴（垂直轴）的艏摇惯量 |
| $I_{xy}, I_{xz}, I_{yz}$ | 惯量积，描述转动耦合 |

### 回转半径

转动惯量也可以用回转半径表示：

$$I_{xx} = m \cdot r_{xx}^2, \quad I_{yy} = m \cdot r_{yy}^2, \quad I_{zz} = m \cdot r_{zz}^2$$

典型船舶的回转半径估算：
- $r_{xx} \approx 0.35 \sim 0.40 \cdot B$（宽度）
- $r_{yy} \approx 0.25 \sim 0.28 \cdot L$（长度）
- $r_{zz} \approx 0.25 \sim 0.28 \cdot L$

## 与其他模块的关系

- **Transform CG to CO模块**：将$M_{RB,CG}$转换到坐标原点CO得到$M_{RB,CO}$
- **Inertial Matrix模块**：将$M_{RB,CO}$与附加质量$M_A$相加得到总惯性矩阵
- **CG Corrected模块**：提供修正后的重心位置

## 数学推导：从CG到CO的转换

如果坐标原点CO与重心G不重合，$\mathbf{r}_G$为G相对于CO的位置向量，则：

$$M_{RB,CO} = \begin{pmatrix}
m I_3 & -m S(\mathbf{r}_G) \\
m S(\mathbf{r}_G) & I_G + m S(\mathbf{r}_G)^T S(\mathbf{r}_G)
\end{pmatrix}$$

右上和左下的非对角块是由于平移-转动耦合产生的。这一转换在Transform CG to CO模块中完成。

## 示例计算

假设：
- 船体质量$m_h = 1000$ kg
- 载荷质量$m_p = 200$ kg
- 船体转动惯量：$I_{xx,h} = 500$ kg·m², $I_{yy,h} = 2000$ kg·m², $I_{zz,h} = 2100$ kg·m²
- 载荷位于新重心前方2m，高1m：$\mathbf{r}_{p/G} = (2, 0, 1)^T$ m

总质量：$m = 1200$ kg

载荷对总转动惯量的贡献：
$$\Delta I = m_p \begin{pmatrix}
0^2+1^2 & 0 & -2\cdot1 \\
0 & 2^2+1^2 & 0 \\
-2\cdot1 & 0 & 2^2+0^2
\end{pmatrix} = 200 \begin{pmatrix}
1 & 0 & -2 \\
0 & 5 & 0 \\
-2 & 0 & 4
\end{pmatrix}$$

## 注意事项

1. **对称性**：$M_{RB}$必须是对称正定矩阵
2. **惯量积**：对称船体的$I_{xy}$和$I_{yz}$应为零
3. **单位一致性**：确保质量(kg)和距离(m)单位一致

## 总结

MRB_CG模块提供了船舶刚体惯性的基本描述。以重心为参考点的惯性矩阵形式最简单，便于理解物理意义。后续通过坐标变换可以得到任意参考点的惯性矩阵，以满足动力学方程的需求。
