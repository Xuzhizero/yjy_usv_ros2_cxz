# Transform CG to CO 重心到原点坐标变换模块分析

## 模块功能说明

**Transform CG to CO（Center of Gravity to Coordinate Origin）**模块将定义在重心（CG）处的惯性矩阵转换到坐标原点（CO）处。这是6DOF动力学计算所必需的，因为方程统一以CO为参考点。

### 转换的必要性

刚体惯性矩阵$M_{RB}$最简单的形式是在重心处定义的（对角块形式），但动力学方程：

$$(M_{RB} + M_A)\dot{\nu} = \tau_{net}$$

中的速度$\nu$和力矩$\tau$都是相对于CO定义的。因此需要将$M_{RB,CG}$转换为$M_{RB,CO}$。

## 输入输出端口

### 输入

- **$M_{RB,CG}$**：在重心处定义的刚体惯性矩阵（6×6）

  $$M_{RB,CG} = \begin{pmatrix}
  m I_3 & 0 \\
  0 & I_G
  \end{pmatrix}$$

- **$\mathbf{r}_G$**：重心CG相对于坐标原点CO的位置向量$(x_g, y_g, z_g)^T$

### 输出

- **$M_{RB,CO}$**：在坐标原点处定义的刚体惯性矩阵（6×6）

## 核心计算公式

### 惯性矩阵变换

根据平行轴定理和刚体力学，惯性矩阵从CG转换到CO：

$$M_{RB,CO} = \begin{pmatrix}
m I_3 & -m S(\mathbf{r}_G) \\
m S(\mathbf{r}_G) & I_G + m S(\mathbf{r}_G)^T S(\mathbf{r}_G)
\end{pmatrix}$$

其中$S(\mathbf{r}_G)$是$\mathbf{r}_G$的反对称矩阵：

$$S(\mathbf{r}_G) = \begin{pmatrix}
0 & -z_g & y_g \\
z_g & 0 & -x_g \\
-y_g & x_g & 0
\end{pmatrix}$$

### 展开形式

$$M_{RB,CO} = \begin{pmatrix}
m & 0 & 0 & 0 & mz_g & -my_g \\
0 & m & 0 & -mz_g & 0 & mx_g \\
0 & 0 & m & my_g & -mx_g & 0 \\
0 & -mz_g & my_g & I_{xx}' & -I_{xy}' & -I_{xz}' \\
mz_g & 0 & -mx_g & -I_{xy}' & I_{yy}' & -I_{yz}' \\
-my_g & mx_g & 0 & -I_{xz}' & -I_{yz}' & I_{zz}'
\end{pmatrix}$$

其中转动惯量部分：
$$\begin{aligned}
I_{xx}' &= I_{xx} + m(y_g^2 + z_g^2) \\
I_{yy}' &= I_{yy} + m(x_g^2 + z_g^2) \\
I_{zz}' &= I_{zz} + m(x_g^2 + y_g^2) \\
I_{xy}' &= I_{xy} + m x_g y_g \\
I_{xz}' &= I_{xz} + m x_g z_g \\
I_{yz}' &= I_{yz} + m y_g z_g
\end{aligned}$$

### $S^T S$的展开

$$S(\mathbf{r})^T S(\mathbf{r}) = \begin{pmatrix}
y^2+z^2 & -xy & -xz \\
-xy & x^2+z^2 & -yz \\
-xz & -yz & x^2+y^2
\end{pmatrix}$$

## 物理意义

### 非对角块的意义

$M_{RB,CO}$的非对角块$\pm m S(\mathbf{r}_G)$代表平移-转动耦合：

- 当在CO点施加纯力时，如果CG不在CO点，会产生力矩
- 当在CO点施加纯力矩时，会导致CG处的平移加速度

### 对角转动惯量增加

根据平行轴定理，关于CO的转动惯量大于关于CG的转动惯量：

$$I_{CO} = I_{CG} + m d^2$$

其中$d$是CG到CO的距离。

## 计算示例

假设：
- 总质量：$m = 1000$ kg
- 重心位置：$\mathbf{r}_G = (0.5, 0, 0.3)^T$ m（CG在CO前方0.5m、下方0.3m）
- 重心处惯性：$I_{xx} = 500$, $I_{yy} = 2000$, $I_{zz} = 2100$ kg·m²

则非对角块：
$$-m S(\mathbf{r}_G) = -1000 \begin{pmatrix}
0 & -0.3 & 0 \\
0.3 & 0 & -0.5 \\
0 & 0.5 & 0
\end{pmatrix} = \begin{pmatrix}
0 & 300 & 0 \\
-300 & 0 & 500 \\
0 & -500 & 0
\end{pmatrix}$$

转动惯量增加：
$$\begin{aligned}
I_{xx}' &= 500 + 1000(0^2 + 0.3^2) = 590 \text{ kg·m}^2 \\
I_{yy}' &= 2000 + 1000(0.5^2 + 0.3^2) = 2340 \text{ kg·m}^2 \\
I_{zz}' &= 2100 + 1000(0.5^2 + 0^2) = 2350 \text{ kg·m}^2
\end{aligned}$$

## 与其他模块的关系

- **MRB_CG模块**：提供在重心处定义的惯性矩阵$M_{RB,CG}$
- **CG Corrected模块**：提供修正后的重心位置$\mathbf{r}_G$
- **Inertials模块**：接收$M_{RB,CO}$并与附加质量矩阵相加

## 对动力学的影响

### 科氏力计算

刚体科氏矩阵$C_{RB}$的计算也需要使用$M_{RB,CO}$：

$$C_{RB}(\nu) = \begin{pmatrix}
0_{3\times3} & -m S(\omega) - m S(\mathbf{v}) S(\mathbf{r}_G)/|\mathbf{v}| \\
-m S(\omega) & -S(I_O \omega)
\end{pmatrix}$$

正确的惯性矩阵变换确保科氏力计算的一致性。

### 牛顿-欧拉方程

在CO点的牛顿-欧拉方程：

$$\begin{pmatrix}
m I_3 & -m S(\mathbf{r}_G) \\
m S(\mathbf{r}_G) & I_O
\end{pmatrix}
\begin{pmatrix}
\dot{\mathbf{v}} \\
\dot{\boldsymbol{\omega}}
\end{pmatrix}
= \begin{pmatrix}
\mathbf{f} \\
\boldsymbol{\tau}
\end{pmatrix}$$

只有使用正确变换后的$M_{RB,CO}$，方程才能正确描述刚体运动。

## 注意事项

1. **对称性验证**：$M_{RB,CO}$必须是对称正定矩阵

2. **数值稳定性**：当$\mathbf{r}_G$接近零时，$M_{RB,CO}$接近$M_{RB,CG}$

3. **载荷变化**：当载荷改变导致$\mathbf{r}_G$变化时，需要重新计算$M_{RB,CO}$

4. **一致性检查**：动能公式应给出相同结果：
   $$T = \frac{1}{2}\nu^T M_{RB,CO} \nu = \frac{1}{2}\nu_{CG}^T M_{RB,CG} \nu_{CG}$$

## 总结

Transform CG to CO模块是6DOF动力学建模的关键步骤。它将直观定义在重心的惯性特性转换到动力学方程统一使用的参考点CO，同时正确引入平移-转动耦合项。这一变换确保了动力学方程的物理正确性和数学一致性。
