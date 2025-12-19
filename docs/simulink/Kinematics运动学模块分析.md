# Kinematics 运动学模块分析

<p align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1752662846838-d0a0bbde-1cdf-49f5-8855-1a7b0f634fb6.png" alt="Kinematics模块结构图" />
</p>

这是 Kinematics 模块中的具体构造，本文档将分析它的功能、机理和输入输出的含义。

---

## 一、功能概要

Kinematics 模块完成"船身坐标系下的速度 $[u,v,w,p,q,r]$"和"当前姿态角 $(\varphi,\theta,\psi)$"向"惯性系下的位置变化率 $(\dot{x},\dot{y},\dot{z})$"以及"欧拉角变化率 $(\dot{\varphi},\dot{\theta},\dot{\psi})$"的映射：

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{z} \\
\dot{\varphi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
R_b^e(\varphi,\theta,\psi) & 0 \\
0 & T(\varphi,\theta)
\end{bmatrix}
}_{J(\varphi,\theta,\psi)}
\begin{bmatrix}
u \\
v \\
w \\
p \\
q \\
r
\end{bmatrix}
$$

---

## 二、模块机理

### 1. 线速度到惯性系速度

上半部分（黑框中的 DCMbe → $A^T$ → J1）：

1. 用"Rotation Order: ZYX"把 $(\varphi,\theta,\psi)$ 转成从船身系到惯性系的方向余弦矩阵 $R_b^e$
2. 再做转置（$A^T$），得到从船身系速度到惯性系速度的映射矩阵 $J_1 = R_b^e$
3. $J_1$ 乘以 $[u,v,w]$ 就得出 $\dot{x}, \dot{y}, \dot{z}$

### 2. 角速度到欧拉角速率

下半部分（COS/SIN → A11…A33 → J2）：

1. 根据欧拉角（ZYX顺序），有标准的耦合矩阵：

$$
T(\varphi,\theta) =
\begin{bmatrix}
1 & \sin\varphi\tan\theta & \cos\varphi\tan\theta \\
0 & \cos\varphi & -\sin\varphi \\
0 & \frac{\sin\varphi}{\cos\theta} & \frac{\cos\varphi}{\cos\theta}
\end{bmatrix}
$$

2. 这里用基本的 sin/cos 计算出矩阵各元 A₁₁…A₃₃，拼成 $J_2 = T(\varphi,\theta)$
3. $J_2$ 乘 $[p,q,r]$ 就得出 $\dot{\varphi}, \dot{\theta}, \dot{\psi}$

### 3. 拼合输出

- 用一个大矩阵 MUX，把 $J_1$ 和 $J_2$ 两块的结果分别拼到一起，形成 6×6 的复合雅可比矩阵 $J$
- 最后用一次 Matrix Multiply，把它与 6×1 的速度向量 $[u,v,w,p,q,r]$ 相乘，输出 6×1 的 $\dot{x}, \dot{y}, \dot{z}, \dot{\varphi}, \dot{\theta}, \dot{\psi}$

---

## 三、输入/输出对照

| 端口名 | 维度 | 含义 |
|--------|------|------|
| **phi_rad, theta_rad, psi_rad** | 3×1 | 当前欧拉角（ZYX顺序） |
| **u_mps, v_mps, w_mps, p_rps, q_rps, r_rps** | 6×1 | 船身坐标系下的线速度 (m/s) 和角速度 (rad/s) |
| **dot_x_y_z_phi_theta_psi** | 6×1 | 惯性系下的位置/姿态变化率：$\dot{x}, \dot{y}, \dot{z}, \dot{\varphi}, \dot{\theta}, \dot{\psi}$ |

---

## 小结

- **Kinematics** 负责"运动学"映射，不涉及力学
- 它把船体速度和姿态角，分别通过**方向余弦矩阵**和**欧拉角耦合矩阵**，转出**位置速率**和**姿态速率**，供后续积分得到位置和姿态

---

## 问答补充

### Q1: 变换次序难道不能直接变换成旋转矩阵吗？为什么需要多一次转置？

<p align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1755075840332-7d738e93-1650-4a52-9627-ad2d9d6f8e78.png" alt="旋转变换示意图" />
</p>

这是一个很好的问题！让我来解释一下为什么需要这个转置操作。

#### 旋转顺序 vs 矩阵乘法顺序

关键在于理解**旋转顺序（Rotation Order）和矩阵乘法顺序**之间的关系：

当我们说"ZYX旋转顺序"时，通常指的是：

1. 先绕Z轴旋转 $\psi$
2. 再绕Y轴旋转 $\theta$
3. 最后绕X轴旋转 $\varphi$

但是，如果直接按这个顺序写矩阵乘法：

$$
R = R_x(\varphi) \cdot R_y(\theta) \cdot R_z(\psi)
$$

这实际上是**从右到左**执行的：先 $R_z$，再 $R_y$，最后 $R_x$ - 这正好是我们想要的 ZYX 顺序。

#### 为什么需要转置？

问题出现在**坐标系约定**上：

**如果使用列向量约定**（向量写成列）：
- 旋转表示为：$\mathbf{v}' = R \cdot \mathbf{v}$
- ZYX 顺序的矩阵：$R = R_x(\varphi) \cdot R_y(\theta) \cdot R_z(\psi)$

**如果使用行向量约定**（向量写成行）：
- 旋转表示为：$\mathbf{v}' = \mathbf{v} \cdot R$
- 需要转置：$R = [R_x(\varphi) \cdot R_y(\theta) \cdot R_z(\psi)]^T$

转置的原因是：

- 行向量约定下，矩阵乘法顺序需要"翻转"
- $(ABC)^T = C^T B^T A^T$

所以转置操作是为了**适配不同的坐标系约定**，而不是因为旋转顺序本身有问题。如果你始终使用列向量约定，就不需要转置。

---

### Q2: 雅可比变换矩阵 J 为什么要带一个 "$_\Theta$"？不带不行吗？

理论上确实可以不带 $\Theta$，但带上它有重要的**区分和提醒作用**。

#### 为什么要带 $\Theta$？

**1. 区分不同的雅可比矩阵**

在复杂的控制系统中，可能存在多个雅可比矩阵：

- $J_\Theta(\eta)$：欧拉角变换雅可比
- $J_p(\mathbf{x})$：位置变换雅可比
- $J_v(\mathbf{v})$：速度变换雅可比
- 等等...

如果都写成 $J$，容易混淆。

**2. 明确变换的性质**

$\Theta$ 明确表示这是**姿态/旋转相关**的变换矩阵，提醒读者：

- 这个雅可比处理的是角度变换
- 它有奇点问题（如万向锁）
- 需要特别注意角度的取值范围

**3. 学术规范**

在机器人学和控制理论中，这是一种**标准记法**：

- 让读者立即知道这是什么类型的雅可比
- 便于文献交流和理解

#### 不带可以吗？

**技术上完全可以**！如果上下文很清楚，写成 $J(\eta)$ 也没问题。但在正式的学术文献中，为了**严谨性和可读性**，通常会保留这个下标。

就像我们写 $R_{body}^{world}$ 而不是简单的 $R$ 一样——虽然可以简化，但明确的标记能避免歧义，让公式的物理含义更清晰。

---

### Q3: $\nu_c$ 代表水流流速吗？如果是且它不为 0 的话，那根据 $\nu_r = \nu - \nu_c$，为什么下图式子成立（即 $\nu_r = \nu$）？

<p align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1755076307587-abec8ae6-ffce-4ec7-9741-e6ffe94a43b1.png" alt="海流速度公式1" />
</p>

<p align="center">
  <img src="https://cdn.nlark.com/yuque/0/2025/png/2408029/1755076317412-4965462d-7951-4170-854e-3855157a0827.png" alt="海流速度公式2" />
</p>

这个观察很敏锐！这里确实需要仔细理解。

#### $\nu_c$ 的含义

是的，$\nu_c$ 代表**海流速度**（ocean current velocity），包括：

- $v_c^b$：机体坐标系下的线速度分量
- $\omega_c^b$：机体坐标系下的角速度分量（通常为 0）

#### 为什么公式中 $\nu_r = \nu$？

关键在于理解 **Property 10.1 的适用条件**：

**1. 理论推导 vs 实际应用**

- 上面的公式 $\dot{\eta} = J_\Theta(\eta)[\nu_r + \nu_c]$ 是**一般性的动力学关系**
- 下面的 Property 10.1 是在**特定假设下的简化结果**

**2. Property 10.1 的假设**

Property 10.1 假设：

- **海流是常数**：$\dot{\nu_c} = 0$
- **海流的角速度分量为 0**：$\omega_c^b = 0$
- 在这些条件下，可以证明某些变换关系

**3. 坐标变换的特殊性质**

当海流是常数且无旋转分量时，**姿态变换**（$\dot{\eta}$）主要由船体自身的角速度决定，海流的线速度分量在姿态动力学中被"抵消"了。

#### 物理解释

虽然 $\nu_r = \nu - \nu_c$（相对速度 = 绝对速度 - 海流速度），但在**姿态变换**中：

- 海流不直接影响船体的旋转
- $J_\Theta(\eta)$ 主要处理角速度到欧拉角变化率的变换
- 因此在特定条件下，可以得到简化的关系

这是海洋工程中的一个重要性质，允许在有海流的情况下简化姿态控制的设计。
