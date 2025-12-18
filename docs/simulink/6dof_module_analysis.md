# Simulink 6DOF 模块分析

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752654042056-21d83be9-7836-45ed-9311-50faef67d564.png)

</div>

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656951194-c0e5773a-9a6a-4e45-9880-94ff942be654.png)

</div>

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656991031-745ea1a0-30fd-494c-856c-d9583f8501e0.png)

</div>

## 提问

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752654076180-4342586d-0b15-4ae6-b411-7f25e2c9b10b.png)

</div>

这个是Simulink，6 DOF模块的具体内部构造。告诉我这个模块里面的输入输出的含义，这个模块的功能和产生此功能的机理。

## 回答

下面分三部分来说明这个"6DOF (In CO)"模块的功能、机理，以及各输入输出的含义。

---

### 一、功能概述

"6DOF"模块就是在坐标原点（CO）系下，根据已求得的合力和合力矩，以及船体的总惯性矩阵，算出六自由度航行器的加速度、速度、位置和姿态。

数学上它求解的就是：

**动力学与运动学模型：**

$$
(M_{RB}+M_A)\,\dot{\nu} = \tau_{\text{net}}
\quad,\quad
\dot{\eta} = J(\eta)\,\nu
$$

其中

+ $\nu=[u,v,w,p,q,r]^T$ 是船体在船身坐标系下的速度和角速度，
+ $\eta=[x,y,z,\phi,\theta,\psi]^T$ 是在惯性坐标系下的位置和欧拉角，
+ $J(\eta)$ 是从船体速度到位置／欧拉角导数的几何映射矩阵。

---

### 二、模块结构与机理

```plain
Inputs:
 ┌─────────────────────────────┐
 │  Forces and Moments (6×1)   │──┐
 └─────────────────────────────┘  │
                                  │  ┌────────┐      ┌───────────┐
 ┌─────────────────────────────┐  │  │ Inv    │      │ 1/s (x6) │
 │      Inertials (6×6)       │──┼──▶│ M⁻¹×   │──▶──▶│ Integrate │──▶───► ν (6×1)
 └─────────────────────────────┘  │  └────────┘      └───────────┘
                                  │
 ┌─────────────────────────────┐  │    ┌─────────────────────────┐
 │   Initial Conditions (18×1)│──┴───▶│    Kinematics (J)      │
 └─────────────────────────────┘       └─────────────────────────┘

Outputs:
 ── Surge Velocity u, Sway v, Heave w,
    Roll p, Pitch q, Yaw r
 ── Position x,y,z
 ── Attitude φ,θ,ψ
 ── Linear Accel. ẍ,ÿ,z̈
 ── Angular Accel. ṗ, q̇, ṙ
```

**1. Inv 块 (M⁻¹·τ)**

- **输入**：
  * "Forces and Moments" $\tau_{\text{net}}$（6×1）
  * "Inertials" $M$（6×6，总质量和转动惯量矩阵）
- **计算**：

$$
\dot{\nu} = M^{-1}\,\tau_{\text{net}}
$$

- **输出**：身体坐标系下的六分量加速度 $\dot{\nu}$。

**2. Integrate 块 (1/s)**

- **输入**：
  * $\dot{\nu}$（6×1）
  * 初始条件 "IC.u0, v0, w0, p0, … , ψ0" 用于第一次仿真时设定积分起点。
- **计算**：

$$
\nu(t) = \int \dot{\nu}\,dt + \nu(0)
$$

- **输出**：速度/角速度 $\nu=[u,v,w,p,q,r]$。

**3. Kinematics 块 (J)**

- **输入**：当前速度向量 $\nu$ 及角度 $\eta$（通过反馈回路获得）。
- **计算**：

$$
\begin{bmatrix}\dot{x}\\\dot{y}\\\dot{z}\\\dot{\phi}\\\dot{\theta}\\\dot{\psi}\end{bmatrix}
= J(\phi,\theta,\psi)\,
\begin{bmatrix}u\\v\\w\\p\\q\\r\end{bmatrix}
$$

其中 $J$ 把船体系速度转换成惯性系下的位置／欧拉角变化率。

- **输出**：位置和姿态导数 $\dot{\eta}$。

**4. 位置／姿态积分**

- 将 $\dot{x},\dot{y},\dot{z},\dot{\phi},\dot{\theta},\dot{\psi}$ 再串入一组 1/s 积分块，得到位置 $(x,y,z)$ 和姿态角 $(\phi,\theta,\psi)$。

---

### 三、各端口含义对照

| 端口名 | 尺寸 | 含义 |
| --- | --- | --- |
| **Forces and Moments** | 6×1 | 来自"Sum of Forces and Moments"模块的合力（Surge→Yaw） |
| **Inertials** | 6×6 | 总惯性矩阵 = 刚体惯性 + 附加质量惯性 |
| **Initial Conditions** | 18×1 | 初始速度(u0,v0,w0,p0,q0,r0)、初始位置(x0,y0,z0)和姿态(φ0,θ0,ψ0) |
| **Surge Velocity (u_mps)** … **Yaw Rate (r_rps)** | 6×1 | 船体六轴速度输出 |
| **x (m), y (m), z (m)** | 3×1 | 船体在惯性系下的位置 |
| **Roll Angle (rad), Pitch Angle, Yaw Angle** | 3×1 | 欧拉角姿态 |
| **Acc X, Acc Y, Acc Z** | 3×1 | 线加速度 = $\dot{u},\dot{v},\dot{w}$ |
| **Angular Acc X,Y,Z** | 3×1 | 角加速度 = $\dot{p},\dot{q},\dot{r}$ |

---

#### 机理小结

1. **动力学**：用 $M^{-1}\tau$ 直接算出加速度。
2. **速度积分**：加速度在线积分得速度。
3. **运动学**：用速度和当前姿态，通过几何矩阵 $J$ 计算位置／角度变化率。
4. **位置／姿态积分**：变化率在线积分得最终位置和姿态。

如此就完成了六自由度从"力→加速度→速度→位置姿态"的全链路仿真。

---

## 继续追问

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752657584059-5fd8fef4-649a-457c-8c34-8a26d51e0f46.png)

</div>

## 回答

这是因为"6DOF"模块里实际上把动力学积分（Acceleration→Velocity）和运动学积分（Velocity→Position/姿态）两条路都给做了：

---

### 1. 动力学积分：把加速度积分到速度

+ 方程中有

$$
(M_{RB}+M_A)\,\dot{\nu} = \tau_{\rm net},
$$

求出 $\dot{\nu}$（6×1 加速度）后，必须做一个积分

$$
\nu(t) = \int \dot{\nu}\,dt + \nu(0)
$$

这就是第一个"1/s"积分类比块，它把"Inv·τ" 结果（加速度）积分得到速度分量 $[u,v,w,p,q,r]$。

---

### 2. 运动学积分：把速度积分到位置和姿态

+ 同时，航行器的位置 $\eta=[x,y,z,\phi,\theta,\psi]$ 并不是直接等于速度 $\nu$，而是要经过几何映射矩阵 $J(\eta)$ 才变成位置／角度的瞬时导数：

$$
\dot{\eta} = J(\eta)\,\nu.
$$

+ 这一步在 Simulink 里是"Kinematics (J)"块：它拿刚算出的速度 $\nu$ 和当前姿态 $\eta$，输出 $\dot{x},\dot{y},\dot{z},\dot{\phi},\dot{\theta},\dot{\psi}$。
+ 然后再经过第二个"1/s"积分类比，把 $\dot{\eta}$ 积分，得到新的位置 $(x,y,z)$ 和姿态角 $(\phi,\theta,\psi)$。

---

#### 为什么要两次积分？

+ **一次积分**（加速度→速度）是动力学定律本身要求的：$\dot{\nu}\to\nu$。
+ **第二次积分**（速度→位置／姿态）是为了把船体"动起来"往哪个方向、旋转到什么朝向，也必须把速度积分成位移和欧拉角。

把这两段分开，不仅结构清晰，也能准确实现

$$
\begin{cases}
(M_{RB}+M_A)\,\dot{\nu} = \tau,\\
\dot{\eta} = J(\eta)\,\nu.
\end{cases}
$$

因此，Simulink 里既要对 $\dot{\nu}$ 做积分，也要对 $J\,\nu$ 做积分——两者分别得到速度和位置/姿态，这正是惯性导航系统里常见的"动力学＋运动学"双积分流程。
