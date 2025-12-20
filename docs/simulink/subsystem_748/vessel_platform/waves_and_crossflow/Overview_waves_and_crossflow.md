# Waves and Crossflow 模块解析

本文档详细解析 `crossFlowDrag` 函数的功能、机理以及输入输出含义，并对比分析 Simulink 中 Damping 模块与横流拖曳（crossFlowDrag）的区别。

---

## 一、crossFlowDrag 函数源码解析

### 1.1 源码

```matlab
function tau_crossflow = crossFlowDrag(L, B, T, nu_r, rho)

% tau_crossflow = crossFlowDrag(L,B,T,nu_r) computes the cross-flow drag
% integrals for a marine craft using strip theory. Application:
%
%  M d/dt nu_r + C(nu_r)*nu_r + D*nu_r + g(eta) = tau + tau_crossflow
%
% Inputs: L:  length
%         B:  beam
%         T:  draft
%         nu_r = [u-u_c, v-v_c, w-w_c, p, q, r]': relative velocity vector
%
% Output: tau_crossflow = [0 Yh 0 0 0 Nh]:  cross-flow drag in sway and yaw
%
% Author:     Thor I. Fossen
% Date:       25 Apr 2021
% Revisions:

% rho = 1026;             % density of water
n = 20;                 % number of strips

dx = L/20;
Cd_2D = Hoerner(B,T);   % 2D drag coefficient based on Hoerner's curve

Yh = 0;
Nh = 0;
for xL = -L/2:dx:L/2
    v_r = nu_r(2);          % relative sway velocity
    r = nu_r(6);            % yaw rate
    Ucf = abs(v_r + xL * r) * (v_r + xL * r);
    Yh = Yh - 0.5 * rho * T * Cd_2D * Ucf * dx;         % sway force
    Nh = Nh - 0.5 * rho * T * Cd_2D * xL * Ucf * dx;    % yaw moment
end

tau_crossflow = [0 Yh 0 0 0 Nh]';

end

function CY_2D = Hoerner(B,T)
% Hoerner computes the 2D Hoerner cross-flow form coeff. as a function of B and T.
% he data is digizied and interpolation is used to compute other data points
% than those in the table
%
%  CY_2D = Hoerner(B,T)
%
% Output:
%    CY_2D:    2D Hoerner cross-flow form coefficient
%
% Inputs:
%    T:      draft (m)
%    B:      beam  (m)
%
% Author: Thor I. Fossen
% Date:   2007-12-01
%
% Reference:
% A. J. P. Leite, J. A. P. Aranha, C. Umeda and M. B. conti (1998).
% Current force in tankers and bifurcation of equilibrium of turret
% systems: hydrodynamic model and experiments.
% Applied Ocean Research 20, pp. 145-256.

% CD_DATA = [B/2T  C_D]
CD_DATA = [...
0.0108623 1.96608
0.176606 1.96573
0.353025 1.89756
0.451863 1.78718
0.472838 1.58374
0.492877 1.27862
0.493252 1.21082
0.558473 1.08356
0.646401 0.998631
0.833589 0.87959
0.988002 0.828415
1.30807 0.759941
1.63918 0.691442
1.85998 0.657076
2.31288 0.630693
2.59998 0.596186
3.00877 0.586846
3.45075 0.585909
3.7379 0.559877
4.00309 0.559315];

CY_2D = interp1(CD_DATA(:,1),CD_DATA(:,2),B/(2*T));
end
```

### 1.2 文件结构

这个文件里包含了两部分内容：

1. **`crossFlowDrag` 函数** —— 利用条带理论（strip theory）计算横向来流产生的横荡阻力和偏航力矩。
2. **`Hoerner` 子函数** —— 根据 Hoerner 曲线（经验表）插值得到二维阻力系数。

---

## 二、`crossFlowDrag` 函数

```matlab
function tau_crossflow = crossFlowDrag(L, B, T, nu_r, rho)
% tau_crossflow = [0 Yh 0 0 0 Nh]'
```

### 2.1 功能

- 计算船体在水流横向作用（sway + yaw）下的附加阻力 `Yh` 和扭矩 `Nh`，并返回一个 6×1 向量 `tau_crossflow`，其中只有 sway（第二分量）和 yaw（第六分量）非零，其它分量为 0。
- 该力矩会被加到总力矩方程中：

$$
M\dot{\nu} + C(\nu)\nu + D\nu + g(\eta) = \tau_{\text{thrust}} + \tau_{\text{crossflow}}
$$

### 2.2 输入

| 参数 | 含义 |
| --- | --- |
| **L** | 船体总长（m） |
| **B** | 船体最大梁宽（m） |
| **T** | 吃水深度（m） |
| **nu_r** | 相对速度向量 $[u_r, v_r, w_r, p, q, r]^T$ |
| **rho** | 水体密度（kg/m³），常取约 1026 |

### 2.3 机理

#### 2.3.1 条带离散

将船体纵向从 $-L/2$ 到 $+L/2$ 等分成 $n=20$ 段，每段长度 `dx = L/20`。

#### 2.3.2 局部速度合成

在第 $i$ 段位置 $x_L$ 处，横向来流速度近似为：

$$
U_{\mathrm{cf}}(x_L) = v_r + x_L \cdot r
$$

其中 $v_r$ 是相对 sway 速度，$r$ 是相对 yaw 角速度。

**理论基础**：这是基于刚体运动学中的速度合成原理。当一个刚体同时进行平移和旋转运动时，刚体上任意一点的总速度等于**平移速度 + 旋转速度**：

- $v_r$ 代表物体的平移速度分量（相对sway速度）
- $r$ 代表角速度（相对yaw角速度）
- $x_L$ 代表该点距离旋转中心的距离
- $x_L \cdot r$ 代表该点由于旋转运动产生的线速度

#### 2.3.3 局部阻力与力矩

每个截面按二维板块模型产生阻力：

$$
dY = -\frac{1}{2} \rho \cdot T \cdot C_{2D} \cdot |U_{\mathrm{cf}}| \cdot U_{\mathrm{cf}} \cdot dx
$$

$$
dN = -\frac{1}{2} \rho \cdot T \cdot C_{2D} \cdot x_L \cdot |U_{\mathrm{cf}}| \cdot U_{\mathrm{cf}} \cdot dx
$$

**各参量含义**：

| 参量 | 含义 |
| --- | --- |
| $\frac{1}{2}$ | 动压系数，流体力学中标准的动压公式系数 |
| $\rho$ | 流体密度，对应源码中的 `rho = 1026` (kg/m³)，海水密度 |
| $T$ | 吃水深度（draft），船舶的吃水深度 |
| $C_{2D}$ | 二维横流阻力系数，由 `Hoerner(B,T)` 计算，基于Hoerner曲线 |
| $\|U_{\mathrm{cf}}\| \cdot U_{\mathrm{cf}}$ | 对应源码中的 `abs(v_r + xL * r) * (v_r + xL * r)` |
| $dx$ | 积分微元，对应源码中的 `dx = L/20` |

> **注意**：负号表明阻力方向总是与运动方向相反。$C_{2D}$ 是截面阻力系数，由 `Hoerner` 函数给出。

#### 2.3.4 积分求和

沿船长把各段的 `dY`、`dN` 累加，就得到总的横荡力 `Yh` 和偏航力矩 `Nh`。

#### 2.3.5 输出组装

最终返回：

```matlab
tau_crossflow = [0; Yh; 0; 0; 0; Nh];
```

---

## 三、`Hoerner` 子函数

```matlab
function CY_2D = Hoerner(B,T)
```

### 3.1 功能

根据 Hoerner 曲线的经验数据，插值得到二维横流阻力系数 $C_{2D}$，用于上面 `crossFlowDrag` 里的每段阻力计算。

### 3.2 输入

| 参数 | 含义 |
| --- | --- |
| **B** | 浮筒/舰体梁宽（m） |
| **T** | 吃水深度（m） |

### 3.3 机理

1. 预定义了一组 $\frac{B}{2T}$ 对应的 $C_D$ 数据表 `CD_DATA`。
2. 计算当前几何比值 $\frac{B}{2T}$。
3. 用 `interp1` 对表格进行线性插值，输出对应的二维阻力系数 `CY_2D`。

---

## 四、整体调用流程

1. **用户** 调用：

```matlab
tau_crossflow = crossFlowDrag(L, B, T, nu_r, rho);
```

2. **内部**：
   - 先 `Cd_2D = Hoerner(B,T)`；
   - 再做 20 段条带积分，算出 `Yh`、`Nh`；
   - 返回 6×1 的 `tau_crossflow`。

3. **系统** 在总的力矩求和里把 `tau_crossflow` 加上，就把横流拖曳效应带进了整船动力学方程。

---

## 五、小结

- **作用**：模拟船体/浮筒在横向来流下的非线性阻力和扭矩。
- **机理**：基于"每段横流速度 → 局部阻力积分 → 求和"的条带理论，结合 Hoerner 经验系数。
- **输入**：船体尺寸 (L, B, T)、相对速度向量 nu_r、水密度 rho。
- **输出**：6×1 向量 `tau_crossflow`，只有横荡和偏航分量非零，供后续动力学方程使用。

> **注意**：流的效果不是施加力，而是只有速度的效果。

---

## 六、Damping 模块与 crossFlowDrag 的区别

### 6.1 结论

两个"带阻尼"的块**不是一类东西**，作用机理也不同，放在同一套 6-DOF 方程里是**互补**而非重复——但在 **sway/yaw** 通道可能产生部分"重叠耗散"，需要按试验数据调参。

### 6.2 一眼对比

| 维度 | Simulink 里的 **Damping（Hydro dynamics）** | **crossFlowDrag（横流拖曳）** |
| --- | --- | --- |
| 物理来源 | 以**等效线性阻尼**表征的辐射/粘性耗散；heave/roll/pitch 用 $C=2\zeta M\omega$ | **黏性形状阻力**（分离/涡脱落主导）的**横流二次阻力** |
| 数学形式 | 近似线性：$\tau_{\text{damp}}=-D \cdot \nu$（多取对角）；个别通道加经验非线性 | 非线性二次：$dY=-\frac{1}{2}\rho T C_{2D} \|U_{\mathrm{cf}}\| U_{\mathrm{cf}} dx$，沿艏艉积分 |
| 依赖参数 | $\omega_3, \omega_4, \omega_5$（垂荡/横摇/纵摇的**固有频率**）、阻尼比 $\zeta$、质量/转动惯量 | Hoerner 系数 $C_{2D}(B/2T)$、吃水 $T$、相对速度 $v_r$ 和艏摇角速 $r$ |
| 作用自由度 | 6 个 DOF 都能给；其中 **heave/roll/pitch** 强依赖 $\omega$、$\zeta$ | **仅给 sway（Y）和 yaw（N）**：$\tau_{\text{crossflow}}=[0\ Y_h\ 0\ 0\ 0\ N_h]^T$ |
| 对速度的阶次 | 主要是一阶（与速度成正比），便于控制/线性化分析 | 二阶（$\|v\|v$），随速度/角速显著增大 |
| 典型适用 | 小幅振动、波浪线性响应、控制器线化设计 | 操纵性、侧滑/转弯、大角速下的横流耗散 |

### 6.3 Damping（Hydro dynamics）块到底在做什么

- **核心机理**：把 heave/roll/pitch 当作二阶系统，用

$$
C = 2 \zeta M \omega
$$

构造**线性阻尼系数**（如 $Z_w, K_p, M_q$），其中：

- $M$ 为对应自由度的等效质量/转动惯量（如 $M_{33}, I_{xx}, I_{yy}$）；
- $\omega_{3,4,5}$ 是这三个模态的**固有角频率**（你问到的那组 $\omega$ 就用于此处）；
- $\zeta$ 是工程给定的**阻尼比**。

- **其余通道**（surge/sway/yaw）常用经验线性/弱非线性系数（例如你之前看到的 $Y_v=-(1+10|u|)$ 之类），也是为了在**不引入频率依赖**的前提下，给 6-DOF 方程提供一个"可调的等效耗散"。

> **小结**：这个块更多是"**等效线性化**的水动阻尼"，$\omega$ 的作用是通过 $C=2\zeta M\omega$ 把目标模态阻尼水平（$\zeta$）**落地成系数**。

### 6.4 crossFlowDrag 在做什么

- **条带理论**把船长分段：每个截面在局部横向相对流速

$$
U_{\mathrm{cf}}(x) = v_r + x \cdot r
$$

下产生二维**横流黏性阻力**（二次项），再沿 $x \in [-L/2, L/2]$ 积分。

- 只输出 **Y** 和 **N**，捕捉"横移 + 偏航"下的**非线性**耗散，**不**涉及 heave/roll/pitch，**不**用到固有频率。

### 6.5 它们怎么配合？会不会"重复计算阻尼"

- **不重复的部分**：
  - heave/roll/pitch 的阻尼几乎**全靠 Damping 块**（crossFlowDrag 不给 Z/K/M）。

- **可能重叠的部分**（需要调参）：
  - sway/yaw：Damping 块里已有 $Y_v, N_r$ 的等效线性/经验项；crossFlowDrag 又额外给了**非线性二次**阻尼 $Y_h, N_h$。两者叠加会增大耗散。

- **工程建议**：
  1. **基线法**：保留 Damping 的 $Y_v, N_r$ 作为小幅/低速的"线性耗散"，再叠加 crossFlowDrag 以覆盖大角速/强横流的非线性。
  2. **不重叠法**：若发现操纵性被"过阻尼"，可**下调或关闭** $Y_v, N_r$，让 sway/yaw 主要由 crossFlowDrag 负责。
  3. **以数据为准**：用 PMM/CFD/海试曲线（如 $Y(v_r), N(r)$）拟合，校准 Hoerner 系数或 Damping 系数，避免过度耗散。

### 6.6 何时优先用谁

- **要做耐波/小振幅响应（RAO、控制线化）**：优先依赖 **Damping（+ 频率相关辐射阻尼更佳）**。
- **要做操纵性/转向/侧滑**：必须引入 **crossFlowDrag**；单靠线性 $Y_v, N_r$ 往往不够。

### 6.7 一句话总结

- **Damping 块** = 等效线性阻尼（$\omega$ 用来定系数），适合小幅/频域友好；
- **crossFlowDrag** = 横流非线性二次阻力（条带积分），专攻 sway/yaw 的大幅/强分离耗散。

两者可叠加，但 **sway/yaw 要校核是否"过阻尼"**，按试验数据调小一边。
