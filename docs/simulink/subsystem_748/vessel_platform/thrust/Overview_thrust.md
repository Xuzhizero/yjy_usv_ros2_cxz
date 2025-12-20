# Thrust 模块分析

## 提问

<div align="center">

![Thrust Module](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752741366123-cc54ef20-4752-4163-afc9-b881186f7875.png)

</div>

这个是Thrust模块的具体构造，分析它的功能和机理，以及输入输出的含义

## 回答

下面分块说明这个 **Thrust** 子系统的功能、机理，以及每一路信号的输入输出含义。

---

## 一、Thrust 模块总览

1. **功能**：
   - 将来自上层的 **推进器转速命令**（左、右螺旋桨）转换成实际的螺旋桨旋转速度（rad/s），
   - 再根据正、反向的**推力系数**计算出每个推进器产生的推力（N），
   - 最后合成左右推进器在 Surge、Sway、Yaw 三个方向上的力／力矩输出。

2. **总体流程**：

```
PropCmd_L,R  ─► Propeller Dynamics ─► n_L,R (rad/s) ─► Thrust Computation ─► [T_L, T_R]
                                                                   │
                                           Geometry (lever arms) ◀┘
                                                                   ↓
                                                         [Fx,Fy,Mz] 合成
```

---

## 二、具体构造与机理

### 1. 螺旋桨一阶动力学（Propeller Dynamics）

**输入**
- `VesselCmds.PropellerCommand_L_rps`、`…_R_rps`：左、右推进器的**命令转速** (rad/s)。

**模型**
- 每路经一个一阶低通：

$$G(s)=\frac{1}{s+1}$$

- 模拟推进器从命令转速过渡到实际转速的**惯性延迟**。

**输出**
- **未饱和**的推进器实际转速 `n_raw_L,R` (rad/s)。

### 2. 转速饱和限幅 (n_min, n_max)

**机制**
- 先根据重力加速度 $g$ 与**推力系数** $k_{pos}$、$k_{neg}$ 计算正反两向的**最大安全转速**：

$$n_{max} = +\sqrt{\frac{0.5 \cdot \rho_{+} \cdot g}{k_{pos}}} \quad , \quad n_{min} = -\sqrt{\frac{0.5 \cdot \rho_{-} \cdot g}{k_{neg}}}$$

其中常数 $0.5 \times 24.4$ 和 $0.5 \times 13.6$ 以及对应 $\rho_{\pm}$ 一起是经验修正因子。

**实现**
- 顶部子网：`0.5·24.4 × g ÷ k_pos → sqrt → n_max`
- 底部子网：`0.5·13.6 × g ÷ k_neg → sqrt → n_min`（再取负号）

**输出**
- 饱和上下限 `n_max`、`n_min`（图中 `[−102,104] rad/s`）。

**应用**
- 用一个 **Saturation** 块把 `n_raw_L,R` 限幅到 `[n_min,n_max]`，得到最终 `n_L,R`。

### 3. 推力计算 (Quadratic Thrust Law)

**输入**
- 限幅后的推进器转速 `n_L,R`
- 正反向推力系数 `k_pos`、`k_neg`

**机理**
- 典型的**二次推力模型**：

$$T = k \cdot n \cdot |n|$$

其中：
- $k = k_{pos}$ 当 $n>0$（正向推）；
- $k = k_{neg}$ 当 $n<0$（反向推）。

**实现**
1. 一个比较块 `n>0` 选出 $k_{pos}$ 或 $k_{neg}$。
2. 取绝对值 `|n|`，再与 `n` 以及选出的 `k` 做乘法：

$$T = k \times n \times |n|$$

**输出**
- 左右推进器的推力 `T_L` 和 `T_R`（单位 N），各自范围大约 $[-67,120]$ N。

### 4. 力／力矩合成 (Lever Arms)

**输入**
- 两个推力 `T_L, T_R`
- 左右推进器到船体质心的横向杠杆臂 `l1`、`l2` (m)

**机理**
- Surge 分量：`Fx = T_L + T_R`
- Sway 分量：通常为 0（推进器不产生横向直接力）
- Yaw 分量：推力差 × 杠杆 = $M_z = l_1 T_L - l_2 T_R$

**实现**
- 一个 **Bus Creator** 收集

$$[F_x, 0, 0, 0, 0, M_z]^T$$

其中

$$M_z = l_1 \cdot T_L - l_2 \cdot T_R$$

**输出**
- 一个 6×1 总推力/力矩向量 `Thrust`，送到 "Sum of Forces and Moments" 模块。

---

## 三、端口列表

| 端口/变量 | 含义 |
| --- | --- |
| **输入：** |  |
| VesselCmds.PropellerCommand_L_rps | 左推进器命令转速 (rad/s) |
| VesselCmds.PropellerCommand_R_rps | 右推进器命令转速 (rad/s) |
| k_pos, k_neg | 正/反推力系数 (N/(rad/s)²) |
| gravity (g) | 重力加速度，9.81 m/s² |
| 0.5·24.4, 0.5·13.6 | 经验常数，用于饱和限幅的计算 |
| l1, l2 | 左/右推进器横向杠杆臂 (m) |
| **输出：** |  |
| Thrust (6×1) | 推进贡献的总力/力矩向量 `[Fx; Fy; Fz; Mx; My; Mz]` |

---

## 四、小结

- **Propeller Dynamics**：模拟推进器响应延迟。
- **限幅**：根据推力系数和经验常数，算出安全转速上下限并饱和。
- **二次推力律**：正反不同系数，$T=k \cdot n \cdot |n|$。
- **杠杆合成**：推力叠加产生 Surge 力，力差产生 Yaw 力矩。

这样，Thrust 模块就把"高层的转速命令"变成了"物理量级的推力/力矩"，供 6DOF 动力学方程使用。
