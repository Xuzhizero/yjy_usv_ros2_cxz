# Simulink动力学模型方程对应关系分析

本文档详细分析了船舶动力学理论方程与Simulink实现之间的对应关系。

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752653980678-3ae628ec-5a9f-4ed6-97e0-15ca73937d5d.png)

</div>

## 提问

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752653414585-516e7497-7019-46b3-93f4-eb26a919e75c.png)

</div>

Simulink中，动力学模型基于上述方程，

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752653525780-c45f2380-15c3-43bb-9a1a-7984568cbb13.png)

</div>

这两个图像，彼此是如何对应的？

## 回答

上面那张是理论上的「相对运动方程」，下面那张是它在 Simulink 里的"力矩求和"实现。二者一一对应关系可以这样看：

---

### 1. 理论方程（第一张图）

$$
\underbrace{M_{RB}\,\dot{\nu} \;+\;C_{RB}(\nu)\,\nu \;+\;g(\eta)\;+\;g_0}_{\text{刚体惯性项 + 刚体科氏项 + 静水恢复项 + 压舱恢复项}}
\;+\;
\underbrace{M_A\,\dot{\nu}_r \;+\;C_A(\nu_r)\,\nu_r \;+\;D(\nu_r)\,\nu_r}_{\text{附加质量惯性 + 附加质量科氏 + 阻尼}}
\;=\;
\tau_{\rm wind} \;+\;\tau_{\rm wave}\;+\;\tau_{\rm thrust}
$$

并且

$$
\nu_r=\nu-\nu_c
$$

---

### 2. Simulink "Forces and Moments" 求和块（第二张图）

| 方程中对应的项 | Simulink 模块名称 | 求和符号 |
| --- | --- | --- |
| **控制推进力** $\tau_{\rm thrust}$ | Thrust | "+" |
| **附加质量阻尼** $D(\nu_r)\,\nu_r$ | Hydrodynamics.Damping | "+" |
| **波浪／横流激励** $\tau_{\rm wave}$ | Crossflow (Waves & Crossflow) | "+" |
| **附加质量科氏力** $C_A(\nu_r)\nu_r$ | Hydrodynamics.Coriolis | "–" |
| **静水恢复力** $g(\eta)$ | Hydrostatic.Hydrostatic | "–" |
| **压舱（水）恢复力** $g_0$ | Hydrostatic.Ballast | "–" |

> **注意**：
>
> + 方程里的惯性项 $M_{RB}\dot{\nu}$ 和附加质量惯性 $M_A\dot{\nu}_r$ 其实在 6DOF 积分模块里（Integrate Acceleration）里一起处理，不是通过这个求和块直接相加。
> + 科氏项分为刚体科氏 $C_{RB}(\nu)\nu$ 和附加质量科氏 $C_A(\nu_r)\nu_r$。这里 Simulink 仅把"附加质量科氏"拿出来作为 Hydrodynamics.Coriolis，刚体科氏则隐含在动量积分里。

---

#### 工作流程

1. **各模块各自计算自己那一项**（推进器力、波浪力、阻尼力、科氏力、静水恢复力、压舱恢复力）。
2. **送入 Sum [+ + + – – –] 求和块**，根据符号正负叠加出总力矩向量

$$
\tau_\text{net}
     = \tau_{\rm thrust}
     + D(\nu_r)\nu_r
     + \tau_{\rm wave}
     - C_A(\nu_r)\nu_r
     - g(\eta)
     - g_0
$$

3. **再交给 6DOF 模块** 去除以总惯性 $M_{RB}+M_A$，积分出加速度、速度、位置姿态。

---

这样，Simulink 里的"求和"方块就精确实现了理论方程中的非惯性项。

注意：

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752657180552-bd99d742-86b5-4644-a7db-08bfb8cedc5f.png)

</div>

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752657213976-effd4710-9349-4a09-b351-9c58ef82fa21.png)

</div>

## 继续追问

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656384057-3a8681ab-bde9-44f2-9941-badfe4fbce78.png)

</div>

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752656392054-756ad5ef-5ea3-46b6-9054-00af29c5c95f.png)

</div>

先问第一个问题：

从方程中来看，为了得到方程里的总惯性，阻尼项应该是负的才对，为什么simulink中是正的？

## 回答

在理论方程里，我们把阻尼项写作

$$
+\,D(\nu_r)\,\nu_r
$$

看起来是"加号"，可别忘了物理上 $D$ 本身就是一个**负定矩阵**（即所有对角元素都是负值），所以 $D\,\nu_r$ 输出的力恰好是"— 阻碍运动"的方向。

---

#### Simulink 里为什么用"+"

看一下 Simulink 求和块里这一行：

```plain
…
+  Hydrodynamics.Damping
+  Crossflow
−  Coriolis
−  Hydrostatic
−  Ballast
…
```

这里的"+""−"只是告诉 Sum 块"把这个端口的输出是加到总和里"还是"从总和里减去"。它并不代表该物理项本身是正向力还是负向力。

+ **Hydrodynamics.Damping 模块内部** 其实做了一个负号操作：

$$
F_\text{damp} \;=\; -\,D_\text{critical}\,\nu_r
    \quad\text{再乘以阻尼比 ζ}
    \;\Longrightarrow\; \text{一串计算后输出的就是一个负值向量}
$$

  因此这个模块**输出的信号本身就是负的**，表示"力的方向和速度相反"。

+ 在 Sum 里你只要用"+"把它加进来，就相当于

$$
\tau_\text{net} \;+=\; (\underbrace{-\,C\,\nu_r}_{F_\text{damp}})
$$

  达到了理论中"+ D(ν_r) ν_r（D<0）"的效果。

---

##### 小结

+ **理论方程**：写成 "+ D ν" 只是数学上把 D ν 放到左边；D 自带负号。
+ **Simulink 求和块**：模块输出已包含负号，你在 Sum 里用"+"号即可，不需要双重负。

所以看上去"阻尼项被加号"其实没错，真正让它"负起来"的，是 Hydrodynamics.Damping 里那个"× –1"的内部实现。

## 继续追问

图中，我能理解Vc的一阶导数为0时，V的导数和V_r的导数可以相等，所以左侧第一项和右侧第一项可以相等，但是为什么左侧第二项和右侧第二项相等呢？

<div align="center">

![](https://cdn.nlark.com/yuque/0/2025/png/2408029/1755076815565-00423246-4177-47eb-8722-c8a9a14f038f.png)

</div>

## 回答

这是一个关于刚体动力学中科里奥利力的问题。让我来解释为什么方程(10.129)中左右两侧的第二项相等。

### 关键理解点

方程 (10.129) 表示的是：

$$
M_{RB}\nu + C_{RB}(\nu)\nu = M_{RB}\nu_r + C_{RB}(\nu_r)\nu_r
$$

其中 $\nu_r = \nu - \nu_c$ 是相对速度。

### 为什么第二项相等

这里的关键假设是**海流速度** $\nu_c$ **是常数**（irrotational and constant），这意味着：

1. $\dot{\nu}_c = 0$（时间导数为零）
2. $\nu_c$ **在空间上是均匀的**（无旋转，无剪切）

#### 科里奥利矩阵的性质

科里奥利矩阵 $C_{RB}(\nu)$ 包含了由于刚体旋转产生的陀螺效应和离心效应。对于6自由度刚体运动：

$$
C_{RB}(\nu) = \begin{bmatrix}
0_{3×3} & -S(M_{11}u + M_{12}\omega) \\
-S(M_{11}u + M_{12}\omega) & -S(M_{21}u + M_{22}\omega)
\end{bmatrix}
$$

其中 $S(·)$ 是反对称矩阵算子。

#### 为什么 $C_{RB}(\nu)\nu = C_{RB}(\nu_r)\nu_r$

当海流速度恒定且均匀时：

1. **线性部分**：由于 $\nu_c$ 是常数平移速度，不产生额外的科里奥利效应
2. **旋转部分**：刚体的角速度 $\omega$ 在绝对坐标系和相对坐标系中是相同的（因为海流只是平移，不包含旋转）

因此：

+ $C_{RB}(\nu)$ 中的旋转相关项与 $C_{RB}(\nu_r)$ 中的相同
+ 科里奥利力只依赖于刚体自身的旋转运动，不受恒定平移海流的影响

这就是为什么在恒定、无旋海流假设下，两个科里奥利项相等的物理原因。这个性质使得我们可以直接在相对速度坐标系中建立动力学方程，大大简化了海洋工程中的控制问题。
