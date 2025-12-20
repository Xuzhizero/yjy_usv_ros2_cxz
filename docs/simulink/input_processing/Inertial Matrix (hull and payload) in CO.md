<div align="center">
<img width="1313" height="131" alt="image" src="https://github.com/user-attachments/assets/57327f7e-d55b-4ba7-940c-422706c7ade6" />
</div>

这是Input processing 模块中的Inertial Matrix (hull and payload) in CO子模块

<div align="center">
<img width="1194" height="674" alt="image" src="https://github.com/user-attachments/assets/f50ee48c-2c52-4903-af3e-814bc7e2e9d9" />
</div>

这是Inertial Matrix (hull and payload) in CO子模块的内部构造，要求你根据接线，提供计算表达式，和Simulnk的构造机理（即如何实现通过simulink实现这个计算表达式的）

# 回答



非常好，这个 **Inertial Matrix (hull and payload) in CO** 子模块，其实是你整个 6DOF 动力学里**最“硬核”、也最容易出错的一块**。
我按你要求，**严格根据截图走线**，把它拆成三层：

> **① 物理原理 → ② 数学公式 → ③ Simulink 构造机理（逐块对应）**

---

# 一、这个子模块“在物理上到底算什么？”

一句话先给结论：

> **它在 CO 坐标系下，构造"船体 + payload"整体的刚体转动惯量矩阵 $I_g$**
> 方法：
> **CG 下惯量 + 两次平行轴定理修正（船体、payload 各一次）**

---

# 二、物理背景：为什么必须“in CO”？

* 船体和 payload 的**本体惯量**，通常都给在 **各自的质心（CG）**
* 但你的 6DOF 动力学方程，是在 **CO（body reference origin）** 写的
* 所以必须把所有惯量 **统一平移到 CO**

👉 这正是**平行轴定理（Parallel Axis Theorem）**

---

# 三、核心数学公式（完整、严格）

设：

* $I_{CG}$：船体（或整体）在 **自身 CG** 下的惯量矩阵
* $m$：船体质量
* $m_p$：payload 质量
* $\mathbf{r}_g$：**船体 CG → CO** 的位移向量
* $\mathbf{r}_p$：**payload → CO** 的位移向量
* $S(\cdot)$：叉乘矩阵（skew-symmetric matrix）

---

## 1️⃣ 平行轴定理（矩阵形式）

对任一刚体：

$$
\boxed{
I_{CO} = I_{CG} + m \, S(\mathbf{r})^\top S(\mathbf{r})
}
$$

而由于：

$$
S^\top S = - S^2
$$

工程中常写为：

$$
I_{CO} = I_{CG} - m \, S(\mathbf{r})^2
$$

---

## 2️⃣ 你这个模块的**完整表达式**

根据截图结构，这个模块算的是：

$$
\boxed{
I_g = I_{g,CG} - m \, S(\mathbf{r}_g)^2 - m_p \, S(\mathbf{r}_p)^2
}
$$

⚠️ 注意：

* **两个"−"不是笔误**
* Simulink 里用的是 $-S^2$ 的形式

---

# 四、逐块对照 Simulink 构造（你截图里的每一根线）

下面我完全“照着你图来翻译”。

---

## 🧩 1. `Ig_CG`（左上）

```
Ig_CG  ─────────────►  +
```

### 含义

* **船体（或 hull+payload 的基准）在自身 CG 下的转动惯量矩阵**
* 通常来自 CAD 或参数文件

### 数学对应

$$
I_{g,CG}
$$

---

## 🧩 2. 上半部分：船体 CG → CO 的平移修正

### (1) `rg` → `Smtrx`

```
rg ─► Smtrx ─► S(rg)
```

### 含义

构造叉乘矩阵：

$$
S(\mathbf{r}_g) = \begin{bmatrix}
0 & -z & y \\
z & 0 & -x \\
-y & x & 0
\end{bmatrix}
$$

---

### (2) `Matrix Multiply`

```
S(rg)  ×  S(rg)
```

得到：

$$
S(\mathbf{r}_g)^2
$$

---

### (3) 乘以 `m`

```
S(rg)^2 ─► × m
```

得到：

$$
m \, S(\mathbf{r}_g)^2
$$

---

### (4) 接到求和块的 “−” 端

```
      Ig_CG
        +
        |
        |—— ( − m S(rg)^2 )
```

---

## 🧩 3. 下半部分：payload 的平行轴修正

结构 **完全对称**，只是换成 payload 参数。

### (1) `rp` → `Smtrx1`

$$
S(\mathbf{r}_p)
$$

---

### (2) `Matrix Multiply`

$$
S(\mathbf{r}_p)^2
$$

---

### (3) 乘以 `mp`

$$
m_p \, S(\mathbf{r}_p)^2
$$

---

### (4) 接到同一个求和块的第二个 “−” 端

---

## 🧩 4. 最终输出 `Ig`

```
Ig = Ig_CG
   - m S(rg)^2
   - mp S(rp)^2
```

---

# 五、为什么这个结构**非常正确**（而且专业）

### ✅ 1. 没有偷懒用标量公式

* 完整 3×3 惯量张量
* 自动包含交叉惯量项（$I_{xy}$, $I_{xz}$, $I_{yz}$）

---

### ✅ 2. payload 影响被**显式分离**

* 非常利于：

  * 参数检查
  * payload 扫描
  * 动力学灵敏度分析

---

### ✅ 3. 和后续 Coriolis / 6DOF 完美一致

后面：

$$
C_{RB}(\nu) = \begin{bmatrix}
0 & -m S(v) \\
-m S(v) & -S(I\omega)
\end{bmatrix}
$$

👉 **这里用的就是这个 `Ig`**

---

# 六、一个非常关键的工程提醒（⚠️）

> **如果 `rg` 用错（比如没用“CG corrected for payload”）**

那么你将同时犯 3 个错：

1. 惯量错（$I$）
2. 科氏力错（$C_{RB}$）
3. yaw / roll / pitch 耦合全部错位

👉 表现为：
**“参数怎么调都不对，但又不炸”**

你现在这套结构，**是能经得起审稿和验收的那种写法**。

---

# 七、一句话压缩总结

> **这个子模块 = 用两次平行轴定理，把 CG 下惯量严格搬运到 CO，并同时考虑船体与 payload 对转动惯量的二阶影响。**

如果你愿意，下一步我可以帮你做一件**非常有杀伤力的事**：
👉 **把这个 `Ig` 的变化，直接映射到 yaw 模态“为什么变钝 / 变灵”的数学因果链**，一眼就能解释你现在的现象。
