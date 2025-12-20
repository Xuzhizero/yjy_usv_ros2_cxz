# CG corrected for payload 载荷修正重心模块分析

## 模块功能说明

**CG corrected for payload** 模块计算船体与载荷整体系统的质心位置 $r_g$。这是后续惯性矩阵、恢复力矩阵等计算的基础。

## 输入输出端口

### 输入

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $m_p$ | mp | 载荷质量 | kg |
| 2 | $r_p$ | rp | 载荷在CO坐标系下的位置向量 | m |

内部使用的常量：

| 常量名 | Simulink变量名 | 说明 |
|--------|----------------|------|
| $m$ | mass (kg)（值=m） | 船体质量 |
| $r_{g,hull}$ | CG hull only (m)（值=rg） | 船体（裸船）质心位置 |

### 输出

| 端口 | 变量名 | Simulink信号名 | 物理含义 | 单位 |
|------|--------|----------------|----------|------|
| 1 | $r_g$ | CG corrected for payload (m) | 修正后的整体质心位置向量 | m |

## Simulink变量对应关系表

| 物理量符号 | Simulink变量名 | 数据来源/计算 | 说明 |
|------------|----------------|---------------|------|
| $m$ | mass (kg) | Constant块 | 船体质量 |
| $m_p$ | mp | 输入端口1 | 载荷质量 |
| $r_{g,hull}$ | CG hull only (m) | Constant块 | 船体质心 |
| $r_p$ | rp | 输入端口2 | 载荷位置 |
| $m \cdot r_{g,hull}$ | Product输出 | $m \times r_{g,hull}$ | 船体一阶矩 |
| $m_p \cdot r_p$ | Product1输出 | $m_p \times r_p$ | 载荷一阶矩 |
| $m + m_p$ | Add2输出 | 总质量 | 分母 |
| $m \cdot r_{g,hull} + m_p \cdot r_p$ | Add1输出 | 总一阶矩 | 分子 |
| $r_g$ | Divide1输出 | 质心公式结果 | 最终输出 |

## 核心计算公式

$$
\boxed{
\mathbf{r}_g = \frac{m \cdot \mathbf{r}_{g,\text{hull}} + m_p \cdot \mathbf{r}_p}{m + m_p}
}
$$

---

# 详细说明

<p align="center">
<img width="1545" height="152" alt="image" src="https://github.com/user-attachments/assets/c246801b-1c43-4e3c-86f0-8889fb7193fe" />
</p>

<p align="center">
<img width="922" height="340" alt="image" src="https://github.com/user-attachments/assets/80c8c524-db03-4c59-99f6-42d880645f3b" />
</p>

很好，这个子模块其实**非常"干净"**，它在 Simulink 里做的事情与经典力学中的**质心合成公式**完全一致，没有任何经验项或隐藏假设。下面我按**物理 → 数学 → Simulink 结构逐一对齐**来说明。

---

## 一、rg 是什么物理量？

**rg = CG corrected for payload (m)**
表示：

> **"船体 + 载荷（payload）整体系统的质心位置"**

而且是**相对于同一个参考原点（CO）**表达的位移向量（通常只在 surge 方向非零，但数学上是向量）。

---

## 二、rg 的物理计算机理（核心思想）

这是一个**典型的多刚体质心合成问题**：

* 船体本体：

  * 质量：`m`（mass）
  * 质心位置：`rg_hull`（你图里标成 *CG hull only*）
* 载荷 payload：

  * 质量：`mp`
  * 位置：`rp`

👉 **合成质心 = 质量加权平均**

---

## 三、数学表达式（唯一正确的公式）

如果把所有量都表达在 **同一参考点 CO** 下，那么：

$$
\boxed{
\mathbf{r}_g = \frac{m \cdot \mathbf{r}_{g,\text{hull}} + m_p \cdot \mathbf{r}_p}{m + m_p}
}
$$

其中：

* $\mathbf{r}_g$：**最终输出的 CG corrected for payload**
* $m$：船体质量
* $\mathbf{r}_{g,\text{hull}}$：船体"裸船"质心
* $m_p$：payload 质量
* $\mathbf{r}_p$：payload 在 CO 下的位置

> 这不是近似式，也不是工程经验式，而是**严格的质心定义**。

---

## 四、Simulink 模块逐块对应解释（对照你截图）

我直接把你图里的每一块翻译成数学操作。

---

### ① 上支路：船体本体贡献

```
m ──► × ──┐
rg_hull ──┘
```

对应数学项：

$$
m \cdot \mathbf{r}_{g,\text{hull}}
$$

---

### ② 下支路：payload 贡献

```
mp ──► × ──┐
rp ────────┘
```

对应数学项：

$$
m_p \cdot \mathbf{r}_p
$$

---

### ③ 中间求和：总一阶矩（质量 × 位置）

```
(m·rg_hull) + (mp·rp)
```

数学上是：

$$
\sum m_i \mathbf{r}_i
$$

---

### ④ 右下角：总质量

```
m + mp
```

数学上是：

$$
m_{\text{total}}
$$

---

### ⑤ 最右侧：除法（质心定义）

```
(m·rg_hull + mp·rp) ÷ (m + mp)
```

输出即：

$$
\mathbf{r}_g
$$

---

## 五、这个 rg 在整个动力学模型中的"地位"

这是一个**极其关键的中间量**，后续至少影响：

### 1️⃣ 刚体惯性矩阵（平行轴定理）

$$
M_{RB}^{CO} = \begin{bmatrix}
m I_3 & -m S(\mathbf{r}_g) \\
m S(\mathbf{r}_g) & I_O
\end{bmatrix}
$$

> 只要 rg 偏离 CO，**yaw / roll / pitch 惯量都会被放大**

---

### 2️⃣ 科氏力矩阵 $C_{RB}$

刚体科氏项里显式含有 $S(\mathbf{r}_g)$，rg 一变，科氏力结构立刻变。

---

### 3️⃣ 静水恢复力矩阵（通过 CG–CF 相对位置）

rg 影响 GM、回复力矩方向与幅值。

---

## 六、几个非常重要但容易忽略的结论

### ✅ 1. 这是"唯一合理"的做法

这个模块**没有任何调参空间**，也不应该有。

如果有人试图：

* 手动改 rg
* 或绕开 payload 合成

那一定会在惯性矩阵、yaw 阻尼、稳态响应上**出现系统性错误**。

---

### ✅ 2. payload 位置比质量更"危险"

因为惯性里有：

$$
I \sim m \cdot r^2
$$

* payload 质量翻倍 → 惯量翻倍
* payload 距离翻倍 → 惯量 **四倍**

👉 **这正是你之前 yaw 动不动的根源之一**

---

### ✅ 3. 这个模块是"物理正确性锚点"

只要：

* 这个模块对
* 惯性矩阵模块对
* 坐标原点选得合理

那么 **yaw / roll / pitch 的"量级"一定是可解释的**，而不是玄学。

---

## 七、一句话总结（给你压缩成工程级结论）

> **`CG corrected for payload` 子模块做的事情只有一件：
> 用质量加权平均，把 payload 合法地"并入"船体质心，
> 得到后续所有动力学计算唯一可信的 `rg`。**

如果你愿意，下一步我可以帮你 **直接把这个 rg 在 yaw 失效链条里的作用画成一条"因果路径图"**，把你现在遇到的问题一次性"钉死"。
