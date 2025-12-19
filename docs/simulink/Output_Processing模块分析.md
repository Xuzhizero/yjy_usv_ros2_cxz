# Output Processing模块分析

<div align="center">

![Output Processing模块结构图1](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752664630220-7ed267a2-5f19-4ff0-a844-32f819d16171.png)

</div>

<div align="center">

![Output Processing模块结构图2](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752664645585-0abac352-87ee-4587-a76d-16d5951cdb1f.png)

</div>

## 模块概述

"Output Processing"子系统的核心作用就是把 6DOF 模块吐出来的原始状态向量（位置、姿态、线速度、角速度、加速度、角加速度）进行"善后处理"——拆包、单位换算、进一步的衍生量计算（航速、航向、地理坐标、四元数、DCM）——最后按一个统一的 Bus（VesselStates）打包输出。

下面按功能块依次说明它的机理和每一路的输入／输出含义。

---

## 1. 原始状态打包

### 输入
- X 向量（18×1）

$$
[u,v,w,p,q,r, x,y,z, \phi,\theta,\psi, Acc_X,Acc_Y,Acc_Z, AngularAcc_X,AngularAcc_Y,AngularAcc_Z]
$$

### 处理
- 用一个 **Bus Creator** 把这 18 路信号按顺序映射到输出总线 `VesselStates`

### 输出
- `VesselStates.SurgeVelocity` … `VesselStates.YawRate`（线速、角速）
- `VesselStates.x, y, z`
- `VesselStates.RollAngle_rad, PitchAngle_rad, YawAngle_rad`
- `VesselStates.Acc_X`…`AngularAcc_Z`

---

## 2. 角度单位转换

### 输入
- $\phi, \theta, \psi$（rad）

### 处理
- 串一个 **Rad to Deg**（Gain = 180/π）

### 输出
- `VesselStates.RollAngle_deg`、`PitchAngle_deg`、`YawAngle_deg`

---

## 3. 航速与航向

### 3.1 航速（speed）

**输入**：`u_mps, v_mps`

**处理**：

$$
speed = sign(u) \times \sqrt{u^2 + v^2}
$$

其中先算速度大小 $\sqrt{u^2+v^2}$，再乘以 $sign(u)$ 保留正负方向。

**输出**：`VesselStates.speed`（signed m/s）

### 3.2 航向（course）

**输入**：`u_mps, v_mps`

**处理**：
- 用 **Atan2(v, u)** 块算出航向角（rad）
- 再 Rad→Deg 转换

**输出**：`VesselStates.course`（rad & deg 两路）

---

## 4. 地理坐标（Latitude / Longitude / Altitude）

### 输入
- `x_m, y_m`：ENU 坐标偏移（米）
- 初始位置经度、纬度 `lat0_deg, lon0_deg`（来自 IC 块）

### 处理
简化地球模型：

$$
\Delta lat = \frac{y_m}{R_{earth}} \times \frac{180}{\pi}
$$

$$
\Delta lon = \frac{x_m}{R_{earth} \cos(lat0)} \times \frac{180}{\pi}
$$

- 加到初始经纬度上
- Altitude 一般固定为 0 或者 IC 给的初始高度

### 输出
- `VesselStates.latitude_deg`、`longitude_deg`、`altitude_m`

---

## 5. 姿态矩阵 DCM & 四元数

### 输入
$\phi, \theta, \psi$（rad）

### 处理
- 用 Simulink 自带 "DCM (ZYX)" 块算出从船体系到地球系的方向余弦矩阵 `DCM_be`
- 用同样的输入再经 "Angles to Quaternion" 块得四元数 $[q_0,q_1,q_2,q_3]$

### 输出
- `VesselStates.DCM_be`（3×3 矩阵）
- `VesselStates.Quaternion`（4×1 向量）

---

## 6. 加速度与角加速度

### 输入
`Acc_X,Acc_Y,Acc_Z, AngularAcc_X,Y,Z`

### 输出
直接打包到 `VesselStates` 对应的字段，无额外处理。

---

## 整体机理小结

1. **拆包**：先把 6DOF 的原始信号整个拆出来，映射到内部命名一致的端口。

2. **衍生**：对角度、速度、航向、地理坐标、DCM/四元数做必要的数学变换和单位换算。

3. **打包**：再把所有通道按照 `VesselStates` Bus 的定义整齐地打包输出，供上层控制器、可视化或记录使用。

如此一来，上游只需关注一个 `VesselStates` bus，就能拿到完整的实时船体状态及各种常用衍生量，大大简化了后续信号路由和管理。
