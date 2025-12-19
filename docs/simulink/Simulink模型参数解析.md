# Simulink模型参数解析

<div align="center">

![参数加载界面1](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752666268451-97fbedaa-ad89-4c3a-896e-69aa5ad36983.png)

</div>

<div align="center">

![参数加载界面2](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752666276846-d0aced1a-fd27-4d15-9e09-395d5507878a.png)

</div>

<div align="center">

![参数加载界面3](https://cdn.nlark.com/yuque/0/2025/png/2408029/1752666285782-94307059-9cd0-47ff-978c-1a5a4befa8f6.png)

</div>

## 提问

为了运行以上Simulink，需要预先加载参数和输入到matlab工作区当中，如图所示。两个结构体（分别为IC和dt）的具体内容也在图中。解析各个参数的含义

## 回答

下面把你在 Workspace 里看到的那些变量，以及两个 struct（`IC`、`dt`）一一列出，说明它们的物理/仿真含义和使用场景。

---

## 一、标量参数

| 名称 | 值 | 含义 |
| --- | --- | --- |
| **L** | 2 | 整船（双体）船体的总长 (m)。 |
| **B** | 1.08 | 整船最大梁宽 (m)。 |
| **m** | 55 | 船体总质量 (kg)。 |
| **Aw_pont** | 0.3750 | 单个浮筒的水线面积 $A_w$ (m²)，等于 $C_{w\_pont} \cdot L \cdot B_{pont}$。 |
| **B_pont** | 0.25 | 单个浮筒的梁宽 (m)。 |
| **Cb_pont** | 0.40 | 浮筒的方形系数（block coefficient），$V_{\text{geom}}/(L \cdot B_{pont} \cdot H)$。 |
| **Cw_pont** | 0.75 | 浮筒的水线面系数（waterplane coefficient），用于计算浮筒水线面积。 |
| **y_pont** | 0.3950 | 单个浮筒中心相对于船体纵向对称面的横向偏移 (m)。 |
| **L1, L2** | –0.3950, 0.3950 | 两个浮筒中心相对于船体质心沿纵向（X 轴）的偏移 (m)。 |
| **rg** | [0.2;0;−0.2] | 船体重心（CG）相对于坐标原点（CO）的偏移向量 $[x,y,z]$ (m)。 |
| **LCF** | –0.2000 | 浮心（center of flotation, CF）相对于 CO 的纵向偏移 $x_{CF}$ (m)。 |
| **Ig_CG** | 3×3 矩阵 | 刚体惯性矩阵 $M_{RB}$ 在 CG（6DOF／CO 前）的 3×3 子块 (kg·m²)。 |
| **Delta_m_L** | 0.2667 | 用于静态配载或横向修整时的"重量差×杠杆"参数 (kg·m)。通常用来计算因载荷偏移产生的配载力矩。 |
| **l_T** | 0.1200 | 初始吃水或修整杠杆长度的猜测值 (m)，用于静水体积-吃水迭代；也可理解为静水模型的初始参数。 |
| **kappa** | 0.1000 | 静水恢复刚度的缩放系数（dimensionless），在非线性恢复力模型里用来调节斜率。 |
| **k_neg, k_pos** | 0.0064, 0.0111 | 对应负滚角和正滚角区域的分段线性恢复刚度 $K_{\phi}$ (N·m/rad)。 |
| **R44, R55, R66** | 0.4320, 0.5000, 0.5000 | heave/roll/pitch 三个方向的阻尼比 ζ，用于计算 $C = 2\zeta M\omega$。 |
| **T_yaw** | 1 | 偏航自然周期 $T_{\text{yaw}}$ (s)，用于计算 $N_r = -I_{zz}/T_{\text{yaw}}$。 |
| **R_switch_m** | 5 | 横流拖曳模型中切换线性/非线性 yaw 模式的速度阈值 (m/s)。 |
| **Umax** | 3.0864 | 波浪阻尼中 surge 方向线性阻尼 $X_u$ 的基准最大速度 (m/s)，用来归一化系数。 |
| **trim_setpoint** | 280 | 航向控制器的目标航向 (deg)。 |

---

## 二、初始条件 Struct：`IC`

该结构体给出了仿真启动时的全船状态初值，字段含义如下：

| 字段 | 初始值 | 含义 |
| --- | --- | --- |
| **u_mps** | 0 | Surge 方向速度 (m/s) |
| **v_mps** | 0 | Sway 方向速度 (m/s) |
| **w_mps** | 0 | Heave 方向速度 (m/s) |
| **p_rps, q_rps, r_rps** | 0,0,0 | Roll/Pitch/Yaw 角速度 (rad/s) |
| **x_m, y_m, z_m** | 0,0,0 | 初始位置 (m) |
| **phi_rad, theta_rad, psi_rad** | 0,0,0 | 初始欧拉角 Roll/Pitch/Yaw (rad) |
| **latitude_deg, longitude_deg** | 51.5033, −0.1201 | 初始地理位置（WGS84 纬度/经度, deg） |

---

## 三、时间步长 Struct：`dt`

该结构体每个字段都是对应某个子系统或传感器的离散更新周期（单位：秒）。例如：

| 字段 | 值 | 含义 |
| --- | --- | --- |
| **TopModel** | 0.05 | 顶层仿真主循环的步长 |
| **vessel** | 0.10 | 船体动力学子系统更新步长 |
| **VelocityController** | 0.10 | 速度控制器更新周期 |
| **YawRateController** | 0.10 | 偏航率控制器更新周期 |
| **HeadingController** | 0.10 | 航向控制器更新周期 |
| **IMU** | 0.10 | 惯性测量单元传感器采样周期 |
| **GPS** | 0.10 | GPS 传感器采样周期 |
| **INS** | 0.10 | 惯导系统解算周期 |
| **map** | 10.00 | 地图/航路点发布周期 |
| **cuboid** | 0.10 | 障碍物管理子系统更新周期 |
| **PublishToUnity** | 0.10 | 与 Unity 可视化接口的数据发布周期 |
| **WaypointFollow** | 0.10 | 航点跟踪逻辑更新周期 |
| **undock, dock** | 0.10 | 起锚/靠泊逻辑更新周期 |
| **StopMode, StandbyMode** | 0.10 | 停船/待机模式逻辑周期 |
| **CollisionAvoidance** | 0.10 | 避碰逻辑更新周期 |
| **dashboard, HMI** | 0.10 | 仪表盘与人机界面刷新周期 |
| **BehaviorPlanner** | 0.10 | 行为规划器更新周期 |
| **AHRS** | 0.10 | 姿态基准系统更新周期 |
| **communication** | 0.10 | 通信链路/遥测更新周期 |

（字段名可能略有不同，但都是"子系统名称 = 更新/采样时间"。）

---

## 总结

在开始仿真前，把上面那些标量和两个 struct `IC`、`dt` 加载到 MATLAB Workspace，Simulink 就能按预期得到初始状态、参数和各子系统的离散步长，保证整个船体动态、控制和传感器模型正常运行。
