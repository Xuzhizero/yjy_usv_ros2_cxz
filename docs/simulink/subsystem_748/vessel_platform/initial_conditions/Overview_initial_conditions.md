# Initial Conditions 初始条件模块分析

## 模块功能说明

**Initial Conditions（初始条件）**模块用于设定仿真开始时船体的状态，包括初始速度和初始位置/姿态。由于6DOF求解器通过对加速度积分得到速度和对姿态变化率积分得到位置，为使仿真以期望的初态开始，必须提供初始积分值。Initial Conditions模块正是提供这些初始值给积分器。

在Simulink实现中，通常有两种方式注入初始条件：

1. **Integrator模块内部参数：** 直接在Integrator (1/s)块中填写初始值向量。
2. **外部初始条件端口：** 使用Initial Condition模块或IC块，将初始值作为信号通过加法节点注入积分器。

根据模型结构，Vessel Platform内包含两个Initial Condition模块（SID 333和 SID 350）。这可能对应于：

- **Initial Condition (SID 333)：** 提供初始速度和角速度$(u_0, v_0, w_0, p_0, q_0, r_0)$。
- **Initial Condition1 (SID 350)：** 提供初始位置和姿态角$(x_0, y_0, z_0, \phi_0, \theta_0, \psi_0)$。

这些初始值通常由仿真场景（Scenario模块）或用户设置。例如，用户希望模拟船从静止、特定位置开始，则$u_0=\dots=r_0=0$，位置取相应坐标，姿态$\psi_0$设置初始航向角等。

Initial Conditions模块确保在$t=0$时，6DOF模块的状态等于设定值，而非默认0。这对于仿真准确性很重要，尤其是仿真短时间响应或再现特定情景。

## 输入输出端口

### 输入

可能有来自上层Scenario或参数的设定值输入。如果Initial Condition模块被实现为简单的Constant常量块，则无显式输入，用户通过编辑常量值得到想要的初态。如果实现为可配置模块，Scenario模块可能在仿真开始前（或通过脚本）将值传递给它。

### 输出

- **Initial Velocity (初始速度)**：6×1向量，包含$(u_0, v_0, w_0, p_0, q_0, r_0)$，作为Integrator（速度积分）块的初始值输入。

- **Initial Position/Attitude (初始位置/姿态)**：6×1向量，包含$(x_0, y_0, z_0, \phi_0, \theta_0, \psi_0)$，作为Integrator（位置积分）块的初始值输入。

在Simulink 6DOF模块实现中，这些输出很可能通过Mux合成为一个18×1向量传递，也可能分别进入两套Integrator。

## 注意事项

- 设定初始条件时应确保自洽。如初始位置在水面上，则$z_0$应对应吃水深度；初始姿态若非水平，需要与其它模块（如Hydrostatic）的平衡状态匹配，否则一开始可能产生静力不平衡力矩。

- 初始速度不应违背物理限制，例如通常船初始转速$p_0,q_0$取0除非已经在旋转状态。

- 若不提供初始条件，Simulink默认以0初始化，会导致比如本应有前进速度的仿真错误地从静止算起。因此应明确设置。

## 核心公式

Initial Conditions模块本身没有计算，仅输出设定值。不过从状态角度，其对应的数学意义：

$$\mathbf{\nu}(0) = \mathbf{\nu}_0, \qquad \mathbf{\eta}(0) = \mathbf{\eta}_0$$

其中$\mathbf{\nu}_0$和$\mathbf{\eta}_0$分别是初始时刻的速度和位置/姿态矢量。

积分器因此满足：

$$\mathbf{\nu}(t) = \int_0^t \dot{\mathbf{\nu}}\, dt + \mathbf{\nu}(0)$$

$$\mathbf{\eta}(t) = \int_0^t \dot{\mathbf{\eta}}\, dt + \mathbf{\eta}(0)$$

这些初始条件的选择不改变系统动力学本身，但决定了解的特解。例如，一个零输入系统，如果$\mathbf{\nu}(0)\neq0$，则会出现纯惯性运动。

通常在稳态模拟中，可能希望初始条件符合稳态（如$\phi_0=0$无倾斜）。而在特定情景如倾覆试验，可能赋予一定初始角速度或倾角来观察动态恢复。

## 总结

Initial Conditions模块保证仿真从正确的状态出发，为后续动态计算提供恰当的起点。正确配置初态对仿真可信度尤其关键，因为错误的初态可能导致非物理的瞬态响应干扰我们对结果的判断。
