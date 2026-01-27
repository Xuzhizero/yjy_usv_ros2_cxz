# Simulink模型参数解析

本文档用于说明本项目 Simulink 动力学模型的关键参数来源、加载方式，以及当 `.slx` / `.mat` 更新时应如何同步更新文档。

## 1. 当前版本（2026-01-27）

- **模型文件**：`src/ROS2_simulink_UE5_260106.slx`
- **变量文件**：`src/large_vessel_params_260106.mat`
- **模型概述**：[`Overview_Model.md`](Overview_Model.md)
- **变量/参数表**：[`仿真变量参考.md`](仿真变量参考.md)

> 说明：`large_vessel_params_260106.mat` 中既包含传统的船体/水动力参数（如 L、B、nabla、I_T/I_L、k_pos/k_neg、l1/l2），也包含仿真/接口相关变量（如 `dt`、`img_matrix`、`xlimits`、`waypoints_m` 等）。文档更新时以该 `.mat` 为“真值源”。

## 2. 参数与文档的对应关系（推荐阅读路径）

- **几何/静水/水动力参数的来源与获取方法**：[`船体水动力参数获取方法.md`](船体水动力参数获取方法.md)
- **推进器与推力系数**：`Subsystem/Vessel Platform/Thrust` → [`Overview_thrust.md`](subsystem_748/vessel_platform/thrust/Overview_thrust.md)
- **输入处理（GM、吃水等）**：`Subsystem/Vessel Platform/Input Processing` → [`Overview_input_processing.md`](subsystem_748/vessel_platform/input_processing/Overview_input_processing.md)
- **惯性参数**：[`惯性参数说明`](subsystem_748/vessel_platform/inertials/Overview_inertials.md)

## 3. 更新流程（当 .slx / .mat 发生变化）

### 3.1 更新 .slx（模块与接线）

目标：更新模型概述、顶层/关键子系统结构，以及（需要时）接线表。

参考：[`Overview_Model.md`](Overview_Model.md) 中的“接线表导出脚本”章节（MATLAB 导出到 CSV / Excel）。

### 3.2 更新 .mat（参数数值）

目标：将 `.mat` 中的最新数值同步到文档（通常是 `仿真变量参考.md`、`船体规格参数.md` 以及与 GM/吃水/推力等相关的章节）。

参考：[`仿真变量参考.md`](仿真变量参考.md) 中的“从 MATLAB 工作区导出变量”脚本（导出 `variables.md` 后再人工或脚本化同步到文档）。

## 4. 常见注意事项

- **符号约定**：推进器横向杠杆臂 `l1/l2` 的正负号以模型坐标系约定为准（当前文件为 \(l_1=-1.005\) m，\(l_2=1.005\) m）。
- **I_T / I_L 单位**：在 GM 相关计算中，`I_T` 与 `I_L` 通常代表水线面面积二次矩（单位 \(m^4\)），并以 \(BM_T=I_T/\\nabla\)、\(BM_L=I_L/\\nabla\) 的形式进入稳性计算。

