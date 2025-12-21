# 图片文件说明 (Image Files Documentation)

本目录存放 Simulink 与 UE5 性能分析相关的图片文件。

## 需要保存的图片

请将以下 4 张图片保存到当前目录：

### 1. simulink_pure_test_1.png
**描述**：Simulink 纯环境测试配置图 1
**内容**：显示 VesselCommand 和 VesselStates 模块，以及各种信号连接（Yaw Angle, x, y, speed, Yaw Rate），右上角显示数值 -91.47

### 2. simulink_pure_test_2.png
**描述**：Simulink 纯环境测试配置图 2
**内容**：类似的 Simulink 配置，右上角显示数值 112.4，包含 "To Video Display" 连接，下方显示海洋场景预览图

### 3. realtime_test_stopwatch_start.png
**描述**：实时测试开始时间（秒表截图）
**内容**：手机秒表应用截图，显示时间 12:01.20

### 4. realtime_test_stopwatch_end.png
**描述**：实时测试结束时间（秒表截图）
**内容**：手机秒表应用截图，显示时间 12:03.86

## 图片用途

这些图片用于证明：
- 在**不连接 UE5 的纯 Simulink 环境**下，系统可完美实现 **1:1 实时仿真调速**
- 逻辑计算与理论时间高度匹配
- 通过秒表记录的真实时间（约 2分43秒）与仿真时间完全一致

## 引用位置

这些图片在 `Simulink_UE5_Performance_Analysis.md` 文档的 "2.1 纯 Simulink 环境表现" 章节中被引用。

---

**创建日期**: 2025-12-21
