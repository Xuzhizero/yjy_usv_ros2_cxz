---
title: simulink这边重新第二次仿真时直接UE5崩溃
date: 2025-06-04
source: https://blog.csdn.net/a2213086589/article/details/148433740
tags: [UE5, Simulink, MATLAB, Crash, Debugging, Co-simulation]
category: UE5
---

# Simulink与UE5联合仿真第二次运行崩溃问题

## 文章信息
- **标题**: simulink这边重新第二次仿真时，直接UE5崩溃，然后simulink没有响应
- **作者**: 氩氪氙氡_xuzhi
- **发布日期**: 2025-06-04
- **阅读量**: 1k+ | 点赞: 23 | 收藏: 8

---

## 问题描述

用户在使用MATLAB R2024b与Unreal Engine 5.3进行联合仿真时遇到重现性问题。按照官方推荐顺序（先停止Simulink仿真，再停止UE5播放），首次仿真运行正常，但启动第二次仿真时UE5直接崩溃，Simulink无响应。

**配置信息**：
- MATLAB R2024b
- UE5.3
- MathWorks插件：MathWorksAutomotiveContent、MathWorksSimulation
- 使用Simulink中的3D Scene Configure模块

---

## 解决方案总结

### 尝试过的方法

**实验1** - 清除MEX文件（失败）
> "先后输入clear mex和clear functions"后重新运行仿真，结果仍出现同样问题

**实验2** - 重启UE编辑器（成功）
完全关闭Unreal Editor，让Simulink在下次运行时重新启动，问题解决。

### 推荐方案

1. **临时解决**: 每次仿真后完全重启Unreal Editor
2. **版本验证**: 确保MATLAB、插件与UE5版本完全同步
3. **更新检查**: 安装R2024b最新补丁，可能包含联合仿真稳定性修复
4. **技术支持**: 如持续出现，建议联系MathWorks官方支持

---

## 结论

问题源于UE5编辑器而非Simulink端。暂时采用"重启UE编辑器"的方案规避此问题。
