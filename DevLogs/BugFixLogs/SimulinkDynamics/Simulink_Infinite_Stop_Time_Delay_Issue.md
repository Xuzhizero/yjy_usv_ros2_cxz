# 解决 Simulink 无限仿真 Stop 延迟响应问题

## 问题描述

在 Simulink 里运行仿真如果时间设为 **`inf`（无限）**，那么点击 Stop 后仿真不会立即停止，通常需要十几秒才能完全停下来。

---

## 问题原因分析

### 1. 无限仿真不自动结束

当仿真设置了：

```
Stop Time = inf
```

Simulink 会认为仿真是一个"**实时运行的过程**"（常见于硬件-in-the-loop 或长期运行仿真）。
这种情况下：

* 模拟器不会在某个固定时间点停止运行
* 即使点击 Stop，也要等当前计算周期完整地结束才能退出
* 如果模型很复杂、计算量大，Simulink 需要完成当前全部函数调用链才返回 Stop —— 这就造成了 **延迟响应**

这与设为固定 Stop Time (比如 10) 的情况不同，固定 Stop Time 的仿真在时间到后内部可以提前判断并迅速结束。

---

### 2. 模型内部阻塞或长计算步骤

即便点了 Stop，Simulink 也只能等当前正在执行的模块代码执行完毕后才真正停止。如果模型中有：

* MATLAB Function 处理大数组
* ROS2 Publish / Subscription 等通信块
* 复杂的图像处理或循环
* 真实硬件接口（如远程依赖）

这些代码往往不会立即中断，必须完整执行完当前时间步：

```
SimTime: t → t + Δt
```

Simulink 才响应 Stop 按钮，这就是为什么 **要过十几秒才停下来**。

---

### 3. Real-time Pacer / ROS2 Exec

如果模型开启了实时执行模式（例如用 Real-Time Pacer 或者有 ROS2 Blocking Publish/Subscribe），执行线程可能会 **等待网络延迟、DDS 响应或计算完成** 才返回，这也会造成：

> "我点了 Stop，但要等一会才真的停下来"

---

## 解决方案

### 方法 1 — 不用 `inf`，用一个大但有限的 Stop Time

例如：

```
Stop Time = 1e6
```

而不是：

```
Stop Time = inf
```

这样 Simulink 内部能预测仿真将结束，因此 Stop 按钮响应更快。

**优先级：⭐⭐⭐⭐⭐（推荐首选）**

---

### 方法 2 — 开启 "Fast Restart"

在 Model Settings → Simulation → **Fast Restart = Enabled**

可以加速重启和 Stop 的响应。

**优先级：⭐⭐⭐**

---

### 方法 3 — 减少每个时间步的计算量

例如：

* 降低 solver 步长
* 减少大型 MATLAB Function 内的循环
* 在 ROS2 Publish/Subscribe 前执行一次批处理而不是逐帧

这样当前仿真步执行得更快，Stop 响应更快。

**优先级：⭐⭐⭐⭐**

---

### 方法 4 — `Ctrl+C` 强制停止

如果 Stop 等太久，可以在 MATLAB 命令窗口按：

```
Ctrl + C
```

这会强制终止仿真循环，但注意可能会丢失数据。

**优先级：⭐⭐（应急方案）**

---

### 方法 5 — Enable Immediate Stop in Simulation Settings

在 Model Settings → Solver → 有些 Simulink 版本提供：

```
Enable immediate stop (checkbox)
```

勾选后 Stop 会对正在执行的时间步更敏感、更快响应。

**优先级：⭐⭐⭐⭐**

---

### 方法 6 — 用暂停点检查当前状态

如果仿真比较复杂，可以在关键位置加一个：

```matlab
if stopRequested
    return
end
```

在 MATLAB Function 中检查是否需要提前退出。

**优先级：⭐⭐**

---

## 技术原理

### 为什么 Simulink 不能立刻停止

Simulink 内核是按照"**时间步 + 连续/离散执行图**"来运行的：

1. 先计算当前时间步所有模块
2. 再推进到下一个时间
3. Only then 可以响应 Stop 界面

这就意味着 Stop 只能在 **一个完整时间步结束后才能生效**。
如果这个时间步执行很久（比如 ROS2 Publish 发送图像、MATLAB Function 处理图像数组等），就会造成延迟。

---

## 响应速度对比

| 情况                              | 响应 Stop 快吗？   |
| ------------------------------- | ------------- |
| Stop Time 有限                    | 🟢 一般很快       |
| Stop Time = inf                 | 🔴 需要等当前时间步结束 |
| 模型含重计算                          | 🔴 可能延迟明显     |
| Real-time Pacer / Publish Block | 🟡 可能取决于回调响应  |

---

## 推荐操作（按优先级）

1. ⭐⭐⭐⭐⭐ 把 Stop Time 设成足够大而不是 `inf`
2. ⭐⭐⭐⭐ 提升模型执行效率，减少大数据处理在单步内
3. ⭐⭐⭐⭐ 在设置里打开 Immediate Stop / Fast Restart
4. ⭐⭐ 必要时用 Ctrl+C 强制终止

---

## 进一步诊断

如需进一步分析具体模型中哪些部分导致 Stop 响应慢，可以：

* 检查 MATLAB Function 内部循环耗时
* 分析 ROS2 Publish callback latency
* 使用 Simulink Profiler 查看各模块执行时间
* 检查是否有阻塞式的硬件接口调用

---

## 相关文档

* [Simulink Simulation](https://www.mathworks.com/help/simulink/simulation.html)
* [Configure Simulation](https://www.mathworks.com/help/simulink/ug/configuring-simulation-parameters.html)
* [ROS 2 Toolbox](https://www.mathworks.com/help/ros/)

---

**更新日期：** 2026-01-15
**问题状态：** 已解决
**解决方案验证：** 待实际测试验证
