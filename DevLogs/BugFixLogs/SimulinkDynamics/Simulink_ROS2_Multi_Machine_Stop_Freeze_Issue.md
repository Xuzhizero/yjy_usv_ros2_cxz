# Simulink 多机 ROS2 仿真 Stop 卡死问题分析

## 问题描述

为什么开启多机仿真以后，如果订阅端没有停止订阅，就在simulink中先stop仿真，结果会导致simulink卡死？

---

## 🔥 一、核心现象

出现这种 **Simulink "卡死"现象（先 Stop 仿真 → 模型不响应、甚至卡死）**，在 **开启多机/分布式仿真（Multiple Machine Simulation with ROS2 Publish/Subscribe）** 的情况下，很常见，本质上是由于 **ROS2 通信层和 Simulink 的执行线程无法优雅终止正在进行的订阅/发送行为** 所导致的，而不是 Simulink 界面的问题。

**具体表现：**

> 在 Simulink 中启动了一个含 ROS2 Subscriber 的模型，然后在 Ubuntu 上启动了多个 ROS2 节点（多机/多上下文）
>
> 如果 **订阅端没有正确停止订阅**
>
> 然后在 Simulink 里点击 Stop
>
> **Result**: Simulink 卡死、甚至整个 MATLAB 不响应

这种现象常发生在含有长期运行、异步接收 ROS2 消息循环的模型是真的阻塞了执行线程。

---

## 🔍 二、为什么会卡死？

### 1️⃣ ROS2 Subscriber 的内部通信线程一直活跃

在 ROS2 的底层架构里：

* ROS2 订阅者是基于 **DDS 运行的接收线程**
* 当有订阅者存在时，它内部会不停地等待数据、调用回调、维持 QoS 连接

如果模型内部的订阅者线程 **还在运行**（没有被优雅地停止），Simulink 就会在 Stop 时 **等待该线程退出**。

因为 ROS2 订阅者正在阻塞（等待新的数据），Simulink 主线程就会卡住在某个阻塞点上，导致界面不响应。

> **本质原因**：Simulink 在 Stop 时要清理正在运行的 ROS2 Subscriber transport / middleware 链接，而底层通信没有正常退出，所以 Stop 事件被阻塞。

这和 ROS2 话题通信不能被强制取消、持续等待网络层反馈有关。系统会保持订阅者 alive，除非：

✔ 订阅者显式 "shutdown"
✔ Publisher 关闭话题
✔ 或者 TCP/DDS 超时

否则订阅者线程会长期阻塞在网络等待状态，Simulink Stop 就等待这个阻塞线程结束 → 卡死。

> StackOverflow/MathWorks 官方论坛也有类似 ROS2 Subscriber 在 Simulink 中 Transport 停止问题的讨论（虽然没有直接解决方案指出，但问题症状一致）。 [参考链接](https://jp.mathworks.com/matlabcentral/answers/1836848-simulink-ros2-subscriber-transport-stopped?utm_source=chatgpt.com)

---

### 2️⃣ 多机/多上下文通信更难优雅退出

当你在多个机器上启动 ROS2 Publish 和 Subscribe 时：

* DDS 的节点发现（Discovery）使得通信链路长时间处于活跃状态
* 即使你在单机上 Stop Simulink 模型，DDS 底层可能仍试图与其他节点保持连接
* Simulink 在释放 Subscriber transport 时会等待 DDS 释放资源，而 DDS 可能会因为网络交互延迟 → 阻塞

这种情况比单机单节点的情况"更难终止"。

---

### 3️⃣ Simulink 与 MATLAB 的 Stop 机制不是"强退出"

当你点击 Stop 时：

✔ Simulink 会尝试优雅停止运行
✔ 等待所有 MATLAB Function / ROS2 Blocks 执行完当前时间步
✔ 等待所有通信 transport 线程退出（Publisher/Subscriber）
✔ 再释放资源

在含 ROS2 Subscriber 时，Simulink 可能会卡在：

> 等待底层网络线程停止 + 等待 ROS2 Transport 停止

这就导致你描述的 "Stop 之后卡很久甚至卡死"。

---

## 🧠 三、解决建议

这个问题不能单靠 Simulink 调整界面设置来解决，因为根源在于 **ROS2 话题 transport thread 尚未结束** 导致 Stop 阻塞。建议如下：

---

### 🧩 解决方案 A — 先显式停止订阅

在分布式仿真里，**先让订阅端先显式退出节点/订阅关系**，再点击 Stop。

比如：

1. **如果你是在 Simulink 内部用 ROS2 Subscribe Block**

   * 使用一个 Flag 或触发机制触发订阅器退出（需要自定义逻辑）

2. **在 ROS2 Python/C++ 端**

   启动通讯的节点后，先退出订阅者节点 → 然后再在 Simulink Stop

这样 Simulink Stop 时不会等待阻塞线程。

---

### 🧩 解决方案 B — 控制 Subscriber 在 Stop 时优雅关闭

你可以在 Simulink 模型的生命周期内编写逻辑，确保：

✔ ROS2 Subscriber 分离
✔ ROS2 Node shutdown
✔ 或将 Subscriber 停止

通常可以借助 MATLAB Function + ROS2 API 手动调用：

```matlab
rosshutdown;
```

或者：

```matlab
node.destroy_node();
rclcpp.shutdown();
```

在 Stop 事件前执行，这样 Simulink stop 时不会再阻塞。

---

### 🧩 解决方案 C — 使用 Simulink 事件决策优先停止 Subscriber

如果你能在模型里添加一个控制 Subscriber 是否激活的开关（比如 Alive/Stop），在 Stop 时让订阅处理提前退出。

（注意：Simulink 的 Stop 是异步的，所以需要提前判断退出条件）

---

### 🧩 解决方案 D — 避免无限循环等待

在 Subscriber 框里（如果是自定义 Subscriber），确保：

✔ 加上 timeout 或最大循环次数
✔ 不要 blocks on spin forever
✔ 在 no data 时及时返回

这样 Stop 时线程不会长时间阻塞。

---

## 📌 小结

❗ **问题原因：** ROS2 Subscriber Internal Thread 在等待网络数据，没有正常停止，导致 Simulink Stop 阻塞而卡住。这是由于 ROS2 底层 transport + DDS discovery 模式 的特性，与 Simulink 的 Stop 执行流程冲突。

❗ **核心原理：** Simulink Stop 必须等所有正在执行的线程结束，尤其是 Subscriber transport，而 ROS2 Subscriber 可能会 blocked on network IO。

🔧 **关键解决方向：**

* 先停止订阅/销毁节点再 Stop
* 在 Stop 前显式调用 shutdown
* 避免无限阻塞的 subscribe spin

这样 Simulink 在 Stop 时不会被 DDS/ROS2 阻塞线程卡住。

---

## 👉 额外说明

如果你正在用 **官方 ROS2 Subscribe Block 而不是自定义 MATLAB Function**，请确认：

✔ Subscribe Block 不使用 blocking 回调
✔ 有设置合理的 timeout
✔ 不要在 MATLAB Function 里用 while true spin，没有 break 条件

这些细节也会影响 Stop 时是否卡死。

---

## 参考资料

- [MathWorks - Simulink ROS2 subscriber transport stopped](https://jp.mathworks.com/matlabcentral/answers/1836848-simulink-ros2-subscriber-transport-stopped?utm_source=chatgpt.com)

---

**文档创建日期：** 2026-01-15
**最后更新日期：** 2026-01-15
