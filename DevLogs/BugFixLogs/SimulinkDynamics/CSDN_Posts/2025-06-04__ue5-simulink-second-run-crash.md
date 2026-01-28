---
title: simulink这边重新第二次仿真时直接UE5崩溃
date: 2025-06-04
source: https://blog.csdn.net/a2213086589/article/details/148433740
tags: [UE5, Simulink, MATLAB, Crash, Debugging, Co-simulation, Lifecycle]
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

### 进一步发现

如果 UE5 的蓝图中**没有**与 Simulink 有关的 Actor（或者说没有与 Simulink 做数据交互的 Actor）时，则**不会**出现上述现象。只有存在数据交互 Actor 时才会报错。

### Crash Log

```
LoginId:9192d73e439052396bef95b255534309
EpicAccountId:c2cf773902ad49f0ae8e75a61569b2aa

Unhandled Exception: EXCEPTION_ACCESS_VIOLATION reading address 0x0000000000000000

libmwshared_sim3d_io
UnrealEditor_MathWorksSimulation!ASim3dGetFloat::ReadVectorFloat()
  [C:\TEMP\Bsim3d_2843960_2780\...\Sim3dGetFloat.cpp:44]
UnrealEditor_MathWorksSimulation!ASim3dGetFloat::execReadVectorFloat()
  [C:\TEMP\Bsim3d_2843960_2780\...\Sim3dGetFloat.gen.cpp:26]
UnrealEditor_CoreUObject
UnrealEditor_Engine
...
```

---

## 核心原因分析

这是一个典型的 **Unreal Engine 插件生命周期管理（Lifecycle Management）** 问题，特别是在涉及外部 DLL 通信（如 MathWorks 的 `libmwshared_sim3d_io`）时。

### 从 Crash Log 分析

| 项目 | 内容 |
|-----|------|
| **错误代码** | `EXCEPTION_ACCESS_VIOLATION reading address 0x00...00`（空指针引用） |
| **崩溃位置** | `ASim3dGetFloat::ReadVectorFloat()` |
| **发生时机** | 第二次运行时 |

### 根本原因

当第一次仿真结束（Simulink 停止，UE5 停止）时：
1. Simulink 端断开了连接
2. 但 UE5 内存中的某些**静态指针或句柄（Handle）没有被正确清理或重置**

当启动第二次仿真时：
1. UE5 中的 Actor（`ASim3dGetFloat`）试图立即调用底层的 C++ 函数去读取数据
2. 但此时底层的通信对象可能**尚未重新初始化完成**
3. 或者它仍然持有指向上一次仿真周期的"**僵尸指针**"
4. 导致访问了非法内存（0x0000000000000000）

---

## 解决方案

### 方案一：重启 UE 编辑器（临时方案）

**实验1** - 清除MEX文件（失败）
> "先后输入clear mex和clear functions"后重新运行仿真，结果仍出现同样问题

**实验2** - 重启UE编辑器（成功）
完全关闭Unreal Editor，让Simulink在下次运行时重新启动，问题解决。

**缺点**：每次仿真都要重启 UE5，效率低下。

---

### 方案二：Delay + Boolean Gate（推荐方案）

通过**时序控制（Timing）**解决初始化阶段的崩溃问题，无需重启 UE5。

#### 原理解析：一场"赛跑"

| 选手 | 特性 | 行为 |
|-----|------|------|
| **选手 A**（UE5 的 Tick 循环） | 急性子 | 游戏一开始，Actor 的 Tick 每秒运行几十次，每次都试图调用 `ReadVectorFloat` 拿数据 |
| **选手 B**（MathWorks 插件） | 慢性子 | 第二次运行时需要先清理上一次的残留内存，重新建立连接，然后初始化 C++ 指针 |

**崩溃原因**：选手 A 跑得太快，在选手 B 还没把路铺好（指针还没初始化完）的时候，选手 A 就一脚踩了上去，结果踩空了。

**Delay 的作用**：给选手 A 强行按个暂停键，让它等 2 秒。这时候选手 B 早就把路铺好了，再让选手 A 去跑，就安全了。

#### 蓝图实现步骤

> ⚠️ **警告**：Delay 加的位置非常关键。绝对不能直接在 Event Tick 里连一个 Delay 节点（那样会导致每一帧都卡顿）。必须用**"布尔锁（Boolean Gate）"**的方式来写。

**第一步：定义一个开关变量**

1. 在左侧 Variables 栏，新建一个 **Boolean** 变量
2. 命名为 `bIsSimReady`（或 `bSafeToRead`）
3. **Default Value 必须设为 False**（未勾选）

**第二步：在 BeginPlay 设置定时器**

```
Event BeginPlay
    └─ Delay (Duration = 2.0)
        └─ Set bIsSimReady = True
```

- 如果电脑较慢，可以设置 Duration 为 3.0 更稳

**第三步：在 Tick 添加关卡**

```
Event Tick
    └─ Branch (Condition = bIsSimReady)
        ├─ True  → ReadVectorFloat 及后续逻辑
        └─ False → (什么都不连，留空)
```

#### 完整蓝图结构

```
┌─────────────────────────────────────────────────────────┐
│  Variables:                                             │
│    bIsSimReady : Boolean = False                        │
├─────────────────────────────────────────────────────────┤
│  Event BeginPlay                                        │
│      │                                                  │
│      ▼                                                  │
│  ┌─────────┐                                            │
│  │  Delay  │ Duration = 2.0                             │
│  └────┬────┘                                            │
│       │                                                 │
│       ▼                                                 │
│  ┌──────────────────┐                                   │
│  │ Set bIsSimReady  │ ☑ True                            │
│  └──────────────────┘                                   │
├─────────────────────────────────────────────────────────┤
│  Event Tick                                             │
│      │                                                  │
│      ▼                                                  │
│  ┌─────────┐                                            │
│  │ Branch  │◄─── bIsSimReady                            │
│  └────┬────┘                                            │
│       │                                                 │
│   ┌───┴───┐                                             │
│   ▼       ▼                                             │
│ True    False                                           │
│   │       │                                             │
│   ▼       ╳ (不连接)                                    │
│ ReadVectorFloat                                         │
│   │                                                     │
│   ▼                                                     │
│ (后续逻辑...)                                           │
└─────────────────────────────────────────────────────────┘
```

#### 预期结果

| 时间段 | 行为 |
|-------|------|
| **前 2 秒** | 画面在动，Simulink 可能在跑，但 UE5 不会去读数据（避免崩溃） |
| **2 秒后** | `bIsSimReady` 变为 True，数据流打通，物体开始根据 Simulink 的指令运动 |

#### 关于延迟时间

- **2 秒**是经验值，通常插件初始化需要几百毫秒到 1 秒
- 2 秒是一个"安全冗余量"
- 如果 2 秒还是偶尔崩，改成 **5 秒**试试
- 如果 5 秒还崩，那就不是时序问题，需要修改 C++ 代码

---

### 其他建议

1. **版本验证**: 确保MATLAB、插件与UE5版本完全同步
2. **更新检查**: 安装R2024b最新补丁，可能包含联合仿真稳定性修复
3. **技术支持**: 如持续出现，建议联系MathWorks官方支持

---

## 结论

问题源于 UE5 编辑器中的 **MathWorks 插件生命周期管理**问题。当存在与 Simulink 数据交互的 Actor 时，第二次仿真启动时插件尚未完成初始化，导致空指针访问崩溃。

| 方案 | 优点 | 缺点 |
|-----|------|------|
| 重启 UE 编辑器 | 100% 有效 | 效率低，每次仿真都要重启 |
| Delay + Boolean Gate | 无需重启，效率高 | 需要修改蓝图，前几秒无数据交互 |

**推荐使用方案二**，通过蓝图中的 Delay + Boolean Gate 实现时序控制，避免在插件初始化完成前访问数据。
