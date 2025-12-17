---
title: 如何用 Simulink 驱动 UE5 的海况切换
date: 2025-11-03
source: https://blog.csdn.net/a2213086589/article/details/154351247
tags: [UE5, Simulink, Waterline Pro, Blueprint, Sea State]
category: UE5
---

# 如何用 Simulink 驱动 UE5 的海况切换

**发布日期：** 2025-11-03 15:32:13
**作者：** 氩氪氙氡_xuzhi
**阅读量：** 832 | **点赞：** 29 | **收藏：** 16

---

## 核心目标

实现低频（小时级）宏观海况参数在运行时热更新，使 Waterline Pro 海面与船体颠簸平稳过渡，**不改动海洋 Actor 本体蓝图**。

---

## 两种方案对比

### 方案 A：Simulink 消息推送（推荐）

- **特点**："一帧完整的海况参数包"由 Simulink 发送
- **优势**：事件驱动、原子更新、低耦合、无文件读写
- **实现**：旁路管理器蓝图 Actor 订阅消息，执行参数插值过渡

### 方案 B：JSON 轮询（备选）

- **特点**：外部脚本定期写入 `Saved/sea_state.json`
- **优势**：实现简单、无需 Simulink
- **劣势**：需要文件操作、轮询机制、容错处理复杂

**结论**：运行期由 Simulink 主控时采用方案 A；离线/演示机器可回退 JSON。

---

## 方案 A 的三种落地形态

| 形态 | 说明 | 推荐度 |
|------|------|--------|
| **A-1** | 独立"海况管理器" Actor，零改海洋蓝图 | ⭐⭐⭐ |
| **A-2** | 逻辑写入 Level Blueprint | ⭐⭐ |
| **A-3** | Simulink 仅发"档位名"，UE 查表 | ⭐⭐ |

**最终选择 A-1 的理由**：

> "工程化最稳、海况收发与参数校验封装独立、后续易扩展、完全满足不改Ocean_Sim1蓝图的要求。"

---

## 方案 A-1 完整实现流程

### Simulink 端配置

**参数结构体字段示例：**

| 字段 | 范围 | 作用 |
|------|------|------|
| `preset` | 文本 | Beaufort 等级预设 |
| `windSpeed` | 0–25 m/s | 风速 |
| `windDirDeg` | 0–360° | 风向 |
| `waveAmp` | 0–7 m | 浪高/振幅 |
| `wavelengthMin/Max` | 3–30 m | 波长范围 |
| `steepness` | 0–1 | 浪尖锐度 |
| `gamma` | 1–7 | JONSWAP 形状系数 |
| `version` | uint | 版本号（幂等性） |

**消息发送**：使用 _Simulation 3D Message Set_，Topic 为 `/wlp/sea_state`

---

### UE5 端实现（不改 Ocean_Sim1）

#### 步骤 1：给海面 Actor 打标签

在关卡中选中 `Ocean_Sim1` → Details → Tags 添加 `"Ocean_Sim1"`

#### 步骤 2：创建 `BP_SeaStateManager` 蓝图

**BeginPlay 逻辑：**

```
GetAllActorsWithTag("Ocean_Sim1")
  → 取第一个 → 缓存为 OceanActorRef
GetComponentByClass(StaticMesh/ProceduralMesh)
  → CreateDynamicMaterialInstance
  → 缓存为 MID_Ocean
```

**关键：只创建一次 MID，避免反复生成导致 GC 压力**

#### 步骤 3：OnMessage 消息回调

**流程：**

1. 检查版本号递增性（去重）
2. 对所有数值执行 Clamp（范围校验）
3. 启动 Timeline（2–5 秒插值）
4. 逐项调用 `Set Scalar/Vector Parameter Value`：
   - `WaveAmplitude`、`WavelengthMin`、`WavelengthMax`
   - `Steepness`、`BreakingThreshold`、`FoamIntensity`
   - 风向、阈值等

#### 步骤 4：参数映射

"消息字段 → 材质参数名"的对应关系（**需按项目材质实际参数调整**）：

| 消息字段 | 材质参数名 |
|---------|----------|
| `waveAmp` | `WaveAmplitude` |
| `windDirDeg` | `WindDirectionDeg` |
| `steepness` | `Steepness` |
| `foamIntensity` | `FoamIntensity` |

---

## 蓝图伪代码

```
[BP_SeaStateManager]

BeginPlay:
  OceanRef ← GetAllActorsWithTag("Ocean_Sim1")[0]
  MID_Ocean ← CreateDynamicMaterialInstance(OceanRef)
  CacheMID_Ocean

OnMessage(SeaState):
  if SeaState.version ≤ LastVersion: return
  LastVersion ← SeaState.version

  Target.waveAmp ← Clamp(SeaState.waveAmp, 0, 7)
  Target.steepness ← Clamp(SeaState.steepness, 0, 1)

  StartTimeline(LerpSeconds = 3):
    for t in [0,1]:
      MID_Ocean.SetScalar("WaveAmplitude",
        Lerp(Current.WaveAmplitude, Target.waveAmp, t))
      MID_Ocean.SetScalar("Steepness",
        Lerp(Current.Steepness, Target.steepness, t))

  PrintString("Applied SeaState v" + LastVersion)
```

---

## 常见坑与排错

| 问题 | 解决方案 |
|------|--------|
| 找不到海面 Actor | 用 Tag 查找，避免名字模糊匹配 |
| 参数名对不上 | 打开材质实例面板逐一核对 |
| 视觉变化生硬 | 加入 2–5 秒 Timeline 插值 |
| 单位错位 | 角度(度↔弧度)、风速/浪高量级校准 |
| 多人冲突 | 管理器独立资源，避免 Level BP |
| 性能下降 | 低频消息、插值只改少量参数，几乎无开销 |

---

## 可选备通道：JSON 回退

管理器中添加 `EnableJsonFallback` 开关：

- 若 N 分钟未收到 Simulink 消息
- 读取 `Saved/sea_state.json` 并套用参数
- 日志标注 `source=JSON`
- 同样执行插值过渡

---

## 总结

1. **方案选型**：消息推送原子一致、工程化更稳，优于 JSON 轮询
2. **形态选择**：A-1 最符合"零改现有蓝图、可复用扩展"的长期维护需求
3. **落地要点**：
   - 用 Tag 查引用
   - 只创建一次 MID（缓存）
   - 参数映射表先校准
   - 2–5 秒平滑过渡
   - 打印应用版本便于联调
