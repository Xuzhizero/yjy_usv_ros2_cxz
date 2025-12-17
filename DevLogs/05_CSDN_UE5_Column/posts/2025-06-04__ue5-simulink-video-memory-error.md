---
title: UE 5 和simulink联合仿真显存不足错误
date: 2025-06-04
source: https://blog.csdn.net/a2213086589/article/details/148433646
tags: [UE5, Simulink, MATLAB, Video Memory, GPU, Resource Management]
category: UE5
---

# UE5 and Simulink Co-simulation: Video Memory Depletion After Stopping Play

**Published:** June 4, 2025, 19:52:49
**Author:** 氩氪氙氡_xuzhi
**Views:** 4.6k | **Likes:** 27 | **Saves:** 26

---

## Problem Description

The user reports a critical issue when using UE5 with Simulink co-simulation:

> "During Play mode, no errors occur even for extended periods. However, after ending Play (regardless of duration), approximately one minute later the engine reliably produces 'Out of video memory' errors."

This pattern suggests the problem isn't during gameplay but rather in the shutdown/resource cleanup process.

---

## Root Cause Analysis

The expert response identifies this as a **resource management failure during Play mode exit**, not a memory exhaustion issue during execution. Key observations:

- **Play phase:** Resources remain allocated and actively used (no error)
- **Post-Play phase:** Shutdown triggers errors after ~60 seconds
- **Pattern:** Occurs consistently regardless of Play duration

### Likely Culprits

1. **Simulink Plugin GPU Resource Leak** – The external simulation interface may not properly deallocate GPU resources during termination

2. **Actor/Component Lifecycle Failure** – Objects displaying simulation data may not execute proper `EndPlay` cleanup routines

3. **RenderTarget/Material Persistence** – Dynamic textures and materials (`UTextureRenderTarget2D`) remain unreleased in video memory

4. **Orphaned Timers/Threads** – Background processes continue consuming resources after Play ends

---

## Recommended Solutions

**Immediate Actions:**

- Implement proper `EndPlay` event handlers with resource cleanup
- Call `ReleaseResource()` on all render targets explicitly
- Clear timers using `ClearTimer()` before shutdown

**Debugging:**

Use console commands to monitor memory:
```
stat RHI
stat GPU
rhi.DumpMemory
```

**Code-Level Fix (C++):**

Ensure destructor cleanup:
```cpp
MyTexture->ConditionalBeginDestroy();
MyRenderTarget->ReleaseResource();
MyComponent->DestroyComponent();
```

---

## Tags
`#UE5` `#MATLAB` `#Robotics`
