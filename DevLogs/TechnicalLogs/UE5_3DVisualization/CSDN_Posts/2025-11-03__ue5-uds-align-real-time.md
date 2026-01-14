---
title: 如何让UE5的插件Ultra Dynamic Sky的光照对齐真实时间？
date: 2025-11-03
source: https://blog.csdn.net/a2213086589/article/details/154352003
tags: [UE5, Ultra Dynamic Sky, UDS, Lighting, Time, Blueprint]
category: UE5
---

# How to Align Ultra Dynamic Sky Lighting with Real Time in UE5

**Published:** November 3, 2025
**Author:** 氩氪氙氡_xuzhi
**Views:** 454 | **Likes:** 4 | **Saves:** 9

## Question

How can the Ultra Dynamic Sky (UDS) plugin's time parameter in UE5 be conveniently synchronized with actual system time?

## Solution

The approach involves retrieving system time and converting it to UDS's "Time of Day" format, then updating it periodically.

### Blueprint Implementation Steps

1. **Get System Time:**
   - Use the `Now` or `UtcNow` node to obtain current system datetime
   - Extract hours, minutes, and seconds
   - Calculate: `TimeFloat = Hour + Minute/60 + Second/3600`

2. **Set UDS Time:**
   - Reference the UDS actor in your level
   - Use either the `Set Time of Day using Time Code (H:M:S)` function or assign `TimeFloat` directly to the `Time of Day` variable

3. **Maintain Synchronization:**
   - Create a timer that repeats every 1-60 seconds
   - Execute the synchronization logic repeatedly

4. **Disable Conflicting Systems:**
   - Turn off UDS's automatic day-night cycle by setting `Day Length` to 0 to prevent conflicts

### Important Considerations

- **Time Zone:** `Now()` returns local time; `UtcNow()` requires manual timezone offset adjustment
- **Precision:** UDS provides artistic rather than astronomically accurate time mapping
- **Alternative:** For precise geographic calculations, use the built-in Sun and Sky Actor instead
- **Resources:** Community blueprints offer ready-made time systems with documentation

The author offers to provide a detailed blueprint diagram for specific project needs.
