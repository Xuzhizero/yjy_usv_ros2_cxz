---
title: UE5 UDS动态天气天空如何采用官方光源和雾
date: 2025-11-13
source: https://blog.csdn.net/a2213086589/article/details/154790994
tags: [UE5, UDS, Ultra Dynamic Sky, Lighting, Fog]
category: UE5
---

# UE5 UDS (Ultra Dynamic Sky) - Using Official Light Sources and Fog

**Publish Date:** November 13, 2025

## Article Summary

This article addresses how to properly configure UDS (Ultra Dynamic Sky) in UE5 to work with official engine light sources rather than relying solely on UDS's built-in lighting components.

### Key Points:

**UDS Built-in Components:**
- UDS includes an internal DirectionalLight (sun) and SkyLight (sky reflection)
- Even without placing separate lights in the level, scenes remain illuminated
- Internal components are blueprint-based and not visible in the World Outliner

**Professional Configuration (Recommended):**

The article emphasizes binding external lighting to UDS through the Details panel:
- **Sun Light Actor** → External DirectionalLight
- **Moon Light Actor** → Separate light or disabled (never share with sun light)
- **Sky Light Actor** → External SkyLight
- **Height Fog Actor** → ExponentialHeightFog

**Critical Steps:**

Disable internal light sources by toggling settings like:
- "Use Built-in Sun Light" → OFF
- "Use Built-in Skylight" → OFF
- "Use Built-in Moon Light" → OFF

This approach:
- Enables Outliner management of light properties
- Supports compatibility with other systems (Waterline Pro, volumetric clouds)
- Provides better control over shadows and exposure
- Follows UE5 best practices

The configuration ensures single unified lighting across your project while allowing UDS to drive dynamic sky effects.
