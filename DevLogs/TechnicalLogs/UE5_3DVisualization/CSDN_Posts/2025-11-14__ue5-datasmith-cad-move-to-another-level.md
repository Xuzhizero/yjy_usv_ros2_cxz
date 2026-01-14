---
title: UE如何把里面用DataSmith导入的CAD模型放到另一个level里面
date: 2025-11-14
source: https://blog.csdn.net/a2213086589/article/details/154819817
tags: [UE5, Datasmith, CAD, Level, Migration]
category: UE5
---

# UE5: Moving DataSmith-Imported CAD Models to Another Level

**Published:** November 14, 2025

## Problem Statement

The user imported a CAD model from Rhino using DataSmith in UE5 but discovered the resulting Actor cannot be directly copied to other levels.

## Solution

The issue stems from DataSmith Scene Actors being a specialized type that lacks standard duplication options.

### Method 1: Content Browser Import
1. Locate the **Datasmith Scene Asset** in Content Browser (not World Outliner)
2. Right-click the asset
3. Select **"Import to Level"** or equivalent import option

### Method 2: Manual Reconstruction (Most Reliable)
1. Expand the DataSmith Scene Actor in World Outliner
2. Locate all **Static Mesh Components** within
3. Select the needed Static Meshes
4. Copy components (Ctrl+C)
5. Create a new Blueprint Actor
6. Open Blueprint and paste (Ctrl+V) into the Components panel

## Key Insight

"DataSmith Scene Actor" represents a unique asset type that doesn't support direct copying between levels like standard actors do. These workarounds extract the underlying mesh components for reuse.

**Tags:** #UE5
