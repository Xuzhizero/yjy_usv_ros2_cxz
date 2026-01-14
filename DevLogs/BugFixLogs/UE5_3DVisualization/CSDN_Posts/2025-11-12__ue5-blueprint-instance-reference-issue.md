---
title: UE5 我在一个Level A里的一个Boat复制到另一个Level B里面，为什么在Level B里的对Boat的编辑会影响Level A 里面Boat？
date: 2025-11-12
source: https://blog.csdn.net/a2213086589/article/details/154729108
tags: [UE5, Blueprint, Level, Instance, Reference]
category: UE5
---

# UE5 Blueprint Instance Reference Issue: Complete Article Summary

## Article Metadata
- **Author:** 氩氪氙氡_xuzhi
- **Published:** 2025-11-12
- **Views:** 599 | Likes: 22 | Saves: 5

## Core Problem
When copying a Boat Blueprint instance from Level A to Level B, edits made to the Level B instance also affect the Level A version.

## Root Cause Explanation
The issue stems from a fundamental misunderstanding about UE5's reference architecture. Both levels contain instances pointing to the identical Blueprint Class definition file (`BP_Boat.uasset`). When accessing "Edit in Blueprint," you're modifying the shared class definition rather than level-specific instances.

## Key Distinction
| Action | Scope | Result |
|--------|-------|--------|
| "Edit in Blueprint" | All levels | Affects every instance |
| Modify instance properties in viewport | Current level only | Instance-specific changes |
| Yellow-flagged overrides | Current level only | Isolated modifications |

## Solution Methods

**Method 1: Create Separate Blueprint Classes**
Duplicate `BP_Boat` as `BP_Boat_LevelB` in Content Browser to establish independent definitions.

**Method 2: Instance Property Overrides**
Modify parameters directly in the Details panel while avoiding blueprint editing mode to preserve level-specific settings.

**Method 3: Level Isolation**
Use separate Level Instances or Sublevels to maintain independent actor management.

## No Images or Code Blocks Present
The article contains primarily text explanation with a reference table structure.
