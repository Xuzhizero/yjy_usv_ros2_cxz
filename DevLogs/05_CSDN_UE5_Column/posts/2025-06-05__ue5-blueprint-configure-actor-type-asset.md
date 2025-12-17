---
title: 在 UE5 蓝图中配置Actor类型的Asset
date: 2025-06-05
source: https://blog.csdn.net/a2213086589/article/details/148444410
tags: [UE5, Blueprint, Actor, Transform, Location, Rotation]
category: UE5
---

# UE5 Blueprint: Configuring Actor-Type Assets for Location and Rotation Targets

**Published:** June 5, 2025
**Author:** 氩氪氙氡_xuzhi

## Overview

This article addresses how to configure an Actor Blueprint (based on `sim3dActor`) to function as a valid target input for the "Set Actor Location and Rotation" node in UE5's event graph.

## Key Concepts

### Blueprint Terminology

The `SetGetActorLocationRotation` blueprint created in your project is formally called:
- **Actor Blueprint** - most common term
- **Blueprint Class** - technical designation for content browser assets
- Based on `sim3dActor` (an Actor subclass), it inherently possesses Location and Rotation properties

### Important Finding

"All `AActor` subclasses naturally possess Transform properties (Location, Rotation, Scale) through engine architecture" - you don't need explicit Event Graph nodes to enable these attributes. They're available by default when placing the Actor in a level.

## Implementation Approach

**Correct workflow:** Only add "Set Actor Location and Rotation" in your Level Blueprint, not in the Actor Blueprint itself.

**Rationale:** The Actor Blueprint already has Location/Rotation capabilities. You only need the Set node where you're controlling it externally (Level Blueprint). This follows proper encapsulation principles.

## Summary

Your Actor Blueprint requires no special configuration to support Location and Rotation parameters. Configure the "Set" logic solely in your Level Blueprint when controlling the Actor from that scope.
