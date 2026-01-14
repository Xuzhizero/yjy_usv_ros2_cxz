---
title: UE5 创建了一个C++类如何加入蓝图类
date: 2025-06-04
source: https://blog.csdn.net/a2213086589/article/details/148414709
tags: [UE5, C++, Blueprint, Component, Integration]
category: UE5
---

# UE5 C++ Class Integration with Blueprint Classes

## Article Metadata
- **Title**: UE5 创建了一个C++类，现在我还有一个蓝图类，我想将编写的C++类中包含的功能加入到这个蓝图类里面，如何做到
- **Author**: 氩氪氙氡_xuzhi
- **Published**: 2025-06-04 11:15:11
- **Updated**: 2025-06-04 14:26:15
- **Tags**: #ue5, #c++, #机器人

## Summary

This comprehensive guide addresses how to integrate C++ class functionality into existing Blueprint classes in Unreal Engine 5. The article presents two primary approaches:

### **Method 1: Blueprint Inheritance**
"Directly inherit your Blueprint class from the C++ class you've written, allowing it to access all properties and functions immediately."

### **Method 2: Component-Based Composition**
Create a C++ component class (inheriting from `UActorComponent`), then add it as a component to existing blueprints without modifying their core structure.

## Key Discussion Points

The author demonstrates thoughtful decision-making by selecting the component approach because they:
- Work with third-party blueprints of unknown internal structure
- Want to avoid breaking existing functionality
- Prefer non-invasive extension through modular components
- Need to maintain code maintainability and reversibility

## Component Characteristics (15 Key Features)

1. **Modular functionality encapsulation**
2. **Composability** - multiple components per Actor
3. **Hierarchical relationships** - parent-child component structures
4. **Independent properties and events**
5. **Visual editing capabilities**
6. **Runtime dynamic control**
7. **Inheritance and customization support**
8. **Physics simulation integration**
9. **Collaboration with other nodes**
10. **Cross-blueprint reusability**
11. **Diverse type support** (meshes, colliders, audio, etc.)
12. **Event-driven architecture**
13. **Performance optimization options**
14. **C++ integration**
15. **Blueprint visual scripting support**

## Technical Implementation

The guide provides detailed steps for exposing C++ functions to blueprints using decorators like `UFUNCTION(BlueprintCallable)` and `UPROPERTY(EditAnywhere, BlueprintReadWrite)` to control visibility and accessibility within the blueprint editor.
