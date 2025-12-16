# 关卡迁移与版本兼容 (Level Migration)

**Tags:** `integration`, `migration`, `version-compatibility`, `ue5`, `level-upgrade`

**日期**: 2025-12-16
**状态**: 模板 / 待填写

---

## 1. 背景 (Background)

[描述关卡迁移的背景和触发原因]

**场景**:
- 从 UE 4.27 迁移到 UE 5.x
- 项目结构调整或重构
- 资产更新与兼容性处理

**目标**:
- 完成关卡无损迁移
- 保证所有蓝图正常编译
- 验证功能完整性

---

## 2. 问题描述 (Problem)

### 2.1 主要问题

[详细描述遇到的问题]

**示例问题**:
- ❌ 资产引用丢失
- ❌ 材质渲染异常
- ❌ 物理碰撞失效
- ❌ 蓝图节点过时

### 2.2 错误信息

```
[贴入具体的错误日志或截图]
```

### 2.3 影响范围

- [ ] 核心游戏逻辑
- [ ] 渲染效果
- [ ] 物理模拟
- [ ] ROS2 通信接口

---

## 3. 原因分析 (Root Cause)

### 3.1 根本原因

[分析问题的根源]

**可能原因**:
- UE 版本升级导致 API 变化
- 资产路径变更
- 蓝图节点被弃用
- 插件版本不兼容

### 3.2 关键因素

1. **版本差异**:
   - 旧版本: [UE 4.27 / UE 5.0]
   - 新版本: [UE 5.3 / UE 5.4]

2. **依赖关系**:
   - 外部插件: [列表]
   - 第三方库: [列表]

---

## 4. 处理方案 (Solution)

### 4.1 迁移步骤

**Step 1: 备份原始项目**

```bash
# 备份整个项目
cp -r OriginalProject OriginalProject_Backup_20251216
```

**Step 2: 清理中间文件**

```bash
# 删除临时文件
rm -rf Binaries/ Intermediate/ Saved/ DerivedDataCache/
```

**Step 3: 使用 UE 内置迁移工具**

1. 打开 UE 编辑器
2. 选择 `File > Open Project`
3. 选择项目文件，UE 会自动提示升级
4. 点击 `Convert in-place` 或 `Copy and Convert`

**Step 4: 手动修复问题**

[详细记录手动修复的步骤]

```cpp
// 示例：修复过时的 API 调用
// 旧代码:
// Actor->SetActorLocation(NewLocation);

// 新代码:
// Actor->SetActorLocation(NewLocation, false, nullptr, ETeleportType::None);
```

### 4.2 蓝图修复

**修复过时节点**:
- 使用 `Refresh All Nodes` 功能
- 手动替换不兼容的节点
- 更新材质函数调用

### 4.3 资产重新导入

```bash
# 批量重新导入纹理
for texture in Assets/Textures/*.tga; do
    # 在 UE 中重新导入
done
```

---

## 5. 验证结果 (Verification)

### 5.1 功能验证清单

- [ ] 关卡可以正常打开
- [ ] 所有蓝图编译通过
- [ ] 材质渲染正常
- [ ] 物理碰撞正常
- [ ] ROS2 节点通信正常
- [ ] 运行时无错误日志

### 5.2 性能对比

| 指标 | 迁移前 | 迁移后 | 变化 |
|------|--------|--------|------|
| FPS | 60 | 58 | -3.3% |
| 内存占用 | 2.5GB | 2.8GB | +12% |
| 加载时间 | 5s | 6s | +20% |

### 5.3 测试方法

```bash
# 运行自动化测试
./RunTests.sh integration_test

# 手动测试关键功能
# 1. 启动仿真
# 2. 检查 USV 行为
# 3. 验证传感器数据
```

---

## 6. 经验总结 (Lessons Learned)

### 6.1 最佳实践

1. **迁移前务必备份**:
   - 完整备份项目文件
   - 记录当前版本信息

2. **分阶段迁移**:
   - 先迁移核心关卡
   - 再迁移辅助资产
   - 最后处理插件依赖

3. **自动化验证**:
   - 编写自动化测试脚本
   - 定期回归测试

### 6.2 常见陷阱

⚠️ **避免的错误**:
- 直接在原项目上升级（应该先复制）
- 跳过清理中间文件步骤
- 忽略插件兼容性检查

### 6.3 改进建议

- 建立版本控制分支策略
- 定期进行版本兼容性测试
- 维护插件版本兼容性列表

---

## 7. 关联资源 (References)

### 7.1 相关 Commit

```bash
# 关联的 git commit (如有)
git log --oneline --grep="migration"
```

- Commit: `abc1234` - "Migrate to UE 5.3"
- Commit: `def5678` - "Fix blueprint compilation errors after migration"

### 7.2 相关文档

- [UE5 迁移指南](https://docs.unrealengine.com/5.3/en-US/migrating-projects-to-unreal-engine-5/)
- [蓝图 API 变更列表](https://docs.unrealengine.com/5.3/en-US/API/)
- [项目架构文档](../../docs/architecture.md)

### 7.3 外部参考

- [Unreal Engine Migration Best Practices](https://example.com)
- [UE5 Breaking Changes](https://example.com)

---

**记录人**: [姓名]
**审核人**: [姓名]
**完成日期**: [YYYY-MM-DD]
