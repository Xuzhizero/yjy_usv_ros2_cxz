# 软件开发过程记录 / 开发日志

**Tags:** `ue5`, `worldpartition`, `blueprint`, `migration`, `simulink`, `devlog`

## 摘要

本文档记录了在UE5仿真系统中，将摄像头与Simulink接口从Map2迁移至Map1的完整过程。主要解决了World Partition机制下关卡蓝图Actor引用失效、变量丢失以及迁移资产蓝图编译错误等问题。通过重建Actor引用、补全变量、启用插件及添加输入映射等措施，成功完成了跨关卡集成，并总结了UE5 World Partition环境下的架构优化建议和操作规范。

---

## 基本信息
| 项目名称 | 无人船视觉仿真系统 |
|----------|--------------------|
| 记录日期 | 2025年12月16日     |
| 记录人   | 曹勖之             |
| 关联模块 | UE5仿真场景集成    |

---

## 问题记录一：关卡蓝图跨关卡迁移引用失效

### 问题（目标）描述
在将摄像头与Simulink接口从Map2（摄像头模型关卡）集成至Map1（含桥梁、风机、电塔、集装箱船、快艇、军舰、瑶光1号、岛礁、沙滩及动态天气元素的场景关卡）过程中，因UE5 World Partition机制，原关卡蓝图中对Actor的直接引用在新关卡中失效，显示为"Unknown"，导致蓝图编译错误。

### 原因分析
1. **Level Blueprint的Actor引用机制**：UE5 Level Blueprint中的Actor引用绑定的是关卡内实例的唯一标识（包含关卡路径/对象GUID/内部名），而非按名称匹配。跨关卡复制节点后，即使将同名Actor搬入新关卡，其实例身份已变化，UE无法自动匹配原有引用。
2. **World Partition特性限制**：目标关卡启用了World Partition，Actor不再隶属于传统的Persistent Level/SubLevel体系，而是属于World Partition Cell。在此模式下，Level Blueprint中标注"from Persistent Level"的引用在跨关卡复制后必然失效。
3. **变量未自动迁移**：关卡蓝图中的变量（如LastLocation、LastRotation、boolLocValid、boolRotValid等）属于关卡本身，不是独立资产，复制节点时不会自动携带至新关卡。

### 处理措施
1. **重建Actor引用**：
   - 在目标关卡Level Blueprint中删除所有显示为"Unknown"的引用节点
   - 在场景中选中正确的目标Actor
   - 返回Level Blueprint空白处右键，选择"Create a Reference to [Actor名称]"
   - 将新建的引用节点接回原有逻辑链路
2. **补全缺失变量**：
   - 在目标关卡蓝图的My Blueprint → Variables中新建以下变量：
     - LastLocation：类型Vector
     - LastRotation：类型Rotator
     - boolLocValid：类型Boolean
     - boolRotValid：类型Boolean
   - 对于误识别为变量"True"的节点，删除后用Boolean常量节点替代
3. **验证与测试**：完成上述修复后，点击Compile验证蓝图编译通过，并执行Play测试确认功能正常。

### 结论
系统功能恢复正常，摄像头与Simulink接口成功集成至目标场景关卡，未对原关卡及核心资产造成影响。

---

## 问题记录二：迁移资产蓝图编译错误

### 问题（目标）描述
在关卡集成过程中，BP_Base_Truck和Mine_Waterline_6_DemoV3两个蓝图出现编译错误，阻止PIE（Play In Editor）运行。

### 原因分析
1. **BP_Base_Truck错误原因**：
   - 缺少VR/HeadMountedDisplay相关插件，导致IsHeadMountedDisplayEnabled和ResetOrientationAndPosition函数找不到
   - 项目输入设置中缺少Action Mapping（Light、SwitchCamera），导致InputAction事件引用未知Action
   - 部分节点签名变化，需要刷新
2. **Mine_Waterline_6_DemoV3错误原因**：
   - 状态变量缺失（同问题记录一）
   - Actor引用失效显示为Unknown（同问题记录一）

### 处理措施
1. **BP_Base_Truck修复**：
   - Edit → Plugins中启用Head Mounted Display插件，重启UE
   - Edit → Project Settings → Input中添加Action Mappings：Light、SwitchCamera
   - 对报红节点执行右键 → Refresh Node
2. **Mine_Waterline_6_DemoV3修复**：按问题记录一的处理措施执行变量补全和引用重建
3. **收尾操作**：
   - 对迁移资源根目录执行Fix Up Redirectors in Folder
   - File → Save All保存所有修改

### 结论
两个蓝图均编译通过，PIE运行正常，系统集成完成。

---

## 经验总结与改进建议
1. **架构优化建议**：将Level Blueprint中的核心逻辑（Simulink通信、摄像头管理、状态变量等）迁移至独立的Actor Blueprint（如BP_SimManager），Level Blueprint仅保留初始化绑定，可大幅降低后续关卡迁移的维护成本。
2. **操作规范**：UE5中移动Map/资产应在Content Browser内通过拖拽或Move操作完成，移动后务必执行Fix Up Redirectors；禁止在系统文件管理器中直接操作.umap文件。
3. **World Partition注意事项**：在World Partition关卡中，应避免在Level Blueprint中直接引用放置的Actor，推荐使用GetAllActorsOfClass、Tag查找或Manager模式实现引用解耦。

---

## 附件
- 修复后关卡蓝图截图（2张）
  - ![Fix reference screenshot 1](img/fix_reference_01.png)
  - ![Fix reference screenshot 2](img/fix_reference_02.png)
