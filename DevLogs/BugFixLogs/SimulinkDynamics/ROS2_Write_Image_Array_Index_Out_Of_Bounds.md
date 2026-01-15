# ROS2 Write Image 数组越界错误修复指南

## 问题描述

### 错误信息

```
错误:仿真期间出现错误,仿真终止
原因:
    Index exceeds array dimensions. Index value 30001 exceeds valid range [1-30000].
    Error in WriteImageFcnBlock.m (line 25)
        Data(1:DataLength) = Data1(1:DataLength);
    'ROS2_simulink_UE5_260106/Write Image/MATLAB Function' (line 30) 中出现错误
```

### 场景

- 从使用 `img_matrix` 改为读取本地 PNG 格式文件
- ROS2 的 Variable Size Messages 中的 `data length` 设置为 30000
- 使用 ROS2 Write Image 块发布图像消息时出现数组越界

---

## 错误根本原因分析

### ✅ 核心问题

当从 PNG 文件读取图像时：

```matlab
img = imread("xxx.png");
```

图像的 **像素数量 × 通道数** 通常远大于预设的最大长度。

**举例说明：**

如果图像是 **400×300×3** (RGB)，那么有效 data 长度：

```
400 × 300 × 3 = 360,000
```

这远大于在 **Variable Size Messages** 里设置的最大长度 **30000**。

### ✅ 越界发生时机

Simulink 在试图把图像 data 复制到消息结构时：

```matlab
Data(1:DataLength) = Data1(1:DataLength);
```

当 `DataLength = 360,000` 时，但数组最大容量只有 30000，就会报错：

```
Index exceeds array dimensions… 30001 exceeds valid range [1-30000]
```

### ✅ ROS2 Write Image 块的内部工作流程

1. 从 Image 端口读入图像矩阵（M×N×3 或 M×N）
2. 展平成列向量：
   ```matlab
   data = reshape(img, [], 1)
   ```
3. 赋给 ROS2 消息 Bus 的 `data` 字段
4. 同时设置 CurrentLength

**关键约束：**
```
numel(data) ≤ MaxLength（Variable Size Messages 里设置的最大值）
```

否则就会在写数据时越界。

---

## 常见图像尺寸的 Data Length

| 图像大小      | Channels | Data Length |
|------------|----------|-------------|
| 640×480    | 3        | 921,600     |
| 800×600    | 3        | 1,440,000   |
| 1024×768   | 3        | 2,359,296   |
| 400×300    | 3        | 360,000     |
| 320×240    | 3        | 230,400     |
| 200×200    | 3        | 120,000     |

---

## 解决方案

### ✅ 方法 A — 重新设置 Variable Size Messages 的最大长度（推荐）

需要把 `sensor_msgs/Image.data` 的最大长度改为 **图像最大可能字节数**。

**计算公式：**
```
MaxLength = rows × cols × channels
```

**推荐设置：**

1. **对于通用场景（支持高清图像）：**
   ```
   MaxLength = 3000000  (3 million)
   ```

2. **对于已知最大尺寸的场景：**
   根据实际最大图像尺寸计算，例如 800×600×3：
   ```
   MaxLength = 1440000
   ```

**配置步骤：**

1. 打开 Simulink 模型
2. 找到 **ROS Toolbox → Variable Size Messages** 配置
3. 选择 `sensor_msgs/Image` 消息类型
4. 设置 `data` 字段的 `MaxLength = 3000000`（或根据需求调整）
5. 应用并保存配置

---

### ✅ 方法 B — 限制输入图像尺寸

如果只想支持小图像，可以设置较小的 MaxLength，但必须确保输入图像不超过限制。

**示例：**

对于 200×200×3 的图像：
```
200 × 200 × 3 = 120,000
```

则设置：
```
MaxLength = 120000
```

**⚠️ 注意：** 如果读入的 PNG 图像大于此尺寸，仍然会越界。

---

### ✅ 方法 C — 在读取后对图像进行缩放

在读取 PNG 文件后，使用 `imresize` 缩放到合适尺寸：

```matlab
img = imread("xxx.png");
img = imresize(img, [240 320]);  % 缩放到 240×320
```

**Data Length 计算：**
```
240 × 320 × 3 = 230,400
```

配合设置：
```
MaxLength = 250000
```

**在 Simulink 中实现：**

可以使用 MATLAB Function 块：

```matlab
function img_resized = resize_image(img_path)
    img = imread(img_path);
    img_resized = imresize(img, [240 320]);
end
```

---

### ✅ 方法 D — 使用消息拆分（高级，不推荐）

将大图像拆分成多个消息发布，但这种方法较为复杂，需要自定义接收端重组逻辑，不常用。

---

## 推荐配置流程（标准做法）

### 🔧 完整步骤

1. **确定图像最大尺寸**
   - 检查要发布的所有 PNG 图像的分辨率
   - 计算最大可能的 data length

2. **配置 Variable Size Messages**
   ```
   sensor_msgs/Image → data → MaxLength = (计算得出的最大值)
   ```

3. **构建 Simulink 模型**
   ```
   [Image From File] → [ROS2 Write Image] → [Publish]
   ```

4. **验证配置**
   - 运行仿真，确保不再出现越界错误
   - 在 RViz2 中订阅 topic，确认图像正常显示

---

## 注意事项

| 要点 | 说明 |
|-----|------|
| **MaxLength 越大** | 在 Simulink 端占用的信号存储空间越大 |
| **动态分辨率** | 如果图像分辨率动态变化很大，需设定足够大的最大值 |
| **CurrentLength** | 必须满足 `0 ≤ CurrentLength ≤ MaxLength` |
| **内存限制** | 极大的 MaxLength 可能导致内存占用过高，需权衡 |

---

## 关键点总结

| 问题 | 原因 |
|------|------|
| 越界错误 `30001 > 30000` | 实际 data 长度 > 配置的最大长度 30000 |
| ROS2 Write Image 块内部数组赋值越界 | Simulink Bus 的最大长度配置过小 |
| RViz2 能看到 height、width，但 data 数组不正确 | Publish 之前已经在数据写入阶段越界报错 |

---

## 预防措施

### ✅ 在项目初期

1. **明确图像规格**
   - 确定要使用的图像最大分辨率
   - 计算对应的 data length

2. **预留余量**
   - MaxLength 设置时预留 20-30% 余量
   - 例如：实际最大 1,440,000，设置为 2,000,000

3. **文档记录**
   - 在项目文档中记录 Variable Size Messages 配置
   - 说明支持的最大图像尺寸

### ✅ 在开发过程中

1. **测试多种尺寸**
   - 使用不同分辨率的图像测试
   - 确保边界情况都能正常处理

2. **监控内存使用**
   - 观察仿真时的内存占用
   - 根据实际情况调整 MaxLength

---

## 扩展阅读

### 相关文档

- [ROS2 sensor_msgs/Image 消息定义](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg)
- [Simulink Variable-Size Signal 文档](https://www.mathworks.com/help/simulink/ug/variable-size-signals.html)
- [ROS Toolbox 使用指南](https://www.mathworks.com/help/ros/)

### 常见问题

**Q: 为什么之前用 img_matrix 没有问题？**

A: img_matrix 可能是在 Simulink 中预定义的固定大小矩阵（如 100×100×3），其大小小于 30000。而读取 PNG 文件时，图像尺寸通常更大，导致越界。

**Q: 可以动态调整 MaxLength 吗？**

A: 不可以。MaxLength 必须在编译时确定，无法在运行时动态调整。需要在设计阶段设置足够大的值。

**Q: 设置很大的 MaxLength 会影响性能吗？**

A: 会增加内存占用，但对计算性能影响较小。建议根据实际需求设置合理的值，既满足功能需求，又不过度占用内存。

---

## 相关日志

- [ROS 2 Write Image Block 使用指南](../../DevNotes/API_Usage/ROS2_Write_Image_Block_Usage_Guide.md)
- [UE5 与 ROS2 图像通信配置](../UE5_3DVisualization/)

---

**日期:** 2026-01-15
**状态:** 已解决
**影响范围:** ROS2 图像消息发布功能
**修复方法:** 配置 Variable Size Messages 的 MaxLength 参数
