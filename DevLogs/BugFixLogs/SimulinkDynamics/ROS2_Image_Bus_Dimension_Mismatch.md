# ROS2 图像消息总线维度不匹配问题 (ROS2 Image Message Bus Dimension Mismatch)

**Tags:** `simulation`, `simulink`, `ros2`, `sensor_msgs/Image`, `bus-assignment`, `variable-length-array`, `debugging`

**日期**: 2026-01-15
**状态**: ✅ 已定位根本原因 / 待配置总线参数
**严重程度**: 🔴 高 - 导致仿真无法启动

---

## 1. 问题描述 (Problem Statement)

### 1.1 系统背景

在 Simulink 中创建 ROS 2 图像消息（`sensor_msgs/Image`），用于将仿真图像通过 ROS 2 发布。

**系统架构**:

<!-- TODO: 插入系统架构图 -->
![simulink_ros2_image_bus](img/simulink_ros2_image_bus.png)

**关键模块**:
- **Image 输入**: 100×100×3 的 RGB 图像矩阵
- **createROS2ImageMsg (MATLAB Function)**: 将图像矩阵转换为 ROS 2 Image 消息格式
- **Bus Assignment**: 将各字段组装成 sensor_msgs/Image 总线

### 1.2 MATLAB Function 代码

```matlab
function [height, width, encoding, step, data] = createROS2ImageMsg(img_matrix)
    % 获取图像的尺寸
    [height, width, channels] = size(img_matrix);

    % 将编码设置为固定长度的字符串数组
    encoding_str = 'rgb8';
    encoding = uint8(zeros(1, 128));  % 创建128元素的uint8数组
    encoding(1:length(encoding_str)) = uint8(encoding_str);  % 填入字符串

    % 计算每行字节数
    step = uint32(width * channels);

    % 将RGB图像展平为字节数组
    data = uint8(reshape(img_matrix, 1, []));

    % height 和 width 保持为 uint32 类型
    height = uint32(height);
    width = uint32(width);
end
```

**输入**:
```matlab
img_matrix = uint8(cat(3, rand(100, 100) * 255, rand(100, 100) * 255, rand(100, 100) * 255));
% 尺寸: [100x100x3]
```

### 1.3 错误信息

仿真运行后报错：

```
错误 1:
端口宽度或维度出错。'ROS2_simulink_UE5_260106/MATLAB Function' 的'输出端口 5' 是 [1x30000] 矩阵。

错误 2:
端口宽度或维度出错。'ROS2_simulink_UE5_260106/Bus Assignment2' 的'输入端口 6' 是有 128 个元素的一维向量。

组件:Simulink | 类别:Model 错误
```

### 1.4 信号流与维度分析

```
┌─────────┐     [100x100x3]     ┌─────────────────────┐
│  Image  │────────────────────▶│   img_matrix        │
└─────────┘                     │                     │
                                │  createROS2ImageMsg │
                                │                     │
                                │  输出端口:          │
                                │   1. height    [1]  │
                                │   2. width     [1]  │
                                │   3. encoding  [1x128] │
                                │   4. step      [1]  │
                                │   5. data      [1x30000] ← 实际输出
                                └─────────────────────┘
                                         │
                                         ▼
                                ┌─────────────────────┐
                                │   Bus Assignment    │
                                │                     │
                                │   期望 data 端口:   │
                                │   [1x128] ← 默认配置│
                                │                     │
                                │   30000 ≠ 128 ❌    │
                                └─────────────────────┘
```

---

## 2. 根本原因分析 (Root Cause Analysis)

### 2.1 核心问题定位

**问题本质**: **Simulink 总线（Bus）定义与实际信号尺寸不匹配**

错误提示非常明确：
> "端口宽度或维度出错...'Bus Assignment2' 的'输入端口 6' 是有 128 个元素的一维向量。"

这表明：
1. **代码产生的信号**: `data` 是 100×100×3 的图像数据，展平成了 **30,000** 个元素的向量 (`[1x30000]`)
2. **Simulink 预期的信号**: ROS 2 消息总线（`sensor_msgs/Image`）中，`data` 字段默认最大长度被限制为 **128**

### 2.2 为什么默认是 128？

MATLAB/Simulink 在自动生成 ROS 2 消息的 Bus Object 时，对于**可变长度数组（Variable-length array）**：
- 为了节省内存，默认给一个较小的初始上限（通常是 128 或 256）
- 除非手动配置过最大长度

**`sensor_msgs/Image` 消息定义**:
```
# ROS 2 sensor_msgs/Image 消息结构
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data      ← 可变长度数组，默认上限 128
```

### 2.3 数学验证

```
图像尺寸: 100 × 100 × 3 = 30,000 字节

Bus 默认配置: data 最大长度 = 128 字节

30,000 >> 128  → 维度不匹配 ❌
```

### 2.4 问题影响范围

| 字段 | 实际输出 | 总线默认配置 | 状态 |
|------|---------|-------------|------|
| height | 1 (uint32) | 1 | ✅ |
| width | 1 (uint32) | 1 | ✅ |
| encoding | 1×128 (uint8) | 1×128 | ✅ |
| step | 1 (uint32) | 1 | ✅ |
| **data** | **1×30000** | **1×128** | ❌ **不匹配** |

---

## 3. 解决方案 (Solutions)

### 3.1 方案概述

需要修改 ROS 2 消息对应的 **Simulink Bus Object** 的属性，将 `data` 字段的维度扩大。

### 3.2 方案 1: 使用 ros2genmsg 重新生成 Bus（推荐）

在 MATLAB 命令窗口中配置消息属性：

```matlab
% 步骤 1: 获取当前的 Bus 定义
busInfo = ros2("msg", "show", "sensor_msgs/Image");

% 步骤 2: 设置 data 字段的最大长度
% 对于 100x100x3 的图像，需要至少 30000
ros2genmsg('sensor_msgs', 'Image', 'MaxArrayLength', 50000);
```

### 3.3 方案 2: 通过 Simulink Bus Editor 手动修改

1. 在 MATLAB 命令窗口输入 `buseditor` 打开 Bus Editor
2. 找到 `sensor_msgs_Image` 总线定义
3. 找到 `data` 字段
4. 修改 `Dimensions` 属性：
   - 原值: `[128]` 或 `[1 128]`
   - 新值: `[30000]` 或更大（如 `[50000]`，留余量）

### 3.4 方案 3: 使用 MATLAB 脚本配置

```matlab
% 创建或修改 Bus Object
% 设置 data 字段的维度为 [1 50000]

% 获取现有 Bus 定义
busObj = Simulink.Bus.createObject('sensor_msgs/Image');

% 找到 data 元素并修改维度
for i = 1:length(busObj.Elements)
    if strcmp(busObj.Elements(i).Name, 'data')
        busObj.Elements(i).Dimensions = [1 50000];
        busObj.Elements(i).DimensionsMode = 'Fixed';
        break;
    end
end

% 保存修改
assignin('base', 'sensor_msgs_Image', busObj);
```

### 3.5 方案 4: 在模型中使用 Variable-Size Signals

如果图像尺寸可能变化：

1. 打开 Simulink 模型配置参数
2. 导航到 **Diagnostics > Data Validity**
3. 设置 **Signal resolution** 为 **Variable-size signal**
4. 在 Bus Object 中设置 `DimensionsMode = 'Variable'`

---

## 4. 实施步骤 (Implementation)

### 4.1 推荐实施流程

#### Step 1: 确定所需的最大 data 长度 (5分钟)

```matlab
% 计算所需的 data 字段长度
image_height = 100;
image_width = 100;
channels = 3;  % RGB

data_length = image_height * image_width * channels;
fprintf('所需 data 长度: %d\n', data_length);  % 输出: 30000

% 建议设置为 1.5~2 倍余量
recommended_length = ceil(data_length * 1.5);
fprintf('推荐设置长度: %d\n', recommended_length);  % 输出: 45000
```

#### Step 2: 打开 Simulink 模型的 Bus Editor (3分钟)

```matlab
% 方法 1: 使用命令打开
buseditor

% 方法 2: 在 Simulink 中
% 1. 打开模型
% 2. 菜单: View > Bus Editor
```

#### Step 3: 修改 data 字段维度 (5分钟)

1. 在 Bus Editor 左侧树中找到 `sensor_msgs_Image`
2. 展开该总线，找到 `data` 字段
3. 在右侧属性面板中:
   - **Dimensions**: 改为 `50000`（或所需值）
   - **Complexity**: 保持 `real`
   - **Data Type**: 保持 `uint8`
4. 点击 **Apply** 保存更改

#### Step 4: 验证并运行仿真 (5分钟)

```matlab
% 验证 Bus 配置
bus = evalin('base', 'sensor_msgs_Image');
for i = 1:length(bus.Elements)
    fprintf('%s: %s\n', bus.Elements(i).Name, ...
            mat2str(bus.Elements(i).Dimensions));
end

% 运行仿真
sim('ROS2_simulink_UE5_260106');
```

### 4.2 完整验证清单

```yaml
□ Step 1: 确认图像参数
  □ 图像高度 (height): _____ 像素
  □ 图像宽度 (width): _____ 像素
  □ 通道数 (channels): _____
  □ 计算 data 长度: height × width × channels = _____

□ Step 2: 修改 Bus Object
  □ 打开 Bus Editor
  □ 找到 sensor_msgs_Image 总线
  □ 修改 data 字段的 Dimensions
  □ 应用并保存更改

□ Step 3: 验证 MATLAB Function 输出
  □ 检查 data 输出端口维度
  □ 确认与 Bus 定义匹配

□ Step 4: 运行仿真测试
  □ 无端口维度错误
  □ 消息成功发布
  □ 接收端正确解析图像
```

---

## 5. 验证结果 (Verification)

### 5.1 成功指标

- ✅ 仿真无维度不匹配错误
- ✅ MATLAB Function 输出端口与 Bus Assignment 输入端口维度一致
- ✅ ROS 2 Image 消息成功发布
- ✅ 外部节点能正确订阅并显示图像

### 5.2 验证脚本

```matlab
% 验证 Bus Object 配置
function verifyImageBus()
    % 获取 Bus 定义
    if evalin('base', 'exist(''sensor_msgs_Image'', ''var'')')
        bus = evalin('base', 'sensor_msgs_Image');
    else
        error('Bus sensor_msgs_Image not found in base workspace');
    end

    % 检查 data 字段
    dataFound = false;
    for i = 1:length(bus.Elements)
        if strcmp(bus.Elements(i).Name, 'data')
            dataFound = true;
            dims = bus.Elements(i).Dimensions;
            if prod(dims) < 30000
                warning('data 维度 %s 可能不足以容纳 100x100x3 图像', ...
                        mat2str(dims));
            else
                fprintf('✅ data 维度配置正确: %s\n', mat2str(dims));
            end
            break;
        end
    end

    if ~dataFound
        error('data 字段未在 Bus 中找到');
    end
end
```

---

## 6. 经验总结 (Lessons Learned)

### 6.1 核心教训

#### 1. ROS 2 可变长度数组需要手动配置

```
Simulink 对 ROS 2 消息的可变长度数组（如 uint8[] data）
默认使用较小的上限（通常 128 或 256）。

对于图像数据等大数组，必须手动配置最大长度。
```

#### 2. 维度计算公式

```
对于 RGB 图像:
data_length = height × width × 3

对于灰度图像:
data_length = height × width

对于 RGBA 图像:
data_length = height × width × 4
```

#### 3. 建议预留余量

```
推荐配置 = 实际所需 × 1.5 ~ 2.0

原因:
- 避免未来图像尺寸变化时需要重新配置
- 防止边界情况导致的溢出
```

### 6.2 常见图像尺寸参考

| 分辨率 | 通道 | data 长度 | 推荐配置 |
|--------|------|-----------|---------|
| 100×100 | RGB | 30,000 | 50,000 |
| 320×240 | RGB | 230,400 | 350,000 |
| 640×480 | RGB | 921,600 | 1,400,000 |
| 1280×720 | RGB | 2,764,800 | 4,000,000 |
| 1920×1080 | RGB | 6,220,800 | 9,000,000 |

### 6.3 最佳实践

1. **在项目初始化时配置 Bus**
   ```matlab
   % 在模型 InitFcn 回调中配置
   setupROS2ImageBus(max_height, max_width, channels);
   ```

2. **使用参数化配置**
   ```matlab
   % 定义全局参数
   IMAGE_MAX_HEIGHT = 1080;
   IMAGE_MAX_WIDTH = 1920;
   IMAGE_CHANNELS = 3;
   DATA_MAX_LENGTH = IMAGE_MAX_HEIGHT * IMAGE_MAX_WIDTH * IMAGE_CHANNELS;
   ```

3. **添加维度检查**
   ```matlab
   % 在 MATLAB Function 中添加断言
   assert(numel(data) <= DATA_MAX_LENGTH, ...
          'data 长度 %d 超过 Bus 配置的最大长度 %d', ...
          numel(data), DATA_MAX_LENGTH);
   ```

---

## 7. 关联资源 (References)

### 7.1 MathWorks 官方文档

- [Manage Array Sizes in Simulink ROS](https://www.mathworks.com/help/ros/ug/manage-array-sizes-in-simulink-ros.html) - 管理 Simulink ROS 中的数组大小
- [Working with ROS 2 Messages in Simulink](https://www.mathworks.com/help/ros/ug/work-with-ros-2-messages-in-simulink.html) - 在 Simulink 中使用 ROS 2 消息
- [Simulink Bus Editor](https://www.mathworks.com/help/simulink/slref/buseditor.html) - Bus Editor 使用指南

### 7.2 ROS 2 消息定义

**sensor_msgs/Image 消息结构**:
```
# sensor_msgs/msg/Image.msg
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

**常用 encoding 值**:
- `rgb8`: RGB 8-bit per channel
- `bgr8`: BGR 8-bit per channel
- `mono8`: 8-bit 灰度
- `rgba8`: RGBA 8-bit per channel
- `32FC1`: 32-bit 单通道浮点

### 7.3 相关文档

- `DevLogs/BugFixLogs/SimulinkDynamics/GM_Negative_Natural_Frequency_NaN_Issue.md` - Simulink 参数配置问题
- `DevLogs/BugFixLogs/SimulinkDynamics/Ship_Propeller_Trajectory_Issue.md` - Simulink 建模问题

---

## 8. 下一步行动 (Next Steps)

### 8.1 立即行动 (必做)

- [ ] 打开 Simulink Bus Editor
- [ ] 找到 `sensor_msgs_Image` 总线定义
- [ ] 修改 `data` 字段维度为 `50000`（或根据实际图像计算）
- [ ] 保存 Bus 配置
- [ ] 重新运行仿真验证

### 8.2 后续优化 (建议)

- [ ] 创建 Bus 配置初始化脚本
- [ ] 将配置参数化（支持不同分辨率）
- [ ] 添加维度检查断言
- [ ] 编写验证测试脚本

### 8.3 文档完善

- [ ] 补充实际修复后的截图
- [ ] 记录最终配置参数
- [ ] 添加常见问题 FAQ

---

## 9. 附录 (Appendix)

### 附录 A: 完整 MATLAB Function 代码（带注释）

```matlab
function [height, width, encoding, step, data] = createROS2ImageMsg(img_matrix)
    %createROS2ImageMsg 将图像矩阵转换为 ROS 2 Image 消息格式
    %
    % 输入:
    %   img_matrix - [H x W x C] 的 uint8 图像矩阵
    %                H: 高度, W: 宽度, C: 通道数 (1=灰度, 3=RGB, 4=RGBA)
    %
    % 输出:
    %   height   - 图像高度 (uint32)
    %   width    - 图像宽度 (uint32)
    %   encoding - 编码字符串 (uint8 数组, 固定长度 128)
    %   step     - 每行字节数 (uint32)
    %   data     - 图像数据 (uint8 数组, 展平后的像素数据)
    %
    % 注意:
    %   data 输出的长度 = height × width × channels
    %   需要确保 Simulink Bus 中 data 字段的维度足够大

    % 获取图像的尺寸
    [height, width, channels] = size(img_matrix);

    % 设置编码字符串 (固定长度 128 字节)
    encoding_str = 'rgb8';
    encoding = uint8(zeros(1, 128));
    encoding(1:length(encoding_str)) = uint8(encoding_str);

    % 计算每行字节数
    step = uint32(width * channels);

    % 将图像展平为一维字节数组
    % 顺序: 按行展开, 每个像素的通道按 R-G-B 顺序
    data = uint8(reshape(img_matrix, 1, []));

    % 转换输出类型
    height = uint32(height);
    width = uint32(width);
end
```

### 附录 B: Bus 配置脚本

```matlab
function setupROS2ImageBus(max_height, max_width, channels)
    %setupROS2ImageBus 配置 ROS 2 Image 消息的 Bus Object
    %
    % 输入:
    %   max_height - 最大图像高度
    %   max_width  - 最大图像宽度
    %   channels   - 通道数 (默认 3)

    if nargin < 3
        channels = 3;
    end

    % 计算所需的 data 长度
    data_max_length = max_height * max_width * channels;

    % 添加 50% 余量
    data_max_length = ceil(data_max_length * 1.5);

    fprintf('配置 sensor_msgs/Image Bus:\n');
    fprintf('  最大图像尺寸: %d x %d x %d\n', max_height, max_width, channels);
    fprintf('  data 最大长度: %d\n', data_max_length);

    % 创建 Bus Elements
    headerBus = Simulink.Bus;
    % ... (省略 Header 定义)

    % 创建 Image Bus
    imageBus = Simulink.Bus;

    % height element
    height = Simulink.BusElement;
    height.Name = 'height';
    height.Dimensions = 1;
    height.DataType = 'uint32';

    % width element
    width = Simulink.BusElement;
    width.Name = 'width';
    width.Dimensions = 1;
    width.DataType = 'uint32';

    % encoding element
    encoding = Simulink.BusElement;
    encoding.Name = 'encoding';
    encoding.Dimensions = [1 128];
    encoding.DataType = 'uint8';

    % step element
    step = Simulink.BusElement;
    step.Name = 'step';
    step.Dimensions = 1;
    step.DataType = 'uint32';

    % data element (关键配置)
    data = Simulink.BusElement;
    data.Name = 'data';
    data.Dimensions = [1 data_max_length];  % 配置最大长度
    data.DataType = 'uint8';

    % 组装 Bus
    imageBus.Elements = [height; width; encoding; step; data];

    % 保存到 base workspace
    assignin('base', 'sensor_msgs_Image', imageBus);

    fprintf('✅ Bus 配置完成\n');
end
```

### 附录 C: 错误信息解读

| 错误信息关键词 | 含义 | 解决方向 |
|--------------|------|---------|
| "端口宽度或维度出错" | 信号维度与期望不匹配 | 检查 Bus 定义中的 Dimensions |
| "[1x30000] 矩阵" | 实际输出是 30000 元素的向量 | 这是代码正确输出 |
| "128 个元素的一维向量" | Bus 定义只允许 128 元素 | 需要扩大 Bus 中的维度配置 |

---

**记录人**: Claude (AI Assistant) & CXZ
**技术审查**: 待审核
**完成日期**: 2026-01-15
**文档版本**: v1.0

---

**文档状态**: ✅ 完成 - 根因已定位，待实施修复
**最后更新**: 2026-01-15
