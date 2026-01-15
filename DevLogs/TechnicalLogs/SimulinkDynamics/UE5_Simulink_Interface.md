# UE5 与 Simulink 仿真接口 (UE5-Simulink Interface)

**Tags:** `simulation`, `ue5`, `simulink`, `co-simulation`, `interface`, `matlab`

**日期**: 2025-12-16
**状态**: 模板 / 待填写

---

## 1. 背景 (Background)

[描述开发仿真接口的背景和需求]

**需求**:
- 集成 UE5 高保真度视觉仿真
- 利用 Simulink 进行控制算法验证
- 实现双向数据通信
- 支持实时/非实时仿真模式

**应用场景**:
- USV 控制算法验证
- 传感器数据仿真
- 人在环 (Human-in-the-loop) 仿真
- 硬件在环 (Hardware-in-the-loop) 测试

---

## 2. 问题描述 (Problem)

### 2.1 技术挑战

[描述在接口开发过程中遇到的问题]

**主要挑战**:
- ❌ UE5 与 MATLAB 通信协议不兼容
- ❌ 数据同步问题（时间戳不一致）
- ❌ 仿真频率不匹配
- ❌ 数据格式转换开销大
- ❌ 实时性能不满足要求

### 2.2 具体问题

**问题 1: 通信延迟**
```
UE5 (60 FPS) <---???---> Simulink (1000 Hz)
延迟: 50-100ms (不可接受)
目标: < 10ms
```

**问题 2: 数据丢失**
```
[ERROR] Lost 15% of packets during simulation
Packet loss detected between frame 1000-1500
```

**问题 3: 坐标系不一致**
```
UE5: 左手坐标系 (X forward, Y right, Z up)
Simulink: 右手坐标系 (X forward, Y left, Z up)
需要坐标转换
```

---

## 3. 原因分析 (Root Cause)

### 3.1 通信架构分析

**现有方案**:
```
UE5 → UDP Socket → MATLAB Simulink
     ← UDP Socket ←
```

**问题根源**:
- UDP 不保证顺序和可靠性
- 缺少时间同步机制
- 无流量控制

### 3.2 性能瓶颈

**瓶颈识别**:
1. **序列化开销**: JSON 格式序列化耗时 5-10ms
2. **网络延迟**: 本地回环 ~1ms，交换机 ~5ms
3. **Simulink 处理**: 模型计算耗时 ~20ms
4. **UE5 渲染**: 帧渲染耗时 ~16ms (60 FPS)

**关键路径**:
```
Total latency = Serialize + Network + Simulink + Deserialize
             = 7ms + 5ms + 20ms + 7ms
             = 39ms (不满足实时要求)
```

---

## 4. 处理方案 (Solution)

### 4.1 架构设计

**新架构**:
```
┌─────────────────────────────────────────────────────┐
│                    UE5 Simulation                    │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐   │
│  │  Physics   │  │  Rendering │  │  Sensors   │   │
│  └─────┬──────┘  └──────┬─────┘  └─────┬──────┘   │
│        │                 │              │           │
│        └─────────┬───────┴──────────────┘           │
│                  ▼                                   │
│         ┌─────────────────┐                         │
│         │ ROS2 Bridge     │                         │
│         │ (Fast-DDS)      │                         │
│         └────────┬────────┘                         │
└──────────────────┼──────────────────────────────────┘
                   │ ROS2 Topics
                   │ (DDS Protocol)
┌──────────────────┼──────────────────────────────────┐
│                  ▼                                   │
│         ┌─────────────────┐                         │
│         │ ROS2-MATLAB     │                         │
│         │ Interface       │                         │
│         └────────┬────────┘                         │
│                  ▼                                   │
│  ┌────────────────────────────────────────────┐    │
│  │        MATLAB/Simulink Model               │    │
│  │  ┌──────────┐  ┌──────────┐  ┌─────────┐  │    │
│  │  │Controller│  │ Guidance │  │ Planner │  │    │
│  │  └──────────┘  └──────────┘  └─────────┘  │    │
│  └────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────┘
```

**优势**:
- ✅ 使用 ROS2 作为中间层，标准化接口
- ✅ DDS 协议提供可靠通信
- ✅ 易于扩展和调试
- ✅ 支持多语言客户端

### 4.2 详细实现

#### Step 1: UE5 端 ROS2 插件集成

**安装 rclUE 插件**:

```bash
# 克隆 rclUE 插件
cd ~/UnrealEngine/Engine/Plugins
git clone https://github.com/rapyuta-robotics/rclUE.git

# 重新生成项目文件
cd ~/MyUE5Project
./GenerateProjectFiles.sh
```

**UE5 蓝图配置**:

```cpp
// C++ 代码示例
#include "ROS2Node.h"
#include "ROS2Publisher.h"

UCLASS()
class MYPROJECT_API AUSVActor : public AActor
{
    GENERATED_BODY()

public:
    AUSVActor();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

private:
    UROS2NodeComponent* ROS2Node;
    UROS2Publisher* StatePublisher;

    void PublishState();
};

void AUSVActor::BeginPlay()
{
    Super::BeginPlay();

    // 创建 ROS2 节点
    ROS2Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2Node"));
    ROS2Node->Init();

    // 创建发布者
    StatePublisher = ROS2Node->CreatePublisher(
        TEXT("/usv/state"),
        TEXT("geometry_msgs/msg/Twist"),
        UROS2QoS::Default
    );
}

void AUSVActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    PublishState();
}

void AUSVActor::PublishState()
{
    // 获取 USV 状态
    FVector Location = GetActorLocation();
    FRotator Rotation = GetActorRotation();
    FVector Velocity = GetVelocity();

    // 坐标系转换: UE5 (左手) → ROS2 (右手)
    // UE: X(forward), Y(right), Z(up)
    // ROS: X(forward), Y(left), Z(up)
    FVector ROSLocation(Location.X / 100.0f, -Location.Y / 100.0f, Location.Z / 100.0f);  // cm to m
    FVector ROSVelocity(Velocity.X / 100.0f, -Velocity.Y / 100.0f, Velocity.Z / 100.0f);

    // 发布消息
    TSharedPtr<FROSBridgeMsgGeometrymsgstwist> Msg = MakeShareable(new FROSBridgeMsgGeometrymsgstwist());
    Msg->linear.x = ROSVelocity.X;
    Msg->linear.y = ROSVelocity.Y;
    Msg->angular.z = Rotation.Yaw * PI / 180.0f;

    StatePublisher->Publish(Msg);
}
```

#### Step 2: MATLAB/Simulink 端接口

**MATLAB ROS2 工具箱配置**:

```matlab
% setup_ros2_interface.m

% 初始化 ROS2 环境
ros2('disconnect');  % 断开已有连接
ros2('domain', 42);  % 设置域 ID

% 创建节点
node = ros2node('/simulink_controller');

% 创建订阅者（接收 UE5 数据）
sub_state = ros2subscriber(node, '/usv/state', 'geometry_msgs/Twist');

% 创建发布者（发送控制指令）
pub_cmd = ros2publisher(node, '/usv/thrust_cmd', 'pid_interfaces/Command');

% 在 Simulink 中使用
% 1. 添加 "Subscribe" block -> Topic: /usv/state
% 2. 添加 "Publish" block -> Topic: /usv/thrust_cmd
```

**Simulink 模型**:

```
[UE5 State Input] → [Controller] → [Thrust Command Output]
      ↓                   ↓                    ↓
  (Subscribe)        (S-Function)         (Publish)
   /usv/state                           /usv/thrust_cmd
```

**S-Function 示例** (控制算法):

```matlab
function [sys,x0,str,ts] = pid_controller_sfun(t,x,u,flag)
% PID 控制器 S-Function

switch flag
    case 0  % 初始化
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 3;  % [integral, prev_error, prev_time]
        sizes.NumOutputs     = 2;  % [surge_force, yaw_moment]
        sizes.NumInputs      = 6;  % [x, y, theta, vx, vy, omega]
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;
        sys = simsizes(sizes);

        x0  = [0; 0; 0];
        str = [];
        ts  = [0.02 0];  % 50 Hz

    case 2  % 更新离散状态
        % PID 计算
        kp = 2.0;
        ki = 0.1;
        kd = 0.5;

        target_heading = 0;  % 目标航向
        current_heading = u(3);

        error = target_heading - current_heading;
        integral = x(1) + error * 0.02;
        derivative = (error - x(2)) / 0.02;

        sys = [integral; error; t];

    case 3  % 输出
        error = u(3);  % 简化示例
        yaw_moment = 2.0 * error;  % PID 控制
        surge_force = 10.0;  % 常推力

        sys = [surge_force; yaw_moment];

    case {1,4,9}
        sys = [];

    otherwise
        error(['Unhandled flag = ',num2str(flag)]);
end
```

#### Step 3: 图像数据发布 — ROS 2 Write Image Block

Simulink 中的 **ROS 2 Write Image Block** 用于将 Simulink 中的图像数据转换成标准的 ROS 2 `sensor_msgs/Image` 消息，它封装了所有需要的字段（包括 height、width、encoding、step、data 等），无需手动构造复杂的 Bus 或手写 encoding 字符串。

**Block 功能示意图**:

<!-- TODO: 插入 ROS 2 Write Image Block 功能示意图 -->
![ROS 2 Write Image Block 功能示意](./images/ros2_write_image_block_diagram.png)

##### 📥 输入端口（Inputs）

| 端口 | 说明 |
|------|------|
| **Image（必需）** | 传入的图像数据信号（一般是从相机、图像处理模块生成的矩阵） |

**支持的尺寸**:
- 彩色图像：M×N×3
- 灰度图像：M×N

**支持的数据类型**:
- `single`、`double`、`int8`、`uint8`、`uint16` 等

##### 📤 输出端口（Outputs）

| 端口 | 说明 |
|------|------|
| **Msg** | 非虚拟 Bus 信号，表示完整合法的 ROS 2 `sensor_msgs/Image` 消息 |

**输出 Bus 自动包含的标准字段**:
- `header`（可外接 Header Assignment 模块修改）
- `height`, `width`, `step`
- `encoding`（由 Block 参数设置）
- `data`（图像像素矩阵线性展开后的数组）

##### ⚙️ 参数设置（Image Encoding）

在 Block 的参数对话框中可以指定 **Image Encoding**：

| 编码格式 | 说明 |
|----------|------|
| `rgb8`（默认） | RGB 8位彩色图像 |
| `rgba8` | RGBA 8位彩色图像（含透明通道） |
| `mono8` | 8位灰度图像 |
| `bgr8` | BGR 8位彩色图像 |
| ... | 其他 ROS 支持的编码格式 |

此参数决定：
- ✔ `encoding` 字段在消息中的值
- ✔ `data` 字段的解释方式（通道顺序等）

##### 🔌 与 ROS 2 Publish 模块的连接

**Write Image Block 输出的 Msg bus 可以直接连接到 ROS 2 Publish 块**：

```
[图像信号] → Write Image → [Msg (Image)] → ROS2 Publish
```

**Simulink 连线示意**:

<!-- TODO: 插入 Simulink 连线示意图 -->
![Write Image 与 Publish 连接示意](./images/write_image_publish_connection.png)

**在 ROS 2 Publish Block 中配置**:
- Message Type: `sensor_msgs/Image`
- Topic: 设为目标主题（如 `/camera/image_raw`）

##### 🚀 使用优势

| 优势 | 说明 |
|------|------|
| 自动填充 height/width/step | 不用手动做 Bus Assignment |
| 直接设置 encoding 参数 | 避免字符串赋值错误（如 Unsupported Encoding） |
| 数据类型支持丰富 | 自动处理 Simulink 信号数据到 sensor_msgs/Image |
| 与 Publish 块直接兼容 | 可直接连线，发布到 ROS 2 网络 |

##### 📝 使用流程示例

```matlab
% 简单使用流程：
% 1. 把相机/图像矩阵信号连接到 Write Image Block
% 2. 在 Block 参数里选择合适的 encoding（如 rgb8）
% 3. 将 Write Image 输出的 Msg 连接到 ROS2 Publish Block
% 4. 在 Publish Block 设置 Topic、Message Type 即可
```

**完整连接示意**:

<!-- TODO: 插入完整连接示意图 -->
![完整图像发布流程](./images/complete_image_publish_flow.png)

##### 📚 参考文档

> *The Write Image block writes image data to a ROS or ROS 2 image message. You can specify the encoding for the output image message. Use the ROS Publish or ROS 2 Publish block to publish the output image message to an active topic on the network.*
> — [MathWorks 官方文档](https://www.mathworks.com/help/ros/ref/writeimageblock.html)

---

#### Step 3.5: 从文件直接读取图像（Image From File）

在实际开发中，我们常常需要在 Simulink 中读取 JPG/PNG 等图像文件作为输入信号，而不是先在 MATLAB 中将图像存储到变量（如 `img_matrix`）再传递给 Simulink。Simulink 提供了多种直接从文件读取图像的方法，使得图像处理流程更加直观和灵活。

##### 🎯 为什么要直接从文件读取图像？

相比于先在 MATLAB 变量中存储图像再传递给 Simulink，直接从文件读取具有以下优势：

| 优势 | 说明 |
|------|------|
| ✅ 更直观 | 直接指定文件路径，无需手动构造矩阵变量 |
| ✅ 易于调试 | 更换图像只需修改文件路径，无需重新赋值变量 |
| ✅ 批量测试友好 | 便于自动化测试不同图像的处理效果 |
| ✅ 灵活性高 | 更改图像尺寸时流程更灵活 |
| ✅ 标准化流程 | 与 ROS2 Write Image 配合更加标准化 |

##### 📦 方法一：使用 Image From File 模块（推荐）

Simulink 自带的 **Image From File** 模块位于 **Computer Vision Toolbox → Sources** 库中，专门用于从图像文件读取数据。

**如何使用**:

1. 打开 Simulink 模型
2. 从 Library Browser 中找到：
   ```
   Computer Vision Toolbox → Sources → Image From File
   ```
3. 将模块拖入模型
4. 在模块参数中指定图像路径，例如：
   ```
   './test_image.jpg'
   或
   'C:\path\to\your\image.png'
   ```
5. 输出端口会自动输出一个图像矩阵：
   - 彩色图像：M×N×3（RGB）
   - 灰度图像：M×N×1

**与 ROS2 Write Image 配合使用**:

```
[Image From File] → [ROS2 Write Image] → [ROS2 Publish]
     ↓                      ↓                   ↓
  读取图像文件            设置 encoding        发布到 ROS2 话题
  (jpg/png/bmp)          (rgb8/mono8)        (sensor_msgs/Image)
```

**完整示例流程**:

```matlab
% 1. 添加 Image From File 模块
%    - File name: './test.png'
%    - Output data type: uint8

% 2. 添加 ROS2 Write Image 模块
%    - Image Encoding: rgb8 (彩色) 或 mono8 (灰度)

% 3. 添加 ROS2 Publish 模块
%    - Message Type: sensor_msgs/Image
%    - Topic: /camera/image_raw

% 4. 连线并运行
```

**注意事项**:

⚠️ **数据类型要求**:
- ROS2 Write Image 要求输入数据类型为 `uint8`
- 如果 Image From File 输出的数据类型不是 `uint8`，需要在中间添加 **Data Type Conversion** 模块进行转换

⚠️ **图像尺寸**:
- 确保图像尺寸符合预期（例如 ROS2 消息的尺寸要求）
- 可以使用 Image Resize 模块进行尺寸调整

##### 📦 方法二：使用 From Multimedia File 模块

**From Multimedia File** 模块位于 **Computer Vision Toolbox → From Multimedia File** 库中，支持读取多种格式：

**支持的格式**:
- 静态图像：JPG、PNG、BMP、GIF
- 视频文件：MP4、AVI、MOV 等

**特点**:
- 支持时间序列图像帧读取
- 即便只有一张图片也可以输出为单帧
- 适合视频流或连续图像序列的处理

**使用场景**:
```
[From Multimedia File] → [ROS2 Write Image] → [ROS2 Publish]
       ↓                       ↓                    ↓
   视频帧或图像序列         转换为 ROS2 格式       发布图像流
```

##### 📦 方法三：使用 MATLAB Function 读取

如果需要更灵活的控制，可以在 Simulink 中使用 **MATLAB Function** 模块：

```matlab
function img = readImage()
%#codegen
img = imread("./test_image.png");
end
```

**注意事项**:

⚠️ 必须开启代码生成支持（`%#codegen`）
⚠️ 文件路径需要在编译时可访问
⚠️ 可能影响实时性能，不推荐用于硬件在环测试

##### ⏱️ Sample Time 参数详解

在 **Image From File** 模块中，**Sample Time** 参数控制图像更新的频率。

**默认值: `inf` 的含义**

| Sample Time 值 | 含义 |
|---------------|------|
| `inf`（默认） | **常量采样时间**：图像在仿真开始时读取一次，之后保持不变，不随仿真时间更新 |
| `0` | 连续时间信号（适用于连续系统） |
| 正数（如 `0.1`） | 离散采样时间：每隔指定时间（如 0.1 秒）更新一次 |
| `-1` | 继承采样时间：从连接的模块自动继承 |

**`inf` 的实际意义**:

> **`inf` 表示该模块的输出是一个常量信号，在仿真初始化时读取一次，之后不再自动刷新**

这样设计的原因：
- ✅ 对于静态图像（如单张 JPG/PNG），不需要每个时间步都重新读取磁盘
- ✅ 节省计算资源，避免无意义的重复 I/O 操作
- ✅ 保持输出一致性，适合作为恒定的测试输入

**何时保留 `inf`（推荐）**:

✔️ 图像内容不需要随仿真时间变化
✔️ 只发布同一张静态图像
✔️ 用于测试和调试场景
✔️ 节省仿真计算量

**示例流程**:
```
Image From File (Sample Time = inf)
         ↓
    [固定图像输出]
         ↓
ROS2 Write Image (encoding = rgb8)
         ↓
ROS2 Publish (/camera/image_raw)
         ↓
    [发布固定图像到 ROS2]
```

**何时需要修改 Sample Time**:

如果需要图像随时间变化（例如模拟视频流或图像序列），则应设置具体的采样周期：

```
Sample Time = 0.1  % 每 0.1 秒更新一次（10 Hz）
```

**使用场景对比**:

| 场景 | Sample Time 设置 | 说明 |
|------|-----------------|------|
| 发布单张静态图像 | `inf`（默认） | 图像只读取一次，适合测试 |
| 模拟视频流（10 FPS） | `0.1` | 每 0.1 秒读取/更新一次 |
| 模拟相机（30 FPS） | `0.0333` | 每约 33 ms 更新一次 |
| 图像序列处理 | 自定义周期 | 根据实际需求设定更新频率 |

**实际建议**:

对于发布静态图像到 ROS2 的场景：

1. **保留默认 `inf`** —— 图像在仿真开始时读取一次
2. **ROS2 Publish 的采样时间** 可以独立设置为具体值（如 `0.1`），控制消息发布频率
3. 这样既节省资源，又能按需控制发布频率

```
[Image From File (Ts=inf)] → [ROS2 Write Image] → [ROS2 Publish (Ts=0.1)]
        ↓                           ↓                       ↓
   读取一次固定图像              转换为 ROS2 格式         每 0.1s 发布一次
```

##### 🎯 实战建议

**推荐的图像发布流程**:

```
┌──────────────────────┐
│ Image From File      │  Sample Time = inf
│ (test_image.jpg)     │  Output: uint8 (M×N×3)
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ ROS2 Write Image     │  Encoding = rgb8
│                      │  Output: sensor_msgs/Image
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ ROS2 Publish         │  Topic = /camera/image_raw
│                      │  Message Type = sensor_msgs/Image
└──────────────────────┘
```

**关键设置要点**:

| 模块 | 关键参数 | 推荐设置 |
|------|---------|---------|
| Image From File | File name | './test_image.jpg' 或绝对路径 |
| Image From File | Sample Time | `inf`（静态图像） |
| Image From File | Output data type | `uint8` |
| ROS2 Write Image | Image Encoding | `rgb8`（彩色）或 `mono8`（灰度） |
| ROS2 Publish | Topic | `/camera/image_raw` |
| ROS2 Publish | Message Type | `sensor_msgs/Image` |

**验证方法**:

在 ROS2 终端中检查发布的图像：

```bash
# 查看话题列表
ros2 topic list

# 查看图像消息（不显示数组内容）
ros2 topic echo /camera/image_raw --no-arr

# 使用 RViz 可视化图像
ros2 run rviz2 rviz2
# 在 RViz 中添加 Image 显示插件，订阅 /camera/image_raw
```

**常见问题排查**:

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 图像无法显示 | Encoding 设置错误 | 检查 Write Image 的 encoding 参数是否与实际图像格式匹配 |
| 数据类型错误 | 输出不是 uint8 | 添加 Data Type Conversion 模块转换为 uint8 |
| 图像尺寸不符 | 原始图像太大/太小 | 使用 Image Resize 模块调整尺寸 |
| RViz 显示异常 | 话题名称不匹配 | 确认 Publish 的 Topic 名称与 RViz 订阅一致 |

##### 📝 完整示例代码

**MATLAB 脚本设置**:

```matlab
% setup_image_publisher.m
% 配置从文件读取图像并发布到 ROS2

% 1. 检查图像文件是否存在
image_file = './test_images/test.jpg';
if ~isfile(image_file)
    error('图像文件不存在: %s', image_file);
end

% 2. 读取图像查看属性（可选，用于调试）
img = imread(image_file);
fprintf('图像尺寸: %d x %d x %d\n', size(img, 1), size(img, 2), size(img, 3));
fprintf('数据类型: %s\n', class(img));

% 3. 打开 Simulink 模型
model_name = 'image_publisher_model';
open_system(model_name);

% 4. 设置 Image From File 模块参数
set_param([model_name '/Image From File'], ...
    'FileName', image_file, ...
    'SampleTime', 'inf', ...
    'OutputDataType', 'uint8');

% 5. 设置 ROS2 Write Image 模块参数
set_param([model_name '/ROS2 Write Image'], ...
    'Encoding', 'rgb8');  % 彩色图像用 rgb8，灰度用 mono8

% 6. 设置 ROS2 Publish 模块参数
set_param([model_name '/ROS2 Publish'], ...
    'Topic', '/camera/image_raw', ...
    'MessageType', 'sensor_msgs/Image');

% 7. 运行仿真
sim(model_name);
```

**在 ROS2 终端验证**:

```bash
# 查看图像话题信息
ros2 topic info /camera/image_raw

# 查看消息内容（不显示 data 数组）
ros2 topic echo /camera/image_raw --no-arr

# 使用 image_tools 保存接收到的图像
ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
```

##### 📚 方案对比总结

| 方案 | 是否直接读取 JPG/PNG | 是否能发布 ROS2 Image | 易用性 | 推荐度 |
|------|---------------------|---------------------|--------|--------|
| **Image From File** | ✅ | ✅ | ⭐⭐⭐⭐⭐ | 🏆 **强烈推荐** |
| From Multimedia File | ✅ | ✅ | ⭐⭐⭐⭐ | 适合视频流 |
| MATLAB Function + imread | ✅ | ⚠️ 需要配置代码生成 | ⭐⭐⭐ | 仅用于特殊需求 |
| 现有 img_matrix 变量 | ⚠️ 间接 | ✅ | ⭐⭐ | 不推荐 |

---

#### Step 4: 时间同步

**同步策略**:

```cpp
// UE5 端：使用 ROS2 时间
void AUSVActor::PublishState()
{
    // 获取 ROS2 时间戳
    auto now = ROS2Node->GetCurrentTime();

    Msg->header.stamp = now;
    StatePublisher->Publish(Msg);
}
```

```matlab
% MATLAB 端：时间对齐
function output = align_timestamps(ros_msg)
    % 提取 ROS2 时间戳
    ros_time = double(ros_msg.header.stamp.sec) + ...
               double(ros_msg.header.stamp.nanosec) * 1e-9;

    % 与 Simulink 时间同步
    sim_time = get_param(bdroot, 'SimulationTime');

    % 计算延迟
    delay = sim_time - ros_time;

    if delay > 0.05  % 延迟超过 50ms
        warning('High latency detected: %.3f s', delay);
    end

    output = ros_msg;
end
```

### 4.3 性能优化

**优化 1: 减少序列化开销**

```cpp
// 使用二进制格式代替 JSON
// 使用 ROS2 CDR (Common Data Representation)
```

**优化 2: 调整 QoS 策略**

```cpp
// UE5 端
UROS2QoS CustomQoS;
CustomQoS.Reliability = UROS2QoSReliability::BestEffort;  // 降低可靠性要求
CustomQoS.Durability = UROS2QoSDurability::Volatile;
CustomQoS.History = UROS2QoSHistory::KeepLast;
CustomQoS.Depth = 1;  // 只保留最新消息

StatePublisher = ROS2Node->CreatePublisher(
    TEXT("/usv/state"),
    TEXT("geometry_msgs/msg/Twist"),
    CustomQoS
);
```

**优化 3: 使用共享内存** (仅限本地通信)

```bash
# 配置 Fast-DDS 使用共享内存
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_shm.xml
```

```xml
<!-- fastdds_shm.xml -->
<profiles>
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>SHMTransport</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

---

## 5. 验证结果 (Verification)

### 5.1 功能验证

**测试清单**:
- [ ] UE5 数据成功发送到 Simulink
- [ ] Simulink 控制指令成功发送到 UE5
- [ ] 坐标系转换正确
- [ ] 时间戳同步正确
- [ ] 无数据丢失

**验证脚本**:

```bash
# 启动 UE5 仿真
./MyUE5Project.sh

# 在 MATLAB 中运行
matlab -r "run_simulink_test; exit"

# 检查数据
ros2 topic echo /usv/state
ros2 topic echo /usv/thrust_cmd
```

### 5.2 性能测试

| 指标 | 优化前 | 优化后 | 目标 | 达成 |
|------|--------|--------|------|------|
| 端到端延迟 | 39ms | 8ms | < 10ms | ✅ |
| 数据丢失率 | 15% | 0.1% | < 1% | ✅ |
| CPU 占用 (UE5) | 45% | 38% | < 50% | ✅ |
| CPU 占用 (MATLAB) | 30% | 25% | < 40% | ✅ |
| 吞吐量 | 50 msg/s | 100 msg/s | > 50 msg/s | ✅ |

**延迟分解**:
```
Total: 8ms
├─ UE5 Serialize: 1ms
├─ Network (DDS): 1ms
├─ MATLAB Deserialize: 1ms
├─ Simulink Compute: 4ms
└─ Return path: 1ms
```

### 5.3 稳定性测试

```matlab
% 长时间运行测试
duration = 3600;  % 1 小时
sim('usv_simulation', duration);

% 分析结果
analyze_simulation_results('simulation_results.mat');
```

**稳定性指标**:
- ✅ 1 小时连续运行无崩溃
- ✅ 内存占用稳定（无泄漏）
- ✅ 延迟波动 < ±2ms

---

## 6. 经验总结 (Lessons Learned)

### 6.1 最佳实践

1. **选择合适的通信协议**:
   - ROS2 DDS 优于原始 UDP/TCP
   - 提供可靠性和标准化接口

2. **坐标系转换**:
   - 在接口层统一处理坐标系转换
   - 避免在每个模块中重复转换

3. **时间同步**:
   - 使用统一的时间源（ROS2 clock）
   - 记录时间戳用于延迟分析

4. **性能调优**:
   - 使用二进制序列化格式
   - 根据需求选择合适的 QoS 策略
   - 本地通信优先使用共享内存

### 6.2 避免的陷阱

❌ **常见错误**:
- 忘记坐标系转换（导致行为异常）
- QoS 策略不匹配（导致通信失败）
- 时间戳不同步（导致数据关联错误）
- 回调阻塞（导致实时性下降）

### 6.3 可复用的代码模板

**UE5 蓝图模板**: `BP_ROS2_Interface`
**Simulink 模型模板**: `usv_control_template.slx`
**配置文件**: `ros2_ue5_config.yaml`

---

## 7. 关联资源 (References)

### 7.1 相关 Commit

- Commit: `abc1234` - "Add ROS2 bridge in UE5"
- Commit: `def5678` - "Implement MATLAB ROS2 interface"
- Commit: `ghi9012` - "Optimize communication performance"

### 7.2 相关文件

- `Plugins/rclUE/` - UE5 ROS2 插件
- `MATLAB/ros2_interface/` - MATLAB 接口脚本
- `Config/fastdds_config.xml` - DDS 配置

### 7.3 参考文档

- [rclUE 插件文档](https://github.com/rapyuta-robotics/rclUE)
- [MATLAB ROS2 工具箱](https://www.mathworks.com/help/ros/)
- [Fast-DDS 性能调优](https://fast-dds.docs.eprosima.com/en/latest/)
- [项目架构文档](../../docs/architecture.md)

### 7.4 相关日志

- [Node_Integration.md](../03_ROS2/Node_Integration.md) - ROS2 节点集成

---

**记录人**: [姓名]
**审核人**: [姓名]
**完成日期**: [YYYY-MM-DD]
