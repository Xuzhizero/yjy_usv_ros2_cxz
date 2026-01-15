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
