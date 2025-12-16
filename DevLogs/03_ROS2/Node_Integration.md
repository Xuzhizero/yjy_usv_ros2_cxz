# ROS2 节点集成记录 (ROS2 Node Integration)

**Tags:** `ros2`, `node-integration`, `communication`, `debugging`, `dds`

**日期**: 2025-12-16
**状态**: 模板 / 待填写

---

## 1. 背景 (Background)

[描述 ROS2 节点集成的背景和需求]

**集成目标**:
- 新增 XXX 功能节点
- 优化现有节点通信
- 修复节点间通信问题
- 提升系统稳定性

**涉及节点**:
- `usvSim` - USV 仿真节点
- `pidController` - PID 控制器节点
- `thrustAllocation` - 推力分配节点

---

## 2. 问题描述 (Problem)

### 2.1 问题现象

[详细描述遇到的问题]

**示例问题**:
- ❌ 节点无法启动
- ❌ 话题数据丢失
- ❌ 通信延迟过高
- ❌ 节点异常退出
- ❌ QoS 策略不匹配

### 2.2 错误信息

```bash
[ERROR] [1702737600.123456789] [my_node]:
Failed to create subscription: incompatible QoS policies

[WARN] [1702737600.234567890] [my_node]:
Topic '/usv/state' not found after 5 seconds

[ERROR] [1702737600.345678901] [my_node]:
Failed to call service '/reset_simulation': timeout
```

### 2.3 环境信息

- **ROS2 版本**: Humble / Foxy
- **DDS 实现**: Fast-DDS / Cyclone DDS
- **操作系统**: Ubuntu 22.04
- **网络环境**: 本地 / 多机通信

### 2.4 复现步骤

```bash
# 1. 启动第一个节点
ros2 run control_planner usvSim

# 2. 在新终端启动第二个节点
ros2 run control_planner pidController

# 3. 观察错误输出
# Expected: 节点正常通信
# Actual: [描述实际现象]
```

---

## 3. 原因分析 (Root Cause)

### 3.1 问题分类

**A. 节点启动问题**
- 环境变量未加载
- 依赖包缺失
- 参数配置错误

**B. 通信问题**
- QoS 策略不匹配
- DDS 域 ID 不一致
- 网络配置错误
- 话题名称拼写错误

**C. 性能问题**
- 消息频率过高
- 消息体过大
- 序列化开销大

**D. 生命周期问题**
- 节点初始化顺序错误
- 回调函数阻塞
- 定时器频率设置不当

### 3.2 根本原因

[深入分析问题根源]

**本次问题分析**:
- **直接原因**: 发布者使用 `RELIABLE` QoS，订阅者使用 `BEST_EFFORT` QoS
- **根本原因**: 未统一配置 QoS 策略
- **触发条件**: 网络波动时消息丢失

**诊断命令**:
```bash
# 查看节点列表
ros2 node list

# 查看话题信息
ros2 topic info /usv/state -v

# 查看 QoS 策略
ros2 topic info /usv/state --verbose

# 监控话题频率
ros2 topic hz /usv/state

# 查看节点信息
ros2 node info /usvSim
```

---

## 4. 处理方案 (Solution)

### 4.1 快速修复

**Step 1: 统一 QoS 策略**

```python
# 发布者端
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(
    Twist,
    '/usv/state',
    qos_profile
)
```

```python
# 订阅者端
self.subscription = self.create_subscription(
    Twist,
    '/usv/state',
    self.listener_callback,
    qos_profile  # 使用相同的 QoS
)
```

**Step 2: 检查环境变量**

```bash
# 确保加载了正确的工作空间
source ~/yjy_usv_ros2_cxz/install/setup.bash

# 检查 ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# 设置相同的域 ID (如果多机通信)
export ROS_DOMAIN_ID=42
```

**Step 3: 验证话题连接**

```bash
# 查看话题列表
ros2 topic list

# 实时监控话题数据
ros2 topic echo /usv/state

# 查看话题连接关系
ros2 topic info /usv/state
```

### 4.2 详细实现方案

#### 方案 1: 优化节点启动顺序

**使用 Launch 文件**:

```python
# launch_usvSim.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 先启动仿真节点
        Node(
            package='control_planner',
            executable='usvSim',
            name='usv_sim',
            output='screen'
        ),

        # 延迟 2 秒后启动控制器
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='control_planner',
                    executable='pidController',
                    name='pid_controller',
                    output='screen'
                )
            ]
        ),
    ])
```

#### 方案 2: 添加健壮性检查

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 等待话题可用
        self.wait_for_topic('/usv/state', timeout_sec=10.0)

        # 创建订阅
        self.subscription = self.create_subscription(
            Twist,
            '/usv/state',
            self.listener_callback,
            10
        )

        # 设置看门狗定时器
        self.watchdog_timer = self.create_timer(
            5.0,  # 5秒检查一次
            self.watchdog_callback
        )
        self.last_message_time = self.get_clock().now()

    def wait_for_topic(self, topic_name, timeout_sec=5.0):
        """等待话题变为可用"""
        start_time = self.get_clock().now()
        while rclpy.ok():
            topics = self.get_topic_names_and_types()
            if topic_name in [t[0] for t in topics]:
                self.get_logger().info(f'Topic {topic_name} is now available')
                return True

            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().error(f'Timeout waiting for topic {topic_name}')
                return False

            time.sleep(0.1)

    def listener_callback(self, msg):
        self.last_message_time = self.get_clock().now()
        # 处理消息

    def watchdog_callback(self):
        """检测通信超时"""
        time_since_last_msg = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9
        if time_since_last_msg > 5.0:
            self.get_logger().warn(f'No message received for {time_since_last_msg:.1f} seconds')
```

#### 方案 3: 性能优化

**减少消息频率**:

```python
# 使用定时器控制发布频率
self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

def timer_callback(self):
    msg = Twist()
    # 填充消息
    self.publisher_.publish(msg)
```

**使用零拷贝通信** (仅支持特定消息类型):

```python
from rclpy.qos import QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions

qos = QoSProfile(depth=10)
qos.use_intra_process_communication = True

self.publisher_ = self.create_publisher(
    Twist,
    '/usv/state',
    qos
)
```

---

## 5. 验证结果 (Verification)

### 5.1 功能验证

**测试清单**:
- [ ] 节点成功启动
- [ ] 话题数据正常发布
- [ ] 订阅端正确接收数据
- [ ] 无错误或警告日志
- [ ] 通信延迟在可接受范围内

**验证命令**:

```bash
# 1. 检查节点运行状态
ros2 node list
# Expected: /usvSim, /pidController

# 2. 检查话题连接
ros2 topic info /usv/state
# Expected: Publishers: 1, Subscribers: 1

# 3. 测量话题频率
ros2 topic hz /usv/state
# Expected: ~50 Hz

# 4. 测量延迟
ros2 topic delay /usv/state
# Expected: < 10ms

# 5. 检查消息内容
ros2 topic echo /usv/state --once
```

### 5.2 性能测试

| 指标 | 优化前 | 优化后 | 目标 |
|------|--------|--------|------|
| 消息频率 | 不稳定 | 50 Hz | 50 Hz |
| 通信延迟 | 50ms | 5ms | < 10ms |
| CPU 占用 | 40% | 15% | < 20% |
| 消息丢失率 | 5% | 0% | 0% |

### 5.3 压力测试

```bash
# 启动多个订阅者
for i in {1..10}; do
    ros2 run control_planner test_subscriber &
done

# 监控系统资源
htop

# 检查消息是否都能正常接收
```

---

## 6. 经验总结 (Lessons Learned)

### 6.1 最佳实践

1. **统一 QoS 配置**:
   - 在配置文件中定义标准 QoS 策略
   - 所有节点使用相同的策略

2. **使用 Launch 文件**:
   - 管理节点启动顺序
   - 统一配置参数
   - 便于调试和部署

3. **添加健壮性检查**:
   - 等待话题可用后再订阅
   - 实现通信超时检测
   - 添加自动重连机制

4. **性能优化**:
   - 合理设置消息频率
   - 避免在回调中进行耗时操作
   - 使用进程内通信 (intra-process)

### 6.2 常见问题速查表

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 节点无法启动 | 环境未加载 | `source install/setup.bash` |
| 话题不可见 | DDS 域 ID 不同 | 统一 `ROS_DOMAIN_ID` |
| QoS 不兼容 | 策略不匹配 | 统一 QoS 配置 |
| 通信延迟高 | DDS 配置不当 | 优化 DDS 参数 |
| 消息丢失 | 队列深度不足 | 增加 `depth` 参数 |
| CPU 占用高 | 回调阻塞 | 使用多线程执行器 |

### 6.3 调试技巧

✅ **推荐工具**:
- `ros2 node` - 节点管理
- `ros2 topic` - 话题调试
- `ros2 bag` - 数据记录与回放
- `rqt_graph` - 可视化节点关系
- `rqt_plot` - 实时数据绘图

**调试流程**:
```bash
# 1. 检查节点
ros2 node list
ros2 node info /my_node

# 2. 检查话题
ros2 topic list
ros2 topic info /my_topic -v

# 3. 监控数据
ros2 topic echo /my_topic
ros2 topic hz /my_topic

# 4. 可视化
rqt_graph
```

---

## 7. 关联资源 (References)

### 7.1 相关 Commit

```bash
# 查找相关提交
git log --all --grep="ros2" --oneline
```

- Commit: `abc1234` - "Add USV simulation node"
- Commit: `def5678` - "Fix QoS compatibility issues"
- Commit: `ghi9012` - "Optimize node communication performance"

### 7.2 相关代码文件

- `control_planner/usvSim.py` - USV 仿真节点
- `control_planner/pidController.py` - PID 控制器节点
- `control_planner/thrustAllocation.py` - 推力分配节点
- `launch/launch_usvSim.launch.py` - 启动文件

### 7.3 参考文档

- [ROS2 QoS 文档](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS2 Launch 文件教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [DDS 调优指南](https://fast-dds.docs.eprosima.com/en/latest/)
- [项目架构文档](../../docs/architecture.md)

### 7.4 相关日志

- [UE5_Simulink_Interface.md](../04_Simulation/UE5_Simulink_Interface.md) - 仿真接口相关

---

**记录人**: [姓名]
**审核人**: [姓名]
**完成日期**: [YYYY-MM-DD]
