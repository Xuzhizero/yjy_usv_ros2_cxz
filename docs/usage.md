# 使用指南 (Usage Guide)

**Tags:** `usage`, `tutorial`, `api`, `examples`, `getting-started`

**最后更新**: 2025-12-16

---

## 目录

- [使用指南 (Usage Guide)](#使用指南-usage-guide)
  - [目录](#目录)
  - [1. 环境准备](#1-环境准备)
    - [1.1 系统要求](#11-系统要求)
    - [1.2 安装依赖](#12-安装依赖)
  - [2. 构建与安装](#2-构建与安装)
    - [2.1 克隆仓库](#21-克隆仓库)
    - [2.2 构建工作空间](#22-构建工作空间)
  - [3. 快速开始](#3-快速开始)
    - [3.1 启动仿真](#31-启动仿真)
    - [3.2 手动控制](#32-手动控制)
    - [3.3 自动导航](#33-自动导航)
  - [4. 控制器使用](#4-控制器使用)
    - [4.1 PID 控制器](#41-pid-控制器)
    - [4.2 MFAC 控制器](#42-mfac-控制器)
    - [4.3 S-plane 控制器](#43-s-plane-控制器)
  - [5. 路径规划](#5-路径规划)
    - [5.1 航点导航](#51-航点导航)
    - [5.2 曲线跟踪](#52-曲线跟踪)
  - [6. 仿真配置](#6-仿真配置)
    - [6.1 USV 参数](#61-usv-参数)
    - [6.2 环境参数](#62-环境参数)
  - [7. 可视化与调试](#7-可视化与调试)
    - [7.1 RViz2 可视化](#71-rviz2-可视化)
    - [7.2 数据记录与回放](#72-数据记录与回放)
    - [7.3 实时绘图](#73-实时绘图)
  - [8. API 参考](#8-api-参考)
    - [8.1 控制器 API](#81-控制器-api)
    - [8.2 制导律 API](#82-制导律-api)
    - [8.3 推力分配 API](#83-推力分配-api)
  - [9. 常见问题](#9-常见问题)
  - [10. 进阶使用](#10-进阶使用)
    - [10.1 自定义控制器](#101-自定义控制器)
    - [10.2 集成外部传感器](#102-集成外部传感器)
  - [11. 性能优化](#11-性能优化)
  - [12. 故障排查](#12-故障排查)

---

## 1. 环境准备

### 1.1 系统要求

- **操作系统**: Ubuntu 20.04 (Foxy) / 22.04 (Humble)
- **ROS2 版本**: Foxy / Humble
- **Python 版本**: 3.8 或更高
- **内存**: 至少 4GB RAM
- **存储**: 至少 5GB 可用空间

### 1.2 安装依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装 ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop -y

# 安装 Python 依赖
sudo apt install python3-pip python3-numpy python3-matplotlib -y
pip3 install scipy

# 安装 ROS2 开发工具
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# 初始化 rosdep (首次使用)
sudo rosdep init
rosdep update
```

---

## 2. 构建与安装

### 2.1 克隆仓库

```bash
# 克隆项目
git clone <repository-url> ~/yjy_usv_ros2_cxz
cd ~/yjy_usv_ros2_cxz
```

### 2.2 构建工作空间

```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build

# 加载环境
source install/setup.bash

# 验证安装
ros2 pkg list | grep control_planner
```

---

## 3. 快速开始

### 3.1 启动仿真

```bash
# 启动 USV 仿真
ros2 launch control_planner launch_usvSim.launch.py

# 在新终端中查看话题
ros2 topic list
ros2 topic echo /usv/state
```

### 3.2 手动控制

```bash
# 启动键盘控制节点
ros2 run control_planner keyboard_interaction

# 控制说明:
# w - 前进
# s - 后退
# a - 左转
# d - 右转
# space - 停止
```

### 3.3 自动导航

```bash
# 启动自动导航节点
ros2 run control_planner velocityController

# 发送目标航点
ros2 topic pub /mission/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, \
    pose: {position: {x: 10.0, y: 5.0, z: 0.0}}}"
```

---

## 4. 控制器使用

### 4.1 PID 控制器

**启动节点**:

```bash
ros2 run control_planner courseController
```

**参数配置** (`PIDParam.py`):

```python
# 航向控制 PID 参数
heading_kp = 2.0
heading_ki = 0.1
heading_kd = 0.5

# 速度控制 PID 参数
speed_kp = 1.5
speed_ki = 0.05
speed_kd = 0.3
```

**Python API**:

```python
from control_planner.PIDControl import PIDController

# 创建控制器实例
controller = PIDController(kp=2.0, ki=0.1, kd=0.5)

# 计算控制量
control_output = controller.compute(
    setpoint=target_heading,
    measured_value=current_heading,
    dt=0.02  # 时间步长
)
```

### 4.2 MFAC 控制器

**启动节点**:

```bash
ros2 run control_planner sim_MFAC
```

**参数配置**:

```python
# MFAC 参数
rho = 0.5      # 步长因子
lambda_ = 1.0  # 权重系数
eta = 1.0      # 学习率
mu = 0.1       # 遗忘因子
```

### 4.3 S-plane 控制器

**启动节点**:

```bash
ros2 run control_planner controllerBasedOnSplane
```

---

## 5. 路径规划

### 5.1 航点导航

```bash
# 启动路径生成器
ros2 run control_planner generate_curve

# 发送航点序列
ros2 topic pub /path/waypoints nav_msgs/Path \
  "{poses: [ \
    {pose: {position: {x: 0, y: 0}}}, \
    {pose: {position: {x: 10, y: 0}}}, \
    {pose: {position: {x: 10, y: 10}}} \
  ]}"
```

### 5.2 曲线跟踪

```python
from control_planner.curveGenerator import CurveGenerator

# 创建曲线生成器
curve_gen = CurveGenerator()

# 生成圆形路径
waypoints = curve_gen.generate_circle(
    center=(0, 0),
    radius=10.0,
    num_points=50
)

# 生成 B 样条曲线
waypoints = curve_gen.generate_bspline(
    control_points=[(0,0), (5,10), (10,5), (15,0)],
    degree=3
)
```

---

## 6. 仿真配置

### 6.1 USV 参数

**编辑** `usvParam.py`:

```python
# USV 物理参数
mass = 50.0        # 质量 (kg)
length = 1.5       # 长度 (m)
width = 0.8        # 宽度 (m)

# 推进器参数
max_thrust = 20.0  # 最大推力 (N)
thruster_distance = 0.4  # 推进器间距 (m)
```

### 6.2 环境参数

```python
# 风力干扰
wind_speed = 2.0      # m/s
wind_direction = 45   # degrees

# 水流干扰
current_speed = 0.5   # m/s
current_direction = 90  # degrees
```

---

## 7. 可视化与调试

### 7.1 RViz2 可视化

```bash
# 启动预配置的 RViz
ros2 launch control_planner launch_usvSim.launch.py

# 或手动启动
rviz2 -d src/control_planner/launch/test_follow.rviz
```

### 7.2 数据记录与回放

```bash
# 记录所有话题
ros2 bag record -a -o my_experiment

# 回放数据
ros2 bag play my_experiment.db3

# 查看包信息
ros2 bag info my_experiment.db3
```

### 7.3 实时绘图

```bash
# 启动数据绘图节点
ros2 run control_planner dataPlotter

# 或使用 rqt_plot
rqt_plot /usv/state/linear/x /usv/state/angular/z
```

---

## 8. API 参考

### 8.1 控制器 API

**PIDController**

```python
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        """初始化 PID 控制器"""

    def compute(self, setpoint: float, measured_value: float, dt: float) -> float:
        """计算控制输出"""

    def reset(self):
        """重置积分器"""
```

### 8.2 制导律 API

**LOSGuidance**

```python
class LOSGuidance:
    def __init__(self, lookahead_distance: float):
        """初始化 LOS 制导律"""

    def compute_heading(self, current_pos: tuple, target_pos: tuple) -> float:
        """计算目标航向角"""
```

### 8.3 推力分配 API

```python
def allocate_thrust(surge_force: float, yaw_moment: float) -> tuple:
    """
    推力分配函数

    Args:
        surge_force: 纵向力 (N)
        yaw_moment: 偏航力矩 (N·m)

    Returns:
        (left_thrust, right_thrust): 左右推进器推力 (N, N)
    """
```

---

## 9. 常见问题

**Q1: 构建失败，找不到 pid_interfaces**

```bash
# 先单独构建接口包
colcon build --packages-select pid_interfaces
source install/setup.bash
colcon build
```

**Q2: 节点启动后无输出**

```bash
# 检查话题连接
ros2 node list
ros2 topic list
ros2 topic hz /usv/state
```

**Q3: 控制效果不佳**

- 调整 PID 参数
- 检查推力限幅设置
- 查看仿真频率是否稳定

---

## 10. 进阶使用

### 10.1 自定义控制器

```python
# 创建文件: control_planner/MyController.py

from control_planner.PIDControl import PIDController

class MyController:
    def __init__(self):
        self.pid = PIDController(kp=2.0, ki=0.1, kd=0.5)

    def compute_control(self, state, target):
        # 自定义控制逻辑
        error = target - state
        control = self.pid.compute(target, state, dt=0.02)
        return control
```

### 10.2 集成外部传感器

```python
import rclpy
from sensor_msgs.msg import Imu

class MySensorNode(Node):
    def __init__(self):
        super().__init__('my_sensor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # 处理 IMU 数据
        pass
```

---

## 11. 性能优化

- **减少日志输出**: 调整 ROS2 日志级别
  ```bash
  ros2 run control_planner usvSim --ros-args --log-level WARN
  ```

- **提高仿真频率**: 修改 `usvSim.py` 中的 `timer_period`

- **使用多线程执行器**:
  ```python
  executor = MultiThreadedExecutor()
  executor.add_node(node)
  ```

---

## 12. 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 节点无法启动 | 环境未加载 | `source install/setup.bash` |
| 话题无数据 | 节点未连接 | 检查话题名称和节点状态 |
| 控制振荡 | PID 参数不当 | 减小 Kp 或 Kd |
| CPU 占用高 | 循环频率过高 | 降低节点更新频率 |

---

**相关文档**:
- [项目概览](overview.md)
- [系统架构](architecture.md)
- [开发日志](../DevLogs/README.md)

---

**技术支持**:
- 提交 Issue: [GitHub Issues]
- 查看开发日志: [DevLogs/](../DevLogs/)
