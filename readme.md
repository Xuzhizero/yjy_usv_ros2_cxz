# 分支介绍
main分支存储的是包含动力学模型的分支，usvSim-pureAlgorithm存储的是不包含动力学模型的分支
# 基本使用方法

运行顺序：
0. 在maltab的命令行输入setenv('ROS_DOMAIN_ID','5')，建立和Ubuntu ROS2的连接
1. 在StartASVDemo.mlx中运行节，分别是"The Simulink Model is ..." 和"Scenario 1: ..."，加载数据
2. 在matlab里面打开UE5
3. 运行ROS2中算法
4. 点击maltab中的run
5. 点击UE5中的play
以上顺序不能颠倒，完成以上步骤以后正式启动。

运行ROS2算法的流程：
1. 进入工作空间ros2_usv_ws
2. source ./install/setup.bash
3. ros2 launch control_planner launch_usvSim.launch.py 
4. ros2 run control_planner keyboard_interaction 

结束顺序：
1. 在matlab里面点击stop
2. 关闭UE5
3. 关闭ROS2中的节点
上面的顺序也不能乱。


# 代码功能基本介绍
usvSimV3.py is the version for rudder-thrust model



urdf....
my_ball_with_rudderV2.urdf is the version for rudder-thrust model.

dynamics_model...
usvDynamicsV2.py: used for rudder-thrust model

launch_usvSimV2.launch.py: a launch file used for launching rudder-thrust model relative files.

keyboard_interactionV2.py: 模拟方向盘，可以根据按键时长实现持续输入

publish_joint_angleV2.py: 可以订阅来自keyboard_interactionV2.py的消息，更新自己的joint_position, 然后发布话题/joint_states, 负责在Rviz中呈现角度的变化

# 现存问题
1. key_interaction， 按同一个键会不断刷出，只需要出现一次就行
2. key_interaction，按不同的键，出现的字符是呈现阶梯性的，而不是换行顶格

---

# 研华工控机硬件配置汇总

## CPU（中央处理器）

| 项目 | 配置 |
| --- | --- |
| 型号 | Intel 12th Gen Core i7-12700 |
| 架构 | x86_64 |
| 核心 / 线程 | 12 核 / 20 线程 |
| 核心结构 | 8 性能核（P-core） + 4 效率核（E-core） |
| 最大主频 | 4.9 GHz |
| 最小主频 | 0.8 GHz |
| L3 缓存 | 25 MB |
| 虚拟化支持 | VT-x（支持 Docker / VM / 仿真） |

## 内存（RAM）

| 项目 | 配置 |
| --- | --- |
| 总内存 | 31 GB |
| 当前已用 | 3.0 GB |
| 可用内存 | 27 GB |
| Swap | 30 GB（当前未使用） |

> 这内存规模对 ROS + 仿真 + 感知算法非常充裕。

## GPU（显卡）

| 项目 | 配置 |
| --- | --- |
| 型号 | NVIDIA GeForce RTX 4060 |
| 显存 | 8 GB |
| 驱动版本 | 535.274.02 |
| CUDA 版本 | 12.2 |
| 当前 GPU 利用率 | ~37%（主要是桌面 / IDE） |
| 当前显存占用 | ~492 MB |

## GPU 算力（推理相关，近似值）

> 说明：`nvidia-smi` 不直接显示 TOPS，下表为官方规格 + 工程常用估算

| 精度 | 理论算力（约） |
| --- | --- |
| FP32 | ~15 TFLOPS |
| FP16 Tensor | ~120 TFLOPS |
| INT8 Tensor | ~180–200 TOPS |

这是桌面级非常强的 AI 推理算力，已经明显高于大多数嵌入式工控方案。

## 系统与软件环境

| 项目 | 状态 |
| --- | --- |
| 操作系统 | Ubuntu（桌面环境：GNOME） |
| ROS | 已安装 ROS1 |
| ROS2 | 已安装 |
| CUDA | 已可用 |
| NVIDIA 驱动 | 正常加载 |
| 桌面 / 远程 | Xorg、TeamViewer、Sunlogin |

## 工程结论

这台研华工控机属于「性能明显偏上」的感知/仿真/推理平台：

- **CPU**：i7-12700 对 ROS 节点调度、点云处理、多进程仿真非常友好
- **GPU（RTX 4060）**：跑 YOLOv8 / BEV / 多相机感知 / TensorRT 推理完全没压力，INT8 推理余量非常大
- **内存**：同时跑仿真 + 感知 + ROS bag + RViz 都绑绑有余

## 定位建议

从工程角度，这台机器可以被定位为：

> **「感知主机 / 推理主机 + 轻量仿真节点」**

完全没必要受限于"工控机算力不足"的思路，它比很多 Jetson / 工控方案都强。