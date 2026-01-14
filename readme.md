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

---

# Windows 仿真计算机硬件配置汇总

> 此计算机运行 Windows 系统，主要用于 MATLAB/Simulink 仿真和 UE5 可视化渲染。

## 核心配置汇总表

| 硬件项目 | 详细型号 / 参数 | 备注 |
| --- | --- | --- |
| **CPU (处理器)** | Intel Core i7-13700K | 13代酷睿, 16核心 (8P+8E), 24线程 |
| **RAM (内存)** | 32 GB | 总容量 |
| **GPU (显卡)** | NVIDIA GeForce RTX 4070 | 主力显卡 (用于AI/渲染) |
| **VRAM (显存)** | 12 GB (GDDR6X) | 实际可用显存 (以 `nvidia-smi` 为准) |
| **AI 算力 (TOPS)** | 约 466 TOPS | 基于 RTX 4070 INT8 (Sparse) 理论峰值 |
| **显卡驱动** | Version 572.16 | CUDA 版本 12.8 |

## 详细解读

### AI 算力 (TOPS)

系统算力主要由显卡提供。**RTX 4070** 是运行本地大模型（LLM）和 AI 绘图（Stable Diffusion）的高性能显卡。

| 精度 | 理论算力（约） |
| --- | --- |
| AI 理论算力 | ~466 AI TOPS (INT8, 稀疏化 Tensor Core) |
| FP16 算力 | ~29 TFLOPS (常用于非量化模型的训练/推理) |

> **注意：** i7-13700K 虽然强大（Geekbench 多核分约 1.7万+），但没有集成独立的 NPU 单元，因此在涉及"AI TOPS"计算时通常不计入主力，AI 任务主要依赖 GPU。

### 显存容量 (VRAM)

- **实际容量：** 12 GB
- **注意：** PowerShell `Get-CimInstance` 命令可能显示 `4 GB`，这是 Windows WMI 接口常见的识别错误（经常无法正确识别大于 4GB 的显存），**请始终以 `nvidia-smi` 的输出结果 (`12282 MiB`) 为准**
- **用途：** 12GB 显存对于运行 7B~13B 参数量的量化大模型（如 Llama 3 8B, Qwen 14B-Int4）是非常舒适的甜点配置

### 其他发现

- **虚拟显示器：** 系统中检测到 `OrayIddDriver Device`，这是向日葵 (Sunlogin) 或类似远程控制软件创建的虚拟显示适配器，用于在无物理显示器连接时进行远程桌面控制，不影响性能
- **内存：** 32GB 系统内存配合 12GB 显存，是目前非常标准的 AI 开发与游戏配置

## 工程结论

这台 Windows 仿真计算机属于高性能仿真与渲染平台：

- **CPU (i7-13700K)**：比工控机的 i7-12700 更强，16核24线程对 MATLAB 并行计算、UE5 渲染非常友好
- **GPU (RTX 4070)**：12GB 显存比工控机的 RTX 4060 (8GB) 更充裕，UE5 高画质渲染无压力
- **定位**：主要承担 MATLAB/Simulink 仿真运算 + UE5 场景渲染，与 Ubuntu 工控机通过 ROS2 进行通信协同