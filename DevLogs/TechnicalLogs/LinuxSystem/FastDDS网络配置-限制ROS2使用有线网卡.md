# 开发日志：FastDDS 网络配置 - 限制 ROS2 只使用有线网卡

**日期**：2026-01-22
**环境**：Ubuntu + ROS2 Humble + YOLOv5 检测系统

---

## 一、前因：问题发现

### 1.1 背景

在将 YOLOv5 检测系统的视频来源从本地文件改为 ROS2 订阅后，发现系统出现明显卡顿：
- 鼠标移动非常卡
- 向日葵远程控制断开
- 即使在本地显示器上操作也感觉卡顿

### 1.2 网络拓扑

| 机器 | 有线 IP | 无线 | 用途 |
|------|---------|------|------|
| Ubuntu（检测） | 192.168.100.1 | 172.16.21.69 | 运行 YOLOv5 检测 |
| Windows（Simulink） | 192.168.100.2 | 有 WiFi | 发布 ROS2 图像话题 |

**预期**：ROS2 图像流走有线网络，上网/远程控制走无线网络。

### 1.3 问题诊断

通过网络流量监控发现：

```
=== 连接 ROS2 后的网络流量 ===
有线 eno2: ~10 MB/s
无线 wlan: ~10 MB/s ← 异常！应该接近 0
```

**两个网卡都有约 10 MB/s 的流量！**

### 1.4 原因分析

ROS2 使用 DDS（FastDDS）作为通信中间件，**DDS 默认行为**：
- 在所有网络接口上发送/接收数据
- 使用多播进行节点发现

导致：
- ROS2 图像流同时在有线和无线网卡上传输
- 无线网卡带宽被占满
- 向日葵远程控制被挤断

---

## 二、过程：配置 FastDDS

### 2.1 配置思路

**目标**：让 ROS2/FastDDS 只使用有线网卡 `eno2`（192.168.100.1）

**方法**：
1. 创建 FastDDS XML 配置文件，指定网卡白名单
2. 设置环境变量 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向该文件
3. 确保 bridge 子进程也继承该配置

### 2.2 创建配置文件

文件位置：`/home/usv-6081/ros2_usv_ws/fastdds_config.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <!-- 定义只使用有线网卡的传输 -->
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>WiredOnlyTransport</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>192.168.100.1</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>

        <!-- 默认参与者配置：使用上述传输，禁用内置传输 -->
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>WiredOnlyTransport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### 2.3 设置环境变量（永久生效）

在 `~/.bashrc` 中添加：

```bash
# FastDDS 配置：限制 ROS2 只使用有线网卡
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/usv-6081/ros2_usv_ws/fastdds_config.xml
```

### 2.4 修复 bridge 子进程问题

在 LoadROS2 类中，bridge 子进程需要继承 FastDDS 配置。修改 `utils/dataloaders.py`：

```python
# Start bridge subprocess with system Python (for ROS2 compatibility)
# The bridge needs ROS2 environment and FastDDS config
import os
fastdds_config = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
fastdds_export = f"export FASTRTPS_DEFAULT_PROFILES_FILE={fastdds_config} && " if fastdds_config else ""
cmd = f"source /opt/ros/humble/setup.bash && {fastdds_export}/usr/bin/python3 {bridge_script} {topic_name} {self.port}"
```

---

## 三、后果：验证结果

### 3.1 网络流量对比

| 网卡 | 配置前 | 配置后 | 变化 |
|------|--------|--------|------|
| 有线 eno2 | ~10 MB/s | ~145-216 MB/s | 正常（全速接收） |
| 无线 wlan | ~10 MB/s | ~35-50 KB/s | ↓ 降低 99.5% ✅ |

### 3.2 问题解决

✅ 无线网卡流量接近 0
✅ 向日葵远程控制不再断开
✅ 系统响应恢复正常
✅ 鼠标移动流畅

### 3.3 检测功能正常

```
ROS2 /usvimage_from_matlab (frame 1142): 640x640 1 fishing_boat, 1 passenger_ship, 1 cargo_ship, 1 bridge, 5.4ms
```

- 每帧推理：~5-7ms
- 检测目标：fishing_boat, passenger_ship, cargo_ship, bridge 等

---

## 四、使用方法

### 4.1 启动检测

```bash
conda activate perception_trt
cd /home/usv-6081/detect
python detect.py --ros2 --view-img
```

### 4.2 使用 TensorRT 加速

```bash
python detect.py --ros2 --view-img --weights v10.engine
```

### 4.3 参数说明

| 参数 | 说明 |
|------|------|
| `--ros2` | 启用 ROS2 模式 |
| `--view-img` | 显示 OpenCV 窗口 |
| `--ros2-topic /xxx` | 指定 topic（默认 /usvimage_from_matlab） |
| `--nosave` | 不保存结果视频 |
| `--weights xxx.engine` | 使用 TensorRT 引擎 |

---

## 五、回滚方法

如需恢复默认行为（ROS2 使用所有网卡）：

```bash
# 方法1：临时
unset FASTRTPS_DEFAULT_PROFILES_FILE

# 方法2：永久
# 编辑 ~/.bashrc，删除 FASTRTPS_DEFAULT_PROFILES_FILE 相关行
sed -i '/FASTRTPS_DEFAULT_PROFILES_FILE/d' ~/.bashrc
sed -i '/FastDDS 配置/d' ~/.bashrc
```

---

## 六、Windows 系统配置（可选）

如果 Windows（Simulink）端无线带宽也紧张，可以类似配置：

1. 创建 `C:\Users\用户名\fastdds_config.xml`（内容同上，IP 改为 192.168.100.2）
2. 设置系统环境变量 `FASTRTPS_DEFAULT_PROFILES_FILE`
3. 重启 MATLAB/Simulink

**当前结论**：只配置 Ubuntu 端即可解决问题，Windows 端暂不需要配置。

---

## 七、关键文件清单

| 文件 | 说明 |
|------|------|
| `/home/usv-6081/ros2_usv_ws/fastdds_config.xml` | FastDDS 网络配置 |
| `/home/usv-6081/detect/utils/dataloaders.py` | LoadROS2 类（含 bridge 子进程修复） |
| `/home/usv-6081/detect/utils/ros2_bridge.py` | ROS2 图像桥接脚本 |
| `/home/usv-6081/detect/detect.py` | 检测入口（含 --ros2 参数） |
| `~/.bashrc` | 环境变量配置 |
