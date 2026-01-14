# 研华工控机与仿真计算机网络连接配置

**Tags:** `network`, `ros2`, `multi-machine`, `static-ip`, `firewall`
**日期**: 2026-01-14
**状态**: 已解决

---

## 1. 背景 (Background)

需要将研华工控机（Ubuntu，用于感知计算）和仿真计算机（Windows）通过网线直连，实现高带宽、低延迟的ROS2数据通信。同时两台机器各自通过Wi-Fi上网。

### 硬件配置概览

| 项目 | 配置 |
|------|------|
| **CPU** | Intel 12th Gen Core i7-12700 (12核/20线程, 最大4.9GHz) |
| **内存** | 31 GB |
| **GPU** | NVIDIA GeForce RTX 4060 (8GB显存, CUDA 12.2) |
| **GPU算力** | FP32 ~15 TFLOPS, FP16 ~120 TFLOPS, INT8 ~180-200 TOPS |
| **操作系统** | Ubuntu (已安装ROS1, 未安装ROS2) |

---

## 2. 网络配置方案

### 2.1 网络拓扑设计

```
[ Ubuntu 工控机 ]              [ Windows 仿真机 ]
   Wi-Fi: 172.16.21.69            Wi-Fi: 172.16.21.81
   有线:  192.168.100.1    ←网线→   有线:  192.168.100.2
        (eno2)                        (以太网)
```

**设计原则：**
- Wi-Fi：负责上网（各自连接路由器）
- 有线网线：专门跑ROS2/仿真大数据流
- 两个网段完全独立，互不干扰

### 2.2 IP配置

| 机器 | 网卡 | IP | 子网掩码 | 网关 |
|------|------|-----|---------|------|
| 工控机(Ubuntu) | eno2 | 192.168.100.1 | 255.255.255.0 | 不填 |
| 仿真机(Windows) | 以太网 | 192.168.100.2 | 255.255.255.0 | 不填 |

**重要：不设置网关！** 这样系统不会把上网流量错误地走到网线。

---

## 3. 配置步骤

### 3.1 Ubuntu工控机配置 (eno2)

**方法：使用 NetworkManager 图形界面**

1. 右上角 → Settings/设置
2. Network/网络
3. 找到 Wired/有线
4. 点击齿轮（⚙️）
5. 切到 IPv4：
   - Method：Manual
   - Address：192.168.100.1
   - Netmask：255.255.255.0
   - Gateway：**留空**
   - DNS：留空
6. 保存
7. 关闭再打开有线连接

**验证：**
```bash
ip addr show eno2
# 应看到: inet 192.168.100.1/24
```

### 3.2 Windows仿真机配置

**步骤（Windows 10/11）：**

1. 控制面板 → 网络和Internet
2. 网络和共享中心
3. 更改适配器选项
4. 右键"以太网" → 属性
5. 双击 "Internet协议版本4 (IPv4)"
6. 选择"使用下面的IP地址"
7. 填写：
   - IP地址：192.168.100.2
   - 子网掩码：255.255.255.0
   - 默认网关：**留空**
   - DNS：留空
8. 确定 → 关闭

**验证：**
```powershell
ipconfig
# 应看到: IPv4 地址: 192.168.100.2
```

---

## 4. Windows防火墙配置

### 4.1 问题：单向ping不通

**现象：** Windows能ping通Ubuntu，但Ubuntu ping Windows失败。

**原因：** Windows防火墙默认阻止ICMP Echo请求（ping）。

### 4.2 解决方案：放行ICMP

1. 打开 Windows Defender 防火墙
2. 高级设置
3. 左侧 → 入站规则
4. 找到：**文件和打印机共享（回显请求 - ICMPv4-In）**
5. 右键 → 启用规则
6. 建议专用和公用配置文件都启用

### 4.3 网络配置文件设置

**有线网络设为"专用"：**
1. 设置 → 网络和Internet → 以太网
2. 点击当前连接的网络
3. 网络配置文件 → 选择"专用（Private）"

---

## 5. ROS2多网卡通信策略

### 5.1 问题：ROS2走哪个网卡？

当机器同时有Wi-Fi和有线时，ROS2(DDS)默认会"两个都用"，不保证只走网线。

### 5.2 方案对比

| 方案 | 优点 | 缺点 |
|------|------|------|
| **方案1：限制Wi-Fi的ROS2能力** | 简单快速 | 依赖防火墙状态 |
| **方案2：DDS绑定有线接口** | 确定性强，可扩展 | 需配置XML |

### 5.3 方案1实施（推荐用于开发阶段）

**目标：** Wi-Fi上网正常，但ROS2只走网线。

**Windows配置：**

1. **WLAN设为公用网络：**
   - 设置 → 网络和Internet → WLAN → 当前Wi-Fi
   - 网络配置文件 → 公用（Public）

2. **关闭Wi-Fi的发现和共享：**
   - 控制面板 → 网络和共享中心
   - 更改高级共享设置
   - 公用网络下：
     - 关闭网络发现
     - 关闭文件和打印机共享

3. **有线保持专用网络且允许通信**

### 5.4 方案2实施（推荐用于生产环境）

**使用CycloneDDS绑定接口：**

**Ubuntu配置：**
```bash
mkdir -p ~/.config/cyclonedds
```

创建 `~/.config/cyclonedds/cyclonedds.xml`：
```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.100.1"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
```

设置环境变量：
```bash
export CYCLONEDDS_URI=file://$HOME/.config/cyclonedds/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**Windows配置：**
创建类似XML，地址改为 `192.168.100.2`，设置环境变量 `CYCLONEDDS_URI`。

---

## 6. 验证方法

### 6.1 验证IP配置
```bash
# Ubuntu
ip addr show eno2

# Windows
ipconfig
```

### 6.2 验证双向ping
```bash
# 从Ubuntu
ping 192.168.100.2

# 从Windows
ping 192.168.100.1
```

### 6.3 验证数据走网线
```bash
# Ubuntu上确认路由
ip route get 192.168.100.2
# 应看到: dev eno2
```

### 6.4 验证ROS2走网线（抓包）
```bash
# 终端A：抓有线
sudo tcpdump -i eno2 udp

# 终端B：抓Wi-Fi
sudo tcpdump -i wlxfc221c200770 udp

# 然后运行ROS2 demo
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

**预期结果：** eno2终端有UDP包刷新，Wi-Fi终端无/极少。

---

## 7. 路由原理说明

### 7.1 为什么不影响Wi-Fi上网？

```bash
ip route
# 输出示例：
# default via 172.16.21.1 dev wlx...    ← Wi-Fi有默认网关
# 192.168.100.0/24 dev eno2              ← 有线无网关
```

- Wi-Fi有默认网关（172.16.21.1）→ 外网流量走Wi-Fi
- 有线无默认网关 → 只处理192.168.100.x网段

| 流量目标 | 走哪张网卡 |
|---------|-----------|
| 192.168.100.x | 有线eno2 |
| 外网/百度/GitHub | Wi-Fi |

---

## 8. 注意事项

### 禁止操作
- ❌ 给有线网卡设置网关
- ❌ 使用和Wi-Fi相同的网段（172.16.21.x）
- ❌ 两张网卡都抢default route

### 最佳实践
- ✅ 有线用独立网段（192.168.100.x）
- ✅ 有线不设网关
- ✅ ROS2端口只在需要时开放

---

## 9. ROS2 vs ROS1网络配置差异

| 项目 | ROS1 | ROS2 |
|------|------|------|
| ROS_MASTER_URI | 必须 | ❌ 不存在 |
| ROS_IP | 必须 | ❌ 不存在 |
| 自动发现 | ❌ | ✅ |
| 主要风险 | Master/IP配错 | 防火墙/DDS网卡选择 |
| 网络变量 | ROS_MASTER_URI, ROS_IP | ROS_DOMAIN_ID |

**ROS2关键设置：**
```bash
export ROS_DOMAIN_ID=10  # 两台机器设相同值
```

---

## 10. 经验总结 (Lessons Learned)

1. **网线直连需手动配置静态IP**：Windows的169.254.x.x是APIPA自动配置，说明没有正确设置。

2. **单向ping失败通常是防火墙问题**：能出不能进是典型Windows防火墙行为。

3. **ROS2多网卡需要主动管理**：DDS会自动使用所有可用网卡，需要通过配置限制。

4. **方案选择取决于阶段**：
   - 开发阶段：用防火墙策略简单快速
   - 生产环境：用DDS接口绑定确保稳定

---

## 11. 关联资源 (References)

- CycloneDDS配置文档: https://cyclonedds.io/docs/
- ROS2网络配置: https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html
- Windows防火墙ICMP设置: Microsoft官方文档
