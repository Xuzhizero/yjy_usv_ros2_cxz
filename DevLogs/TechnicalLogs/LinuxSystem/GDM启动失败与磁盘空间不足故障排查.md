# GDM 启动失败与磁盘空间不足故障排查日志

**故障时间**: 2026-01-12
**系统环境**: Ubuntu (Linux 4.4.0)
**故障现象**: 开机后长时间无法进入图形界面，提示 "failed to start GNOME Display Manager"

---

## 目录

- [一、故障现象与初步诊断](#一故障现象与初步诊断)
  - [1.1 故障表现](#11-故障表现)
  - [1.2 可能原因分析](#12-可能原因分析)
- [二、进入恢复模式](#二进入恢复模式)
  - [2.1 通过 TTY 终端进入](#21-通过-tty-终端进入)
  - [2.2 通过 GRUB 恢复模式进入](#22-通过-grub-恢复模式进入)
  - [2.3 Recovery Mode 下的关键操作](#23-recovery-mode-下的关键操作)
- [三、故障定位：磁盘空间 100% 满](#三故障定位磁盘空间-100-满)
  - [3.1 使用 df -h 检查磁盘](#31-使用-df--h-检查磁盘)
  - [3.2 问题根源确认](#32-问题根源确认)
- [四、紧急清理：释放根目录空间](#四紧急清理释放根目录空间)
  - [4.1 清理 APT 缓存](#41-清理-apt-缓存)
  - [4.2 清理系统日志](#42-清理系统日志)
  - [4.3 清理 Snap 缓存（关键步骤）](#43-清理-snap-缓存关键步骤)
  - [4.4 清理用户缓存](#44-清理用户缓存)
- [五、理解分区结构：du vs df 的区别](#五理解分区结构du-vs-df-的区别)
  - [5.1 为什么 du 和 df 显示不一致](#51-为什么-du-和-df-显示不一致)
  - [5.2 空间占用分析](#52-空间占用分析)
- [六、长期解决方案：移花接木（软链接）](#六长期解决方案移花接木软链接)
  - [6.1 方案对比：调整分区 vs 软链接](#61-方案对比调整分区-vs-软链接)
  - [6.2 将 APT 缓存搬迁到 /home](#62-将-apt-缓存搬迁到-home)
  - [6.3 将 Snap 数据搬迁到 /home](#63-将-snap-数据搬迁到-home)
- [七、注意事项](#七注意事项)
  - [7.1 软件更新的空间占用](#71-软件更新的空间占用)
  - [7.2 Snap 文件夹的安全删除](#72-snap-文件夹的安全删除)
- [八、总结与预防](#八总结与预防)

---

## 一、故障现象与初步诊断

### 1.1 故障表现

- 开机后长时间卡在启动界面，无法进入桌面
- 按 `Win + P` 快捷键查看信息，提示：
  ```
  failed to start GNOME Display Manager
  See 'systemctl status gdm.service' for details
  ```

### 1.2 可能原因分析

| 原因 | 概率 | 说明 |
|------|------|------|
| **磁盘空间已满** | 最高 | GDM 启动时需要写入临时文件，空间满了就会失败 |
| 显卡驱动冲突 | 中等 | 尤其是 NVIDIA 驱动更新后容易出现 |
| 配置文件损坏 | 较低 | GDM 或 Wayland 配置异常 |

---

## 二、进入恢复模式

### 2.1 通过 TTY 终端进入

在卡住的界面，尝试以下快捷键进入纯命令行模式：

```
Ctrl + Alt + F3  (如果没反应，尝试 F2 到 F6)
```

出现 `login:` 提示后，输入用户名和密码登录。

### 2.2 通过 GRUB 恢复模式进入

如果 TTY 方式无效，重启电脑，在 GRUB 菜单选择：

```
Advanced options for Ubuntu → Ubuntu (recovery mode)
```

进入恢复菜单后，选择：
```
root (Drop to root shell prompt)
```

**注意**：选择 root 后可能需要再按一次回车键确认。

### 2.3 Recovery Mode 下的关键操作

⚠️ **重要**：Recovery Mode 下磁盘默认是**只读**模式，必须先执行以下命令获取写权限：

```bash
mount -o remount,rw /
```

（没有任何输出提示是正常的）

---

## 三、故障定位：磁盘空间 100% 满

### 3.1 使用 df -h 检查磁盘

```bash
df -h
```

查看挂载点为 `/` 的那一行，关注 `Use%` 列。

**本次故障的实际情况**：
```
Filesystem      Size  Used  Avail Use%  Mounted on
/dev/sdb6       28G   27G   0     100%  /
```

### 3.2 问题根源确认

根目录 `/` 的使用率达到 100%，这是导致 GDM 无法启动的直接原因。

---

## 四、紧急清理：释放根目录空间

**目标**：只需释放出 1GB 左右的空间，系统就能正常启动。

### 4.1 清理 APT 缓存

```bash
apt clean
# 或
apt-get clean
```

> 两个命令效果完全相同，会清空 `/var/cache/apt/archives/` 目录。

### 4.2 清理系统日志

```bash
# 删除 systemd 日志
rm -rf /var/log/journal/*

# 删除旧的系统日志归档
rm -f /var/log/syslog.*
rm -f /var/log/kern.log.*
rm -f /var/log/messages.*
rm -f /var/log/daemon.log.*
```

### 4.3 清理 Snap 缓存（关键步骤）

```bash
rm -rf /var/lib/snapd/cache/*
```

**本次故障实际效果**：执行此命令后，瞬间释放 2.4GB 空间，Use% 从 100% 降至 92%。

⚠️ **警告**：`/var/lib/snapd/snaps` 目录**绝对不能**直接删除！这里存放的是 Snap 软件的本体，删除会导致软件无法运行。

### 4.4 清理用户缓存

如果上述步骤仍不够，可以清理用户目录下的缓存：

```bash
# 替换 username 为实际用户名
rm -rf /home/username/.cache/*
rm -rf /home/username/.local/share/Trash/*
rm -rf /home/username/.vscode
rm -rf /home/username/.ros/log/*
rm -rf /home/username/anaconda3/pkgs/*.tar.bz2
rm -rf /home/username/anaconda3/pkgs/*.conda
```

---

## 五、理解分区结构：du vs df 的区别

### 5.1 为什么 du 和 df 显示不一致

执行 `du -h --max-depth=1 /` 可能显示 81G，而 `df -h` 显示根目录只有 28G。

这是因为：

| 工具 | 统计方式 | 比喻 |
|------|----------|------|
| `du` | 统计目录下所有文件大小的逻辑总和 | 像清洁工走遍每个房间称重 |
| `df` | 统计分区的物理容量和使用情况 | 像建筑师只看图纸上的房间面积 |

`du` 会把挂载到 `/` 下的其他分区（如 `/home`）和虚拟文件系统（如 `/snap`）都算进去，所以显示的数值会比实际根分区大很多。

### 5.2 空间占用分析

本次故障的实际分区结构：

| 目录 | 大小 | 是否占用根分区(28G)空间 | 说明 |
|------|------|------------------------|------|
| `/home` | 40G | **否** | 独立分区，不占根目录 |
| `/snap` | 16G | **部分** | 虚拟挂载，大部分是只读镜像 |
| `/usr` | 14G | **是** | 系统软件、库文件 |
| `/var` | 8.5G | **是** | 日志、缓存、Docker/Snap 数据 |

**结论**：`/usr` (14G) + `/var` (8.5G) + 其他系统目录 ≈ 26-27G，几乎占满了 28G 的根分区。

---

## 六、长期解决方案：移花接木（软链接）

### 6.1 方案对比：调整分区 vs 软链接

| 方案 | 优点 | 缺点 |
|------|------|------|
| **调整分区 (GParted)** | 系统结构清晰 | 需要 U 盘启动、有数据丢失风险、操作复杂 |
| **软链接搬迁** | 立即可做、安全、效果立竿见影 | 目录结构略显复杂 |

**推荐方案**：软链接搬迁，将根目录下的大文件夹"搬"到 `/home`，通过软链接保持原有路径可用。

### 6.2 将 APT 缓存搬迁到 /home

这样以后软件更新下载的包都会存在 `/home` 中，不再占用根目录空间。

```bash
# 1. 在 /home 创建新仓库
mkdir -p /home/apt_cache

# 2. 复制现有缓存（保留权限）
sudo cp -a /var/cache/apt/* /home/apt_cache/

# 3. 删除原目录
sudo rm -rf /var/cache/apt

# 4. 创建软链接
sudo ln -s /home/apt_cache /var/cache/apt
```

### 6.3 将 Snap 数据搬迁到 /home

Snap 通常是根目录的最大杀手，搬迁后可释放 5GB+ 空间。

```bash
# 1. 停止 Snap 服务
sudo systemctl stop snapd

# 2. 创建目录并复制数据（耗时较长，请耐心等待）
mkdir -p /home/snap_data
sudo cp -a /var/lib/snapd /home/snap_data/

# 3. 备份原目录
sudo mv /var/lib/snapd /var/lib/snapd_backup

# 4. 创建软链接
sudo ln -s /home/snap_data/snapd /var/lib/snapd

# 5. 重启服务
sudo systemctl start snapd

# 6. 验证
snap list

# 7. 确认正常后删除备份
sudo rm -rf /var/lib/snapd_backup
```

---

## 七、注意事项

### 7.1 软件更新的空间占用

软件更新的**下载**和**安装**过程都会占用根目录空间：

- **下载阶段**：占用 `/var/cache/apt/archives/`
- **安装阶段**：占用 `/usr/bin`, `/usr/lib` 等

⚠️ 如果根目录剩余空间不足 5GB，建议暂缓系统更新，先执行上述搬迁方案。

### 7.2 Snap 文件夹的安全删除

| 目录 | 能否直接删除 | 说明 |
|------|-------------|------|
| `/var/lib/snapd/cache/*` | ✅ 可以 | 下载缓存，删除安全 |
| `/var/lib/snapd/snaps/*` | ❌ 绝对不行 | 软件本体，删除会导致软件失效 |

清理 Snap 旧版本的正确方法：

```bash
sudo sh -c 'snap list --all | awk "/disabled/{print \$1, \$3}" | while read snapname revision; do snap remove "$snapname" --revision="$revision"; done'
```

---

## 八、总结与预防

### 故障原因
根目录分区只有 28GB，对于 ROS/仿真开发环境来说严重不足。随着系统使用，`/usr`（系统软件）和 `/var`（日志、缓存）逐渐膨胀，最终导致磁盘 100% 满，GDM 无法启动。

### 解决方法
1. 进入 Recovery Mode
2. 挂载磁盘为可写模式
3. 清理 `/var/lib/snapd/cache/*` 释放空间
4. 重启进入系统
5. 使用软链接将大目录搬迁到 `/home` 分区

### 预防措施
1. 定期检查磁盘空间：`df -h`
2. 定期清理 APT 缓存：`sudo apt clean`
3. 定期清理 Snap 旧版本
4. 将 APT 缓存和 Snap 数据搬迁到 `/home` 分区
5. 对于开发环境，建议根目录分区至少 50GB

---

**文档版本**: 1.0
**最后更新**: 2026-01-12
