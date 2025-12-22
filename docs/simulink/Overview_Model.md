# ROS2_simulink_UE5_0821 模型概述

## 模型简介

本Simulink模型是一个集成了ROS2通信、虚幻引擎5(UE5)可视化的船舶控制仿真系统，用于自主无人船(ASV)的动力学仿真与控制验证。

## 关键统计数据

| 项目 | 数量 |
|------|------|
| 总系统/子系统数 | 58 |
| 总模块数 | 889 |
| 总连线数 | 824 |
| 顶层子系统数 | 7 |

## 层级架构

```
ROS2_simulink_UE5_0821 (Root)
│
├── ROS2simulink (SID:10)          # ROS2输入处理
│   └── 18个模块, 9条连线
│
├── ROS2simulink1 (SID:813)        # 辅助ROS2输入处理
│   └── 8个模块, 5条连线
│
├── Scenario (SID:779)             # 场景控制和环境设置
│   └── 33个模块, 21条连线
│
├── Simulink2ROS (SID:841)         # Simulink到ROS2输出
│   └── 13个模块, 9条连线
│
├── Subsystem (SID:748)            # 船舶平台主系统 ★核心模块★
│   ├── Environment                 # 环境模型
│   ├── Payload                     # 有效载荷模型
│   └── Vessel Platform             # 船舶平台动力学
│       ├── Input Processing
│       ├── Thrust
│       ├── Waves and Crossflow
│       ├── Hydrodynamics
│       ├── Hydrostatic
│       ├── Sum of Forces and Moments
│       ├── 6DOF
│       └── Output Processing
│
├── simulink2UE1 (SID:751)         # UE5通信接口(备用)
│   └── 46个模块, 29条连线
│
└── simulink2UE5 (SID:20)          # UE5通信接口(主)
    └── 23个模块, 13条连线
```

## 数据流总览

### 1. 输入阶段
- **ROS2接口**: 接收外部控制命令和状态数据
- **常数模块**: 提供螺旋桨转速设定值 (左120rpm, 右121rpm)

### 2. 处理阶段
- **Bus Creator**: 组合控制命令为总线信号
- **Subsystem**: 执行船舶动力学计算
- **Scenario**: 管理仿真场景参数

### 3. 输出阶段
- **Bus Selector**: 提取关键状态量
  - Yaw Angle (deg) - 偏航角
  - x (m) - X位置
  - y (m) - Y位置
  - speed (m/s) - 速度
  - Yaw Rate (rad/s) - 偏航率
- **Simulink2ROS**: 发布数据到ROS2
- **simulink2UE5**: 发送可视化数据到虚幻引擎5
- **Scope**: 实时监控关键参数

## 主要连线关系

```
ROS2simulink ─┬─ out:1 ──→ simulink2UE5 in:1 (psi角度)
              ├─ out:2 ──→ simulink2UE5 in:2 (转速n)
              └─ out:3 ──→ simulink2UE5 in:3

PropellerCommand_L_rpm ──→ Bus Creator2 ──→ Subsystem ──→ Scenario
PropellerCommand_R_rpm ──┘

Scenario ──→ Bus Selector ─┬─ Yaw Angle ──→ Simulink2ROS
                           ├─ x ──────────→ Simulink2ROS
                           ├─ y ──────────→ Simulink2ROS
                           ├─ speed ──────→ Scope3
                           └─ Yaw Rate ───→ Simulink2ROS
```

## 系统关键特性

| 特性 | 说明 |
|------|------|
| 双向ROS2通信 | 既能接收也能发送ROS2消息 |
| UE5集成 | 实时3D可视化仿真结果 |
| 模块化设计 | 清晰的功能分离和层级结构 |
| 备份接口 | 包含备用的UE通信接口(simulink2UE1) |
| 完整船舶模型 | 包含环境、载荷和平台动力学 |
| 实时监控 | 多个Scope和Display用于调试 |

## 文档结构导航

| 目录 | 说明 |
|------|------|
| [ros2_interface/](ros2_interface/) | ROS2通信接口模块文档 |
| [ue5_interface/](ue5_interface/) | UE5可视化接口模块文档 |
| [scenario/](scenario/) | 场景控制模块文档 |
| [environment/](environment/) | 环境模型文档 |
| [payload/](payload/) | 载荷模型文档 |
| [vessel_platform/](vessel_platform/) | 船舶平台动力学文档 |

## 模型分析工具

### 接线表导出脚本

如需获取模型的**全量接线表（CSV格式）**和**按系统拆分的Excel文件（xlsx格式）**，可使用以下MATLAB脚本。

#### 功能说明

该脚本可以：
1. 提取模型中所有连线（line）信息，包括源/目标模块、端口序号、所属系统等
2. 导出为CSV文件（`slx_connections_full.csv`）- 包含所有连线的完整信息
3. 导出为Excel文件（`slx_wiring_by_system.xlsx`）- 按系统/子系统拆分为多个sheet

#### 使用方法

1. **将以下代码保存为** `export_slx_connections.m` 文件（保存到MATLAB工作路径或模型所在目录）

2. **在MATLAB命令行窗口中运行**：
   ```matlab
   % 使用默认模型名（ROS2_simulink_UE5_0821）
   export_slx_connections

   % 或指定其他模型名
   export_slx_connections('your_model_name')
   ```

3. **输出文件**：
   - `slx_connections_full.csv` - 全量接线表
   - `slx_wiring_by_system.xlsx` - 按系统分类的接线表

#### 完整代码

```matlab
% =========================================================================
% 导出 Simulink 模型全量接线表
% 功能：
%   1. 提取模型中所有连线（line）信息
%   2. 导出为 CSV 文件（全量）
%   3. 导出为 Excel 文件（按系统/子系统拆分，多 sheet）
% =========================================================================

function export_slx_connections(mdl)
% mdl: 模型名称（不带 .slx 扩展名），例如 'ROS2_simulink_UE5_0821'
%      如果未提供，使用默认值

% ========= 0) 配置 =========
if nargin < 1
    mdl = 'ROS2_simulink_UE5_0821'; % 默认模型名
end

fprintf('正在加载模型: %s\n', mdl);

try
    % 检查模型是否已加载
    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end
catch ME
    error('无法加载模型 %s: %s', mdl, ME.message);
end

% ========= 1) 拉全量连线 =========
fprintf('正在提取连线信息...\n');

try
    lines = find_system(mdl, 'FindAll','on','Type','line');
catch ME
    error('无法查找连线: %s', ME.message);
end

if isempty(lines)
    warning('未找到任何连线！');
    return;
end

fprintf('找到 %d 条连线\n', numel(lines));

% 预分配 cell 数组以提高性能
rows = cell(numel(lines), 7);
validCount = 0;

for i = 1:numel(lines)
    lh = lines(i);

    try
        srcH = get_param(lh,'SrcBlockHandle');
        dstH = get_param(lh,'DstBlockHandle');

        % 跳过无效连线（源或目标为 -1）
        if srcH == -1 || dstH == -1
            continue;
        end

        srcFull = getfullname(srcH);
        dstFull = getfullname(dstH);

        % ---- 端口：尽量转成"端口序号"方便阅读 ----
        srcPH = get_param(lh,'SrcPortHandle');
        dstPH = get_param(lh,'DstPortHandle');

        srcPortNum = localPortIndex(srcH, srcPH, 'Outport');
        dstPortNum = localPortIndex(dstH, dstPH, 'Inport');

        % ---- 所属系统（父层级） ----
        srcSystem = localParentSystem(srcFull);
        dstSystem = localParentSystem(dstFull);

        % ---- 信号名（如果你给线命名过） ----
        sigName = '';
        try
            sigName = get_param(lh,'Name');
            if isempty(sigName)
                sigName = '';
            end
        catch
            % 忽略获取信号名失败的情况
        end

        validCount = validCount + 1;
        rows(validCount, :) = { ...
            srcFull, srcPortNum, srcSystem, ...
            dstFull, dstPortNum, dstSystem, ...
            sigName ...
        };

    catch ME
        warning('处理第 %d 条连线时出错: %s', i, ME.message);
        continue;
    end
end

% 只保留有效行
rows = rows(1:validCount, :);

if validCount == 0
    warning('没有有效的连线数据！');
    return;
end

fprintf('有效连线数: %d\n', validCount);

% 创建表格
T = cell2table(rows, 'VariableNames', { ...
    'SrcBlock','SrcPort','SrcSystem', ...
    'DstBlock','DstPort','DstSystem', ...
    'SignalName'});

% ========= 2) 写全量 CSV =========
csvName = 'slx_connections_full.csv';
fprintf('正在写入 CSV: %s\n', csvName);

try
    writetable(T, csvName);
    fprintf('✓ CSV 文件已生成\n');
catch ME
    warning('写入 CSV 失败: %s', ME.message);
end

% ========= 3) 写按"系统"拆分的 Excel（多 sheet） =========
xlsxName = 'slx_wiring_by_system.xlsx';
fprintf('正在生成 Excel 文件: %s\n', xlsxName);

% 防止旧文件锁死/残留
if exist(xlsxName,'file')
    try
        delete(xlsxName);
    catch
        warning('无法删除旧文件，可能被占用: %s', xlsxName);
    end
end

try
    % 按 SrcSystem 分组
    sysList = unique(T.SrcSystem);
    fprintf('找到 %d 个系统/子系统\n', numel(sysList));

    % 如果只有一个系统，直接写入
    if numel(sysList) == 1
        sheet = localSafeSheetName(sysList{1});
        writetable(T, xlsxName, 'Sheet', sheet);
        fprintf('✓ Excel 文件已生成（1 个 sheet）\n');
    else
        % 多个系统，分别写入不同 sheet
        for k = 1:numel(sysList)
            sys = sysList{k};
            Tk = T(strcmp(T.SrcSystem, sys), :);

            % Excel sheet 名限制 31 字符 + 不允许 : \ / ? * [ ]
            sheet = localSafeSheetName(sys);

            % 如果 sheet 名已存在，添加序号
            if k > 1
                % 检查是否有重名（简化处理，实际中可能需要更复杂的逻辑）
                sheetBase = sheet;
                sheetCounter = 1;
                while any(strcmp(sheet, sysList(1:k-1)))
                    sheet = sprintf('%s_%d', sheetBase, sheetCounter);
                    sheetCounter = sheetCounter + 1;
                    if strlength(sheet) > 31
                        sheet = extractAfter(sheet, strlength(sheet)-31);
                    end
                end
            end

            writetable(Tk, xlsxName, 'Sheet', sheet);
            fprintf('  - Sheet "%s": %d 条连线\n', sheet, height(Tk));
        end
        fprintf('✓ Excel 文件已生成（%d 个 sheet）\n', numel(sysList));
    end

catch ME
    warning('写入 Excel 失败: %s', ME.message);
end

fprintf('\n完成！\n');
fprintf('  - CSV: %s\n', csvName);
fprintf('  - Excel: %s\n', xlsxName);

end

% =========================================================================
% 本地辅助函数
% =========================================================================

function parent = localParentSystem(fullpath)
% 提取块的父系统路径
% fullpath: "model/Sub1/Sub2/Block"
% parent: "model/Sub1/Sub2"
% 如果只有一层，返回原路径

if isempty(fullpath)
    parent = '';
    return;
end

parts = split(fullpath, '/');
if numel(parts) <= 1
    parent = fullpath;
else
    parent = strjoin(parts(1:end-1), '/');
end

end

function sheet = localSafeSheetName(sysPath)
% 将系统路径转换为 Excel 安全的 sheet 名称
% Excel sheet 名称限制：
%   - 最多 31 个字符
%   - 不能包含 : \ / ? * [ ]
%   - 不能以数字开头（某些版本）

if isempty(sysPath)
    sheet = 'Sheet1';
    return;
end

s = char(sysPath);

% 替换不允许的字符
s = regexprep(s, '[:\/?*\[\]]', '_');

% 如果以数字开头，添加前缀
if ~isempty(s) && isstrprop(s(1), 'digit')
    s = ['S_' s];
end

% 限制长度（保留末尾字符，因为通常末尾是子系统名）
if strlength(s) > 31
    s = extractAfter(s, strlength(s)-31);
end

% 如果处理后为空，使用默认名
if isempty(s)
    s = 'Sheet1';
end

sheet = char(s);

end

function portNum = localPortIndex(blockH, portH, portType)
% 将 PortHandle 转换为端口序号（1,2,3...）
% 如果转换失败，返回 NaN

portNum = NaN;

try
    ph = get_param(blockH, 'PortHandles');
    if ~isfield(ph, portType)
        return;
    end

    plist = ph.(portType);
    if isempty(plist)
        return;
    end

    idx = find(plist == portH, 1);
    if ~isempty(idx)
        portNum = idx;
    end
catch
    % 忽略错误，返回 NaN
end

end
```

#### 输出文件说明

**CSV文件列说明**：
- `SrcBlock`: 源模块完整路径
- `SrcPort`: 源模块输出端口序号
- `SrcSystem`: 源模块所属系统
- `DstBlock`: 目标模块完整路径
- `DstPort`: 目标模块输入端口序号
- `DstSystem`: 目标模块所属系统
- `SignalName`: 信号名称（如果已命名）

**Excel文件特点**：
- 每个系统/子系统对应一个独立的sheet
- 便于按系统查看和分析连线关系
- Sheet名称自动处理特殊字符和长度限制

---

## 文档维护流程

本节描述如何维护和更新 Simulink 模型文档。

### 流程概述

1. **模型文件解析与更新**
   - slx 文件上传 → 要求 Claude Code 解析 → 更新 GitHub 中 `/docs` 内部内容

2. **初始变量值导出与更新**
   - 使用 .m 文件导出初始变量值（具体代码可以查看 `/docs` 文档）→ Claude Code 更新文档

### 说明

- 当模型文件（.slx）更新后，应使用 Claude Code 重新解析模型结构
- 模型的初始变量值可以通过专用的 MATLAB 脚本导出
- 导出的数据应同步更新到对应的文档文件中

---

## 相关文档

- [坐标系统定义](坐标系统定义.md) - 模型使用的坐标系定义
- [Simulink模型参数解析](Simulink模型参数解析.md) - 模型参数详解
- [船体规格参数](船体规格参数.md) - 船体尺寸、吨位和推力曲线
- [仿真变量参考](仿真变量参考.md) - 仿真模型变量及导出方法
- [船体水动力参数获取方法](船体水动力参数获取方法.md) - 水动力参数说明
- [Inertials变量来源说明](Inertials变量来源说明.md) - 惯性参数说明
