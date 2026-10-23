# rebot_taccap_identification

用**静态力矩**辨识 reBot B601 灵足版（RS / RobStride）和达妙版（DM）机械臂各连杆的**质量和质心**，以及末端负载（C1 转接件 + TacCap G1 夹爪）的质量和质心，用于重力补偿。

DM 使用 `--robot.type=rebot_b601_dm`，支持 USB 串口及 SocketCAN，按单臂独立辨识。详见 [DM 接线、模型和完整辨识流程](src/rebot_taccap_identification/robots/rebot_b601_dm/README.md)。静态辨识不包含转动惯量和速度摩擦的动态辨识。

- 数据来自几十个静止位姿的保持力矩。
- 辨识结果写成 URDF（质量、质心替换为辨识值，关于质心的惯量保持原值），可以导出带网格的完整 URDF 包。
- 带图形工作台：3D 预览、碰撞检查、实时曲线、重力补偿阻尼调节，全程按步骤操作。
- 示例：[examples/rebot_b601_rs_taccap_v2](examples/rebot_b601_rs_taccap_v2/) 是一台改装机械臂的完整数据和辨识结果，不接硬件也能复现；导出的 URDF 包在 GitHub Release。

## 目录

1. [安装](#1-安装)
2. [流程](#2-流程)
3. [工作台逐步操作](#3-工作台逐步操作)
4. [结果文件](#4-结果文件)
5. [命令行](#5-命令行)
6. [方法](#6-方法)
7. [安全](#7-安全)

## 1. 安装

pip 分发包名、Python 导入名和启动命令均为 `rebot_taccap_identification`。

夹爪模型由第三方仓库 [XTac-UMI-Description](https://github.com/XenseRobotics-AI/XTac-UMI-Description) 提供。源码安装前，在项目根目录初始化或更新夹爪子模块：

```bash
git submodule update --init --remote -- third_party/XTac-UMI-Description
pip install -e .
```

子模块配置跟踪 `main`，不锁定 tag 或发布版本。Git 提交仍会记录子模块当时的快照；上面的 `--remote` 命令会拉取最新 `main`。更新后重启工作台。运行辨识时只读取本地资产，不自动联网更新。

已有独立夹爪仓库时也可以直接使用它：

```bash
rebot_taccap_identification studio --robot.gripper_repo=../XTac-UMI-Description
```

wheel 不包含第三方仓库；在源码目录之外使用 wheel 时，通过 `--robot.gripper_repo=<仓库目录>` 指定已初始化的夹爪仓库，也可以用 `--robot.gripper_urdf=<URDF文件>` 指定其它独立夹爪模型。

需要 Python ≥ 3.12，Linux（安全停止通过 SIGINT 实现）。

RS 的 CAN 口（PCAN-USB；DM 默认 USB 串口不需要此步骤）：

```bash
#套件里是 PCAN-USB，通常应该直接出现 can0 或 can1
sudo modprobe peak_usb
ip -br link

#如果出现 can0，再设置 bitrate
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

启动工作台：

```bash
# 默认 can0
rebot_taccap_identification studio
# 其它 CAN 口
rebot_taccap_identification studio --robot.channel=can1
# DM：指定当前机械臂的串口
rebot_taccap_identification studio --robot.type=rebot_b601_dm --robot.channel=/dev/ttyACM0
```

RS 机械臂和 C1 转接件的 URDF、网格位于 `src/rebot_taccap_identification/robots/rebot_b601_rs/urdf/rebot_b601_rs/`（来源见 [sources.json](src/rebot_taccap_identification/robots/rebot_b601_rs/urdf/rebot_b601_rs/sources.json)，许可见 [LICENSES/](src/rebot_taccap_identification/robots/rebot_b601_rs/urdf/rebot_b601_rs/LICENSES/)）。夹爪从 `third_party/XTac-UMI-Description` 的资产清单读取裸从夹爪 URDF；运行时与机械臂、C1 装配成带相对网格路径的资源快照，夹爪模型和网格的源文件仍由第三方仓库提供。`link6` 是第六轴输出面的虚拟坐标系，没有质量；C1 和夹爪挂在它下面，整体参与负载辨识。电机配置、默认增益、软限位见 [src/rebot_taccap_identification/robots/rebot_b601_rs/README.md](src/rebot_taccap_identification/robots/rebot_b601_rs/README.md)。

默认机械臂路径使用 `package://rebot_taccap_identification/...` 资源标识，外部仓库可以使用项目相对路径。运行时组合模型会生成含相对网格引用的资源快照；辨识模型同样携带 `meshes/` 和许可文件，移动整个模型目录仍可加载。新结果 JSON 的文件引用相对于 JSON 所在目录解析。

## 2. 流程

```text
 0 设置与连接    检查每个关节能否通信
      ↓
 1 静态核对      在几十个静止位姿测保持力矩（拆掉 C1 和夹爪）
      ↓            → 检查方向、零位，测出力矩标定，得到辨识数据
 2 辨识质量质心  静态数据 → 各连杆质量、质心 → 新 URDF，给出验收表
      ↓
 3 重力补偿      手拖机械臂检验；松手后自己转动的地方自动生成补数据位姿
      ↓
 仍有漂移 → 回到第 1 步采补数据位姿，再辨识一轮
      ↓
 4 负载辨识      装上 C1 和夹爪，采静态数据，只辨识它们合起来的质量、质心
      ↓
 5 导出 URDF     以原始 URDF 为模板导出完整的包（urdf/ + meshes/）
```

工作台默认：50 个训练位姿、20 个换种子的验证位姿；默认保持时间下每个位姿约 9 秒。

## 3. 工作台逐步操作

右侧是 3D 视图（碰撞检查结果）和曲线。顶部红色按钮（或 Esc / 空格）随时**安全停止**：第一次回休息位再失能，第二次立即失能。

### 第 0 步：设置与连接

- 点**连接**：只读检测每个关节是否应答，不使能、不运动。
- **桌面高度**：碰撞检查用，以 URDF 基坐标系的 z 为准。
- **力矩标定**：电机反馈力矩与指令力矩之比（静态核对中测得，假定指令等于实际输出）。第 1 步做完后点“采用测得的力矩标定”自动填入，以后辨识都会把实测力矩除以它。

### 第 1 步：静态核对（拆掉 C1 和夹爪）

默认采样的关节：J1，加上转动后会改变重力力矩的关节（J2–J5）。link6 没有质量，J6 不受重力，本体的静态位姿里 J6 固定不动。位姿表里每个关节可以改成“采样”或“固定”（命令行 `--sample 1,2,3`）。C1 只有 20 g，不拆对 J4/J5 的影响约 0.01–0.03 N·m，可以忽略，但它会算进本体而不是负载。

1. 在“静态训练”行点**生成**（默认 50 个位姿），在“静态验证”行再点**生成**（另一个种子，20 个）。位姿已排除碰撞、排好顺序。
2. 每组先点**预检（不运动）**，确认无误后点**执行**。每个位姿从两侧接近、各测一次，取平均以近似抵消静摩擦。
3. 结果存成 `static/train_….csv`、`static/val_….csv`。第一次做完点**采用测得的力矩标定**。

日志里的表格：斜率 k ≈ 1 说明方向和比例与 URDF 一致，k ≈ −1 说明方向反了；偏置 b 大说明零位或力矩零点有偏差。

### 第 2 步：辨识质量、质心

1. 勾选数据：“静态训练”用于辨识，“静态验证”只用于验收（名称含 `val` 的默认勾在验证，其余默认勾在训练）。
2. **漂移补点权重**（默认 ×5）：文件名以 `drift_` 开头的数据（第 3 步漂移分析生成的、机械臂常用姿态的补点）再乘这个倍数。补点和随机位姿可以同时勾选。
3. 点**开始辨识**。

日志里的**验收表**每个关节三个数：RMS（模型重力与实测之差的均方根）、偏差（平均误差，> 0 表示模型推得太多，机械臂会往 +q 方向漂）、会漂（误差超过静摩擦的位姿数）。表中同时列出原始 URDF 和新版本在“静态验证”上的结果。

### 第 3 步：重力补偿

1. 选模型（刚辨识的版本），模式 `free`：只加重力前馈，手可拖动，松手应停住。`lock`：推动时跟随，不推时保持。
2. **阻尼 kd** 运行中拖动滑条即时生效：kd 越大，阻尼力矩越大。
   **前馈倍率**（每个关节一个，默认 1）：重力前馈乘以它，运行中修改也即时生效。某个姿态区域往下掉（如伸直时 J2 负载大）可把该关节调到 1.02–1.06，往上漂就调小。只用来检验，不改模型；命令行 `--ff-scale 1 1.04 1 1 1 1`。
3. 在各种姿态松手，等几秒再推下一个地方。

结束后自动做**漂移分析**：找出松手后关节自己转动的地方（按速度和方向筛掉疑似手推和联动的片段），生成补数据的静态位姿。点**载入建议位姿 → 第 1 步静态核对**执行，再在第 2 步和原来的数据一起勾选，辨识下一个版本。free 模式运行超过 2 秒才保存记录、做分析。

### 第 4 步：负载辨识（夹爪等）

本体辨识好后，装上 C1 和夹爪（闭合、不动）：

1. 选**本体**模型（如 v3），填负载**名称**（如 `gripper`）。
2. 在“负载训练”（40 个）和“负载验证”（20 个）里分别**生成**，先预检再执行。碰撞检查包含夹爪外形（3D 中橙色），J1–J6 全部参与采样。
3. 数据存在负载自己的数据包 `<结果目录>/gripper/` 里，与本体数据分开。点**辨识负载**。

采负载数据时的重力前馈：所选本体版本已有辨识出的负载模型时用它，否则用本体 + URDF 中的 CAD 负载（命令行 `--payload-cad`）。前馈和实物的差由**负载平衡**补上（工作台采负载数据时开启；命令行加 `--payload-cad`、`--with-tool` 时默认开启，`--no-balance-load` 关闭）：位置误差的受限积分在每个位姿建立平衡力矩，到位静止后冻结，再对电机反馈力矩取平均，所以换夹爪、CAD 不准也不需要先有准确的负载模型。积分最多补力矩上限的一半；kd 还能同比加大的关节同时提高 kp，让这么大的误差在积分跟上之前也不超过跟踪误差上限（阻尼比不变，kd 已到量程的关节不改）。跟踪误差、力矩、温度保护不变。

**保持力矩余量**：模型保持力矩（含上下接近位姿）超过力矩上限 60%（`--max-hold`）的位姿在规划时跳过，留下到安全停止（85%）之间的余量给模型之外的力矩：线缆拉扯、非刚性负载、电机在高负载区的力矩反馈偏差。采集中每测完一个位姿，就用本次已测的位姿拟合末端负载的修正量（质量、一阶矩），重新预测后面的位姿，超出余量的在运行中跳过（日志 `跳过 [i/n] …`），跳过后下一段路径重新做碰撞检查。所以即使前馈完全没有负载，测过几个位姿后也能挡住被负载压到力矩上限附近的位姿。这个修正只用于预测，不改前馈，也不写进数据；每个位姿上模型之外的部分仍由积分补上。**异常位姿**：同一个在线预测带有不确定度。实测力矩偏离预测超过 5 个标准差（`--max-surprise`）的位姿，说明有模型之外的外力，比如夹爪碰到桌面、线缆拉扯或卡滞，这样的位姿不计入数据，另存到数据目录下的 `diagnostics/<名称>_rejected.csv`。负载平衡 6 s 内没能静止的位姿会跳过（判据是 0.4 s 内移动不超过 0.15°、偏离目标不超过 2°，用移动量而不是速度判断，以免 DM 反馈 0.022° 的量化被误判成运动），日志会写出是哪个关节、移动或偏离了多少。这两种情况都会把最近 8 s 的反馈和指令存成 `diagnostics/<名称>_p<序号>.csv`，然后继续采下一个位姿；连续 3 个位姿异常才停止采集。`diagnostics/` 不在 `static/*.csv` 里，不会被当成数据。

安全停止时日志会打出停在哪个位姿的哪个阶段，以及当时的实测/指令角度、实测力矩、模型重力、前馈和积分补偿，停止前 8 s 的反馈存为 `diagnostics/<名称>_stop.csv`。

碰撞检查包含每个位姿两侧 ±`--delta`（默认 5°）的接近动作；桌面高度（第 0 步，`--table-z`）要按实际填，安全距离 `--clearance` 默认 10 mm。
4. 结果在 `gripper/models/<本体版本>/`，第 3 步的模型列表里显示为“辨识 v3+gripper”，可以直接做重力补偿检验。用它做补偿时，漂移分析的补点会存进 `gripper/`，按钮变成“载入建议位姿 → 第 4 步负载静态（gripper）”。

负载关于质心的惯量取运行时组合 URDF 中 `link6` 以下部件（C1、第三方夹爪本体、手指在 0 位）的合成惯量，按辨识质量缩放（静态数据看不到惯量，重力补偿也不需要）；自定义完整 URDF 没有这些部件时按 `--radius` 的实心球估计。初始夹爪资产中这些部件合计约 1.169 kg，质心在 link6 系 [−24.1, 0.3, 104.1] mm；更新第三方模型后以实际加载的 URDF 为准。负载模型旁的 `arm_model/`、`cad_model/` 保存辨识时的本体和 CAD 资源快照，`payload.json` 以相对路径引用它们，并记录 CAD 快照的 SHA-256。导出该结果时使用记录的 CAD 快照，因此后来更新夹爪仓库也不会改变历史结果对应的几何。

⚠ 任一电机驱动（MOS）温度超过 100 °C（`--max-temp`）时自动安全停止。

### 第 5 步：导出完整 URDF

选一个辨识出的模型点**导出**，得到 `export/<包名>/`：

```text
export/rebot_b601_rs_taccap_v3_gripper/
  urdf/rebot_b601_rs_taccap_v3_gripper.urdf          完整 URDF
  urdf/rebot_b601_rs_taccap_v3_gripper_offset.yaml   关节力矩偏置（URDF 写不了）
  meshes/<子目录>/*.STL                              URDF 用到的全部网格（rebot_b601_rs/、c1_adapter/、taccap_description/）
  LICENSES/<资产包>/                                 机械臂、C1 和第三方夹爪的许可与来源声明
```

- 本体模型 → 不带夹爪（去掉 `link6` 以下的 C1、夹爪的连杆、关节和网格）；带负载的模型 → 带夹爪（保留原始 URDF 里 C1、夹爪的连杆、手指关节和网格，质量、质心改成辨识值）。两种包里 `link6` 都没有质量。
- 连杆、关节、外观、碰撞、限位都和原始 URDF 一样，只换惯性参数；网格路径是 `../meshes/…`，整个文件夹可以拷到任何地方。
- 带夹爪的包里两个手指是转动关节（左指 mimic 右指）：用 pinocchio 等加载时模型是 8 个关节，手指给 0 即可（导出时按手指在 0 位分配质量）。
- 导出的包也会出现在第 3 步的模型列表里（“导出 …”），可以直接做重力补偿检验。

## 4. 结果文件

默认在 `results/rebot_b601_rs[_<id>]/`：以源码目录安装（`pip install -e`）时在仓库根目录下，否则在当前工作目录下；可用 `--robot.results_dir=<目录>` 指定。

```text
static/                 第 1 步：静态核对（每个位姿一行：保持力矩、模型重力、静摩擦）
models/v1/              第 2 步：每个版本一个目录
  rebot_b601_rs_identified_v1.urdf         新 URDF（质量、质心为辨识值，关于质心的惯量为原 URDF 的值）
  rebot_b601_rs_identified_v1_offset.yaml  关节力矩偏置（重力补偿和验收自动加上）
  result.json                          质量、质心、偏置、残差
  inputs.json                          用了哪些数据和设置
  meshes/                              辨识模型引用的网格（相对路径）
  LICENSES/                            资源许可与来源声明
gravity/                第 3 步：free 模式重力补偿记录（漂移分析用）
gripper/                第 4 步：负载数据包（static/ gravity/ models/<本体版本>/）
export/                 第 5 步：完整 URDF 包
```

重力日志的 `.meta.json` 使用相对模型路径；使用临时组合模型时，其资源快照保存在 `gravity/models/` 下。移动整个结果目录后，负载导出和漂移分析仍能找到相应模型。

力矩标定存放在 `~/.cache/rebot_taccap_identification/calibration/robots/rebot_b601_rs/<id 或 default>.json`（可用 `--robot.calibration_dir` 指定目录）。

## 5. 命令行

工作台运行真机、辨识和导出时都调用下面的命令，不开界面也能完成全部流程。`static`、`gravity` 默认只预检，加 `--execute` 才运动；`ping` 只读。任意配置字段可用 `--robot.<字段>=<值>` 覆盖（如 `--robot.channel=can1`、`--robot.id=arm2`）。下表路径以结果目录 `results/rebot_b601_rs/` 为当前目录书写。

| 步骤 | 命令 |
|---|---|
| 0 连接 | `rebot_taccap_identification ping` |
| 1 静态核对 | `rebot_taccap_identification static --random 50 [--seed 0] --out static/train_rand50_s0.csv [--execute] [--save-calibration]`（不给 `--out` 时写 `static/check.csv`，会覆盖） |
| 2 辨识 | `rebot_taccap_identification identify --static static/train_….csv static/drift_….csv@5 --val static/val_….csv` |
| 2 验收 | `rebot_taccap_identification evaluate --models v1=… v2=… --static static/val_….csv` |
| 3 重力补偿 | `rebot_taccap_identification gravity --urdf models/v1/….urdf [--mode lock] [--comp-kd 0.2] [--execute]` |
| 3 漂移分析 | `rebot_taccap_identification drift --log gravity/<记录>.csv`（free 模式结束时自动进行；桌面高度、是否完整 URDF 取记录旁的 `.meta.json`） |
| 4 负载 | `rebot_taccap_identification payload --urdf models/v1/….urdf --static gripper/static/train_….csv --val gripper/static/val_….csv --out-urdf gripper/models/v1/rebot_b601_rs_gripper_identified_v1.urdf` |
| 5 导出 | `rebot_taccap_identification package --model gripper/models/v1/….urdf` |
| 检验导出的包 | `rebot_taccap_identification gravity --urdf export/<包>/urdf/<包>.urdf --full-urdf --with-tool --execute` |

装着夹爪运动时加 `--with-tool`：碰撞检查包含机器人 URDF 中 `link6` 以下的夹爪。`--full-urdf` 表示 URDF 是第 5 步导出的完整包，`link6` 以下的夹爪（手指在 0 位）要算进重力（不加会被截掉）。模型 URDF 旁边的 `<模型>_offset.yaml` 会被自动加上（`--no-offset` 不加）。

## 6. 方法

静止时，关节 j 的保持力矩是各连杆重力的力矩加一个常值偏置：

$$
\tau_j = \sum_{k \ge j} \Big( -a_j \cdot \big((o_k - o_j) \times g\big)\, m_k \;-\; \big(R_k^\top (g \times a_j)\big) \cdot h_k \Big) + b_j
$$

- $m_k$：连杆 k 的质量；$h_k = m_k c_k$：一阶质量矩，$c_k$ 是质心在连杆坐标系中的位置；
- $a_j$、$o_j$：关节 j 的轴和原点，$R_k$：连杆 k 的姿态（都在基座坐标系中，由 URDF 运动学算出）；$g$：重力向量；
- $b_j$：关节力矩偏置（电机力矩零点误差等）。

它对 $(m_k, h_k, b_j)$ 是线性的：$\tau = Y(q)\,\theta + b$。所有静态位姿的测量力矩除以力矩标定后堆叠起来，用加权最小二乘求解：

$$
\min_{\theta, b}\ \sum_i w_i \big\|S^{-1}(\tau_i - Y(q_i)\theta - b)\big\|^2 + \lambda \big\|D^{-1}(\theta - \theta_0)\big\|^2
$$

- $w_i$：位姿 i 的权重（`文件@K` 的 K）；$S$：每个关节的方程除以该关节实测力矩的变化范围（不小于 0.2 N·m）；
- 第二项把参数拉向原 URDF 的值 $\theta_0$，使数据看不到的方向（例如 J1 绕竖直轴，重力力矩恒为 0）有确定的解；它作用于所有方向，所以数据看得到的方向也会有少量偏向原值。$D$ 是各参数的典型大小（质量 ~ m，一阶矩 ~ m·L），$\lambda$ = 0.1 × 归一化回归列的平均能量（命令行 `--prior-weight` 改 0.1）；
- URDF 中没有质量的连杆（reBot 的 link6）固定为零质量，不参与辨识。reBot link1–link5 共 20 个质量/质心参数，位姿足够丰富时静态数据最多能确定其中 8 个组合（辨识时打印实际确定的个数），其余由先验决定。单个连杆的值受先验影响，数据确定的是它们的组合（即重力预测）。

关于质心的转动惯量保持原 URDF 的值（静态数据看不到惯量），所以导出的每个连杆质量为正、关于质心的惯量正定。负载辨识固定本体参数和偏置，用普通最小二乘（不加先验、不做关节归一化）只求末端连杆上附加的 $(m_p, h_p)$；数据确定不了的方向会在结果里列出。

## 7. 安全

- 真机命令默认只做预检（碰撞检查、路线、模型力矩），加 `--execute` 才使能。
- 运行中持续监控：实测力矩（电机 MIT 量程峰值的 85%，rs-06 36 N·m、rs-00 14 N·m）、驱动温度（100 °C）、反馈丢失；静态核对还监控跟踪误差（8°），并在规划和采集中跳过模型保持力矩超过上限 60% 的位姿（`--max-hold`）。任何一项超限都安全停止。重力补偿不检查跟踪误差，也不检查安全窗口（窗口外由虚拟墙推回）。
- 安全停止：顶部红色按钮、Esc、空格或 Ctrl+C。第一次以慢速回到休息位再失能（重力补偿中回休息位的路径有碰撞时会先保持、等按回车）；第二次立即失能（机械臂会落下，手要扶住）。
- 重力补偿 free 模式下机械臂完全靠模型托住，模型不准时会缓慢下坠或上漂，手要随时能扶住。安全窗口外有虚拟墙把机械臂推回。
- 不要在装好的机械臂上运行会把当前位置写入电机 flash 作为零点的工具。

## 许可证

[MIT](LICENSE) © 2026 XenseRobotics-AI。随附的机械臂和 C1 资产许可见 [LICENSES/](src/rebot_taccap_identification/robots/rebot_b601_rs/urdf/rebot_b601_rs/LICENSES/)（机械臂 CERN-OHL-W-2.0，C1 转接件许可未确认）。第三方夹爪的 MIT 许可和来源声明由 [XTac-UMI-Description](third_party/XTac-UMI-Description) 仓库提供。
