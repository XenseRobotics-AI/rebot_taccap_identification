# reBot B601 达妙版（`rebot_b601_dm`）

用于 B601 DM 本体与 C1 + TacCap 负载的**静态重力参数辨识**，接入同一套工作台、采集、验证、重力补偿和 URDF 导出流程。

识别的是重力可观测的质量/一阶质量矩组合、等效质心和关节力矩偏置。静态数据不能唯一确定每个连杆的全部参数，也不能辨识转动惯量、速度摩擦和惯性项；本体的质心惯量保留 CAD 值，负载的质心惯量取 CAD 缩放值。结果 JSON 记录可观测秩/惯量来源，不应把导出的惯量当作动态激励辨识结果。

## 启动与连接

在仓库根目录安装/更新环境，初始化夹爪模型：

```bash
git submodule update --init --recursive -- third_party/XTac-UMI-Description
pip install -e .

# 指定当前要辨识的机械臂串口
rebot_taccap_identification studio \
  --robot.type=rebot_b601_dm --robot.channel=/dev/ttyACM0
```

本仓库按单臂独立使用，直接通过 `channel` 选择当前设备。默认 `/dev/ttyACM0`；实际设备编号变化时修改这个参数。也可以使用 `/dev/serial/by-path/<实际路径>`。

默认 `transport=serial`、`dm_serial_baud=921600`，不需要设置 can0。只有实际采用 SocketCAN 适配器时才使用：

```bash
rebot_taccap_identification ping \
  --robot.type=rebot_b601_dm --robot.transport=socketcan --robot.channel=can0
```

`ping` 只读寄存器并请求反馈，不使能、不失能、不改零位、不存储电机参数。其它程序（包括 LeRobot）应先释放同一通信口。

## 电机与模型

- J1–J3：DM `4340P`；J4–J6：DM `4310`。电机 ID 1–6，反馈 ID `0x11`–`0x16`。可以用 `--robot.motors='[...]'` 按 URDF 顺序覆盖，不能用 RS 配置替代。
- 使能前核对 `ESC_ID`、`MST_ID` 和 `PMAX/VMAX/TMAX`。默认量程分别为 `[12.5,10,28]` 和 `[12.5,30,10]`（rad、rad/s、N·m）。不匹配会拒绝使能，不会修改电机量程。
- 默认 `kp=[45,45,45,8,9,8]`、`kd=[5,5,5,1,1,1]`，来自 libpyrebot DM 默认配置。没有放宽到位阈值来掩盖下垂。只有负载静态采集的负载平衡会提高 J4–J6 的 kp（约 25，kd 按 √kp 同比加大，阻尼比不变）；J1–J3 的 kd 已到量程 5，增益不变。
- 默认力矩上限读取 URDF effort：`[27,27,27,7,7,7] N·m`；协议峰值和 URDF 上限都不是连续保持力矩额定值。可按实机条件用 `--robot.torque_limits='[...]'` 降低采集上限，不能超过协议量程。
- 零位必须已按 B601 DM 机械零位完成标定。URDF 使用电机原始坐标，方向全为 `+1`、偏置全为 `0`；J2/J3 工作角为负，如 `[0,-0.8,-0.8,0.5,0,0] rad`。辨识输出的 `_offset.yaml` 是力矩偏置，不是零位校正。
- 默认基座水平、向上安装，重力 `[0,0,-9.80665] m/s²`。其它安装方向需覆盖 `--robot.gravity='[gx,gy,gz]'`。
- 本体质量、质心和惯量来自 `reBotArm_control_py` 当前 DM 控制模型，与 libpyrebot DM 本体一致；几何来自 `reBot-Isaacsim`。仿真仓库同名 DM 模型的质量明显偏小，不用于本体重力初值。这些仍是名义参数，需要实机辨识。去掉原厂末端安装盘和夹爪，`joint6` 位于电机直接输出面（link5 系 x=0.028008 m），与 libpyrebot `DM_Taccap` 对齐；`link6` 是无质量坐标系。
- C1 使用仓库已有 CAD，TacCap 从外部 `XTac-UMI-Description` 组合；没有导入 RS 的辨识质量或偏置。`--robot.gripper_repo` / `--robot.gripper_urdf` 可替换夹爪模型；`--robot.urdf` 指定完整自定义模型时不再自动拼装。
- 来源、文件哈希和修改说明见 [sources.json](urdf/rebot_b601_dm/sources.json)，许可证/声明随资源和导出包一起保存。可用 `scripts/build_dm_assets.py --dm-repo <reBot-Isaacsim目录> --dm-arm-repo <reBotArm_control_py目录>` 重建本体资源。

## 辨识顺序

工作台的六个步骤与 RS 相同。默认结果目录为 `results/rebot_b601_dm/`，无需设置机器人 ID：

1. 拆下 C1 和 TacCap，完成本体静态核对。先生成少量姿态、预检、确认实机零位与模型一致，再执行。数据用实测角度而非命令角度；双向接近并平均保持力矩，减小静摩擦影响。
2. 本体训练数据拟合质量/质心组合与力矩偏置，另一组随机种子的验证数据只用于验收。没有足够激励的力矩增益不能靠静止零位确定。
3. 使用新本体模型检验重力补偿。不能用增大 `reset_tolerance` 替代模型验证。
4. 装上 C1 + TacCap，手指保持模型的固定零位/闭合状态，保持线缆布置一致。重新采集负载训练和验证数据，**固定已辨识本体，只拟合新增负载**。采集时 `--payload-cad` 提供 CAD 负载前馈，并默认开启负载平衡：CAD 与实物的差由受限积分补上，换夹爪或 CAD 不准时不需要先有准确的负载模型。J2 保持力矩预计超过 16.2 N·m（27 N·m 的 60%，`--max-hold`）的位姿会跳过；采集中会用已测位姿在线修正负载后重新预测。
5. 验证本体 + 负载，再导出完整 URDF 包。

下列命令展示单臂流程。先完成每段预检，再单独执行带 `--execute` 的命令；这些示例不是自动运行脚本。

```bash
robot=(--robot.type=rebot_b601_dm --robot.channel=/dev/ttyACM0)
result=results/rebot_b601_dm
rebot_taccap_identification ping "${robot[@]}"

# 拆掉 C1 + TacCap。本体预检：不连接电机，不运动
rebot_taccap_identification static "${robot[@]}" --random 50 --seed 0

# 本体训练与独立验证，只有 --execute 会使能/运动
rebot_taccap_identification static "${robot[@]}" --random 50 --seed 0 \
  --out "$result/static/train.csv" --execute
rebot_taccap_identification static "${robot[@]}" --random 20 --seed 100 \
  --out "$result/static/val.csv" --execute

# 检查输出的力矩增益，按工作台“采用测得的力矩标定”保存；
# CLI 可用 --robot.torque_gain='[...]' 在后续各命令显式传入同一组已验证值。
rebot_taccap_identification identify "${robot[@]}" \
  --static "$result/static/train.csv" --val "$result/static/val.csv"
arm="$result/models/v1/rebot_b601_dm_identified_v1.urdf"

# 装上 C1 + TacCap。先预检包含工具的姿态/路径
rebot_taccap_identification static "${robot[@]}" --urdf "$arm" --payload-cad --random 40 --seed 1
rebot_taccap_identification static "${robot[@]}" --urdf "$arm" --payload-cad --random 40 --seed 1 \
  --out "$result/taccap/static/train.csv" --execute
rebot_taccap_identification static "${robot[@]}" --urdf "$arm" --payload-cad --random 20 --seed 101 \
  --out "$result/taccap/static/val.csv" --execute

loaded="$result/taccap/models/v1/rebot_b601_dm_taccap_identified_v1.urdf"
rebot_taccap_identification payload "${robot[@]}" --urdf "$arm" --name taccap \
  --static "$result/taccap/static/train.csv" --val "$result/taccap/static/val.csv" --out-urdf "$loaded"
rebot_taccap_identification package "${robot[@]}" --model "$loaded"
```

重复辨识默认递增版本，`arm`/`loaded` 路径需使用当次输出。桌面高度用 `--table-z` 指定；预检不能发现实机环境里未建模的障碍物。

## 输出与使用

本体 `result.json` 含质量、质心、力矩偏置、残差、可观测秩；`payload.json` 含总负载质量、link6 系质心、惯量来源及本体/CAD 资源快照。结果没有自动写进电机，也没有自动修改 LeRobot YAML。

用于 LeRobot 时，选择**导出的完整 DM + TacCap URDF**，显式设置 `urdf_path`，并保持关节零位、重力方向和末端坐标系一致。与 URDF 配套的 `_offset.yaml` 单独保存力矩偏置：本仓库验证/补偿会加载它，外部控制器是否加载必须核对实现，单独换 URDF 不代表偏置也生效。不要把 RS 示例参数、未验证的 CAD 值或本仓库协议上限当成实机辨识结果。

## 通信与停止

每个电机都有独立的寄存器应答看门狗（默认 0.5 s）。运行中单次寄存器读取等待 80 ms，超时会记录警告并在通信期限内重试；失败不刷新最后应答时间，其它关节的回复也不会替这个关节续期。单个电机持续无应答达到 `feedback_timeout_s` 会锁定停止，即使之后收到回复也不会自动恢复。串口收发错误、异常寄存器内容、故障码、意外失能、非有限反馈仍立即停止；不自动清故障或重新使能。部分电机使能失败会尝试失能全部关节。MOS/转子温度取较高值交给现有运行监控。

motorbridge 0.5.6 的 Python 状态接口不提供帧接收时间。这里用必须收到新回复的寄存器读取检测通信存活，状态仍来自 SDK 缓存；不能把它解释成每一帧位置/力矩的精确时间同步保证。首次实机采集应保留独立验证集，检查反馈、残差和实际保持姿态。
