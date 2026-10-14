# reBot B601 灵足版（`rebot_b601_rs`）

6 个 RobStride 电机（J1–J3 rs-06，J4–J6 rs-00），一条 CAN 总线，MIT 模式，通过 motorbridge SDK 控制（host id 0xFD，电机原始弧度和 N·m）。末端是 C1 转接件 + TacCap G1 从夹爪（XTac-UMI）。

`link6` 是第六轴电机输出面的虚拟坐标系，没有质量（原厂 link6 安装盘已去掉，C1 直接装在电机输出面上；电机本体算在 link5 里）。`tip_link = "link6"`：本体辨识时忽略下面的 C1、夹爪和两个手指，link6 保持零质量不参与辨识；第 4 步把 C1 + 夹爪整体作为负载辨识。

| URDF 中 link6 以下（负载） | 质量 |
|---|---|
| C1 转接件 `adapter_link` | 0.020 kg |
| 夹爪本体 `taccap_base_link` | 0.415 kg |
| 左右手指（转动关节，左指 mimic 右指） | 2 × 0.367 kg |
| 合计（手指在 0 位，质心在 link6 系 [−24.1, 0.3, 104.1] mm） | 1.169 kg |

```bash
rebot_taccap_identification ping                          # 默认 can0
rebot_taccap_identification static --robot.channel=can1 --random 50
rebot_taccap_identification studio
```

## 前提

- 装好本仓库（`pip install -e .`，已包含 motorbridge）；CAN 口已起来（套件里是 PCAN-USB，通常直接出现 can0 或 can1）：
  ```bash
  sudo modprobe peak_usb
  ip -br link                          # 确认出现 can0
  sudo ip link set can0 down 2>/dev/null
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  ```
- 末端空载，`q = 0` 对应 URDF 零位。电机原始坐标与 URDF 一致（方向 1、偏置 0），静态核对会再验证一遍。
- 本项目只保存机械臂与 C1 的 URDF 和网格，位于 `urdf/rebot_b601_rs/`（来源和网格哈希见 `sources.json`，许可见 `LICENSES/`）。夹爪来自 `third_party/XTac-UMI-Description` 子模块的资产清单，运行时装配，不复制进本包；在项目根目录用 `git submodule update --init --remote -- third_party/XTac-UMI-Description` 跟进最新 `main`。
- `--robot.gripper_repo=<目录>` 可改用独立夹爪仓库，`--robot.gripper_urdf=<文件>` 可直接选择外部夹爪 URDF。安装接口由 `gripper_parent_link`、`gripper_xyz` 和 `gripper_rpy` 配置；默认连接 C1 的 `adapter_link`，沿其 y 轴偏移 14 mm，绕 y、z 各转 90°。使用的是裸从夹爪，不再重复加入 Asm_link 转接件。
- `--robot.urdf=<路径>` 指定的是完整自定义机器人模型，按该文件加载，不额外挂载默认夹爪。上面的 CAD 质量/质心表为初始资产的参考值，更新第三方模型后以新模型为准。
- 默认配置使用包资源 URI，外部仓库示例使用 `--robot.gripper_repo=../XTac-UMI-Description`。组合模型与辨识模型都保存相对网格路径及随附资源，结果 JSON 的模型引用相对于其所在目录解析。
- C1 的安装分度为电机侧 +90°、夹爪侧 −90°，`joint6` 是原厂零位、限位 ±3.14，与 xense-description 的 `seeed_rebotb601rs_single` 装配一致。
- ⚠️ 不要在装好的机械臂上运行会把当前位置写入 flash 作为零点的工具（例如 motorbridge 的 `robstride_all_interfaces_demo.py`）。

## 默认值

| 项 | 值 |
|---|---|
| 指令频率 | 100 Hz（RobStride 每收到一帧 MIT 指令回一帧状态） |
| kp / kd | 50 50 50 30 50 50 / 3 5 5 3 4 4（带重力前馈时的保持增益；按电机量程截断，rs-00 的 kd ≤ 5） |
| 软限位 [°] | J1 ±145，J2 0–170，J3 0–200，J4 −80–90，J5 ±90，J6 ±130（与 URDF 限位取交集，实际 J3 0–180、J4 −80–90） |
| 力矩上限 | rs-06 36 N·m，rs-00 14 N·m（MIT 量程峰值） |

## 经验

- **力矩标定**：反馈力矩比实际输出大，静态核对测得 J2 ≈ 1.055、J3 ≈ 1.072，其余取 1。先用 `rebot_taccap_identification static ... --save-calibration`（或工作台“采用测得的力矩标定”）存入标定，再辨识。
- **常用区域的静态数据**：J2 在 0–30° 附近（机械臂收拢时）重力补偿最敏感。只用随机位姿时这一区域数据不足，补偿会向前倾；在重力补偿里把机械臂推到常用姿态松手，用漂移分析生成的补点（`drift_…`）一起辨识后明显改善。
- **腕部**：本体辨识时 J5 的重力力矩很小（约 0.1 N·m），和静摩擦同一量级，link5 的质量、质心主要由 CAD 先验决定；link6 没有质量，J6 不受重力，本体静态位姿里 J6 固定不动。装上夹爪后负载采样 J1–J6 全部参与。
