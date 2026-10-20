# Demo9 EGO / POINT 航向与位置交接修复候选 V1

**状态：离线验证通过；尚未部署到飞机，尚未进行 ROS 真机无桨集成测试，严禁据此直接放飞。**

## 事故依据

- 对应记录：`demo9_20261019_210756_recovered.bag` + `log_85_2026-10-6-12-49-50.ulg`。
- P2 末期 EGO ROS ENU 指令：yaw +93.56°，位置 (-4.776, 4.472, 0.719)m。
- 未启用的旧 POINT HOLD：yaw -175.01°，位置 (-4.720, 3.518, 0.701)m。
- 两者角差按最短角度约 91°、位置差约 0.96m。ULog 异常短时 yaw 命令恰好对应旧 POINT HOLD 值。
- 正常历史 EGO->POINT 切换曾有约 0.18m 的源间目标偏差，但实际 POSITION HOLD 与 PX4 odom 只差约 0.002m。因此不能无差别拒绝所有 0.12m 指令变化。

## 修复范围（仅两种方法）

1. `trajectory_mux.py`：只在控制源**切换时**进行 ROS ENU 位置/偏航连续性检查，并在切换后 0.25s 防止旧控制源目标的异步突跳。源间允许位置差 ≤0.25m、偏航差 ≤12°。不对正常源内部的起飞轨迹做限幅。检查失败时拒绝选源或停止发布该帧，保持 MUX 既有来源，不伪造 setpoint；下游原有 HOLD / 失效逻辑不变。
2. `uav2_demo5_waypoints.py`：`restore_point_hold()` 先调用原有 `disable_trajectory` 和 `abort_trajectory`，等候至少 3 次**新** POINT setpoint；其位置距 PX4 odom ≤0.12m、偏航距当前航向 ≤12° 才允许 `use_point`；确认 `POINT_ACTIVE` 后才能 `enable_trajectory`。任何失败都抛出异常，**不再吞掉错误并继续启用旧轨迹**。

**没有修改** PX4 参数、RC 门控、T265、PID、失效保护、实时 Demo9、冻结 Demo8 或 runtime。

## 在机载电脑上准备（必须 DISARM、停止 Demo9 进程）

### 1. 解压和离线测试

把本 zip 放入 `~/Downloads/`，执行：

```bash
mkdir -p "$HOME/contest1_uav/dev/projects"
unzip -q "$HOME/Downloads/demo9_yaw_handoff_repair_v1.zip" -d "$HOME/contest1_uav/dev/projects"
cd "$HOME/contest1_uav/dev/projects/demo9_yaw_handoff_repair_v1"
python3 -B -m unittest -v test_guard test_mux_mock test_restore_mock
```

### 2. 找到实际正在使用的当前文件，**只读定位**

```bash
BASE="$HOME/contest1_uav/demo/09_demo8_posctl_integrated_v1"
grep -nE 'trajectory_mux|uav2_demo5_waypoints' "$BASE/start_backend.sh" "$BASE/run_demo9.sh" 2>/dev/null
find "$BASE" "$HOME/contest1_uav/runtime" -type f \
  \( -name trajectory_mux.py -o -name uav2_demo5_waypoints.py \) -print
```

特别注意：现场 MUX 可能放在 `demo8_interface_overlay/`，不能随意选另一个历史版本。下方 `MUX=` 必须换成 **实际的 MUX 脚本路径**。

### 3. 自动生成 DEV 修复候选和 diff（只写 `dev/projects/`）

```bash
cd "$HOME/contest1_uav/dev/projects/demo9_yaw_handoff_repair_v1"
MUX="/请替换为实际生效的/trajectory_mux.py"
MISSION="$HOME/contest1_uav/demo/09_demo8_posctl_integrated_v1/uav2_demo5_waypoints.py"
OUT="$HOME/contest1_uav/dev/projects/demo9_handoff_candidate_$(date +%Y%m%d_%H%M%S)"
python3 -B prepare_dev.py --mux "$MUX" --mission "$MISSION" --out "$OUT"
```

只在历史关键函数的 AST 语义结构与已见版本一致时才会自动修复；**如果现场函数已修改，会 STOP 而不是覆盖未知的新安全逻辑**。停在 STOP 时把两份当前脚本上传给助手做精确合并即可。

生成结果：

- `$OUT/originals/` — 两份原始代码**副本**，用作回退参考；
- `$OUT/candidates/` — 两份 DEV 修复代码和依赖 `yaw_guard_core.py`；
- `$OUT/patch_review.diff` — 逐行改动审核；
- `$OUT/sha256_original.txt` — 输入文件校验值。

**此命令不会将 DEV 候选安装到 Demo9 或 runtime；不会启动 ROS、ARM、切模式或转电机。**

## 部署前必须逐项验证

1. Diff 审阅及候选 Python 编译通过；所有离线测试通过。
2. 现场核实 `/contest1/mavros/setpoint_raw/local` 只有预期的轨迹接口发布者，绝无第二个并行控制器。
3. 在 ROS 地面/无桨环境分别注入“正常交接”“旧 POINT 约 91°/0.96m”“POINT 超时”“模式退出 OFFBOARD”的模拟输入；核实失败时 `enable_trajectory` 未被调用，原有 HOLD 和 RC 接管有效。
4. 机载电脑的进程/供电异常**尚未修复**。必须独立排除导致 OFFBOARD/视觉链路消失的问题后，才可由飞手按照原预飞流程考虑实飞。不要仅凭这些离线单元测试直接飞行。

## 关于异常降落

该补丁处理的是 P2 交接时的目标不连续；ULog 中 +67.89s 的 OFFBOARD 丢失以及之后的定位无效/`blind land` 是**尚未修好的另一条故障链**。日志里 21:08:35 的 `whoopsie Parsing /var/crash/*nodelet*` 只能证明系统处理过一个 crash 文件，不能证明该时刻 nodelet 新崩溃。可执行包内 `inspect_pc_crash.sh` 只读采集该启动的日志及 crash 文件时间戳。
