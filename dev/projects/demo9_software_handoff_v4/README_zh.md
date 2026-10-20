# Demo9 软件交接竞态修复 DEV V4

**状态：完成源码修改与 65 项无硬件离线测试；未部署、未进行 ROS 主节点/PIXHAWK 在环/SITL 全任务回归，不可视为实飞验证通过。**

本包以你上传的 `demo9_handoff_contract_v3.zip`、`demo9_runtime_contract.tar.gz`、`demo9_handoff_review.tar.gz` 和事故 ROS/ULog 的时间线为依据，专注软件。**不修改硬件、PX4 参数、安全门限、冻结 Demo8、当前正式 Demo9 或当前 runtime。**

## 要修复的真实问题

事故的 P2 交接（北京时间 2026-10-19 21:09:17）：

- 21:09:17.746，`disable_trajectory` 成功，下游开始 HOLD。
- 21:09:17.749，`abort_trajectory` 成功，POINT 内部更新当前 HOLD。
- 21:09:17.753，`use_point` 被成功调用；21:09:17.756 即 `enable_trajectory`。
- POINT 定时发布频率为 50Hz（每 20ms），因此上面约 10ms 的交接间隔不能保证 MUX 已收到新的 POINT 指令。旧 POINT 指令约差 0.96m/91°，与 ULog 中瞬时电机差速高度对应。

## V4 实际修改

1. **任务代码 `uav2_demo5_waypoints.py`**：`abort_trajectory` 返回成功后才开始计时；必须收到 POINT *header.stamp* 在本次 abort ACK 之后生成的新样本，不能只看订阅回调到达时间。仍要求 AIRBORNE_HOLD、新鲜里程计、飞行健康、CH7 许可、OFFBOARD、Demo 帧安全、实际 MAVROS HOLD 位置/yaw 匹配、连续 3 条**不同生成时间戳**的新 POINT 指令。支持最多一次额外捕获，不自动跳航点。
2. **MUX `trajectory_mux.py`**：拒绝倒序 POINT 包；切回 POINT 前要求新鲜 AIRBORNE_HOLD、POINT 原始生成时间、位置/yaw 连续、HOLD 无残余速度/加速度/yaw_rate。切换后继续拒绝异常 POINT，不会仅因为 0.25 秒保护窗口结束而放行持续错误目标；原本的 POINT 起飞/降落状态不被无限 HOLD 限制。
3. **恢复使能后验证**：不是收到 `enable_trajectory` ACK 就推进比赛；必须看到使能以后新生成的 POINT 指令与下游 MAVROS 输出连续匹配，才允许进入下一航点。失败会再次尝试 `disable_trajectory`，以保留下游已存在的独立 HOLD（前提是电脑、定位和健康状态仍正常）。
4. **软件链路诊断**（`tools/software_link_witness.py`）：一个纯观察节点，只订阅状态、控制源、轨迹接口、里程计、健康、RC 和下游设定值；每秒将各话题的接收年龄写入 Desktop `.jsonl`，在严重 yaw 跳变时记录事件。完全不发布控制指令，也不请求 OFFBOARD/ARM。若整台机载电脑死机或断电，它也会一起停，**不具备防止掉线或保证悬停的能力**。

**重要边界：** 这里不是“修改 OFFBOARD 失联后自动重新进入模式”。机载 ROS/T265 链路整体停止的问题尚未定位出可以验证的软件修复；V4 不会强行恢复 OFFBOARD，也不会屏蔽 PX4 原有失效保护。这次主要修复异响对应的交接竞态，并补充软件掉线诊断能力。

## 直接在 Ubuntu Desktop 上使用

将 `demo9_software_handoff_v4.zip` 放到 `~/Desktop/`，确认飞机 DISARM、Demo9 没有运行。执行：

```bash
mkdir -p "$HOME/contest1_uav/dev/projects"
unzip -q "$HOME/Desktop/demo9_software_handoff_v4.zip" \
  -d "$HOME/contest1_uav/dev/projects"

cd "$HOME/contest1_uav/dev/projects/demo9_software_handoff_v4"

# 65 项独立离线测试；不连接 ROS 或 PX4。
python3 -B -m unittest discover -s tests -v

# 对照现场源文件 SHA-256，并仅生成一个隔离的 DEV 副本。
python3 -B tools/stage_dev_only.py
```

期望 `Ran 65 tests`、`OK`，再看到 `[OK] Separate DEV source stage:`。它会生成：

```
~/contest1_uav/dev/projects/demo9_v4_stage_<timestamp>/
  ros_src/ego_uav_interface/scripts/trajectory_mux.py
  ros_src/ego_uav_interface/scripts/yaw_guard_core.py
  demo9_app/uav2_demo5_waypoints.py
  dev_manifest.json
```

`stage_dev_only.py` 会对照 5 份已经审阅过的现场源文件哈希（MUX、POINT、Trajectory Interface、任务脚本、launch）。任何不匹配都会 `[STOP]`，**不要强制覆盖**。它不运行新节点，不修改 `ROS_PACKAGE_PATH`，不写 `/runtime/` 和正式 Demo9。

需要查看修改时，可以查看 `diffs/` 内的 V3→V4 差异。原始版本位于 `reference/reviewed_originals/`，用于核验。

### 可选：无桨地面软件链路记录

在将来进行**无桨**地面 ROS 运行稳定性检查时，另开终端：

```bash
source "$HOME/contest1_uav/runtime/env.sh"
python3 -B \
 "$HOME/contest1_uav/dev/projects/demo9_software_handoff_v4/tools/software_link_witness.py"
```

默认在 `~/Desktop/` 生成 `demo9_link_witness_<timestamp>.jsonl`。按 Ctrl+C 结束；该节点本身不执行任务、不控制电机、不改变飞行模式。注意它使用当前 ROS 网络，**不会自动接入 V4 候选 MUX**；在正式接入前只能监视现有系统。

## 验证范围与尚未完成的工作

- 65 个**独立离线单元/局部集成测试**通过，其中包含从你上传的真实 `point_traj_node.py` 抽取 `abort_trajectory` 与 50Hz timer 逻辑，和真实轨迹接口在同一个模拟时钟下运行；事故中的 10ms 旧 POINT 会被拒绝，后续新 HOLD 才被允许。
- 测试覆盖正常 POINT 航点降落指令、持续旧 yaw、位置跳变、POINT stale、乱序消息、旧 header、非零 HOLD 前馈、健康/RC/模式失效，以及 MAVROS 回显与 POINT 更新不同步。
- 未运行真实 ROS master、真实 PX4、SITL/EGO 规划器或整场 P1→P4。不能承诺单次比赛一遍过或电脑停止响应后仍能 HOLD。
- 下一阶段是**隔离 ROS 无桨集成验证与 SITL 全任务回归**，确认新的 DEV ROS 包实际启动、最终 MAVROS 连续发布、拒绝危险交接后可靠保持下游 HOLD，以及 RC/PX4 失效保护没有被破坏。**在完成之前不要将 V4 复制进正式 runtime 或带桨试飞。**

## 再下一步：独立 ROS Master 的无硬件集成冒烟测试

V4 另外附带 `tools/isolated_ros_smoke.sh`，用于在 `127.0.0.1:11457` **另起独立 ROS Master**，仅运行 DEV 候选 MUX、复制出来的原始 `trajectory_interface.py` 和合成传感器消息。它不启动 MAVROS、PX4、摄像头、飞行任务或电机，也不改变正在运行的 ROS Master。可以在保留官方 Demo9 不动的前提下，测试真正的 ROS 订阅/服务/消息转发（区别于 Python 假总线离线测试）。

**本隔离冒烟脚本已做语法和静态审查，但因为这里没有 ROS Noetic 环境，尚未在真实 ROS Master 上执行；必须由你在 Ubuntu 完成后才知道是否通过。**

先执行上面的离线测试和 DEV staging，记下 `[OK] Separate DEV source stage` 后的路径，运行：

```bash
# 仅使用在 Ubuntu 生成的 DEV stage 路径，不是正式 runtime 路径。
STAGE="$HOME/contest1_uav/dev/projects/demo9_v4_stage_这里填实际时间戳"

source /opt/ros/noetic/setup.bash
bash "$HOME/contest1_uav/dev/projects/demo9_software_handoff_v4/tools/isolated_ros_smoke.sh" "$STAGE"
```

预期出现 `PASS: isolated ROS synthetic handoff and downstream HOLD`；日志保存在 `$STAGE/isolated_smoke_logs/`。这个合成测试会尝试旧 POINT 错误 yaw 拒绝、切换到新 POINT、异常再出现时降级 HOLD、目标恢复后继续输出。**它不是完整 SITL P1→P4，也不是实际控制器带电测试，不意味着可以实飞。**
