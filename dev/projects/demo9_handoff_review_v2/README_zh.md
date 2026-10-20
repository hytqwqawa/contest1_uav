# Demo9 EGO↔POINT 交接 DEV V2（源码审核后修正）

**状态：仅供开发、离线审查和无桨地面集成；不是可以直接飞比赛的部署包。**

本包以你上传的 `demo9_handoff_review.tar.gz` 中 **真实 DEV 候选** 为输入，额外审核 `start_backend.sh`、`start_core.sh`、`run_demo9.sh`。原来的 Demo9、`runtime/`、冻结版 Demo8 和 PX4 参数均未修改。

## 审核结论

- `start_backend.sh` 调用 `roslaunch "$DEMO/uav2_demo5_multi_waypoint_interface.launch"`，因此生效 MUX 取决于 launch 的 `pkg/type` 和运行时 ROS 包解析结果。上传包**缺少此 launch**，不能声称 DEV 候选已经被当前飞行程序使用。
- V1 的切换校验可以拦截事故中约 0.96m / 91° 的旧 POINT 目标，但在 Guard 拒绝时不发布该帧，不能仅凭 MUX 就承诺飞机能悬停；下游 `trajectory_interface` 的独立 HOLD 必须实际核验。
- V1 原有 18 项测试中，MUX 和恢复函数相关模拟主要在**历史 fixture** 代码上动态打补丁后执行，而不是运行现场 DEV 候选本身；因此本包改成**直接加载 candidates 代码**的独立测试。
- 事故前恢复 bag 中三次确实发生过的正常 MUX 切换，其切换前源间差值分别约 **0.084m / 0.09°、0.179m / 1.43°、0.032m / 0.43°**。本包沿用位置 **0.25m**、最短航向差 **12°** 的护栏，这三次已记录的交接均不会因差值被拒绝。**不能据此保证未见过的重规划/竞赛条件全部通过。**

## V2 改动

`candidates/trajectory_mux.py`：
- 原子地读取源消息和接收时间，避免服务回调读到不一致的消息/时间戳。
- 时间新鲜度检查拒绝未来时间戳；切换前检查新目标有效性；保留来源连续性与 0.25 秒后续突跳检查。
- 守护组件只能拒绝可疑 setpoint，**不会制造新的位置目标，也不会绕过健康/RC/PX4 保护**。

`candidates/yaw_guard_core.py`：
- 要求 ROS `PositionTarget.coordinate_frame == FRAME_LOCAL_NED (1)`，保留 XYZ/YAW 有效性、数值有限性、位置/yaw 交接门限。

`candidates/uav2_demo5_waypoints.py`：
- 恢复 EGO→POINT 前先调用 `disable_trajectory`，等待外部 `trajectory_interface` 的 HOLD 状态。
- 经原有 `abort_trajectory` 捕获当前机体位置/yaw；要求新 POINT 目标、最新里程计、当前姿态、机体速度和 `AIRBORNE_HOLD` 共同验证通过；连续 3 个新样本才接受。
- 如果仅仅是第一次 HOLD 捕获不及时，**只额外重试一次 abort/capture**；不自动重复航点、不跳过目标、不重新 ARM。
- 对当前 `ARMED+OFFBOARD`、`flight_healthy`、CH7 任务授权、`demo_frame_safe`、odom 时效和下游状态进行持续检查。
- `use_point` 后需要新鲜 `POINT_ACTIVE`；`enable_trajectory` 后还必须看到 `trajectory_interface=TRAJECTORY_ACTIVE` 及新鲜 MAVROS 输出与 POINT 一致，才允许继续下一个航点。
- 即使 `enable_trajectory` 服务 ACK 丢失，也尽力再次调用 `disable_trajectory` 退回已有 HOLD。这里的“退回 HOLD”仍依赖真实下游节点在线、位姿健康；不能在电脑断电时保证悬停。

## 直接测试了新候选源码

在包的 `tests/` 下执行：

```bash
python3 -B -m unittest discover -v
```

本包构建时测试结果：**27 tests, OK**。包含：旧 POINT 突跳被拒绝、正常切换允许、无效帧与 NaN、POINT 重新捕获、第一次捕获延迟自动重试、授权/健康/OFFBOARD 丢失、里程计陈旧、MUX ACK 失败、MAVROS 输出未确认、enable 服务 ACK 丢失。模拟使用 fake `rospy`，**没有真实 ROS、控制器或 PX4 在环**。

## 在 Ubuntu 的 Desktop 上使用

把压缩包放到 `~/Desktop/`，只做开发测试：

```bash
mkdir -p "$HOME/contest1_uav/dev/projects"
unzip -q "$HOME/Desktop/demo9_handoff_review_v2.zip" \
  -d "$HOME/contest1_uav/dev/projects"
cd "$HOME/contest1_uav/dev/projects/demo9_handoff_review_v2/tests"
python3 -B -m unittest discover -v
```

**不要将 `candidates/` 复制覆盖到当前 `runtime/` 或 Demo9，更不要据此直接试飞。**

如需进一步接入前审核，在无人机已落地且 DISARM 时运行包根目录的只读脚本：

```bash
bash "$HOME/contest1_uav/dev/projects/demo9_handoff_review_v2/collect_runtime_contract.sh"
```

脚本会从当前 `rospack find ego_uav_interface` 解析的路径收集 launch、POINT 节点、MUX、桥接和轨迹接口代码，输出 `~/Desktop/demo9_runtime_contract.tar.gz`。上传该文件即可继续进行**准确的启动路径整合和无桨在环测试**。脚本不会调用 ARM、解锁或修改参数。

## 尚未解决、实飞前必须完成

1. 核实真实 `uav2_demo5_multi_waypoint_interface.launch`、`point_traj_node.py`、`trajectory_interface.py`，并通过地面无桨模拟确认 Guard 拒绝时仍稳定发布 HOLD、不会丢失 OFFBOARD。
2. ROS 实际运行路径的 DEV 隔离接入、单节点验证、EGO/POINT 交接回放，以及完整 P1→P4 任务仿真；不能只做抽象 27 项测试。
3. **2026-10-19 事故中的电脑/ROS/视觉链路中断仍是独立故障。** 此补丁不解决 computer freeze、nodelet 崩溃、OFFBOARD 丢失、EKF 失效或 PX4 blind land。
4. 无桨与仿真通过后，仍需要检查电机/电调/桨叶和独立的机载电脑电源，再由现场飞手决定任何实际飞行。

**比赛目标：正常交接自动连续完成，短时 HOLD 捕获延迟至多重试一次；若定位、控制权或任务授权失效，则不为了“一遍过”强行自动恢复。**
