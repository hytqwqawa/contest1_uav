# Demo9 控制源交接 DEV V3：按实际 runtime 合同修复

**状态：开发候选；不允许直接覆盖正式 Demo9、runtime、PX4 或带桨起飞。**

本包根据 `demo9_runtime_contract.tar.gz` 验证当前 ROS 包解析路径：
`/home/yundrone/contest1_uav/runtime/ego_uav_ws/src/ego_uav_interface`。
`uav2_demo5_multi_waypoint_interface.launch` 的 MUX、POINT、EGO Bridge 均来自该 ROS 包；`trajectory_interface.py` 提供独立 50Hz HOLD 及 MAVROS 最终输出。**本 V3 压缩包没有接入飞控。**

## V3 修复的两个实际缺陷

1. **真实下游 HOLD 作为交接参考。** 当 `trajectory_interface_status == HOLD_ACTIVE` 且最终 `/contest1/mavros/setpoint_raw/local` 数据新鲜时，MUX 用最终 HOLD 与新 POINT 目标比较。不再用早已脱离机体的旧 EGO 内部目标作为参考。下游不在确认的 HOLD 或 TRAJECTORY 状态、状态或输出超时均拒绝切换。在 TRAJECTORY_ACTIVE 下，以最后新鲜 MUX 输出作为交接参考。MAVROS/ROS PositionTarget 使用同一 ROS 侧 ENU 数值比较。
2. **拒绝不能在计时器结束后自动解除。** 新源交接后 0.25 秒内出现异常，拒绝窗口会持续延长，直到收到连续的有效目标；不会因为超过 0.25 秒就释放仍异常的 POINT 指令。另外在 POINT `AIRBORNE_HOLD` 的状态话题新鲜时，始终监测相邻 HOLD 指令的骤变，即使已过切换窗口，约 90° 突变仍拒绝。正常起飞 POINT 目标重置不被这个额外规则限幅。

任务脚本 DEV V3 仍先 `disable_trajectory`，等待 POINT `abort_trajectory` 重新捕获最新悬停位置。增加**最终 MAVROS HOLD 与 POINT 新指令**的距离、偏航和数据时效验证；起飞/飞行状态、健康、CH7 RC 消息加入接收时间检查。轨迹启用后需要看到最终 MAVROS 输出与 POINT 匹配，才继续下一航点。POINT 捕获仅允许一次额外重试。服务 ACK 丢失时尽力恢复既有 HOLD。

## 验证范围

构建时 **41 项离线测试通过**。这些测试直接执行 DEV MUX、DEV 任务恢复方法、你上传的真实 `trajectory_interface.py` 控制分支，以及启动/话题静态接线合同。涵盖：旧 POINT 的约 0.96m/91°异常拒绝、EGO 偏离真实 HOLD 时正确切回 POINT、持续异常超过保护窗口也不穿透、POINT 已交接后再次收到旧 yaw 的拒绝、真实 IF 轨迹超时降级 HOLD、错误 RC/健康/OFFBOARD 状态停止自动交接等。

**测试使用假的 ROS 消息总线和时钟；没有运行真实 ROS 主节点、PX4、SITL、EGO 实时规划器或电机，不代表比赛 P1→P4 全任务测试通过。**

## 在 Ubuntu Desktop 上跑离线测试

先将 `demo9_handoff_contract_v3.zip` 放到 `~/Desktop/`，然后：

```bash
mkdir -p "$HOME/contest1_uav/dev/projects"
unzip -q "$HOME/Desktop/demo9_handoff_contract_v3.zip" -d "$HOME/contest1_uav/dev/projects"
cd "$HOME/contest1_uav/dev/projects/demo9_handoff_contract_v3/tests"
python3 -B -m unittest discover -v
```

期望 `Ran 41 tests` 且 `OK`。`candidates/` 内三份文件**仅用于核对、离线验证、未来 DEV ROS 包隔离接入**；不要复制到 `/runtime/`、冻结 Demo8 或正式 Demo9。包内 `changes_from_V2.diff` 和 `changes_from_active_runtime_and_mission.diff` 可审阅。需要退出 DEV 候选时，不启动其脚本即可；正式环境没有被修改。

## 仍需解决的两件事

- **实际 ROS/PX4 无桨在环与 SITL 全程回归**：确认正常 P1→P4 交接、拒绝/超时进入独立 HOLD、地理围栏与原 RC 授权正常。没有这些证据，不应当把 V3 接入带桨比赛。
- **2026-10-19 电脑/ROS 链路中断**：偏航交接补丁不能修复机载电脑失去响应、nodelet 崩溃、T265/外部视觉中断、PX4 OFFBOARD 丢失或 blind land。包内 `collect_20261019_pc_fault.sh` 仅回读故障启动时间段的系统日志，输出到 Desktop；不触发飞行或改动系统。可以在需要时执行 `bash ~/contest1_uav/dev/projects/demo9_handoff_contract_v3/collect_20261019_pc_fault.sh`。

**赛场目标：正常切换连续执行，短时数据更新迟到只有限次数的自动重新捕获；真正丢失健康、定位、控制权或 RC 授权时不强行续飞。**
