# Demo11 R4 — 独立飞行候选（默认拒绝执行）

本包 **不是现场实飞放行证明**。代码保留原 Demo11 三航点、Demo10 的飞手授权与 V4 交接、R3 的最终设定点实时越界拦截和私有 P2 Planner。**没有软件 ARM、软件 DISARM 或自动 LAND。** 不能用关闭退出码 97 取代真机无桨接管检查。

## 版本与任务

- 新目录：`~/contest1_uav/demo/11_2p1_corridor_r4_flight_candidate/`
- 原版 Demo10、Demo11 V1、R2、R3、runtime 不变。安装会核对 R3 的 **149** 个哈希并拒绝覆盖已有 R4。
- 开阔、无障碍、单机；P1 F3 L0，P2 F4.061 L-0.439，P3 F4.5 L-1.5，H0.8，速度上限 0.20m/s。
- 初始直道 F0..3, L[-0.5,+1.0]。候选安全中心 L[-0.05,+0.55]；靠近直道端点的 F[-0.2,+3.2] 仍检查墙约束。三航点 B-spline **不保证沿参考圆弧**；若未找到合法轨迹，拒绝并人工接管，而不是放宽墙体、障碍膨胀或跳过任务检查。
- PX4、RC、定位/深度、物理机架、电源、净空等须由现场飞手和独立安全观察员验收。

## 装载：不启动 ROS、不起飞

```bash
cd ~/Desktop
unzip -n demo11_r4_flight_candidate.zip
python3 -B ~/Desktop/demo11_r4_candidate_package/install_r4_candidate.py
```

## 现场最少必要步骤

**0. 必须拆桨验证真实 PX4/遥控接管与失效保护。** 这不是模拟器验证。连接动力电池，飞控 DISARM，CH6 HIGH(POSCTL)，CH7 LOW；不能在桨已装好时运行无桨挑战。飞手确认无桨条件下已演示真实 PX4 的模式接管和控制失效保护，记录结果；若实际开关/通道与本项目 `rc_supervisor.py` 不一致，停止。

使用**新的 R4 终端**启动 Core 与 Backend（**先关闭旧 R3 Core/Backend**；不能让两个版本同时控制或发布）：

终端 1：

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4_flight_candidate
bash "$D/start_core.sh"
```

终端 2：

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4_flight_candidate
bash "$D/start_backend.sh"
```

终端 3，仍然**拆桨 DISARM**：

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4_flight_candidate
bash "$D/prepare_r4_noprop.sh"
```

该检查只订阅真实 RC/PX4 ROS 状态，不执行模式切换、不 ARM。依提示先拨动 CH6 **HIGH→LOW→HIGH**、再拨动 CH7 **LOW→MID→HIGH→LOW**。两个真实人员（飞手和安全观察员）现场记录已完成真实 PX4 接管/失效保护检查和无遮挡场地净空核验；软件不能代替此项判断。准备成功后候选生成本机、限时 45 分钟的证据文件。**授权不等于自动起飞。**

**1. 完成现场放行前不要装桨。** 由合格飞行负责人确认：电池为当前匹配 6S 包、电芯状态、QGC 与万用表电压接近；开阔净空和安全距离；CH6 物理切回 POSCTL、CH7 任务撤销有效；无桨控制链检查真实成功；飞手和观察员均已就位，现场可立刻中止。

**2. 仅在真实现场放行后**，先关闭测试进程并断开动力电池，按飞手 SOP 安装螺旋桨、检查紧固与桨向；再接电、重新启动 R4 Core/Backend，将飞机安置于明确的地面原点，CH6 HIGH/POSCTL、CH7 LOW、DISARM。不得在带电情况下装桨；必须确认无其他飞行程序在运行。

终端 3 实飞候选入口：

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4_flight_candidate
bash "$D/run_r4_flight.sh"
```

飞行入口会重新核验：代码 SHA、R4 栈进程、私有 EGO、V4 MUX、唯一 MAVROS 输出者、电池范围、RC LOW、DISARM/POSCTL、授权时间。随后**继续执行原 Demo11 真实飞手门控**：Frame Guard 新鲜坐标原点、RViz 航点复核、物理 CH7 LOW→MID→HIGH、飞手实体 ARM。脚本**从不主动 ARM**；R3 末端通道检查继续工作。最终 EGO→POINT HOLD→POSCTL，飞手实体遥控降落、接地后 DISARM。

**遇到 HOLD、没有新 B-spline、越界、yaw 突变、遥测异常时，不要重启任务、重复授权或在空中 Ctrl+C 代替接管。** 飞手用 CH7 LOW + CH6 POSCTL 接管并人工降落。当前软件 HOLD 依赖定位与电源，不等同于独立安全保障。

## 故障/回滚

任何检查失败：`FLIGHT LOCK` / 退出 97，**不要绕过或删除**。重新确认现场条件。若 R4 配置、状态不符，应保持停飞并上传日志。回滚不修改老目录：安全落地 DISARM 后停止 R4，独立启动已知成功版本，不复用 R4 会话。

本文件不证明已通过真实 PX4 无桨控制接管。设计不能凭某个软件 PASS 单独决定实际飞行安全。
