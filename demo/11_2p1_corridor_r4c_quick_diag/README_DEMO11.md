## R2 REVIEW-ONLY / FLIGHT LOCKED
Do not use flight commands in this directory. See README_DEMO11_R2.md.

# Demo11 — 2.1 三航点圆弧中点观察（开发版，未实飞）

来源：现场实际 Demo10 `~/contest1_uav/demo/10_demo9_v4_success/` 复制，
仅使用 2026-10-21 导出 ZIP 审计过的 Demo10 源码，安装时核对其全部 133 个 SHA256。
**Demo10、runtime、原 Demo11 双杆实验以及历史数据不会修改/覆盖。**

## 固定三航点（0.80m 高）

1. P1 `F+3.000 L0.000` EGO，terminal yaw `0°`，模拟直道尽头。
2. P2 `F+4.061 L-0.439` EGO，terminal yaw `-135°`，圆弧**弧长中点**，悬停观察识别源。
3. P3 `F+4.500 L-1.500` EGO，terminal yaw `-180°`，末点交接。

几何参考圆心 `C=(F+3.0,L-1.5)`、半径 `R=1.5m`，P1→P3 为 90° 右转圆弧。
**P2 不在圆心；也不是比赛 180° U 形顶点。**
由于 EGO 按实际障碍地图规划，三个点**不保证真实航迹沿圆弧**。
本版本模拟单机朝向/交接，不包含识别任务、多机协同或 ArUco 自动降落。

## 不变的关键保护

原始 `uav2_demo5_waypoints.py`、`demo8_terminal_yaw.py`、`demo9_posctl_handoff.py`、
`ros_src/ego_uav_interface/`、Frame Guard/RC/health/轨迹连续性全部沿用 Demo10 源码字节。
保留最大速度 0.20m/s、障碍膨胀 0.299m（runtime EGO 配置，在本机再次核对）、
`dist0=0.30m`、地图分辨率 0.15m、高度 0.80m、原软件围栏。
无自动 ARM、无 LAND 请求。最终航点：EGO→POINT 稳定 HOLD 3s→单次 POSCTL 请求→**飞手实体遥控降落并观察 DISARM**。

## 核验与启动（安装后，不是现在执行飞行）

**只读静态核验**（无需 ROS master，不 ARM）：

```bash
D="$HOME/contest1_uav/demo/11_2p1_arc_midpoint_v1"
python3 -B "$D/demo11_verify.py"
python3 -B "$D/test_demo11_offline.py"
```

只有人工现场确认实际障碍与围栏、飞手接管、足够距离、可控机况后，再按 Demo10 原模式：

- 终端 1：`bash "$D/start_core.sh"`
- 终端 2：`bash "$D/start_backend.sh"`
- 无桨，DISARM、POSCTL、CH7 LOW，环境启动后（同时核对 ROS 规划器 4 项实时参数）：
  `source "$HOME/contest1_uav/runtime/env.sh" && python3 -B "$D/demo11_verify.py" --ground`
- 终端 3 真正飞行入口：`bash "$D/run_demo11.sh"` **必须另行人工批准，不作为地面测试**。

**注意**：`start_core.sh`、`start_backend.sh` 会实际启动 ROS 硬件节点，
`run_demo11.sh` 运行到实体飞手权限门后可进入真飞行，不要把它当作审计命令。
只有现场飞手按 Demo10 原流程操纵 CH5/6/7/8；碰撞/异常必须由现场处理。
飞行时禁止 Ctrl+C 代替接管。Ground gate 需要全程 DISARM。

## 独立日志和回滚

- 核心日志：`~/contest1_uav/data/logs/core_11_2p1_arc_midpoint_v1/`
- 后端日志：`~/contest1_uav/data/logs/demo11_2p1_arc_midpoint_v1/`
- Bag：`~/contest1_uav/data/bags/demo11_2p1_arc_midpoint_v1/`
- 任务内部实时日志位于新 Demo11 副本，不写 Demo10。

回滚无需恢复原始文件：**停止程序并确认 DISARM**，选择原 Demo10 脚本重启即可。
若需归档 Demo11，只在确认 DISARM、三个终端已退出且包数据已备份后，
使用 `mv` 将整个 `11_2p1_arc_midpoint_v1` 重命名归档；不要删除 runtime。
旧 `run_mission.sh`、`run_demo10.sh`、`start_demo7.sh` 在本 Demo11 副本中故意禁止执行，
以防旧入口绕过新版本参数审计。
