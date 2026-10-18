# Demo9 V3：可地面联调的受控起飞 / 悬停 / 飞手降落候选版

## 这次为什么先采用飞手降落

机上 `/contest1/mavros/distance_sensor/hrlv_ez4_pub` 自报 `min_range=0.400m`，但旧 POINT 降落的末端切换条件是 `0.10m`，旧着陆确认阈值还使用 `0.08m`。这些读数低于传感器有效范围，因此不能证明实际接地，更不能用来空中停桨。

**V3 严格保留成功的 V1 自动起飞 Gate 和POINT/MUX/控制接口，但仅在悬停结束后要求飞手通过 CH7 LOW + CH6 POSCTL 接管，并由飞手安全降落，CH5 物理 DISARM。** 它不调用自动 LAND、软件 ARM/DISARM，也不改 PX4 参数。该版本能先实飞验证起飞/悬停观测，不代表自动降落已经修好。

## 不污染其他例程

新目录：`~/contest1_uav/demo/09_takeoff_hover_land_v3/`。安装时从机上的 Demo9 V1 读取成功起飞版本，复制独立 Core/MUX/POINT Backend；V1、冻结的 Demo8、runtime 均不写入。安装保存 V1/Demo8 SHA256 清单。V3 增设自己的 Core 包名、日志目录、bag 目录；回退仅移动 V3。

## 只读／离线

```bash
bash ~/contest1_uav/demo/09_takeoff_hover_land_v3/check_demo9_v3_static.sh
```

## 地面联调（物理 DISARM、POSCTL、CH7 LOW、CH8 SAFE）

先确认没有其他 Demo8/9 Core 和 Backend 在运行。

终端 1：
```bash
bash ~/contest1_uav/demo/09_takeoff_hover_land_v3/start_core.sh
```
终端 2：
```bash
bash ~/contest1_uav/demo/09_takeoff_hover_land_v3/start_backend.sh
```
终端 3（只读）：
```bash
source ~/contest1_uav/runtime/env.sh
pgrep -af '[t]rajectory_interface_demo9_v3.py'
timeout 5s rostopic echo -n 1 /contest1/mavros/state
timeout 5s rostopic echo -n 1 /contest1/mavros/extended_state
timeout 5s rostopic echo -n 1 /contest1/ego_uav_interface/point_traj_status
timeout 5s rostopic echo -n 1 /contest1/ego_uav_interface/trajectory_interface_status
timeout 5s rostopic echo -n 1 /contest1/ego_uav_interface/rc_mission_allowed
rostopic info /contest1/ego_uav_interface/point_setpoint
```
**地面合格不自动意味着允许实飞。** 飞手需独立检查飞行空间、定位、遥控接管、急停、起落区域、机身和桨叶、外部人员。

## 未来受控实飞入口（不能当成地面命令）

```bash
bash ~/contest1_uav/demo/09_takeoff_hover_land_v3/run_demo9_v3.sh
```

初始要求 DISARM、POSCTL、CH7 LOW、PX4 landed_state=ON_GROUND、flight_healthy True、setpoint stream ready。随后按既有物理飞手流程 CH7 LOW→MID→HIGH、CH5 ARM，不允许软件 ARM。受保护起飞至 0.8m，完成独立高度稳定检查，悬停 6 秒。**悬停结束后由飞手先 CH7 LOW、CH6 POSCTL 接管，确认 PX4 实际为 POSCTL，然后手动降落、物理 CH5 DISARM。**

起飞阶段如出现偏移、模式变化、定位异常，飞手先按已验证流程接管。不要在空中关闭 Core/Backend，不要通过 Ctrl+C 或向程序发送 kill 来代替接管。控制器不负责在 10cm 停桨。

## 回退

确认安全落地、物理 DISARM，并正常停止 V3 Mission、Backend、Core 后：

```bash
bash ~/contest1_uav/demo/09_takeoff_hover_land_v3/rollback_demo9_v3.sh
```

不会改变 V1、Demo8、runtime。V3 自动降落入口被拒绝是设计行为，不是故障。后续只有在可靠的末端高度/落地判定完成独立验证后，才考虑恢复自动降落和调整下降速度。

## 飞后分析（落地并物理 DISARM，rosbag 已退出后）

```bash
source ~/contest1_uav/runtime/env.sh
BAG="$(ls -t ~/contest1_uav/data/bags/demo9_takeoff_hover_land_v3/*.bag | head -n 1)"
python3 -B ~/contest1_uav/demo/09_takeoff_hover_land_v3/analyze_demo9_v3.py "$BAG"
```

对比之前固定起飞参考下的最大 XY、+0.7m 达标时间、POSCTL 接管前设定点间隔，以及飞手下降最后 15cm 的位置估计。分析器不会把低于 0.4m 的测距值当成安全触地证据。
