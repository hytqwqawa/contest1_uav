# contest1_uav

Stable contest1 UAV project.

## Directory layout

runtime/
    Stable ROS/PX4 runtime.

demo/
    Formal demonstration entrypoints.

data/
    Runtime data, bags, logs, and calibration.

## Stable runtime

Terminal 1:

    ~/contest1_uav/runtime/start_core.sh

Runtime workspaces:

    runtime/sunray_ws
    runtime/Fast-Drone-250
    runtime/ego_uav_ws

The runtime has passed ground integration validation with:

- T265 raw and filtered odometry
- PX4 local odometry
- MAVROS
- external vision pose
- NaN guard
- state monitor
- preflight monitor
- RC supervisor
- OFFBOARD manager

## Formal demos

Demo01:

    ~/contest1_uav/demo/01_4m_pole/start_backend.sh
    ~/contest1_uav/demo/01_4m_pole/run_mission.sh

Demo02:

    ~/contest1_uav/demo/02_waypoint_pole/start_backend.sh
    ~/contest1_uav/demo/02_waypoint_pole/run_mission.sh

## Safety

- NO SOFTWARE ARM
- CH5: manual ARM / DISARM
- CH6: manual POSCTL takeover
- CH7: autonomy permission
- CH8: emergency only
- Do not loosen the 0.08 m first-gap safety limit

## Development rule

Do not experimentally modify accepted demo code in place.

Make experimental copies outside the formal contest1_uav tree.
