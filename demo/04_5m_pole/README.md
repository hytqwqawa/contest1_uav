# 04_5m_pole

Status: UNDER DEVELOPMENT

## Mission

Demo1 tangent/EGO mission with the goal extended from L=4.00 m to L=5.00 m.

The flight sequence is:

    Demo3 takeoff
    -> Demo1 tangent/EGO translation
    -> 5.00 m Demo-frame LEFT goal
    -> Demo3 landing

## Formal entrypoints

Terminal 1:

    ~/contest1_uav/demo/04_5m_pole/start_core.sh

Terminal 2:

    ~/contest1_uav/demo/04_5m_pole/start_backend.sh

Terminal 3:

    ~/contest1_uav/demo/04_5m_pole/run_mission.sh

## Demo3 characteristics inherited

- T265 filtered odometry passes through the Demo3 landing sanitizer.
- PX4 external vision consumes /contest1/t265/odom/landing_safe_dev.
- Takeoff height: 0.80 m.
- Takeoff speed: 0.40 m/s.
- Takeoff acceleration: 0.80 m/s^2.
- EGO max velocity: 0.20 m/s.
- Landing speed: 0.30 m/s.
- Terminal landing speed: 0.20 m/s.
- Touchdown range threshold: 0.08 m.
- Touchdown range timeout: 0.30 s.
- Touchdown stable time: 0.20 s.
- Touchdown minimum drop: 0.30 m.
- No software ARM.
- No forced software DISARM.

## Geometry

- Goal: Demo forward = 0.00 m, left = +5.00 m.
- Authoritative forward fence remains [-1.50, +1.50] m.
- Authoritative left fence is [-0.50, +5.50] m.
- The historical +0.50 m margin beyond the goal is preserved.
- Height fence remains [-0.25, +1.30] m.

## Safety

- CH5: physical ARM / DISARM.
- CH6: POSCTL manual takeover.
- CH7: autonomy permission.
- CH8: emergency only.
- First-gap threshold remains strictly 0.08 m.
- Touchdown threshold remains strictly 0.08 m.

Do not mark this demo accepted until static, DISARM ground, and real-flight
validation have all passed.
