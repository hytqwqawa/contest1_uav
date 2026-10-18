# Demo6 V2 — Accepted QR and POINT Return-to-Home

Real flight: POINT takeoff at 0.80m -> F+0.50,L-0.50 -> fresh QR decode + 5 second hold -> POINT return to captured takeoff XY F0 L0 -> proven auto LAND -> PX4 DISARM.

QR payload: 'HIT'

Both paths are POINT and have NO obstacle avoidance. No software ARM. CH5/CH6/CH7/CH8 safety procedure retained.
Validated flight bag is an EXTERNAL file (path and SHA256 in flight_evidence/FLIGHT_BAG.json); it is not copied into demo.
Runtime sources recorded under runtime_snapshot; compiled runtime workspaces remain external dependencies.
