# R4F: conservative refinement of P2 B-spline preview proof

Source: immutable `R4E` 158-file candidate. New independent install: `11_2p1_corridor_r4f_span_proof`.

## Flight log finding

At 20:58, P1 translation completed. P2 B-spline preview rejected control hull
span 5: F[3.048,3.612], L[-0.236,0.001], against R4E center limit
L[-0.150,+0.650] within guarded F[0.8,3.2]. This is a control hull
bound; without actual spline knots/control positions, one cannot know if
real curve crosses the protected zone.

## Fix

DO NOT widen any wall or center envelope. Keep F1..3 physical walls,
F0.8..3.2 swept airframe, L[-0.15,+0.65], RC, HOLD, PX4, and final
output gate. `demo11_corridor.py` first retains the old convex-hull
fast path. Only if the conservative hull is ambiguous, derive the
per-knot-span componentwise upper bounds on actual B-spline derivatives.
Recursively enclose position at every parameter subinterval by the midpoint
plus/minus speed bound x interval radius, including 1e-8m outward padding.
A subinterval passes only if proven wholly outside the guarded F range or
wholly within allowed L range. Otherwise split, or fail CLOSED at resource
/precision limits. No path editing, clipping, sampled-only shortcuts or
runtime wall changes. B-splines genuinely crossing L<-0.15 at F<=3.2 reject.

**IMPORTANT:** Actual R4E P2 control points and knots were NOT provided,
so it is NOT yet proven that R4F accepts P2. If R4F still refuses, the
trajectory really may violate the wall or is too close to certify; then
redesign the P1->P2 turn rather than weaken the safety envelope.

## Install while DISARM; does not start flight

1. Copy `demo11_r4f_span_proof.zip` to Desktop.
2. `cd ~/Desktop && unzip -n demo11_r4f_span_proof.zip`
3. `python3 -B ~/Desktop/demo11_r4f_adaptive_proof_package/install_r4f.py`

After install, stop prior R4E sessions and ensure PX4 remains DISARM.
Start Core and Backend as usual, then `run_r4_flight.sh` on terminal 3
**only after confirming local clearance, RC takeover and all live gates**.

Terminal 1: `bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/start_core.sh`
Terminal 2: `bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/start_backend.sh`
Terminal 3: `bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/run_r4_flight.sh`

No flight release is implied by passing software tests. Neither the new
curve proof nor the previous R4E has been demonstrated to safely traverse
physical corridor walls in the user's current venue. First dry-run a
captured P2 B-spline offline and ensure the trajectory and actual airframe
satisfy measured wall clearance. Original R4E, Demo10, runtime are unchanged.
