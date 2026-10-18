# Demo6 V2 — QR scan then return to takeoff XY and land

Status: DEVELOPMENT. NOT accepted for flight until real-world validation.

P1: F+0.50, L-0.50 at H0.80, POINT straight-line, yaw held, NO avoidance.
Only after P1 stable arrival: start fresh D435i RGB QR detector (5Hz decode,
3-frame stability, 1Hz terminal updates). A confirmed QR starts a 5.0s clock;
the QR text is printed every second. Then stop QR detector and execute P2.
P2: F0, L0 at H0.80, POINT return to captured takeoff origin XY; stable
arrival required BEFORE the proven Demo5 auto landing and PX4 DISARM.
If no QR within 30s or codes conflict, return and land only while normal
permissions/health still hold; mark task FAILED after landing.

Safety: CH5 physical ARM/DISARM; CH6 POSCTL takeover; CH7 autonomy grant;
CH8 emergency. NEVER software ARM. CH7 must remain HIGH during entire
autonomous mission until landed; manual takeover overrides on anomalies.
POINT does NOT avoid obstacles on either outbound or homecoming path.

Three terminals (after prior ground verification):
    ./start_core.sh
    ./start_backend.sh
    ./check_demo6_static.sh && ./run_demo6.sh

Do NOT execute the old draft `qr_mission.py`.
Demo5 accepted, Demo6 V1 and runtime are never modified by the installer.
