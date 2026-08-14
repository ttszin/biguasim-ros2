"""Workaround for a real ArduSub SITL bug found live: the native GPS backend
detection (GPS_TYPE=AUTO -> simulated u-blox protocol probing, and
GPS_TYPE=100/SITL's direct-injection path) both get permanently stuck for
ArduSub specifically in this environment — confirmed via the dataflash log
("GPS 1: probing for u-blox/SITL at 230400 baud" logged once, then never
advances to "EKF3 IMUx is using GPS", across every ArduSub session tried,
while the equivalent DjiMatrice/Copter session completes in ~75s with the
identical --model JSON setup). Without a real GPS fix, ArduSub's
Sub::position_ok() never returns true and GUIDED mode is permanently
rejected ("Guided requires position").

Workaround: GPS_TYPE=14 (MAV) bypasses the stuck serial-probe path entirely
(AP_GPS.cpp's _detect_instance() explicitly short-circuits GPS_TYPE_MAV
before ever reaching the baud-cycling logic) and just waits for MAVLink
GPS_INPUT (#232) messages instead. This script closes the loop: reads the
vehicle's own GLOBAL_POSITION_INT (already confirmed reliable — the EKF's
predicted position tracks BiguaSim's real synthetic GPS truth closely even
without a fused GPS source) and relays it straight back as a synthetic
GPS_INPUT with a plausible fix (fix_type=3, 10 satellites, hdop=1.0).

Requires t2_biguasim_bluerov2.parm's GPS_TYPE/GPS1_TYPE set to 14.

Usage:
    python3 t2_gps_input_bridge_bluerov2.py
"""

from __future__ import annotations

import time

from pymavlink import mavutil

CONNECTION = "tcp:127.0.0.1:5760"
RATE_HZ = 5.0


def main() -> None:
    m = mavutil.mavlink_connection(CONNECTION)
    print(f"[gps_input_bridge] Connecting to {CONNECTION}...")
    m.wait_heartbeat(timeout=30)
    print(f"[gps_input_bridge] Connected, sysid={m.target_system} compid={m.target_component}")

    period = 1.0 / RATE_HZ
    last_send = 0.0
    sent_count = 0

    while True:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg is None:
            continue

        now = time.time()
        if now - last_send < period:
            continue
        last_send = now

        m.mav.gps_input_send(
            0,  # time_usec (0: let ArduPilot use its own onboard clock)
            0,  # gps_id
            0,  # ignore_flags: 0 — every field below (position, velocity, hdop/vdop,
            # speed/horiz/vert accuracy) is supplied with a real/plausible value.
            0, 0,  # time_week_ms, time_week (unused when time_usec is also 0)
            3,  # fix_type: 3D fix
            msg.lat, msg.lon, msg.alt / 1000.0,  # GLOBAL_POSITION_INT alt is mm
            1.0, 1.0,  # hdop, vdop
            msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0,  # GLOBAL_POSITION_INT vel is cm/s
            0.5,  # speed_accuracy
            1.0,  # horiz_accuracy
            1.0,  # vert_accuracy
            10,  # satellites_visible
        )
        sent_count += 1
        if sent_count % (int(RATE_HZ) * 5) == 0:
            print(f"[gps_input_bridge] sent {sent_count} GPS_INPUT messages "
                  f"(lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f} alt={msg.alt/1000.0:.2f}m)")


if __name__ == "__main__":
    main()
