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

Two more real bugs found live and fixed here, both about *which* MAVLink
link this script uses -- neither was reachable until the Unreal Engine
hang (see biguasim_bridge/README.md) was root-caused and fixed, since this
script never got to run against a live vehicle before that:

1. Connecting to port 5760 (SERIAL0, the same primary link MAVROS/GCS
   uses) starves MAVROS's own connection: ArduPilot's SITL TCP serial
   emulation only actively services one client per port at a time.
   Confirmed live -- with this script also connected to 5760, MAVROS's
   `/mavros/state` froze at `connected: false` indefinitely; killing this
   script's connection let MAVROS recover within seconds. Fixed by using
   port 5762 (SERIAL1, ArduPilot's own separate telemetry port, already
   listening by default) instead -- a fully independent link.
2. Even on its own dedicated link, GLOBAL_POSITION_INT isn't streamed
   automatically -- SITL only streams position data to a link once a
   client explicitly asks (this is what MAVProxy/MAVROS/QGC normally do
   on connect; this script talks raw pymavlink, so it has to ask itself).
   Fixed by sending MAV_CMD_SET_MESSAGE_INTERVAL for message id 33
   (GLOBAL_POSITION_INT) right after the heartbeat handshake.

Usage:
    python3 t2_gps_input_bridge_bluerov2.py
"""

from __future__ import annotations

import time

from pymavlink import mavutil

CONNECTION = "tcp:127.0.0.1:5762"  # SERIAL1 -- independent from MAVROS's SERIAL0 (5760)
RATE_HZ = 5.0


def main() -> None:
    m = mavutil.mavlink_connection(CONNECTION)
    print(f"[gps_input_bridge] Connecting to {CONNECTION}...")
    m.wait_heartbeat(timeout=30)
    print(f"[gps_input_bridge] Connected, sysid={m.target_system} compid={m.target_component}")

    # SITL doesn't stream GLOBAL_POSITION_INT to a link unless asked --
    # request it explicitly (message id 33, 5Hz), since this isn't a GCS
    # that does this automatically on connect.
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        33, int(1e6 / RATE_HZ), 0, 0, 0, 0, 0,
    )

    period = 1.0 / RATE_HZ
    last_send = 0.0
    sent_count = 0
    got_real_fix = False

    while True:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg is None:
            continue

        # Guards against injecting a real garbage/bad fix -- but NOT against
        # (0,0) unconditionally: GPS_TYPE=14/AP_GPS_MAV (see AP_GPS_MAV.cpp)
        # has no other position source, so ArduSub's own GLOBAL_POSITION_INT
        # genuinely reports (0,0) until it gets its very first relayed
        # GPS_INPUT -- skipping every (0,0) message here means it can never
        # bootstrap at all (confirmed live: with this guard unconditional,
        # zero GPS_INPUT messages were ever sent, position stayed at (0,0)
        # forever). Once a real fix has been sent at least once, a sudden
        # jump back to exactly (0,0) really would be bad data worth
        # dropping -- so only guard after that point.
        if got_real_fix and msg.lat == 0 and msg.lon == 0:
            continue
        if msg.lat != 0 or msg.lon != 0:
            got_real_fix = True

        now = time.time()
        if now - last_send < period:
            continue
        last_send = now

        send_alt = msg.alt / 1000.0  # GLOBAL_POSITION_INT alt is mm
        if msg.lat == 0 and msg.lon == 0 and send_alt == 0:
            # AP_AHRS::update_state()'s SITL-only sanity check (AP_AHRS.cpp)
            # calls AP_HAL::panic() -- an infinite for(;;) loop, confirmed
            # live via gdb backtrace -- whenever the EKF reports
            # location_ok=true for a Location where lat/lng/alt are ALL
            # zero (Location::initialised(), Location.h, treats an all-zero
            # tuple as "not initialised" and this is exactly that). The very
            # first bootstrap relay is unavoidably (0,0) for lat/lon --
            # that's genuinely what ArduSub reports before its first fix --
            # but alt happening to be 0 too at that exact instant tips it
            # into the all-zero case that panics. Nudge alt by a physically
            # meaningless amount so the triple is never all-zero; real
            # coordinates arrive in the next few relayed messages once the
            # EKF has something to converge from.
            send_alt = 0.01
        m.mav.gps_input_send(
            0,  # time_usec (0: let ArduPilot use its own onboard clock)
            0,  # gps_id
            0,  # ignore_flags: 0 — every field below (position, velocity, hdop/vdop,
            # speed/horiz/vert accuracy) is supplied with a real/plausible value.
            0, 0,  # time_week_ms, time_week (unused when time_usec is also 0)
            3,  # fix_type: 3D fix
            msg.lat, msg.lon, send_alt,
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
