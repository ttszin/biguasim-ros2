"""Standalone BiguaSim runner for the BlueROV2 (ArduSub), bridged to real
ArduPilot Sub SITL — analogous to biguasim_sim_runner_blueboat.py's approach
for the BlueBoat/Rover, and to how biguasim_sim_runner.py does it for the
DjiMatrice.

Unlike those two, this reimplements ArduBiguaSimRunner's base run() loop
(same pattern as t2_bluerov2_velbridge_runner.py) instead of calling it
directly, to also relay BiguaSim's own ground-truth position/velocity to
ArduSub as GPS_INPUT (#232) on a second MAVLink link (SERIAL1, port 5762).

Why this is needed, found live: ArduSub here runs with GPS_TYPE=14 (MAV) --
see t2_gps_input_bridge_bluerov2.py's docstring and biguasim_bridge/
README.md's "Position-hold for BlueROV2" section for the full story of why
that workaround exists. That script relays ArduSub's own GLOBAL_POSITION_INT
(the EKF's *estimate*) back to it as "GPS" -- a self-referential loop with
no independent ground truth to correct drift. Confirmed live: this produces
a genuine closed-loop instability, not just a stale/noisy estimate -- video
showed the vehicle physically climbing out of the water, tipping hard to
one side, and drifting into the world's bridge geometry, while logged
`velocity_local` grew ~exponentially (12 -> 73 m/s laterally over 105s).
Any EKF drift gets reported back as "GPS truth", the position controller
applies real thrust to correct the now-reinforced error, and that real
motion feeds back into the EKF via the IMU -- with nothing external ever
pulling the estimate back toward truth.

Fixed by sourcing GPS_INPUT from BiguaSim's own LocationSensor/
VelocitySensor instead of ArduSub's GLOBAL_POSITION_INT: this process
already computes exactly that, every tick, via
bridge.build_json_state()['position'] / ['velocity'] (frame.py's
pos_nwu_to_ap()/vel_nwu_to_ned(), already in the right units/frame for
GPS_INPUT's lat/lon/alt and vn/ve/vd fields) to feed ArduSub's own JSON
physics link -- it was simply never also sent as GPS_INPUT. This is a real,
independent position source (the simulation's own ground truth), not an
echo of the vehicle's estimate, so it can actually correct drift the way a
real GPS would. t2_gps_input_bridge_bluerov2.py is no longer needed
(Terminal 4 in biguasim_bridge/README.md's command sequence) -- kept only
for reference/comparison, not as the recommended path.

Usage:
    python3 biguasim_sim_runner_bluerov2.py --viewport
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

from biguasim.ardubridge import ArduBiguaSimRunner
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Same x=25 water-crossing point already validated by t2_land_test.yaml/
# t2_hover_test.yaml/biguasim_sim_runner_blueboat.py. z=-1.0 (already
# submerged 1m below the surface at spawn) rather than the BlueBoat's z=0.2 —
# the BlueROV2 has no hull/buoyancy to float on the surface with, and this
# test's whole point is validating position hold while submerged. Starting
# point only, adjustable if live testing shows a spawn-splash/settle issue
# like the BlueBoat's did.
DEFAULT_LOCATION = [25.0, 0.0, -1.0]

# SERIAL1 (5762), not SERIAL0/5760 (MAVROS's own link) -- ArduPilot's SITL
# TCP serial emulation only actively services one client per port; sharing
# 5760 starves MAVROS's connection (confirmed live, see README).
GPS_CONNECTION = "tcp:127.0.0.1:5762"
GPS_RATE_HZ = 5.0


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim BlueROV2 (ArduSub) SITL Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=DEFAULT_LOCATION,
        metavar=("X", "Y", "Z"),
        help=f"Agent start location in BiguaSim NWU metres (default: {DEFAULT_LOCATION})",
    )
    args = parser.parse_args()

    profile = VEHICLE_REGISTRY["BlueROV2"]
    scenario = ArduBiguaSimRunner.build_scenario(
        profile,
        package_name="SkyDive",
        world="Bridge",
        agent_name="bluerov0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    with ArduBiguaSimRunner(profile, scenario, show_viewport=args.viewport, verbose=True) as runner:
        # Reimplements ArduBiguaSimRunner.run()'s loop (same reason as
        # t2_bluerov2_velbridge_runner.py: need to do something extra per
        # tick that the base loop doesn't support) to also relay ground-truth
        # GPS_INPUT alongside the normal PWM-in/JSON-state-out exchange.
        bridge = runner._bridge
        env = runner._env
        agent = runner._agent_name
        dt = runner._dt

        # Port 5762 (SERIAL1) only starts listening once ArduSub's own
        # accept() on SERIAL0/5760 unblocks -- which needs MAVROS to have
        # connected first (see README) -- so this process can legitimately
        # reach this point before that's happened. Don't block the physics
        # loop on it: BiguaSim/Unreal is already sitting idle waiting for
        # the very first env.step() below, and gating that on an external
        # MAVLink connection would just add another way to reproduce the
        # exact stall this whole file's docstring is about. Connect lazily,
        # retried opportunistically from inside the tick loop instead.
        gps = None
        gps_period = 1.0 / GPS_RATE_HZ
        last_gps_send = 0.0
        last_gps_connect_attempt = 0.0

        bridge.bind()
        motor_cmds = [0.0] * profile.num_motors
        raw = env.step(motor_cmds)
        agent_state = raw[agent][0]
        sim_time = 0.0

        print(f"Running {profile.name} SITL bridge (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is None:
                    continue
                motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                raw = env.step(motor_cmds)
                agent_state = raw[agent][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                now = time.time()
                if gps is None and now - last_gps_connect_attempt >= 2.0:
                    last_gps_connect_attempt = now
                    try:
                        gps = mavutil.mavlink_connection(GPS_CONNECTION)
                        gps.wait_heartbeat(timeout=2)
                        print(f"[gps_ground_truth] Connected to {GPS_CONNECTION}, "
                              f"sysid={gps.target_system} compid={gps.target_component}")
                    except (ConnectionRefusedError, OSError):
                        gps = None

                if gps is not None and json_state is not None and now - last_gps_send >= gps_period:
                    last_gps_send = now
                    lat, lon, alt = json_state["position"]
                    vn, ve, vd = json_state["velocity"]
                    gps.mav.gps_input_send(
                        0,  # time_usec (0: let ArduPilot use its own onboard clock)
                        0,  # gps_id
                        0,  # ignore_flags: every field below is supplied
                        0, 0,  # time_week_ms, time_week (unused when time_usec is also 0)
                        3,  # fix_type: 3D fix
                        int(lat * 1e7), int(lon * 1e7), alt,
                        1.0, 1.0,  # hdop, vdop
                        vn, ve, vd,
                        0.5,  # speed_accuracy
                        1.0,  # horiz_accuracy
                        1.0,  # vert_accuracy
                        10,  # satellites_visible
                    )

                if frame is not None and frame % 200 == 0:
                    print(
                        f"  t={sim_time:.2f}s frame={frame} "
                        f"quat={[f'{v:.3f}' for v in json_state['quaternion']]} "
                        f"motors={[f'{m:.1f}' for m in motor_cmds]}"
                    )
        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()


if __name__ == "__main__":
    main()
