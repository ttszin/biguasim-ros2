"""BiguaSim side of the T8.2 Stage D test (underwater): BlueROV2 (ArduSub SITL) in the SkyDive/Bridge world.

Same recipe as missions/biguasim_sim_runner_bluerov2.py (ground-truth GPS_INPUT on SERIAL1, tcp:5762, drained every
tick) plus what the planner tests need: the scenario's obstacles are spawned as static props and the ground truth
(position, velocity, thruster commands) goes to a CSV.

    python3 rov_runner.py --scenario stage_d/scenarios/d1_poles.yaml --log /tmp/truth.csv [--viewport]

Start ArduSub SITL first (bash missions/t2_sitl_run_bluerov2.sh), connect MAVROS / rov_exec.py to tcp:5760, then this.
Frame of the scenario and of the CSV origin: (north, east, up) with z = 0 at the water surface (BiguaSim x = north,
y = -east). Prints STAGE_D_RUNNER_READY once the world is up and the props are spawned.
"""

from __future__ import annotations

import argparse
import csv
import os
from dataclasses import replace
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

# Yaw-loop sign convention (finding of 2026-09-24, see BIGUASIM_NOTES.md): the ORIGINAL mt_z of HexaCopterFiveDoF together with the
# un-flipped gyro yaw gives a consistent yaw loop (heading held within ~2 deg for 60 s); the library default (mt_z negated + gyro r
# not negated, both added on 2026-08-30) makes the gyro disagree in sign with the attitude quaternion, and ArduSub's GUIDED
# position control then spins the vehicle. --legacy-yaw restores the library behaviour. Must be set before biguasim is imported.
if "--legacy-yaw" not in sys.argv:
    os.environ.setdefault("BIGUA_MTZ_ORIG", "1")

PLANNING = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PLANNING))
sys.path.insert(0, str(PLANNING / "stage_b"))
from common_b import to_bsim  # noqa: E402

from recorder import CAMERAS, Recorder  # noqa: E402
from scenario import load_scenario  # noqa: E402

from biguasim.ardubridge import ArduBiguaSimRunner  # noqa: E402
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY  # noqa: E402

SURFACE = (25.0, 0.0, 0.0)          # BiguaSim position of the scenario origin (x = 25 is the validated water-crossing point)
SPAWN = [25.0, 0.0, -2.0]
GPS_CONNECTION = "tcp:127.0.0.1:5762"
GPS_RATE_HZ = 5.0
LOG_EVERY = 5


def spawn_obstacles(env, sc) -> None:
    for group, material in ((sc.known, "steel"), (sc.hidden, "brick")):
        for o in group:
            if hasattr(o, "radius"):
                c = to_bsim(o.north, o.east, (o.z_min + o.z_max) / 2.0, SURFACE)
                env.spawn_prop("cylinder", location=c, rotation=[0, 0, 0], scale=[2 * o.radius, 2 * o.radius, o.z_max - o.z_min],
                               sim_physics=False, material=material, tag="t82_obstacle")
            else:
                c = to_bsim((o.n_min + o.n_max) / 2, (o.e_min + o.e_max) / 2, (o.u_min + o.u_max) / 2, SURFACE)
                env.spawn_prop("box", location=c, rotation=[0, 0, 0], scale=[o.n_max - o.n_min, o.e_max - o.e_min, o.u_max - o.u_min],
                               sim_physics=False, material=material, tag="t82_obstacle")


def _euler_deg(q) -> tuple[float, float, float]:
    """roll, pitch, yaw (deg) of the body (FRD) in NED from the bridge's quaternion [w, x, y, z]."""
    w, x, y, z = (float(v) for v in q)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(np.degrees([roll, pitch, yaw]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--ticks", type=int, default=200)
    ap.add_argument("--viewport", action="store_true")
    ap.add_argument("--flip-yaw", type=int, choices=[0, 1], default=None,
                    help="override VehicleProfile.flip_gyro_yaw (yaw-rate sign of the gyro sent to ArduSub); default: the profile's")
    ap.add_argument("--record", help="write an mp4 of the flight (chase camera, see recorder.py)")
    ap.add_argument("--label", default="", help="title burnt into the video")
    ap.add_argument("--cam-preset", help="camera preset of recorder.CAMERAS (default: the vehicle kind's)")
    ap.add_argument("--marker-dir", help="directory where the executive touches 'started' / 'finished' (video clock and trimming)")
    ap.add_argument("--trail-bgr", default="255,130,70", help="colour of the flown trail, B,G,R")
    ap.add_argument("--legacy-yaw", action="store_true", help="library yaw convention (mt_z negated, gyro r un-negated)")
    ap.add_argument("--fix-position", action="store_true",
                    help="rewrite the JSON 'position' to NED metres: north/east relative to the spawn, down = -z relative to the WATER SURFACE "
                         "(SITL's water barometer clamps above home altitude 0, so the surface must be the SITL zero, see rov_sitl.sh)")
    a = ap.parse_args()

    sc = load_scenario(a.scenario)
    profile = VEHICLE_REGISTRY["BlueROV2"]
    flip = a.flip_yaw if a.flip_yaw is not None else (1 if a.legacy_yaw else 0)
    profile = replace(profile, flip_gyro_yaw=bool(flip))
    scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Bridge", agent_name="bluerov0",
                                                 location=SPAWN, rotation=[0.0, 0.0, 0.0], ticks_per_sec=a.ticks)
    rec = None
    if a.record:
        rec = Recorder(a.record, a.label, a.ticks, "water", tuple(int(v) for v in a.trail_bgr.split(",")), marker_dir=a.marker_dir, cam=CAMERAS[a.cam_preset] if a.cam_preset else None)
        rec.add_sensor(scenario)
    with ArduBiguaSimRunner(profile, scenario, show_viewport=a.viewport, verbose=False) as runner:
        bridge, env, agent, dt = runner._bridge, runner._env, runner._agent_name, runner._dt
        gps, last_connect, last_send = None, 0.0, 0.0
        bridge.bind()
        motor_cmds = [0.0] * profile.num_motors
        env.step(motor_cmds)
        spawn_obstacles(env, sc)
        env.step(motor_cmds)
        print(f"STAGE_D_RUNNER_READY obstacles={len(sc.known) + len(sc.hidden)}", flush=True)

        sim_t, n = 0.0, 0
        with open(a.log, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["wall", "sim_t", "x", "y", "z", "vx", "vy", "vz", "m1", "m2", "m3", "m4", "m5", "m6", "m7", "m8", "roll", "pitch", "yaw", "gyro_z"])
            try:
                while True:
                    frame, pwm = bridge.receive_pwm()
                    if frame is None:
                        continue
                    motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
                    state = env.step(motor_cmds)[agent][0]
                    sim_t += dt
                    if rec is not None:
                        rec.frame(env, state, sim_t)
                    js = bridge.build_json_state(state, sim_t)
                    gps_pos = list(js["position"]) if js is not None else None   # stock [lat, lon, alt(up)]: what GPS_INPUT needs
                    if js is not None and a.fix_position:
                        loc = np.asarray(state["LocationSensor"], dtype=float)[:3]
                        js["position"] = [loc[0] - SPAWN[0], -(loc[1] - SPAWN[1]), -loc[2]]
                    bridge.send_state(js)

                    now = time.time()
                    if gps is None and now - last_connect >= 2.0:
                        last_connect = now
                        try:
                            gps = mavutil.mavlink_connection(GPS_CONNECTION)
                            gps.wait_heartbeat(timeout=2)
                            print(f"[gps_ground_truth] connected to {GPS_CONNECTION}", flush=True)
                        except (ConnectionRefusedError, OSError):
                            gps = None
                    if gps is not None:
                        while gps.recv_match(blocking=False) is not None:   # never let the TCP buffer fill (stalls ArduSub)
                            pass
                        if js is not None and now - last_send >= 1.0 / GPS_RATE_HZ:
                            last_send = now
                            lat, lon, alt = gps_pos
                            vn, ve, vd = js["velocity"]
                            gps.mav.gps_input_send(0, 0, 0, 0, 0, 3, int(lat * 1e7), int(lon * 1e7), alt, 1.0, 1.0,
                                                   vn, ve, vd, 0.5, 1.0, 1.0, 10)
                    n += 1
                    if n % LOG_EVERY == 0:
                        loc = np.asarray(state["LocationSensor"], dtype=float)[:3]
                        vel = np.asarray(state["VelocitySensor"], dtype=float)[:3]
                        mc = list(motor_cmds) + [0.0] * (8 - len(motor_cmds))
                        w.writerow([f"{time.time():.3f}", f"{sim_t:.3f}", *(f"{v:.3f}" for v in loc), *(f"{v:.3f}" for v in vel),
                                    *(f"{v:.2f}" for v in mc[:8]), *(f"{v:.1f}" for v in _euler_deg(js["quaternion"])),
                                    f"{js['imu']['gyro'][2]:.3f}"])
                        if n % (LOG_EVERY * 40) == 0:
                            fh.flush()
            except KeyboardInterrupt:
                print("runner stopped", flush=True)
            finally:
                fh.flush()
                bridge.close()
                if rec is not None:
                    rec.close()


if __name__ == "__main__":
    main()
