"""BiguaSim side of the T8.2 Stage B test.

Runs the Hydrone (DjiMatrice profile) in the SkyDive/Bridge world, bridges it to ArduPilot SITL (JSON
interface, UDP 9002), spawns the scenario's obstacles as static props, and logs the ground truth
(position, velocity and the total rotor thrust BiguaSim actually applies) to a CSV.

    python3 sitl_runner.py --scenario stage_b/scenarios/b1_poles.yaml --log /tmp/truth.csv [--viewport]

Start ArduPilot SITL first (bash aircraft_resources/missions/t2_sitl_run.sh), then this, then sitl_exec.py.
Prints STAGE_B_RUNNER_READY once the world is up and the props are spawned.

State fix: the "position" field of the JSON state is rewritten to NED metres relative to home (see run()); the stock
biguasim bridge sends [lat, lon, alt] there, which ArduPilot's JSON backend does not read that way.

Thrust: BiguaSim computes rotor thrust as k_eta * omega^2 per rotor (DjiMatrice k_eta = 6.64e-5); the motor
speeds it receives from ArduPilot's PWM are logged, so the total thrust is exact for the plant. The power
curves that turn thrust into watts are still the planner's placeholder ones (BiguaSim has no power model).
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common_b import HOME_BSIM, from_bsim, to_bsim  # noqa: E402

from recorder import Recorder  # noqa: E402
from scenario import load_scenario  # noqa: E402

from biguasim.ardubridge import ArduBiguaSimRunner  # noqa: E402
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY  # noqa: E402
from biguasim.dynamics.agents import DjiMatrice  # noqa: E402

LOG_EVERY = 5   # steps



class StageBRunner(ArduBiguaSimRunner):
    def __init__(self, profile, scenario, sc, log_path, recorder=None, **kwargs):
        super().__init__(profile, scenario, **kwargs)
        self._rec = recorder
        self._sc = sc
        self._log_path = log_path
        self._k_eta = float(DjiMatrice._params["k_eta"])

    def _spawn_obstacles(self) -> None:
        env = self._env
        for group, material in ((self._sc.known, "steel"), (self._sc.hidden, "brick")):
            for o in group:
                if hasattr(o, "radius"):
                    c = to_bsim(o.north, o.east, (o.z_min + o.z_max) / 2.0)
                    env.spawn_prop("cylinder", location=c, rotation=[0, 0, 0],
                                   scale=[2 * o.radius, 2 * o.radius, o.z_max - o.z_min],
                                   sim_physics=False, material=material, tag="t82_obstacle")
                else:
                    c = to_bsim((o.n_min + o.n_max) / 2, (o.e_min + o.e_max) / 2, (o.u_min + o.u_max) / 2)
                    env.spawn_prop("box", location=c, rotation=[0, 0, 0],
                                   scale=[o.n_max - o.n_min, o.e_max - o.e_min, o.u_max - o.u_min],
                                   sim_physics=False, material=material, tag="t82_obstacle")

    def run(self) -> None:
        bridge, env, agent, dt = self._bridge, self._env, self._agent_name, self._dt
        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        env.step(motor_cmds)
        self._spawn_obstacles()
        env.step(motor_cmds)
        print(f"STAGE_B_RUNNER_READY home_bsim={HOME_BSIM} obstacles={len(self._sc.known) + len(self._sc.hidden)}", flush=True)

        sim_t, n = 0.0, 0
        with open(self._log_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["wall", "sim_t", "x", "y", "z", "vx", "vy", "vz", "thrust_n", "w1", "w2", "w3", "w4"])
            try:
                while True:
                    frame, pwm = bridge.receive_pwm()
                    if frame is None:
                        continue
                    motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)
                    raw = env.step(motor_cmds)
                    state = raw[agent][0]
                    sim_t += dt
                    if self._rec is not None:
                        self._rec.frame(env, state, sim_t)
                    js = bridge.build_json_state(state, sim_t)
                    if js is not None:
                        # biguasim's bridge puts [lat, lon, alt(up)] in "position", but ArduPilot's JSON backend reads
                        # "position" as NED metres relative to home (lat/lon have their own keys). Read that way the
                        # horizontal position never changes and altitude has the wrong sign: the EKF "descends" while
                        # the drone climbs, and the controller pushes harder (runaway climb). Send what ArduPilot expects.
                        north, east, up = from_bsim(*np.asarray(state["LocationSensor"], dtype=float)[:3])
                        js["position"] = [north, east, -up]
                        js.pop("pressure", None)   # not a key of ArduPilot's JSON backend
                    bridge.send_state(js)
                    n += 1
                    if n % LOG_EVERY == 0:
                        loc = np.asarray(state["LocationSensor"], dtype=float)[:3]
                        vel = np.asarray(state["VelocitySensor"], dtype=float)[:3]
                        om = np.asarray(motor_cmds, dtype=float)
                        thrust = self._k_eta * float(np.sum(om ** 2))
                        w.writerow([f"{time.time():.3f}", f"{sim_t:.3f}", *(f"{v:.3f}" for v in loc),
                                    *(f"{v:.3f}" for v in vel), f"{thrust:.3f}", *(f"{v:.1f}" for v in om)])
                        if n % (LOG_EVERY * 40) == 0:
                            fh.flush()
            except KeyboardInterrupt:
                print("runner stopped", flush=True)
            finally:
                fh.flush()
                bridge.close()
                if self._rec is not None:
                    self._rec.close()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--port", type=int, default=9002)
    ap.add_argument("--ticks", type=int, default=250, help="BiguaSim tick rate; ArduPilot needs gyro rate >= 1.8 x SCHED_LOOP_RATE (=216 Hz at 120)")
    ap.add_argument("--viewport", action="store_true")
    ap.add_argument("--record", help="write an mp4 of the flight (chase camera, see recorder.py)")
    ap.add_argument("--label", default="", help="title burnt into the video")
    ap.add_argument("--marker-dir", help="directory where the executive touches 'started' / 'finished' (video clock and trimming)")
    ap.add_argument("--trail-bgr", default="255,130,70", help="colour of the flown trail, B,G,R")
    a = ap.parse_args()

    sc = load_scenario(a.scenario)
    # same profile as the T2 runner: the DepthSensor feeds the pressure field of the JSON state (SITL barometer)
    profile = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)
    scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Bridge", agent_name="hydrone0",
                                                 location=list(HOME_BSIM), rotation=[0.0, 0.0, 0.0], ticks_per_sec=a.ticks)
    rec = None
    if a.record:
        rec = Recorder(a.record, a.label, a.ticks, "air", tuple(int(v) for v in a.trail_bgr.split(",")), marker_dir=a.marker_dir)
        rec.add_sensor(scenario)
    with StageBRunner(profile, scenario, sc, a.log, recorder=rec, port=a.port, show_viewport=a.viewport) as runner:
        runner.run()


if __name__ == "__main__":
    main()
