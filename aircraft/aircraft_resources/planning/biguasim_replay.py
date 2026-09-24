"""Replay a T8.2 mission inside BiguaSim (Stage A: simple path follower, no autopilot).

    # replay a trace saved by the benchmark campaign
    python3 biguasim_replay.py --trace results/campaign/traces/C6_K1_None_rrt_star_None_0.json --viewport

    # or plan + fly a scenario right now with the harness, then replay it
    python3 biguasim_replay.py --scenario C6 --planner astar --condition K1 --viewport

What you see: the scenario's obstacles spawned as static props (known = steel,
hidden/unmapped = brick), the planned path drawn as a coloured polyline with a marker
at every waypoint, and the vehicle moved along the executed trajectory (kinematic
teleport each tick, like the fly-by mode of biguasim_sim_runner.py). The WhiteBoat of
C2 cannot be a moving prop, so its footprint is redrawn as a wireframe every step.

Frame: planner (north, east, up), z = 0 at the water surface, maps to BiguaSim as
    [x, y] = home + R(yaw) . [north, -east],   z = up
With --yaw-deg 90 (default) planner north runs along BiguaSim +y, i.e. along the
LONG axis of the Bridge channel (about 470 m), so long routes stay over the water.
--home is where planner (0, 0) sits in BiguaSim x/y. Check it in the viewport: the
bridge structure is not modelled by the planners, keep the route clear of it.
Only yaw multiples of 90 deg are supported (box props are axis-aligned).

This script needs the BiguaSim engine (GPU). It cannot run headless; use --viewport.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from scenario import load_scenario  # noqa: E402

SCENARIO_FILES = {"C1": "c1_long_air", "C2": "c2_moving_boat", "C3": "c3_buoy_close",
                  "C4": "c4_transition", "C5": "c5_underwater", "C6": "c6_full_mission"}
PATH_COLORS = {"AR": [0, 200, 255], "TRANSICAO": [255, 200, 0], "AGUA": [0, 80, 255]}


class Frame:
    def __init__(self, home_xy, yaw_deg: float):
        if abs(yaw_deg) % 90 > 1e-9:
            raise SystemExit("--yaw-deg must be a multiple of 90")
        self.home = np.asarray(home_xy, dtype=float)
        th = math.radians(yaw_deg)
        self.R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        self.swap = int(round(yaw_deg / 90)) % 2 == 1   # box axes swap for odd multiples of 90 deg
        self.yaw = yaw_deg

    def point(self, p) -> list[float]:
        xy = self.home + self.R @ np.array([p[0], -p[1]])
        return [float(xy[0]), float(xy[1]), float(p[2])]

    def heading_deg(self, a, b) -> float:
        d = np.array(self.point(b)[:2]) - np.array(self.point(a)[:2])
        return math.degrees(math.atan2(d[1], d[0])) if np.linalg.norm(d) > 1e-6 else self.yaw

    def box_size(self, n_size, e_size, u_size):
        return [e_size, n_size, u_size] if self.swap else [n_size, e_size, u_size]


def medium_of(z: float, mu: float = 0.5) -> str:
    return "AR" if z > mu else ("AGUA" if z < -mu else "TRANSICAO")


def spawn_obstacles(env, sc, fr: Frame) -> None:
    def cyl(o, material):
        c = fr.point([o.north, o.east, (o.z_min + o.z_max) / 2.0])
        # BiguaSim props are 1 m primitives scaled per axis: a cylinder's x/y scale is its diameter.
        env.spawn_prop("cylinder", location=c, rotation=[0, 0, 0], scale=[2 * o.radius, 2 * o.radius, o.z_max - o.z_min],
                       sim_physics=False, material=material, tag="t82_obstacle")

    def box(o, material):
        c = fr.point([(o.n_min + o.n_max) / 2, (o.e_min + o.e_max) / 2, (o.u_min + o.u_max) / 2])
        env.spawn_prop("box", location=c, rotation=[0, 0, 0],
                       scale=fr.box_size(o.n_max - o.n_min, o.e_max - o.e_min, o.u_max - o.u_min),
                       sim_physics=False, material=material, tag="t82_obstacle")

    for group, material in ((sc.known, "steel"), (sc.hidden, "brick")):
        for o in group:
            (cyl if hasattr(o, "radius") else box)(o, material)


def draw_paths(env, planned_paths, fr: Frame) -> None:
    for path in planned_paths:
        pts = [fr.point(p) for p in path]
        for a, b, pa, pb in zip(pts[:-1], pts[1:], path[:-1], path[1:]):
            mid_z = 0.5 * (pa[2] + pb[2])
            env.draw_line(a, b, color=PATH_COLORS[medium_of(mid_z)], thickness=6.0, lifetime=0)
        for p in pts:
            env.draw_point(p, color=[255, 40, 40], thickness=18.0, lifetime=0)


def draw_box_wireframe(env, box, fr: Frame, lifetime: float, color=(255, 60, 60)) -> None:
    corners = [[n, e, u] for n in (box.n_min, box.n_max) for e in (box.e_min, box.e_max) for u in (box.u_min, box.u_max)]
    pts = [fr.point(c) for c in corners]
    edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    for i, j in edges:
        env.draw_line(pts[i], pts[j], color=list(color), thickness=4.0, lifetime=lifetime)


def load_run(a):
    if a.trace:
        d = json.loads(Path(a.trace).read_text())
        name = d["row"]["scenario"]
        return load_scenario(HERE / "scenarios" / f"{SCENARIO_FILES[name]}.yaml"), np.array(d["trace"]), [np.array(p) for p in d["planned_paths"]], d["row"]
    from config import Config
    from execution import run_mission

    sc = load_scenario(HERE / "scenarios" / f"{SCENARIO_FILES[a.scenario]}.yaml")
    r = run_mission(sc, Config.load(), a.planner, a.condition, a.seed, resolution=a.resolution)
    if not r.trace:
        raise SystemExit(f"nothing to replay: mission ended with {r.status} ({r.detail})")
    print(f"mission: {r.status}, planned {r.energy_planned_wh:.3f} Wh / executed {r.energy_exec_wh:.3f} Wh, {r.n_replans} replans")
    return sc, np.array(r.trace), r.planned_paths, {"scenario": a.scenario, "planner": a.planner, "condition": a.condition}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--trace", help="trace JSON from the benchmark campaign (results/<run>/traces/)")
    src.add_argument("--scenario", choices=list(SCENARIO_FILES))
    ap.add_argument("--planner", choices=["astar", "rrt_star"], default="astar")
    ap.add_argument("--condition", default="K1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resolution", type=float, default=2.0, help="A* resolution when planning with --scenario")
    ap.add_argument("--home", nargs=2, type=float, default=[0.0, -215.0], metavar=("X", "Y"),
                    help="BiguaSim x y of planner (0, 0)")
    ap.add_argument("--yaw-deg", type=float, default=90.0)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor (1 = real time)")
    ap.add_argument("--ticks", type=int, default=50)
    ap.add_argument("--viewport", action="store_true")
    ap.add_argument("--hold-s", type=float, default=15.0, help="seconds to keep the final frame on screen")
    a = ap.parse_args()

    sc, trace, planned, meta = load_run(a)
    fr = Frame(a.home, a.yaw_deg)

    import biguasim
    from dataclasses import replace
    from biguasim.ardubridge import ArduBiguaSimRunner
    from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

    profile = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)
    start = fr.point(trace[0][1:])
    scenario = ArduBiguaSimRunner.build_scenario(profile, package_name="SkyDive", world="Bridge", agent_name="hydrone0",
                                                 location=start, rotation=[0.0, 0.0, a.yaw_deg], ticks_per_sec=a.ticks)
    print(f"replaying {meta} in BiguaSim; planner (0,0) at bsim {a.home}, yaw {a.yaw_deg}; start at {np.round(start, 1).tolist()}")
    dt_sim = 1.0 / a.ticks
    zeros = [0.0] * profile.num_motors

    with biguasim.make(scenario_cfg=scenario, show_viewport=a.viewport) as env:
        env.step(zeros)
        spawn_obstacles(env, sc, fr)
        draw_paths(env, planned, fr)
        env.step(zeros)

        wall0 = time.perf_counter()
        max_err, n_checked, prev_want = 0.0, 0, np.array(start)
        for k, (t, n, e, u) in enumerate(trace):
            nxt = trace[min(k + 1, len(trace) - 1)]
            yaw = fr.heading_deg([n, e, u], nxt[1:]) if k + 1 < len(trace) else a.yaw_deg
            env._agent.teleport(location=np.array(fr.point([n, e, u]), dtype=np.float32), rotation=np.array([0.0, 0.0, yaw], dtype=np.float32))
            for m in sc.moving:
                if k % 5 == 0:
                    draw_box_wireframe(env, m.box_at(t), fr, lifetime=0.6)
            raw = env.step(zeros)
            # objective check: where does the engine say the agent is? (teleport applies on the next tick,
            # so accept the closest of the current and previous expected positions)
            loc = raw["hydrone0"][0].get("LocationSensor") if isinstance(raw.get("hydrone0"), (list, tuple)) else None
            if loc is not None:
                want = np.array(fr.point([n, e, u]))
                errs = [float(np.linalg.norm(np.asarray(loc)[:3] - w)) for w in (want, prev_want)]
                max_err = max(max_err, min(errs))
                n_checked += 1
            prev_want = np.array(fr.point([n, e, u]))
            # pace the replay against wall-clock (trace samples are every 0.4 s of mission time)
            due = (t - trace[0][0]) / a.speed
            slack = due - (time.perf_counter() - wall0)
            if slack > 0:
                time.sleep(slack)
        end = time.perf_counter()
        while time.perf_counter() - end < a.hold_s:
            env.step(zeros)
            time.sleep(dt_sim)
    print(f"replay finished; engine position vs expected trajectory: max error {max_err:.2f} m over {n_checked} steps"
          if n_checked else "replay finished (no LocationSensor reading available to verify positions)")


if __name__ == "__main__":
    main()
