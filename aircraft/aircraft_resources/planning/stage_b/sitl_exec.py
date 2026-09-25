"""T8.2 Stage B executive: plan with A* or RRT*, fly the plan in ArduPilot SITL (GUIDED) over MAVLink.

The flight logic is PlannedFlight (planning/flight_executive.py), the same one the mission_node runs; this file only
supplies the vehicle side over pymavlink.

    python3 sitl_exec.py --scenario stage_b/scenarios/b1_poles.yaml --planner rrt_star --condition K2 \
        --seed 0 --out /tmp/run.json

It uses the same `Mission` machinery as the Stage A harness (map, energy model, planners, replanners,
simulated perception) but the vehicle is the real ArduCopter in SITL: positions come from the EKF
(LOCAL_POSITION_NED, origin = home) and waypoints go out as SET_POSITION_TARGET_LOCAL_NED. Local altitudes
are relative to home, which is what GUIDED uses.

How GUIDED is driven (this is what Stage B measures):
  * one position target per waypoint; the next one is sent when the vehicle gets within the waypoint's
    fly-by radius (radius capped by the waypoint clearance), so ArduPilot blends instead of stopping;
  * at a sharp turn or a tight spot the vehicle is made to STOP at the waypoint first (stop-and-go), because
    GUIDED position targets cannot carry an arrival speed;
  * when the remaining route is cut by a newly sensed obstacle the vehicle is told to hold position while the
    replan runs, then the new route is sent.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common_b import sitl_config  # noqa: E402

from flight_executive import PlannedFlight  # noqa: E402
from scenario import load_scenario  # noqa: E402

POS_MASK = 0b110111111000   # position only: ignore velocity, acceleration, force, yaw, yaw rate
LOOP_S = 0.1
STOP_SPEED = 0.25           # m/s: "stopped" for stop-and-go waypoints
MAX_MISSION_S = 420.0


class Sitl:
    """Minimal MAVLink client for ArduCopter SITL."""

    def __init__(self, url: str):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.pos = None          # (boot_s, north, east, up, vn, ve, vu, wall)
        self.armed = False
        self.mode = ""
        self.texts: list[str] = []

    def pump(self) -> None:
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "LOCAL_POSITION_NED":
                self.pos = (msg.time_boot_ms / 1000.0, msg.x, msg.y, -msg.z, msg.vx, msg.vy, -msg.vz, time.time())
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = mavutil.mode_string_v10(msg)
            elif t == "STATUSTEXT":
                self.texts.append(str(msg.text))
                print(f"  [ardupilot] {msg.text}", flush=True)

    def wait_for(self, cond, timeout: float, what: str) -> None:
        t0 = time.time()
        while not cond():
            self.pump()
            if time.time() - t0 > timeout:
                raise TimeoutError(f"timeout waiting for {what}")
            time.sleep(0.05)

    def goto(self, north: float, east: float, up: float) -> None:
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, POS_MASK,
            float(north), float(east), float(-up), 0, 0, 0, 0, 0, 0, 0, 0)

    def command(self, cmd, *params) -> None:
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)


def bring_up(s: Sitl, altitude: float) -> None:
    print("waiting for heartbeat...", flush=True)
    s.m.wait_heartbeat(timeout=300)
    s.m.mav.request_data_stream_send(s.m.target_system, s.m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    s.wait_for(lambda: s.pos is not None, 120, "LOCAL_POSITION_NED")
    print("guided + arm...", flush=True)
    t0 = time.time()
    while not s.armed:
        s.pump()
        if time.time() - t0 > 240:
            raise TimeoutError("could not arm: " + " | ".join(s.texts[-4:]))
        s.m.set_mode("GUIDED")
        s.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        for _ in range(30):
            s.pump()
            if s.armed:
                break
            time.sleep(0.1)
    print(f"armed, mode {s.mode}; takeoff to {altitude:.1f} m", flush=True)
    s.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
    s.wait_for(lambda: s.pos is not None and s.pos[3] >= altitude - 0.4, 90, "takeoff altitude")
    t0 = time.time()
    while time.time() - t0 < 4.0:            # let the climb settle
        s.pump()
        time.sleep(0.1)


FRAME_OFFSET = np.zeros(3)      # scenario frame = home-relative frame + FRAME_OFFSET (Stage E: the scenario is in the water-surface frame)


def local_pos(s: Sitl) -> np.ndarray:
    return np.array([s.pos[1], s.pos[2], s.pos[3]]) + FRAME_OFFSET


def local_vel(s: Sitl) -> np.ndarray:
    return np.array([s.pos[4], s.pos[5], s.pos[6]])


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--planner", choices=["astar", "rrt_star"], required=True)
    ap.add_argument("--condition", choices=["K1", "K2", "K3", "K5", "K6"], default="K1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resolution", type=float, default=1.0)
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame-offset", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("N", "E", "U"),
                    help="scenario frame minus the vehicle's home-relative frame (Stage E: the home in the surface frame)")
    ap.add_argument("--frame-origin-bsim", nargs=3, type=float, help="BiguaSim position of the scenario frame origin (recorded for the analysis)")
    ap.add_argument("--no-land", action="store_true", help="stay in GUIDED at the end (Stage E: the route ends above the water)")
    a = ap.parse_args()

    global FRAME_OFFSET
    FRAME_OFFSET = np.array(a.frame_offset, dtype=float)
    cfg = sitl_config()
    sc = load_scenario(a.scenario)
    change = None
    if a.condition == "K5" and sc.mission_change:
        change = sc.mission_change
    flight = PlannedFlight(sc, cfg, a.planner, a.condition, a.seed, resolution=a.resolution, mission_change=change)

    if a.frame_origin_bsim:
        flight.rec["home_bsim"] = list(a.frame_origin_bsim)

    def save() -> None:
        Path(a.out).write_text(json.dumps(flight.result()))

    try:
        ok, status, detail = flight.plan()
        print(f"plan {a.planner}: {status} {detail}", flush=True)
        save()
        if not ok:
            return

        s = Sitl(a.url)
        bring_up(s, flight.takeoff_altitude - FRAME_OFFSET[2])
        print(f"hovering at {np.round(local_pos(s), 2).tolist()}, starting the route", flush=True)
        boot0 = s.pos[0]

        def send(cmd) -> None:
            if cmd is not None:
                s.goto(*((cmd.target if cmd.kind == "goto" else local_pos(s)) - FRAME_OFFSET))
                print(f"  {cmd.kind} ({cmd.reason}) -> {np.round(cmd.target, 1).tolist()}", flush=True)

        (Path(a.out).parent / "started").touch()          # the video recorder's clock starts here
        send(flight.begin(local_pos(s), time.time(), 0.0))
        t_cmd0 = time.time()
        while not flight.done and time.time() - t_cmd0 < MAX_MISSION_S:
            s.pump()
            if s.pos is not None:
                send(flight.step(local_pos(s), local_vel(s), s.pos[0] - boot0, time.time()))
            time.sleep(LOOP_S)
        flight.abort("mission time limit", time.time(), local_pos(s))
        (Path(a.out).parent / "finished").touch()
        if flight.status == "SUCESSO":
            for _ in range(30):          # hover a moment so the log ends at rest
                s.pump()
                time.sleep(0.1)
            if not a.no_land:
                s.m.set_mode("LAND")
    except Exception as e:  # noqa: BLE001  (the orchestrator needs the reason, whatever it is)
        flight.abort(f"{type(e).__name__}: {e}", time.time())
        print("ERROR:", f"{type(e).__name__}: {e}", flush=True)
    finally:
        save()


if __name__ == "__main__":
    main()
