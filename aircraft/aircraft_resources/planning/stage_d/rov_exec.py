"""T8.2 Stage D executive: plan with A* or RRT*, fly the plan with the BlueROV2 in ArduSub SITL (GUIDED) over MAVLink.

Same flight logic as Stage B (PlannedFlight); the vehicle side differs: ArduSub arms in GUIDED and takes NED position
targets relative to the EKF origin, and the scenario frame is the water-surface one (z = 0 at the surface). The
offset between the two (where the EKF's zero depth is) is measured at start from the known spawn depth.

    python3 rov_exec.py --scenario stage_d/scenarios/d1_poles.yaml --planner astar --condition K2 --out /tmp/run.json
    python3 rov_exec.py --probe --out /tmp/probe.json        # arm, dive, hold, cross: no planner (validates the vehicle side)
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

PLANNING = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PLANNING))
sys.path.insert(0, str(PLANNING / "stage_b"))
from sitl_exec import LOOP_S, Sitl  # noqa: E402

from config import Config  # noqa: E402
from flight_executive import PlannedFlight  # noqa: E402
from scenario import load_scenario  # noqa: E402

SPAWN_UP = -2.0            # BiguaSim spawn depth of the ROV (rov_runner.SPAWN)
ROV_CONFIG = PLANNING / "config" / "planner_rov_sitl.yaml"
ROV_PARAMS = {"ATC_RAT_RLL_I": 0.01}   # t2_biguasim_bluerov2.parm has 0.05: with it the roll oscillation grows exponentially from ~100 s
                                       # (peak 0.1 deg at 100 s -> 56 deg at 210 s, 0.16 Hz) and the ROV capsizes; 0.01 held 0.0-0.1 deg for 220 s
MAX_MISSION_S = 900.0      # the ROV cruises at 0.2 m/s


class Rov(Sitl):
    """ArduSub client: same telemetry as the Copter one, plus the offset between the EKF's z and the water surface."""

    def __init__(self, url: str):
        # ArduSub's TCP port only opens some seconds after sim_vehicle.py starts; and SITL blocks in accept() until this
        # client is connected (see biguasim_bridge/README.md), so this executive has to be up BEFORE the BiguaSim runner
        t0 = time.time()
        while True:
            try:
                super().__init__(url)
                break
            except (ConnectionRefusedError, OSError):
                if time.time() - t0 > 120:
                    raise
                time.sleep(1.0)
        self.up_offset = 0.0         # surface_up = ekf_up + up_offset
        self.spawn_ne = (0.0, 0.0)   # surface-frame north/east of the EKF origin (the spawn); 0, 0 for the tests at the frame origin
        self.spawn_up = SPAWN_UP
        self.extra: dict = {}
        self.params: dict = {}

    def pump(self) -> None:
        """Sitl.pump plus the depth-related telemetry (to find where a depth error comes from)."""
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
            elif t == "SCALED_PRESSURE":
                self.extra["press_abs"] = msg.press_abs
            elif t == "ATTITUDE":
                self.extra["rpy"] = [round(float(np.degrees(v)), 1) for v in (msg.roll, msg.pitch, msg.yaw)]
            elif t == "ATTITUDE_TARGET":
                self.extra["tgt_yaw_rate"] = round(float(np.degrees(msg.body_yaw_rate)), 1)
            elif t == "SERVO_OUTPUT_RAW":
                self.extra["servo"] = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw]
            elif t == "GPS_RAW_INT":
                self.extra["gps_alt"] = msg.alt / 1000.0
            elif t == "EKF_STATUS_REPORT":
                self.extra["vvar"], self.extra["hvar"], self.extra["flags"] = msg.pos_vert_variance, msg.pos_horiz_variance, msg.flags
            elif t == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif t == "VFR_HUD":
                self.extra["hud_alt"] = msg.alt
            elif t == "GLOBAL_POSITION_INT":
                self.extra["gpi_alt"], self.extra["gpi_rel"] = msg.alt / 1000.0, msg.relative_alt / 1000.0

    def surface_pos(self) -> np.ndarray:
        return np.array([self.pos[1] + self.spawn_ne[0], self.pos[2] + self.spawn_ne[1], self.pos[3] + self.up_offset])

    def surface_vel(self) -> np.ndarray:
        return np.array([self.pos[4], self.pos[5], self.pos[6]])

    def goto_surface(self, north: float, east: float, up: float) -> None:
        self.goto(north - self.spawn_ne[0], east - self.spawn_ne[1], up - self.up_offset)


def bring_up(s: Rov) -> None:
    print("waiting for heartbeat...", flush=True)
    s.m.wait_heartbeat(timeout=300)
    s.m.mav.request_data_stream_send(s.m.target_system, s.m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    s.wait_for(lambda: s.pos is not None, 180, "LOCAL_POSITION_NED")
    t0 = time.time()
    while time.time() - t0 < 3:                        # let a few samples in, then take the offset from the known spawn depth
        s.pump()
        time.sleep(0.1)
    s.up_offset = s.spawn_up - s.pos[3]
    print(f"EKF up at spawn = {s.pos[3]:.2f} m -> offset to the surface frame {s.up_offset:+.2f} m", flush=True)
    print("guided + arm...", flush=True)
    t0 = time.time()
    while not s.armed:
        s.pump()
        if time.time() - t0 > 240:
            raise TimeoutError("could not arm: " + " | ".join(s.texts[-4:]))
        s.m.set_mode(os.environ.get("PROBE_MODE", "GUIDED"))
        s.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        for _ in range(30):
            s.pump()
            if s.armed:
                break
            time.sleep(0.1)
    print(f"armed, mode {s.mode}", flush=True)
    for k, v in ROV_PARAMS.items():                      # see ROV_PARAMS
        s.m.mav.param_set_send(s.m.target_system, s.m.target_component, k.encode(), float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
    t0 = time.time()
    while time.time() - t0 < 3:
        s.pump()
        time.sleep(0.1)


def dump_params(s: Rov, names) -> None:
    for n in names:
        s.m.mav.param_request_read_send(s.m.target_system, s.m.target_component, n.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < 4:
        s.pump()
        time.sleep(0.1)
    print("PARAMS", {n: s.params.get(n) for n in names}, flush=True)


def probe(s: Rov, out: Path) -> None:
    """Hold, dive to -4 m, 10 m north, hold; logs the EKF position to compare with the runner's truth CSV."""
    legs = [("hold at spawn", (0.0, 0.0, SPAWN_UP), 15), ("dive", (0.0, 0.0, -4.0), 60), ("north 10 m", (10.0, 0.0, -4.0), 100),
            ("hold", (10.0, 0.0, -4.0), 20)]
    rows = []
    for kv in filter(None, os.environ.get("PROBE_PARAMS", "").split(",")):
        k, v = kv.split("=")
        s.m.mav.param_set_send(s.m.target_system, s.m.target_component, k.encode(), float(v), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.3)
    dump_params(s, ["ATC_ANG_RLL_P", "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_RLL_FLTD", "ATC_RAT_RLL_FLTT", "ATC_ANG_PIT_P",
                    "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_ANG_YAW_P", "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D", "PSC_NE_VEL_P", "PSC_NE_VEL_I", "PSC_NE_VEL_D", "PSC_NE_POS_P", "WP_SPD", "WP_ACC", "WP_RADIUS"])
    dump_params(s, ["EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW", "EK3_ENABLE", "EK2_ENABLE",
                    "AHRS_EKF_TYPE", "EK3_HGT_I_GATE", "EK3_ALT_M_NSE", "BARO1_GND_TEMP", "BARO_ALT_OFFSET", "GPS_TYPE", "GPS_AUTO_CONFIG"])
    for name, tgt, dur in legs:
        print(f"probe: {name} -> {tgt}", flush=True)
        t0 = time.time()
        while time.time() - t0 < dur:
            s.pump()
            s.goto_surface(*tgt)
            if s.pos is not None:
                rows.append([time.time(), s.pos[0], *s.surface_pos().tolist(), *s.surface_vel().tolist(), s.extra.get('rpy', [0, 0, 0])[2], name])
                if len(rows) % 20 == 0:
                    print(f"  {name}: pos {np.round(s.surface_pos(), 2).tolist()} vel {np.round(s.surface_vel(), 2).tolist()} "
                          f"{ {k: (v if isinstance(v, list) else round(v, 2)) for k, v in s.extra.items() if k in ('rpy', 'tgt_yaw_rate', 'servo', 'flags')} }", flush=True)
            time.sleep(0.25)
    out.write_text(json.dumps(rows))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario")
    ap.add_argument("--planner", choices=["astar", "rrt_star"])
    ap.add_argument("--condition", choices=["K1", "K2", "K3", "K5", "K6"], default="K1")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--resolution", type=float, default=1.0)
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--out", required=True)
    ap.add_argument("--probe", action="store_true")
    ap.add_argument("--spawn-n", type=float, default=0.0, help="surface-frame north of the spawn (Stage E hand-off point)")
    ap.add_argument("--spawn-e", type=float, default=0.0, help="surface-frame east of the spawn")
    ap.add_argument("--spawn-up", type=float, default=SPAWN_UP, help="surface-frame z of the spawn (depth, negative)")
    a = ap.parse_args()

    if a.probe:
        s = Rov(a.url)
        bring_up(s)
        probe(s, Path(a.out))
        return

    cfg = Config.load(ROV_CONFIG)
    sc = load_scenario(a.scenario)
    change = sc.mission_change if a.condition == "K5" and sc.mission_change else None
    flight = PlannedFlight(sc, cfg, a.planner, a.condition, a.seed, resolution=a.resolution, mission_change=change,
                           max_flight_s=MAX_MISSION_S)

    flight.rec.update(medium="water", home_bsim=[25.0, 0.0, 0.0], vehicle="BlueROV2")

    def save() -> None:
        Path(a.out).write_text(json.dumps(flight.result()))

    try:
        ok, status, detail = flight.plan()
        print(f"plan {a.planner}: {status} {detail}", flush=True)
        save()
        if not ok:
            return
        s = Rov(a.url)
        s.spawn_ne, s.spawn_up = (a.spawn_n, a.spawn_e), a.spawn_up
        bring_up(s)
        pos0 = s.surface_pos()
        print(f"at {np.round(pos0, 2).tolist()} (surface frame), starting the route", flush=True)
        boot0 = s.pos[0]

        def send(cmd) -> None:
            if cmd is not None:
                s.goto_surface(*(cmd.target if cmd.kind == "goto" else s.surface_pos()))
                print(f"  {cmd.kind} ({cmd.reason}) -> {np.round(cmd.target, 1).tolist()}", flush=True)

        (Path(a.out).parent / "started").touch()          # the video recorder's clock starts here
        send(flight.begin(pos0, time.time(), 0.0))
        t_cmd0 = time.time()
        while not flight.done and time.time() - t_cmd0 < MAX_MISSION_S:
            s.pump()
            if s.pos is not None:
                send(flight.step(s.surface_pos(), s.surface_vel(), s.pos[0] - boot0, time.time()))
            time.sleep(LOOP_S)
        flight.abort("mission time limit", time.time(), s.surface_pos())
        (Path(a.out).parent / "finished").touch()
        if flight.status == "SUCESSO":
            for _ in range(30):
                s.pump()
                s.goto_surface(*flight.m.paths[-1][-1])
                time.sleep(0.1)
    except Exception as e:  # noqa: BLE001
        flight.abort(f"{type(e).__name__}: {e}", time.time())
        print("ERROR:", f"{type(e).__name__}: {e}", flush=True)
    finally:
        save()


if __name__ == "__main__":
    main()
