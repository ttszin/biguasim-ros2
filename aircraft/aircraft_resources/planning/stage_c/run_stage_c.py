"""Stage C: the planners flown THROUGH THE mission_node, in ArduPilot SITL + BiguaSim.

    python3 stage_c/run_stage_c.py --out results/stage_c --matrix full        # the campaign (hours)
    python3 stage_c/run_stage_c.py --out results/stage_c_smoke --matrix smoke # one flight, to check the setup
    python3 stage_c/run_stage_c.py --out results/stage_c --analyse-only

Per flight it starts, on a fresh stack:
  * ArduPilot SITL           (aircraft_resources/missions/t2_sitl_run.sh, same parameters as T2)
  * BiguaSim + obstacle props (stage_b/sitl_runner.py: also fixes the JSON position format, see its docstring)
  * the ROS 2 Humble side in the aircraft-image container: MAVROS, ardupilot_interface and the mission_node
    (stage_c/aircraft_stack.sh). The host's modified mission_node is bind-mounted over the image's copy (the image
    installs the package with --symlink-install), and aircraft_resources/ is mounted so the planning package is found.
The flight is a generated mission YAML: plan_route -> takeoff -> follow_route -> wait. For K5 the new goal is
published on /planner/new_goal DURING the flight (the path a decision from T11 would take), not passed as a parameter.

Metrics are the Stage B ones (BiguaSim ground truth, thrust-based energy); rows and figures reuse run_stage_b.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

STAGE_C = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE_C.parent / "stage_b"))
from run_stage_b import (FIELDS, MISSIONS_DIR, PLANNING, PY, SCENARIO_DIR, STAGE_B, analyse, kill_leftovers,  # noqa: E402
                         start, stop, wait_log)
import run_stage_b  # noqa: E402

from common_b import sitl_config  # noqa: E402
from energy import EnergyModel  # noqa: E402
from scenario import load_scenario  # noqa: E402

AIRCRAFT = PLANNING.parent.parent            # .../aircraft
MISSION_PKG = AIRCRAFT / "aircraft_ws" / "src" / "mission" / "mission"
RESOURCES = AIRCRAFT / "aircraft_resources"
CONTAINER = "aas_stage_c"
IN_RES = "/aas/aircraft_resources"

MATRIX = {
    "smoke": [("b1_poles", "astar", "K1", 0)],
    "smoke2": [("b3_field", "astar", "K5", 0), ("b3_field", "astar", "K6", 0)],
    "full": (
        # the two Stage B scenarios again, now through the mission_node (equivalence check)
        [(s, pl, k, sd) for s in ("b1_poles", "b2_gate") for k in ("K1", "K2") for pl, sd in (("astar", 0), ("rrt_star", 0))]
        # the new scenarios: field, blocks (S-curve), beam (vertical avoidance)
        + [(s, pl, k, sd) for s in ("b3_field", "b4_blocks", "b5_beam") for k in ("K1", "K2")
           for pl, sd in (("astar", 0), ("rrt_star", 0), ("rrt_star", 1))]
        # continuous sensing (K3) and a goal change in flight (K5) on the field
        + [("b3_field", pl, k, sd) for k in ("K3", "K5") for pl, sd in (("astar", 0), ("rrt_star", 0), ("rrt_star", 1))]
        # too little energy: must be refused before takeoff
        + [("b3_field", "astar", "K6", 0)]
    ),
}


def docker(*args: str, check: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True, check=check)


def mission_yaml(scenario: str, planner: str, cond: str, seed: int, res: float, takeoff: float) -> dict:
    return {"steps": [
        {"action": "plan_route", "params": {
            "scenario": f"{IN_RES}/planning/stage_b/scenarios/{scenario}.yaml", "planner": planner, "condition": cond,
            "seed": seed, "resolution": res, "result_file": "/results/exec.json"}},
        {"action": "takeoff", "params": {"takeoff_altitude": takeoff}},
        {"action": "follow_route", "params": {"timeout": 300.0}},
        # No 'land' step: the Land action of ardupilot_interface goes through RTL (climb, return, land), which takes
        # minutes in BiguaSim and is not what is being evaluated. The mission ends here; the vehicle keeps holding
        # the goal in GUIDED until the orchestrator tears the stack down.
        {"action": "wait", "params": {"duration": 3.0}},
    ]}


def publish_new_goal(log: Path, goal, delay_s: float, stop_evt: threading.Event) -> None:
    """K5: wait for the route to start, then publish the new final goal on /planner/new_goal from inside the container."""
    t0 = time.time()
    while not stop_evt.is_set() and time.time() - t0 < 900:
        if log.exists() and "route: following" in log.read_text(errors="ignore"):
            break
        time.sleep(0.5)
    else:
        return
    stop_evt.wait(delay_s)
    if stop_evt.is_set():
        return
    msg = "{x: %.2f, y: %.2f, z: %.2f}" % tuple(goal)
    r = docker("exec", CONTAINER, "bash", "-c",
               f"source /opt/ros/humble/setup.bash && ros2 topic pub --once /planner/new_goal geometry_msgs/msg/Point \"{msg}\"")
    (log.parent / "new_goal.log").write_text(f"published {msg}\nrc={r.returncode}\n{r.stdout}{r.stderr}")


def run_flight(run_id: str, scenario: str, planner: str, cond: str, seed: int, res: float, out: Path,
               boot_timeout: float, flight_timeout: float) -> dict | None:
    d = out / "runs" / run_id
    d.mkdir(parents=True, exist_ok=True)
    scen_path = SCENARIO_DIR / f"{scenario}.yaml"
    sc = load_scenario(scen_path)
    sitl = runner = ctr = None
    stop_evt = threading.Event()
    try:
        kill_leftovers()
        docker("rm", "-f", CONTAINER)
        sitl = start(["bash", str(MISSIONS_DIR / "t2_sitl_run.sh")], d / "sitl.log", cwd=MISSIONS_DIR)
        time.sleep(8.0)
        runner = start([PY, str(STAGE_B / "sitl_runner.py"), "--scenario", str(scen_path), "--log", str(d / "truth.csv")], d / "runner.log")
        if not wait_log(d / "runner.log", "STAGE_B_RUNNER_READY", boot_timeout, runner):
            print(f"  [{run_id}] BiguaSim did not come up (see {d / 'runner.log'})", flush=True)
            return None

        (d / "mission.yaml").write_text(yaml.safe_dump(mission_yaml(scenario, planner, cond, seed, res, float(sc.waypoints[0][2]))))
        ctr = start(["docker", "run", "--rm", "--name", CONTAINER, "--network", "host",
                     "-v", f"{MISSION_PKG}:/aas/aircraft_ws/src/mission/mission:ro",
                     "-v", f"{RESOURCES}:{IN_RES}:ro", "-v", f"{d}:/results",
                     "-e", "DRONE_TYPE=quad", "-e", "DRONE_ID=0", "-e", "AUTOPILOT=ardupilot",
                     "-e", f"AAS_PLANNING_PATH={IN_RES}/planning",
                     "--entrypoint", "bash", "aircraft-image",
                     f"{IN_RES}/planning/stage_c/aircraft_stack.sh", "/results/mission.yaml", "/results"], d / "container.log")
        if cond == "K5" and sc.mission_change:
            threading.Thread(target=publish_new_goal, daemon=True,
                             args=(d / "mission.log", sc.mission_change["goal"], float(sc.mission_change["t"]), stop_evt)).start()
        t0 = time.time()
        while ctr.poll() is None and time.time() - t0 < flight_timeout:
            time.sleep(1.0)
        if ctr.poll() is None:
            print(f"  [{run_id}] container timed out", flush=True)
        return json.loads((d / "exec.json").read_text()) if (d / "exec.json").exists() else None
    finally:
        stop_evt.set()
        docker("rm", "-f", CONTAINER)
        stop(ctr)
        stop(runner)
        stop(sitl)
        kill_leftovers()


def write_notes(out: Path, rows: list[dict]) -> None:
    import pandas as pd

    r = pd.DataFrame(rows)
    for c in ("planned_wh", "exec_wh_thrust", "delta_e_thrust_wh", "flown_length_m", "mission_time_s", "n_replans",
              "min_clearance_m", "max_crosstrack_m", "mean_speed_mps", "max_speed_mps", "final_error_m", "stops_at_waypoints", "replan_ms_max"):
        r[c] = pd.to_numeric(r[c], errors="coerce")
    r["dE_pct"] = 100.0 * r["delta_e_thrust_wh"] / r["planned_wh"]
    lines = [
        "**Como foi rodado.** Os planejadores foram voados **pelo `mission_node`** (ações `plan_route` e `follow_route`, novas), no stack ROS 2 Humble real "
        "(MAVROS + `ardupilot_interface` + `mission`, dentro do `aircraft-image`), contra ArduPilot SITL e BiguaSim (Hydrone, mundo Bridge). "
        "Cada voo tem um stack novo e uma missão gerada: `plan_route` → `takeoff` → `follow_route` → `wait` → `land`. Os waypoints saem pelo serviço "
        "`SetReposition` (GUIDED, `GlobalPositionTarget`); posição e velocidade entram de `/mavros/local_position/{pose,velocity_local}`. "
        "No K5 a nova meta é publicada em `/planner/new_goal` durante o voo. A lógica de voo é a mesma (`PlannedFlight`) do executivo pymavlink da Etapa B.",
        "",
        "**Ainda simulado:** a percepção (pontos de superfície dentro do raio, a partir da posição do EKF); os obstáculos existem de verdade no BiguaSim como props. "
        "**Só parte aérea**, 4 a 9 m acima do home. Energia executada = empuxo aplicado pela planta nas curvas de potência placeholder.",
        "",
        f"**Voos:** {len(r)}; SUCESSO: {int((r['status'] == 'SUCESSO').sum())}; "
        + "; ".join(f"{k}: {v}" for k, v in r["status"].value_counts().items() if k != "SUCESSO"),
    ]
    for key, label in (("scenario", "Cenário"), ("condition", "Condição"), ("planner", "Planejador")):
        for k, g in r.groupby(key):
            ok = g[g["status"] == "SUCESSO"]
            lines.append(f"* **{label} {k}** ({len(g)} voos, {len(ok)} SUCESSO): folga mín. {g.min_clearance_m.mean():.2f} m, "
                         f"tempo {g.mission_time_s.mean():.1f} s, replan. {g.n_replans.mean():.1f}/voo, ΔE {g.dE_pct.mean():+.0f} %, "
                         f"erro final {g.final_error_m.mean():.2f} m.")
    (out / "NOTES.md").write_text("\n".join(lines) + "\n")


def collect_and_report(out: Path) -> None:
    rows = run_stage_b.collect(out)
    if not rows:
        return
    with open(out / "runs_c.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    run_stage_b.figures(out, rows)
    write_notes(out, rows)
    print(f"wrote {out}/runs_c.csv, figures/, NOTES.md ({len(rows)} flights)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(PLANNING / "results" / "stage_c"))
    ap.add_argument("--matrix", choices=list(MATRIX), default="full")
    ap.add_argument("--only", nargs="*", help="restrict to scenarios, e.g. b3_field")
    ap.add_argument("--astar-resolution", type=float, default=1.0)
    ap.add_argument("--boot-timeout", type=float, default=480.0)
    ap.add_argument("--flight-timeout", type=float, default=720.0)
    ap.add_argument("--retries", type=int, default=1)
    ap.add_argument("--analyse-only", action="store_true")
    a = ap.parse_args()

    out = Path(a.out).resolve()      # absolute: docker treats a relative -v source as a volume name
    (out / "runs").mkdir(parents=True, exist_ok=True)
    if not a.analyse_only:
        plan = [m for m in MATRIX[a.matrix] if not a.only or m[0] in a.only]
        print(f"{len(plan)} flights planned", flush=True)
        for scn, pl, cond, seed in plan:
            run_id = f"{scn}_{pl}_{cond}_s{seed}"
            if (out / "runs" / run_id / "exec.json").exists():
                print(f"[{run_id}] already done, skipping", flush=True)
                continue
            for attempt in range(1 + a.retries):
                print(f"[{run_id}] attempt {attempt + 1}", flush=True)
                ex = run_flight(run_id, scn, pl, cond, seed, a.astar_resolution if pl == "astar" else 0.0, out,
                                a.boot_timeout, a.flight_timeout)
                if ex is not None:
                    print(f"[{run_id}] {ex['status']} {ex.get('detail', '')}", flush=True)
                    break
    collect_and_report(out)


if __name__ == "__main__":
    main()
