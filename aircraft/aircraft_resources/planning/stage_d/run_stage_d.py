"""Orchestrate T8.2 Stage D: BiguaSim BlueROV2 + ArduSub SITL (GUIDED), one fresh stack per flight. Underwater counterpart of Stage B.

    python3 stage_d/run_stage_d.py --out results/stage_d --scenarios d1_poles d2_gate --conditions K1 K2 --rrt-seeds 1
    python3 stage_d/run_stage_d.py --out results/stage_d_smoke --scenarios d1_poles --conditions K1 --planners astar --rrt-seeds 0
    python3 stage_d/run_stage_d.py --out results/stage_d --analyse-only

Per flight: ArduSub SITL (rov_sitl.sh) -> rov_exec.py (must connect first: SITL blocks until a client is on tcp:5760) -> rov_runner.py
(BiguaSim; connects to SERIAL1 on its own for the ground-truth GPS_INPUT). Saves <out>/runs/<id>/{exec.json, truth.csv, *.log}; runs_d.csv
has one row per flight. Metrics come from BiguaSim's ground truth like Stage B; the thrust energy uses the sum of |thruster thrust|
(k_eta * V_MAX * |cmd|, k_eta = 3.8e-4 of the BlueROV2 profile, V_MAX = 278.9) in the planner's WATER power curve.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

STAGE_D = Path(__file__).resolve().parent
sys.path.insert(0, str(STAGE_D.parent / "stage_b"))
import run_stage_b as B  # noqa: E402

from config import Config  # noqa: E402
from energy import EnergyModel  # noqa: E402
from scenario import load_scenario  # noqa: E402

SCEN = STAGE_D / "scenarios"
B.SCENARIO_DIR = SCEN                       # figures()/collect() look the scenario files up here
CFG = STAGE_D.parent / "config" / "planner_rov_sitl.yaml"
K_ETA, V_MAX = 3.8e-4, 278.9


def add_thrust_column(truth_csv: Path) -> None:
    """analyse() expects a thrust_n column: total thruster thrust in N from the six logged commands."""
    import pandas as pd
    t = pd.read_csv(truth_csv)
    t["thrust_n"] = K_ETA * V_MAX * t[[f"m{i}" for i in range(1, 7)]].abs().sum(axis=1)
    t.to_csv(truth_csv, index=False)


def run_flight(run_id, scenario, planner, cond, seed, res, out: Path, boot_timeout, flight_timeout, viewport, runner_extra=None, exec_extra=None):
    d = out / "runs" / run_id
    d.mkdir(parents=True, exist_ok=True)
    scen_path = SCEN / f"{scenario}.yaml"
    sitl = runner = ex = None
    try:
        B.kill_leftovers()
        sitl = B.start(["bash", str(STAGE_D / "rov_sitl.sh")], d / "sitl.log", cwd=B.MISSIONS_DIR)
        time.sleep(10.0)
        ecmd = [B.PY, str(STAGE_D / "rov_exec.py"), "--scenario", str(scen_path), "--planner", planner, "--condition", cond,
                "--seed", str(seed), "--resolution", str(res), "--out", str(d / "exec.json")] + (exec_extra or [])
        ex = B.start(ecmd, d / "exec.log")
        time.sleep(3.0)
        rcmd = [B.PY, str(STAGE_D / "rov_runner.py"), "--scenario", str(scen_path), "--log", str(d / "truth.csv"), "--fix-position"]
        if viewport:
            rcmd.append("--viewport")
        rcmd += runner_extra or []
        runner = B.start(rcmd, d / "runner.log")
        if not B.wait_log(d / "runner.log", "STAGE_D_RUNNER_READY", boot_timeout, runner):
            print(f"  [{run_id}] BiguaSim did not come up (see {d / 'runner.log'})", flush=True)
            return None
        t0 = time.time()
        while ex.poll() is None and time.time() - t0 < flight_timeout:
            time.sleep(1.0)
        if ex.poll() is None:
            B.stop(ex)
            print(f"  [{run_id}] executive timed out", flush=True)
        if (d / "truth.csv").exists():
            time.sleep(1.0)
        return json.loads((d / "exec.json").read_text()) if (d / "exec.json").exists() else None
    finally:
        B.stop(ex)
        B.stop(runner)
        B.stop(sitl)
        B.kill_leftovers()
        if (d / "truth.csv").exists():
            try:
                add_thrust_column(d / "truth.csv")
            except Exception as e:  # noqa: BLE001
                print(f"  [{run_id}] could not add the thrust column: {e}", flush=True)


def collect(out: Path) -> list[dict]:
    cfg = Config.load(CFG)
    energy = EnergyModel.from_config(cfg)
    rows = []
    for d in sorted((out / "runs").glob("*")):
        f = d / "exec.json"
        if not f.exists():
            continue
        ex = json.loads(f.read_text())
        sc = load_scenario([p for p in SCEN.glob("*.yaml") if load_scenario(p).name == ex["scenario"]][0])
        rows.append(B.analyse(d, ex, cfg, energy, sc))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(STAGE_D.parent / "results" / "stage_d"))
    ap.add_argument("--scenarios", nargs="*", default=["d1_poles", "d2_gate"])
    ap.add_argument("--conditions", nargs="*", default=["K1", "K2"])
    ap.add_argument("--planners", nargs="*", default=["astar", "rrt_star"], choices=["astar", "rrt_star"])
    ap.add_argument("--rrt-seeds", type=int, default=1)
    ap.add_argument("--astar-resolution", type=float, default=1.0)
    ap.add_argument("--boot-timeout", type=float, default=480.0)
    ap.add_argument("--flight-timeout", type=float, default=1100.0)
    ap.add_argument("--viewport", action="store_true")
    ap.add_argument("--analyse-only", action="store_true")
    ap.add_argument("--retries", type=int, default=1)
    a = ap.parse_args()

    out = Path(a.out).resolve()
    (out / "runs").mkdir(parents=True, exist_ok=True)
    if not a.analyse_only:
        plan = []
        for scn in a.scenarios:
            for cond in a.conditions:
                if "astar" in a.planners:
                    plan.append((scn, "astar", cond, 0, a.astar_resolution))
                if "rrt_star" in a.planners:
                    plan += [(scn, "rrt_star", cond, s, 0.0) for s in range(a.rrt_seeds)]
        print(f"{len(plan)} flights planned", flush=True)
        for scn, pl, cond, seed, res in plan:
            run_id = f"{scn}_{pl}_{cond}_s{seed}"
            if (out / "runs" / run_id / "exec.json").exists():
                print(f"[{run_id}] already done, skipping", flush=True)
                continue
            for attempt in range(1 + a.retries):
                print(f"[{run_id}] attempt {attempt + 1}", flush=True)
                ex = run_flight(run_id, scn, pl, cond, seed, res, out, a.boot_timeout, a.flight_timeout, a.viewport)
                if ex is not None:
                    print(f"[{run_id}] {ex['status']} {ex.get('detail', '')}", flush=True)
                    break
    rows = collect(out)
    if rows:
        with open(out / "runs_d.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=B.FIELDS)
            w.writeheader()
            w.writerows(rows)
        B.figures(out, rows)
        print(f"wrote {out}/runs_d.csv, figures/ ({len(rows)} flights)")


if __name__ == "__main__":
    main()
