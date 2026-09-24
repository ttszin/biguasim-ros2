"""T8.2 experiment campaign: A* vs RRT* over scenarios C1..C6 and conditions K1..K6.

    python3 benchmark.py --seeds 30 --jobs 12 --out results/full        # the full campaign
    python3 benchmark.py --seeds 2 --jobs 8 --out results/smoke --only C3   # quick check

Minimum matrix from the guide: C1, C5 and C6 run under every condition; the others
run under K1 and their most relevant condition (C2: K5, C3: K2, C4: K4).
RRT* runs `--seeds` fixed seeds (0..N-1) per combination; A* is deterministic
(grid jitter is seeded) and runs at two grid resolutions per scenario.

Outputs (in --out):
  runs.csv          one row per run (append-only; an interrupted campaign resumes)
  summary.csv       aggregated per scenario x condition x planner x resolution
  traces/*.json     executed trace + planned paths of the first seed of every combination
  memory.csv        peak memory of the initial planning, measured in a separate pass
Use report.py to turn these into plots and a Markdown summary.

Timing note: every run is pinned to one CPU core, and times are also reported as a
percentage of the real-time budget (time to traverse the leg) so the numbers can be
scaled to the onboard computer. `--cpu-slowdown F` multiplies the reported times by
F (e.g. ~3 for a Raspberry Pi 4 vs this desktop); it is an estimate, not a
measurement -- the guide asks for measurements on the real board.
"""

from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent

SCENARIO_FILES = {
    "C1": "c1_long_air", "C2": "c2_moving_boat", "C3": "c3_buoy_close",
    "C4": "c4_transition", "C5": "c5_underwater", "C6": "c6_full_mission",
}
ALL_K = ["K1", "K2", "K3", "K4", "K5", "K6"]
MATRIX = {
    "C1": ALL_K, "C5": ALL_K, "C6": ALL_K,
    "C2": ["K1", "K5"], "C3": ["K1", "K2"], "C4": ["K1", "K4"],
}
# A* grid resolutions per scenario: at least two, from cheap to fine (memory grows with the cube).
RESOLUTIONS = {"C1": [4.0, 2.0], "C2": [4.0, 2.0], "C3": [1.0, 0.5], "C4": [2.0, 1.0], "C5": [2.0, 1.0], "C6": [4.0, 2.0]}
K4_LEVELS = [0.02, 0.10, 0.20]

FIELDS = ["scenario", "condition", "uncertainty", "planner", "resolution", "seed", "status", "detail",
          "t_initial_ms", "t_first_solution_ms", "n_replans", "replan_ms_mean", "replan_ms_max",
          "initial_pct_budget", "replan_pct_budget_max", "realtime_ok",
          "energy_planned_wh", "energy_exec_wh", "delta_e_wh", "abs_delta_e_wh",
          "e_air_wh", "e_water_wh", "e_transition_wh",
          "length_planned_m", "length_exec_m", "n_waypoints", "min_clearance_m", "mission_time_s", "wall_s"]


def _init_worker(cfg_path: str | None) -> None:
    os.environ["OMP_NUM_THREADS"] = os.environ["OPENBLAS_NUM_THREADS"] = os.environ["MKL_NUM_THREADS"] = "1"
    try:
        cpus = sorted(os.sched_getaffinity(0))
        os.sched_setaffinity(0, {cpus[os.getpid() % len(cpus)]})  # one core per worker
    except (AttributeError, OSError):
        pass


def _run_one(task: dict) -> dict:
    """Runs in a worker process. Returns a flat CSV row (+ optional trace payload)."""
    from config import Config
    from execution import run_mission
    from scenario import load_scenario

    cfg = Config.load(task["config"]).override(**task["overrides"])
    sc = load_scenario(HERE / "scenarios" / f"{SCENARIO_FILES[task['scenario']]}.yaml")
    t0 = time.perf_counter()
    r = run_mission(sc, cfg, task["planner"], task["condition"], task["seed"], resolution=task["resolution"],
                    uncertainty=task["uncertainty"], keep_trace=task["save_trace"])
    wall = time.perf_counter() - t0
    slow = task["cpu_slowdown"]

    legs = [np.linalg.norm(np.diff(p, axis=0), axis=1).sum() for p in r.planned_paths] or [1.0]
    speed = min(cfg.get("energy", "air")["cruise_speed"], cfg.get("energy", "water")["cruise_speed"])
    budget_s = max(min(legs) / speed, 1e-6)                       # shortest leg's traversal time
    replan = np.array(r.replan_ms) if r.replan_ms else np.array([])
    row = {
        "scenario": r.scenario, "condition": r.condition, "uncertainty": task["uncertainty"] if task["uncertainty"] is not None else "",
        "planner": r.planner, "resolution": r.resolution if r.resolution else "", "seed": r.seed,
        "status": r.status, "detail": r.detail,
        "t_initial_ms": r.t_initial_ms * slow, "t_first_solution_ms": r.t_first_solution_ms * slow,
        "n_replans": r.n_replans,
        "replan_ms_mean": replan.mean() * slow if len(replan) else "", "replan_ms_max": replan.max() * slow if len(replan) else "",
        "initial_pct_budget": 100.0 * r.t_initial_ms * slow / 1000.0 / budget_s / max(len(legs), 1) if r.t_initial_ms == r.t_initial_ms else "",
        "replan_pct_budget_max": 100.0 * replan.max() * slow / 1000.0 / budget_s if len(replan) else "",
        "realtime_ok": int(r.realtime_ok),
        "energy_planned_wh": r.energy_planned_wh, "energy_exec_wh": r.energy_exec_wh, "delta_e_wh": r.delta_e_wh,
        "abs_delta_e_wh": abs(r.delta_e_wh) if r.delta_e_wh == r.delta_e_wh else "",
        "e_air_wh": r.energy_by_medium_wh.get("AR", ""), "e_water_wh": r.energy_by_medium_wh.get("AGUA", ""),
        "e_transition_wh": r.energy_by_medium_wh.get("TRANSICAO", ""),
        "length_planned_m": r.length_planned_m, "length_exec_m": r.length_exec_m, "n_waypoints": r.n_waypoints,
        "min_clearance_m": r.min_clearance_m, "mission_time_s": r.mission_time_s, "wall_s": wall,
    }
    payload = None
    if task["save_trace"]:
        payload = {"key": task["key"], "row": {k: row[k] for k in ("scenario", "condition", "planner", "resolution", "seed", "status")},
                   "trace": [list(map(float, p)) for p in r.trace][::2],
                   "planned_paths": [p.tolist() for p in r.planned_paths]}
    return {"row": row, "payload": payload}


def _memory_one(task: dict) -> dict:
    from config import Config
    from execution import run_mission
    from scenario import load_scenario

    cfg = Config.load(task["config"]).override(**task["overrides"])
    sc = load_scenario(HERE / "scenarios" / f"{SCENARIO_FILES[task['scenario']]}.yaml")
    r = run_mission(sc, cfg, task["planner"], "K1", 0, resolution=task["resolution"], measure_memory=True, keep_trace=False)
    return {"scenario": task["scenario"], "planner": task["planner"], "resolution": task["resolution"] or "",
            "peak_mem_mb": round(r.peak_mem_mb, 2), "status": r.status}


def build_tasks(a) -> list[dict]:
    tasks = []
    for scn in a.only or MATRIX:
        for cond in MATRIX[scn]:
            levels = ([K4_LEVELS if a.k4_sweep else [0.10]][0]) if cond == "K4" else [None]
            for u in levels:
                for res in RESOLUTIONS[scn]:
                    for seed in range(a.astar_seeds):
                        tasks.append(dict(scenario=scn, condition=cond, planner="astar", resolution=res, seed=seed, uncertainty=u))
                for seed in range(a.seeds):
                    tasks.append(dict(scenario=scn, condition=cond, planner="rrt_star", resolution=None, seed=seed, uncertainty=u))
    for t in tasks:
        t.update(config=a.config, overrides=a.overrides, cpu_slowdown=a.cpu_slowdown)
        t["key"] = "|".join(str(t[k]) for k in ("scenario", "condition", "uncertainty", "planner", "resolution", "seed"))
        t["save_trace"] = t["seed"] == 0
    return tasks


def parse_overrides(pairs: list[str]) -> dict:
    out = {}
    for p in pairs or []:
        k, v = p.split("=", 1)
        out[k] = json_value(v)
    return out


def json_value(v: str):
    try:
        return json.loads(v)
    except json.JSONDecodeError:
        return v


def summarise(rows: list[dict]) -> list[dict]:
    keys = sorted({(r["scenario"], r["condition"], str(r["uncertainty"]), r["planner"], str(r["resolution"])) for r in rows})
    out = []
    num = ["t_initial_ms", "t_first_solution_ms", "n_replans", "replan_ms_mean", "replan_ms_max", "initial_pct_budget",
           "replan_pct_budget_max", "energy_planned_wh", "energy_exec_wh", "abs_delta_e_wh", "length_planned_m",
           "length_exec_m", "n_waypoints", "min_clearance_m", "mission_time_s"]
    for sc, cond, u, pl, res in keys:
        rs = [r for r in rows if (r["scenario"], r["condition"], str(r["uncertainty"]), r["planner"], str(r["resolution"])) == (sc, cond, u, pl, res)]
        row = {"scenario": sc, "condition": cond, "uncertainty": u, "planner": pl, "resolution": res, "runs": len(rs)}
        statuses = {}
        for r in rs:
            statuses[r["status"]] = statuses.get(r["status"], 0) + 1
        for s in ("SUCESSO", "SEM_CAMINHO", "TIMEOUT", "ENERGIA_INSUFICIENTE", "TEMPO_TOTAL_EXCEDIDO", "COLISAO"):
            row[f"pct_{s}"] = round(100.0 * statuses.get(s, 0) / len(rs), 1)
        ok = [r for r in rs if r["status"] == "SUCESSO"]
        for k in num:
            vals = np.array([float(r[k]) for r in ok if r[k] not in ("", None) and float(r[k]) == float(r[k])])
            row[f"{k}_mean"] = round(float(vals.mean()), 3) if len(vals) else ""
            row[f"{k}_std"] = round(float(vals.std()), 3) if len(vals) else ""
        row["realtime_ok_pct"] = round(100.0 * sum(int(r["realtime_ok"]) for r in ok) / len(ok), 1) if ok else ""
        out.append(row)
    return out


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    fields = fields or list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=30, help="RRT* seeds per combination (guide: 30)")
    ap.add_argument("--astar-seeds", type=int, default=3, help="A* runs per resolution (grid-jitter offsets are seeded)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--only", nargs="*", choices=list(MATRIX), help="restrict to some scenarios")
    ap.add_argument("--config", default=None, help="planner.yaml to use (default: config/planner.yaml)")
    ap.add_argument("--set", dest="set", nargs="*", default=[], help="config overrides, e.g. rrt_star.step=1.5 rrt_star.time_budget_s=2")
    ap.add_argument("--k4-sweep", action="store_true", help="K4 at 2 %%, 10 %% and 20 %% (default: 10 %% only)")
    ap.add_argument("--cpu-slowdown", type=float, default=1.0, help="multiply reported times (estimate for a slower board)")
    ap.add_argument("--no-memory", action="store_true")
    ap.add_argument("--out", default=str(HERE / "results" / "campaign"))
    a = ap.parse_args()
    a.overrides = parse_overrides(a.set)

    out = Path(a.out)
    (out / "traces").mkdir(parents=True, exist_ok=True)
    tasks = build_tasks(a)
    runs_path = out / "runs.csv"
    done = set()
    rows: list[dict] = []
    if runs_path.exists():
        with open(runs_path) as f:
            for r in csv.DictReader(f):
                rows.append(r)
                done.add("|".join(str(r[k]) if r[k] != "" else "None" for k in ("scenario", "condition", "uncertainty", "planner", "resolution", "seed")))
    todo = []
    for t in tasks:
        key = "|".join(str(t[k]) for k in ("scenario", "condition", "uncertainty", "planner", "resolution", "seed"))
        if key not in done:
            todo.append(t)
    print(f"{len(tasks)} runs planned, {len(tasks) - len(todo)} already done, {len(todo)} to go, {a.jobs} workers", flush=True)

    ctx = mp.get_context("spawn")
    t0 = time.perf_counter()
    new_file = not runs_path.exists()
    with ctx.Pool(a.jobs, initializer=_init_worker, initargs=(a.config,)) as pool, open(runs_path, "a", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        if new_file:
            writer.writeheader()
        for n, res in enumerate(pool.imap_unordered(_run_one, todo, chunksize=1), start=1):
            writer.writerow(res["row"])
            fh.flush()
            rows.append(res["row"])
            if res["payload"]:
                (out / "traces" / (res["payload"]["key"].replace("|", "_").replace(".", "p") + ".json")).write_text(json.dumps(res["payload"]))
            if n % 20 == 0 or n == len(todo):
                el = time.perf_counter() - t0
                print(f"  {n}/{len(todo)} runs  {el:.0f}s elapsed  ~{el / n * (len(todo) - n):.0f}s left", flush=True)

        if not a.no_memory:
            mem_tasks = [dict(scenario=s, planner="astar", resolution=r, config=a.config, overrides=a.overrides)
                         for s in (a.only or MATRIX) for r in RESOLUTIONS[s]]
            mem_tasks += [dict(scenario=s, planner="rrt_star", resolution=None, config=a.config, overrides=a.overrides)
                          for s in (a.only or MATRIX)]
            mem = pool.map(_memory_one, mem_tasks, chunksize=1)
            write_csv(out / "memory.csv", mem)

    typed = []
    for r in rows:
        typed.append({k: (r[k] if k in ("scenario", "condition", "planner", "status", "detail", "uncertainty", "resolution") else r[k]) for k in FIELDS})
    summary = summarise(typed)
    write_csv(out / "summary.csv", summary)
    print(f"\nwrote {runs_path}, summary.csv" + ("" if a.no_memory else ", memory.csv") + f" in {out}")


if __name__ == "__main__":
    main()
