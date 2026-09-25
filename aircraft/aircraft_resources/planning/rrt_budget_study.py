"""Does the RRT* energy gap to the A* close with more planning time? (open hypothesis of the algorithm-choice report)

    python3 rrt_budget_study.py --seeds 20 --jobs 12 --out results/rrt_budget

For C1, C4, C5 and C6 (K1: known map, so only the planner's quality matters) the RRT* plans the whole mission with time budgets of 1, 2, 5, 10, 20
and 40 s PER LEG, with the iteration cap lifted (the config's max_iter would otherwise stop it before a long budget is used), 20 seeds each. The
A* reference is the finest grid of Stage A (3 jitter seeds). Output: runs.csv, rrt_budget.png (planned energy gap to the A* against the budget)
and summary.csv. Planned energy, not executed: the follow-up question is only whether the plan gets cheaper.
"""

from __future__ import annotations

import argparse
import csv
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

PLANNING = Path(__file__).resolve().parent
sys.path.insert(0, str(PLANNING))

from config import Config  # noqa: E402
from energy import wh  # noqa: E402
from execution import Mission  # noqa: E402
from scenario import load_scenario  # noqa: E402

SCEN = {"C1": "c1_long_air", "C4": "c4_transition", "C5": "c5_underwater", "C6": "c6_full_mission"}
ASTAR_RES = {"C1": 2.0, "C4": 1.0, "C5": 1.0, "C6": 2.0}          # the finest grid of the Stage A campaign
BUDGETS = [1.0, 2.0, 5.0, 10.0, 20.0, 40.0]


def _plan(job):
    scn, planner, budget, seed = job
    sc = load_scenario(PLANNING / "scenarios" / f"{SCEN[scn]}.yaml")
    cfg = Config.load()
    if planner == "rrt_star":
        cfg = cfg.override(**{"rrt_star.time_budget_s": budget, "rrt_star.max_iter": 10_000_000})
    m = Mission(sc, cfg, planner, "K1", seed, resolution=ASTAR_RES[scn] if planner == "astar" else None)
    t0 = time.perf_counter()
    ok, status, t_total, t_first = m.initial_plan()
    row = {"scenario": scn, "planner": planner, "budget_s": budget if planner == "rrt_star" else "", "seed": seed, "status": status,
           "wall_s": round(time.perf_counter() - t0, 2), "first_solution_ms": round(t_first * 1000, 1)}
    if ok:
        row["energy_wh"] = round(wh(m.planned_energy_j()), 4)
        row["length_m"] = round(float(sum(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)) for p in m.paths)), 2)
        row["waypoints"] = int(sum(len(p) for p in m.paths) - (len(m.paths) - 1))
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--out", default=str(PLANNING / "results" / "rrt_budget"))
    ap.add_argument("--plot-only", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / "runs.csv"
    if not a.plot_only:
        jobs = [(s, "astar", 0.0, k) for s in SCEN for k in range(3)]
        jobs += [(s, "rrt_star", b, k) for s in SCEN for b in BUDGETS for k in range(a.seeds)]
        jobs.sort(key=lambda j: -(j[2] * (3 if j[0] == "C6" else 1)))          # longest first: better packing on the workers
        rows = []
        t0 = time.time()
        with mp.Pool(a.jobs) as pool:
            for i, r in enumerate(pool.imap_unordered(_plan, jobs), 1):
                rows.append(r)
                if i % 20 == 0 or i == len(jobs):
                    print(f"{i}/{len(jobs)} done, {time.time() - t0:.0f} s", flush=True)
        keys = ["scenario", "planner", "budget_s", "seed", "status", "wall_s", "first_solution_ms", "energy_wh", "length_m", "waypoints"]
        with open(f, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(sorted(rows, key=lambda r: (r["scenario"], r["planner"], float(r["budget_s"] or 0), r["seed"])))
    plot(out)


def plot(out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd

    r = pd.read_csv(out / "runs.csv")
    ok = r[r.status == "SUCESSO"]
    ref = ok[ok.planner == "astar"].groupby("scenario").energy_wh.median()
    rr = ok[ok.planner == "rrt_star"].copy()
    rr["gap_pct"] = [100.0 * (e / ref[s] - 1.0) for e, s in zip(rr.energy_wh, rr.scenario)]
    g = rr.groupby(["scenario", "budget_s"]).gap_pct
    summ = pd.DataFrame({"median": g.median(), "q25": g.quantile(0.25), "q75": g.quantile(0.75), "min": g.min(), "n_ok": g.size()}).round(2).reset_index()
    fail = r[(r.planner == "rrt_star") & (r.status != "SUCESSO")].groupby(["scenario", "budget_s"]).size().rename("n_fail").reset_index()
    summ = summ.merge(fail, how="left", on=["scenario", "budget_s"]).fillna({"n_fail": 0})
    summ["astar_wh"] = summ.scenario.map(ref).round(3)
    summ.to_csv(out / "summary.csv", index=False)

    plt.rcParams.update({"font.size": 9.5, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": "#e6e5e1",
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "axes.edgecolor": "#b9b8b2"})
    colors = {"C1": "#2a78d6", "C4": "#eb6834", "C5": "#1baf7a", "C6": "#e87ba4"}
    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    for scn, d in summ.groupby("scenario"):
        ax.fill_between(d.budget_s, d.q25, d.q75, color=colors[scn], alpha=0.15, lw=0)
        ax.plot(d.budget_s, d["median"], "-o", color=colors[scn], lw=1.8, ms=5, label=f"{scn} (A* = {ref[scn]:.2f} Wh)")
    ax.axhline(0, color="#0b0b0b", lw=1.0)
    ax.text(ax.get_xlim()[1], 0.3, " A*", va="bottom", ha="right", fontsize=9, color="#52514e")
    ax.set_xscale("log")
    ax.set_xticks([1, 2, 5, 10, 20, 40])
    ax.set_xticklabels(["1", "2", "5", "10", "20", "40"])
    ax.set_xlabel("orçamento de tempo do RRT* por trecho (s, escala log)")
    ax.set_ylabel("energia planejada do RRT* acima da do A* (%)")
    ax.set_title("O RRT* alcança a energia do A* com mais tempo de planejamento?", loc="left", fontweight="bold")
    ax.legend(frameon=False, loc="upper right")
    fig.text(0.01, 0.005, "Mediana e intervalo interquartil de 20 seeds, mapa conhecido (K1), sem limite de iterações. A* = grade mais fina da campanha, 3 jitters.",
             fontsize=8, color="#52514e")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out / "rrt_budget.png", dpi=130)
    print(summ.to_string(index=False))


if __name__ == "__main__":
    main()
