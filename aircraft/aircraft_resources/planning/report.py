"""Turn a benchmark campaign (runs.csv, memory.csv, traces/) into figures and report.md.

    python3 report.py results/campaign [--convergence]

Figures (PNG, in <dir>/figures): per-scenario paths (top view + altitude profile), planned
energy per planner across seeds, real-time budget usage, peak memory vs A* resolution,
K4 uncertainty sweep, and (with --convergence) RRT* cost-vs-time curves.
report.md has the per-scenario / per-condition tables and the comparison of the two
planners; it states what the data support and what it does not.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Circle, Rectangle

from benchmark import RESOLUTIONS, SCENARIO_FILES
from scenario import load_scenario

HERE = Path(__file__).resolve().parent
COLORS = {"astar": "tab:blue", "rrt_star": "tab:orange"}
LABEL = {"astar": "A*", "rrt_star": "RRT*"}


def load(out: Path):
    runs = pd.read_csv(out / "runs.csv")
    runs["resolution"] = runs["resolution"].astype("float64")
    mem = pd.read_csv(out / "memory.csv") if (out / "memory.csv").exists() else pd.DataFrame()
    return runs, mem


def draw_scenario(ax, sc):
    for o in sc.known:
        _draw_obstacle(ax, o, solid=True)
    for o in sc.hidden:
        _draw_obstacle(ax, o, solid=False)
    for m in sc.moving:
        for t in (0.0, m.track[-1][0]):
            b = m.box_at(t)
            ax.add_patch(Rectangle((b.e_min, b.n_min), b.e_max - b.e_min, b.n_max - b.n_min, fill=False, ec="tab:red", ls="--", lw=1))
        e0, e1 = m.track[0][1][1], m.track[-1][1][1]
        ax.annotate("", xy=(e1, m.track[-1][1][0]), xytext=(e0, m.track[0][1][0]), arrowprops=dict(arrowstyle="->", color="tab:red"))


def _draw_obstacle(ax, o, solid):
    kw = dict(color="0.35") if solid else dict(fill=False, ec="0.35", ls=":", lw=1.2)
    if hasattr(o, "radius"):
        ax.add_patch(Circle((o.east, o.north), o.radius, **kw))
    else:
        ax.add_patch(Rectangle((o.e_min, o.n_min), o.e_max - o.e_min, o.n_max - o.n_min, **kw))


def fig_paths(out: Path):
    (out / "figures").mkdir(exist_ok=True)
    for name, fname in SCENARIO_FILES.items():
        sc = load_scenario(HERE / "scenarios" / f"{fname}.yaml")
        traces = sorted((out / "traces").glob(f"{name}_K1_*_0.json"))
        if not traces:
            continue
        fig, (ax, az) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [1.3, 1]})
        draw_scenario(ax, sc)
        for tp in traces:
            d = json.loads(tp.read_text())
            planner = d["row"]["planner"]
            tr = np.array(d["trace"])
            if len(tr) == 0:
                continue
            res = d["row"]["resolution"]
            label = LABEL[planner] + (f" {res:g} m" if res == res and planner == "astar" else "")
            ls = "-" if planner == "rrt_star" else "--"
            ax.plot(tr[:, 2], tr[:, 1], ls, color=COLORS[planner], alpha=0.8, lw=1.6, label=label)
            dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(tr[:, 1:3], axis=0), axis=1))])
            az.plot(dist, tr[:, 3], ls, color=COLORS[planner], alpha=0.8, lw=1.6, label=label)
        for i, w in enumerate(sc.waypoints):
            ax.plot(w[1], w[0], "k*", ms=11)
            ax.annotate(sc.waypoint_names[i], (w[1], w[0]), textcoords="offset points", xytext=(6, 6), fontsize=8)
        ax.set_xlim(*sc.bounds[1]); ax.set_ylim(*sc.bounds[0]); ax.set_aspect("equal")
        ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)"); ax.set_title(f"{name}: {sc.description}", fontsize=9)
        ax.legend(fontsize=7, loc="best")
        az.axhline(0, color="tab:cyan", lw=1); az.axhspan(-0.5, 0.5, color="tab:cyan", alpha=0.15, label="transition zone")
        az.set_xlabel("horizontal distance flown (m)"); az.set_ylabel("up (m), surface = 0"); az.set_title("altitude profile (executed)")
        fig.tight_layout(); fig.savefig(out / "figures" / f"paths_{name}.png", dpi=120); plt.close(fig)


def _finest(runs, scenario):
    return min(RESOLUTIONS[scenario])


def fig_energy(runs, out: Path):
    ok = runs[(runs.status == "SUCESSO") & (runs.condition == "K1")]
    fig, axes = plt.subplots(1, 6, figsize=(17, 4))
    for ax, name in zip(axes, SCENARIO_FILES):
        d = ok[ok.scenario == name]
        data, labels, cols = [], [], []
        for res in RESOLUTIONS[name]:
            x = d[(d.planner == "astar") & (d.resolution == res)].energy_planned_wh
            if len(x):
                data.append(x.values); labels.append(f"A*\n{res:g} m"); cols.append(COLORS["astar"])
        x = d[d.planner == "rrt_star"].energy_planned_wh
        if len(x):
            data.append(x.values); labels.append("RRT*"); cols.append(COLORS["rrt_star"])
        if data:
            bp = ax.boxplot(data, tick_labels=labels, patch_artist=True)
            for patch, c in zip(bp["boxes"], cols):
                patch.set_facecolor(c); patch.set_alpha(0.5)
        ax.set_title(name); ax.set_ylabel("planned energy (Wh)" if name == "C1" else "")
    fig.suptitle("Planned energy, K1 (RRT*: spread over seeds; A*: grid-jitter offsets)")
    fig.tight_layout(); fig.savefig(out / "figures" / "energy_k1.png", dpi=120); plt.close(fig)


def fig_time(runs, out: Path):
    ok = runs[(runs.status == "SUCESSO")]
    fig, ax = plt.subplots(1, 2, figsize=(14, 4.5))
    rows = []
    for name in SCENARIO_FILES:
        d = ok[(ok.scenario == name) & (ok.condition == "K1")]
        for pl in ("astar", "rrt_star"):
            e = d[d.planner == pl]
            if pl == "astar":
                e = e[e.resolution == _finest(runs, name)]
            rows.append((name, pl, e.t_initial_ms.mean(), e.t_first_solution_ms.mean()))
    x = np.arange(len(SCENARIO_FILES))
    for k, pl in enumerate(("astar", "rrt_star")):
        ax[0].bar(x + (k - 0.5) * 0.38, [r[2] for r in rows if r[1] == pl], 0.38, color=COLORS[pl], alpha=0.5, label=f"{LABEL[pl]} total")
        ax[0].bar(x + (k - 0.5) * 0.38, [r[3] for r in rows if r[1] == pl], 0.38, color=COLORS[pl], label=f"{LABEL[pl]} to 1st solution")
    ax[0].set_yscale("log"); ax[0].set_xticks(x, list(SCENARIO_FILES)); ax[0].set_ylabel("initial planning time (ms)")
    ax[0].set_title("Initial plan (RRT* is anytime: it spends its budget)"); ax[0].legend(fontsize=7)
    rp = ok[ok.n_replans > 0]
    for k, pl in enumerate(("astar", "rrt_star")):
        vals = [rp[(rp.scenario == n) & (rp.planner == pl)].replan_ms_mean.dropna().values for n in SCENARIO_FILES]
        pos = x + (k - 0.5) * 0.38
        bp = ax[1].boxplot([v if len(v) else [np.nan] for v in vals], positions=pos, widths=0.3, patch_artist=True, manage_ticks=False)
        for patch in bp["boxes"]:
            patch.set_facecolor(COLORS[pl]); patch.set_alpha(0.5)
        ax[1].plot([], [], color=COLORS[pl], label=LABEL[pl])
    ax[1].set_yscale("log"); ax[1].set_xticks(x, list(SCENARIO_FILES)); ax[1].set_ylabel("mean replanning time per run (ms)")
    ax[1].set_title("Replanning time (runs that replanned)"); ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(out / "figures" / "times.png", dpi=120); plt.close(fig)


def fig_memory(mem, out: Path):
    if mem.empty:
        return
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for k, name in enumerate(SCENARIO_FILES):
        d = mem[(mem.scenario == name) & (mem.planner == "astar")].sort_values("resolution", ascending=False)
        ax.plot(d.resolution.astype(float), d.peak_mem_mb, "o-", label=f"A* {name}")
        r = mem[(mem.scenario == name) & (mem.planner == "rrt_star")].peak_mem_mb
        if len(r):
            ax.axhline(float(r.iloc[0]), color="tab:orange", alpha=0.25, lw=1)
    ax.set_xscale("log"); ax.set_yscale("log"); ax.invert_xaxis()
    ax.set_xlabel("A* grid resolution (m)"); ax.set_ylabel("peak memory (MB)")
    ax.set_title("Peak memory of the initial plan (orange lines: RRT* per scenario)"); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(out / "figures" / "memory.png", dpi=120); plt.close(fig)


def fig_k4(runs, out: Path):
    d = runs[(runs.condition == "K4") & (runs.status == "SUCESSO")]
    if d.empty:
        return
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    for ax, (col, lab) in zip(axes, (("energy_planned_wh", "planned energy (Wh)"), ("min_clearance_m", "min clearance (m)"), ("n_waypoints", "waypoints after pruning"))):
        for pl in ("astar", "rrt_star"):
            for name in ("C4", "C5", "C6"):
                e = d[(d.scenario == name) & (d.planner == pl)]
                if pl == "astar":
                    e = e[e.resolution == _finest(runs, name)]
                g = e.groupby("uncertainty")[col].mean()
                ax.plot(g.index * 100, g.values, "o-" if pl == "astar" else "s--", color=COLORS[pl], alpha=0.4 + 0.2 * ("C4", "C5", "C6").index(name), label=f"{LABEL[pl]} {name}")
        ax.set_xlabel("position uncertainty (% of distance)"); ax.set_ylabel(lab)
    axes[0].legend(fontsize=6)
    fig.suptitle("K4: underwater position uncertainty")
    fig.tight_layout(); fig.savefig(out / "figures" / "k4.png", dpi=120); plt.close(fig)


def fig_convergence(out: Path):
    from config import Config
    from hybrid_map import HybridMap
    from energy import EnergyModel
    from replan import CLRRTReplanner

    cfg = Config.load()
    fig, axes = plt.subplots(1, 6, figsize=(17, 3.8))
    for ax, (name, fname) in zip(axes, SCENARIO_FILES.items()):
        sc = load_scenario(HERE / "scenarios" / f"{fname}.yaml")
        w = sc.truth_world(0.0)
        hm, en = HybridMap(w, cfg), EnergyModel.from_config(cfg)
        for seed in range(5):
            r = CLRRTReplanner(hm, en, cfg, sc.waypoints[0], sc.waypoints[1], seed=seed).initial(budget_s=5.0, improve=True)
            h = r.extra.get("history", [])
            if h:
                ax.step([x[1] for x in h], [x[2] / 3600.0 for x in h], where="post", alpha=0.7)
        ax.set_title(name); ax.set_xlabel("time (s)"); ax.set_ylabel("best energy (Wh)" if name == "C1" else "")
    fig.suptitle("RRT* convergence, first leg, 5 seeds")
    fig.tight_layout(); fig.savefig(out / "figures" / "rrt_convergence.png", dpi=120); plt.close(fig)


# ---------------------------------------------------------------------------- markdown

def fmt(x, nd=2):
    return "-" if x is None or (isinstance(x, float) and x != x) else f"{x:.{nd}f}"


def table(rows, header):
    s = "| " + " | ".join(header) + " |\n|" + "|".join("---" for _ in header) + "|\n"
    return s + "\n".join("| " + " | ".join(str(c) for c in r) + " |" for r in rows) + "\n"


def cell(d, col, nd=2):
    d = d[col].dropna()
    return "-" if d.empty else f"{d.mean():.{nd}f} ± {d.std(ddof=0):.{nd}f}"


def write_md(runs, mem, out: Path):
    L = ["# T8.2 results: A* vs RRT*\n"]
    L.append(f"{len(runs)} runs. **All energies use PLACEHOLDER curve coefficients** (config/planner.yaml): compare planners with each other, "
             "do not quote absolute Wh until the real Pinheiro et al. coefficients are in. ΔE compares the plan with the flown "
             "trajectory under the *same* curves, so it measures execution deviation, not model fidelity.\n")
    L.append("Statuses are per run: SUCESSO, SEM_CAMINHO, TIMEOUT, ENERGIA_INSUFICIENTE, TEMPO_TOTAL_EXCEDIDO, COLISAO.\n")

    # per scenario, K1 baseline
    L.append("\n## Baseline (K1): per scenario\n")
    rows = []
    for name in SCENARIO_FILES:
        d = runs[(runs.scenario == name) & (runs.condition == "K1")]
        for pl in ("astar", "rrt_star"):
            for res in (RESOLUTIONS[name] if pl == "astar" else [np.nan]):
                e = d[(d.planner == pl) & ((d.resolution == res) if res == res else d.resolution.isna())]
                if e.empty:
                    continue
                ok = e[e.status == "SUCESSO"]
                m = mem[(mem.scenario == name) & (mem.planner == pl) & ((mem.resolution == res) if res == res else mem.resolution.isna())]
                rows.append([name, LABEL[pl] + (f" {res:g} m" if res == res else ""), f"{100 * len(ok) / len(e):.0f}%",
                             cell(ok, "t_initial_ms", 0), cell(ok, "t_first_solution_ms", 0),
                             fmt(float(m.peak_mem_mb.iloc[0])) if len(m) else "-",
                             cell(ok, "energy_planned_wh", 3), cell(ok, "length_planned_m", 1),
                             cell(ok, "n_waypoints", 1), cell(ok, "min_clearance_m", 2), cell(ok, "abs_delta_e_wh", 3)])
    L.append(table(rows, ["scen", "planner", "success", "init ms", "1st sol. ms", "peak MB", "E plan Wh", "length m", "wps", "min clear m", "|ΔE| Wh"]))

    L.append("\n## All conditions: success by category and replanning\n")
    rows = []
    for (name, cond, unc), d in runs.groupby(["scenario", "condition", runs.uncertainty.fillna(-1)]):
        for pl in ("astar", "rrt_star"):
            e = d[d.planner == pl]
            if pl == "astar":
                e = e[e.resolution == _finest(runs, name)]
            if e.empty:
                continue
            st = e.status.value_counts()
            cats = ", ".join(f"{k} {100 * v / len(e):.0f}%" for k, v in st.items() if k != "SUCESSO")
            ok = e[e.status == "SUCESSO"]
            rp = ok[ok.n_replans > 0]
            rows.append([name, cond + (f" {unc * 100:g}%" if unc >= 0 else ""), LABEL[pl] + (f" {_finest(runs, name):g} m" if pl == "astar" else ""),
                         f"{100 * len(ok) / len(e):.0f}%", cats or "-", cell(ok, "n_replans", 1),
                         cell(rp, "replan_ms_mean", 0), cell(rp, "replan_pct_budget_max", 1), cell(ok, "energy_planned_wh", 3), cell(ok, "abs_delta_e_wh", 3)])
    L.append(table(rows, ["scen", "cond", "planner", "success", "failures", "replans", "replan ms", "max % of budget", "E plan Wh", "|ΔE| Wh"]))

    # comparison
    L.append("\n## Comparison\n")
    ok = runs[(runs.status == "SUCESSO")]
    lines = []
    for name in SCENARIO_FILES:
        a = ok[(ok.scenario == name) & (ok.condition == "K1") & (ok.planner == "astar") & (ok.resolution == _finest(runs, name))]
        r = ok[(ok.scenario == name) & (ok.condition == "K1") & (ok.planner == "rrt_star")]
        if a.empty or r.empty:
            continue
        de = 100 * (r.energy_planned_wh.mean() / a.energy_planned_wh.mean() - 1)
        lines.append(f"- **{name}** (K1): RRT* energy is {de:+.1f}% vs A* {_finest(runs, name):g} m; "
                     f"RRT* first solution {r.t_first_solution_ms.mean():.0f} ms vs A* {a.t_initial_ms.mean():.0f} ms.")
    L.append("\n".join(lines) + "\n")
    rp = ok[ok.n_replans > 0]
    for name in SCENARIO_FILES:
        a = rp[(rp.scenario == name) & (rp.planner == "astar")].replan_ms_mean
        r = rp[(rp.scenario == name) & (rp.planner == "rrt_star")].replan_ms_mean
        if len(a) and len(r):
            L.append(f"- **{name}** replanning: D* Lite {a.mean():.0f} ms vs CL-RRT {r.mean():.0f} ms (mean over runs that replanned).\n")
    L.append("\nRead the tables before concluding: the guide asks for the choice to be justified by these data. "
             "Things this campaign does **not** establish: timings on the onboard board (this is a desktop, one core per run), "
             "absolute energy (placeholder coefficients), the real Bridge-world structure (obstacles are synthetic props), "
             "and Stage B (ArduPilot SITL in GUIDED, see README).\n")
    (out / "report.md").write_text("\n".join(L))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--convergence", action="store_true", help="also run 5 short RRT* runs per scenario for convergence curves")
    a = ap.parse_args()
    out = Path(a.out)
    runs, mem = load(out)
    (out / "figures").mkdir(exist_ok=True)
    fig_paths(out); fig_energy(runs, out); fig_time(runs, out); fig_memory(mem, out); fig_k4(runs, out)
    if a.convergence:
        fig_convergence(out)
    write_md(runs, mem, out)
    print(f"wrote {out}/report.md and {out}/figures/")


if __name__ == "__main__":
    main()
