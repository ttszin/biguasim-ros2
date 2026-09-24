"""Comparison charts A* x RRT* (T8.2): Stage A campaign (C1..C6, 30 seeds) and the SITL flights (air: Stage B, water: Stage D).

    python3 make_comparison_plots.py [--out results/comparison]

Writes stage_a_metrics.png, sitl_metrics.png and comparison_summary.csv. Colours are the first two slots of the reference
categorical palette (blue = A*, orange = RRT*), which validate for adjacent pairs and all-pairs (three-slot cap) in both modes.
Text is in Portuguese, like the rest of the report.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

PLANNING = Path(__file__).resolve().parent
RESULTS = PLANNING / "results"
BLUE, ORANGE = "#2a78d6", "#eb6834"
COLOR = {"astar": BLUE, "rrt_star": ORANGE}
NAME = {"astar": "A*", "rrt_star": "RRT*"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e6e5e1"

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9.5, "axes.edgecolor": "#b9b8b2", "axes.labelcolor": INK2, "xtick.color": INK2,
    "ytick.color": INK2, "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "legend.frameon": False, "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
})


def _legend(fig, y=0.995):
    handles = [plt.Rectangle((0, 0), 1, 1, color=COLOR[p]) for p in ("astar", "rrt_star")]
    fig.legend(handles, [NAME["astar"], NAME["rrt_star"]], loc="upper right", ncol=2, bbox_to_anchor=(0.985, y), fontsize=10)


def _grouped(ax, groups, values, errs=None, log=False, fmt="{:.2f}", label_all=False):
    """values[planner] -> list aligned with groups; errs[planner] -> (lo, hi) arrays (absolute values)."""
    x = np.arange(len(groups))
    w = 0.38
    for k, pl in enumerate(("astar", "rrt_star")):
        v = np.asarray(values[pl], dtype=float)
        xs = x + (k - 0.5) * w
        ax.bar(xs, v, w * 0.94, color=COLOR[pl], zorder=2)
        if errs is not None and errs.get(pl) is not None:
            lo, hi = errs[pl]
            ax.vlines(xs, lo, hi, color=INK, lw=1.0, zorder=3)
        if label_all:
            for xi, vi in zip(xs, v):
                if np.isfinite(vi):
                    ax.text(xi, vi * (1.08 if log else 1.0) if log else vi, fmt.format(vi), ha="center", va="bottom", fontsize=7,
                            color=INK2, rotation=90 if len(groups) > 4 else 0)
    ax.set_xticks(x)
    ax.set_xticklabels(groups)
    if log:
        ax.set_yscale("log")
    ax.margins(y=0.18)


def stage_a(out: Path) -> pd.DataFrame:
    r = pd.read_csv(RESULTS / "campaign" / "runs.csv")
    mem = pd.read_csv(RESULTS / "campaign" / "memory.csv")
    scs = sorted(r.scenario.unique())
    summary = []

    def per_scn(df, col, agg="median", by_res=True):
        """median per scenario; for A* the spread between its grid resolutions is the whisker (RRT*: interquartile range)."""
        vals, errs = {}, {}
        for pl in ("astar", "rrt_star"):
            v, lo, hi = [], [], []
            for s in scs:
                d = df[(df.scenario == s) & (df.planner == pl)][col].dropna()
                if d.empty:
                    v.append(np.nan); lo.append(np.nan); hi.append(np.nan)
                    continue
                if pl == "astar":
                    per_res = df[(df.scenario == s) & (df.planner == pl)].groupby("resolution")[col].median()
                    v.append(float(d.median())); lo.append(float(per_res.min())); hi.append(float(per_res.max()))
                else:
                    v.append(float(d.median())); lo.append(float(d.quantile(0.25))); hi.append(float(d.quantile(0.75)))
            vals[pl], errs[pl] = v, (np.array(lo), np.array(hi))
        return vals, errs

    ok = r[r.status == "SUCESSO"]
    fig, axs = plt.subplots(3, 3, figsize=(15.5, 11.5))
    fig.suptitle("A* x RRT* - campanha da Etapa A (BiguaSim, seguidor simples): 1152 execuções, RRT* com 30 seeds, A* em 2 resoluções por cenário",
                 x=0.012, ha="left", fontsize=12.5, fontweight="bold", color=INK, y=0.995)

    panels = [
        ("t_first_solution_ms", "Tempo até o primeiro caminho (ms, escala log)", True, "{:.0f}", ok),
        ("replan_ms_mean", "Tempo de replanejamento por evento (ms, escala log)", True, "{:.0f}", ok[ok.n_replans > 0]),
        (None, "Memória de pico do planejador (MB, escala log)", True, "{:.1f}", None),
        ("energy_planned_wh", "Energia planejada da missão (Wh)", False, "{:.1f}", ok),
        ("abs_delta_e_wh", "|ΔE| = |planejada - executada| (Wh)", False, "{:.2f}", ok),
        ("length_exec_m", "Comprimento voado (m)", False, "{:.0f}", ok),
        ("n_waypoints", "Waypoints depois da poda", False, "{:.0f}", ok),
        ("min_clearance_m", "Folga mínima a obstáculos (m)", False, "{:.2f}", ok),
    ]
    for ax, (col, title, log, fmt, df) in zip(axs.flat, panels):
        if col is None:                                          # memory: peak per scenario (A*: the worst resolution)
            vals, errs = {}, {}
            for pl in ("astar", "rrt_star"):
                m = mem[mem.planner == pl]
                vals[pl] = [float(m[m.scenario == s].peak_mem_mb.max()) for s in scs]
                errs[pl] = (np.array([float(m[m.scenario == s].peak_mem_mb.min()) for s in scs]),
                            np.array([float(m[m.scenario == s].peak_mem_mb.max()) for s in scs])) if pl == "astar" else None
            _grouped(ax, scs, vals, errs, log=log, fmt=fmt)
            ax.set_xlabel("A*: barra = pior resolução, traço = variação entre resoluções", fontsize=8, color=INK2)
            for pl in vals:
                for s, v in zip(scs, vals[pl]):
                    summary.append(dict(metric="peak_mem_mb", scenario=s, planner=pl, value=v))
        else:
            vals, errs = per_scn(df, col)
            _grouped(ax, scs, vals, errs, log=log, fmt=fmt)
            if col == "t_first_solution_ms":              # RRT* keeps refining until its fixed time budget: show it as a dashed tick
                bud, _ = per_scn(df, "t_initial_ms")
                for xi, b in zip(np.arange(len(scs)) + 0.19, bud["rrt_star"]):
                    ax.hlines(b, xi - 0.19, xi + 0.19, color=ORANGE, lw=1.6, ls=(0, (3, 2)), zorder=4)
                ax.set_xlabel("tracejado laranja = orçamento fixo do RRT* (5 s por trecho); ele refina até o fim", fontsize=8, color=INK2)
            if col == "replan_ms_mean":
                ax.text(3, ax.get_ylim()[0] * 1.6, "sem replanejamento\n(C4 só em K1/K4)", ha="center", fontsize=8, color=INK2)
            for pl in vals:
                for s, v in zip(scs, vals[pl]):
                    summary.append(dict(metric=col, scenario=s, planner=pl, value=v))
        ax.set_title(title, loc="left")
    # success by expected outcome: SUCESSO except in K6, where ENERGIA_INSUFICIENTE is the correct answer
    ax = axs.flat[8]
    exp = np.where(r.condition == "K6", "ENERGIA_INSUFICIENTE", "SUCESSO")
    r["esperado"] = r.status.values == exp
    rate = {pl: [100 * r[(r.planner == pl) & (r.scenario == s)].esperado.mean() for s in scs] for pl in COLOR}
    _grouped(ax, scs, rate, None, fmt="{:.0f}")
    ax.set_ylim(0, 112)
    ax.set_title("Resultado esperado (%)", loc="left")
    to = r[r.status == "TIMEOUT"].groupby("planner").size().to_dict()
    ax.set_xlabel(f"esperado = SUCESSO (ENERGIA_INSUFICIENTE em K6). TIMEOUT: A* {to.get('astar', 0)}, RRT* {to.get('rrt_star', 0)}", fontsize=8, color=INK2)
    for pl in rate:
        for s, v in zip(scs, rate[pl]):
            summary.append(dict(metric="expected_outcome_pct", scenario=s, planner=pl, value=v))
    _legend(fig, 0.985)
    fig.text(0.012, 0.004, "Barras = mediana por cenário; traço = variação (A*: entre as duas resoluções; RRT*: intervalo interquartil entre seeds). "
             "Tempos de desktop com 1 núcleo, não extrapolar para a placa embarcada. Coeficientes de energia são placeholders.",
             fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.015, 1, 0.965))
    fig.savefig(out / "stage_a_metrics.png", dpi=130)
    plt.close(fig)
    return pd.DataFrame(summary)


def sitl(out: Path) -> pd.DataFrame:
    rb = pd.read_csv(RESULTS / "stage_b" / "runs_b.csv")
    rd = pd.read_csv(RESULTS / "stage_d" / "runs_d.csv")
    metrics = [("mission_time_s", "Tempo de missão (s)", "{:.0f}"), ("flown_length_m", "Comprimento voado (m)", "{:.1f}"),
               ("min_clearance_m", "Folga mínima (m)", "{:.2f}"), ("max_crosstrack_m", "Erro lateral máx. vs plano (m)", "{:.2f}"),
               ("final_error_m", "Erro final ao alvo (m)", "{:.2f}"), ("exec_wh_thrust", "Energia executada (Wh)", "{:.2f}")]
    fig, axs = plt.subplots(2, len(metrics), figsize=(17, 7.2))
    n_b_ok, n_b_col = int((rb.status == "SUCESSO").sum()), int((rb.status == "COLISAO").sum())
    n_d_ok = int((rd.status == "SUCESSO").sum())
    fig.suptitle(f"A* x RRT* nos voos em SITL - ar (Hydrone/ArduCopter): {n_b_ok} de {len(rb)} SUCESSO, {n_b_col} com folga abaixo do raio do veículo; "
                 f"água (BlueROV2/ArduSub): {n_d_ok} de {len(rd)} SUCESSO",
                 x=0.012, ha="left", fontsize=12.5, fontweight="bold", color=INK, y=0.995)
    rows = []
    for i, (df, lab) in enumerate(((rb, "AR"), (rd, "ÁGUA"))):
        for j, (col, title, fmt) in enumerate(metrics):
            ax = axs[i, j]
            for k, pl in enumerate(("astar", "rrt_star")):
                v = df[df.planner == pl][col].astype(float).values
                xs = k + np.linspace(-0.16, 0.16, len(v)) if len(v) > 1 else np.array([float(k)])
                ax.scatter(xs, v, s=26, color=COLOR[pl], edgecolor="#fcfcfb", linewidth=1.0, zorder=3)
                ax.hlines(v.mean(), k - 0.3, k + 0.3, color=INK, lw=1.6, zorder=4)
                ax.text(k + 0.34, v.mean(), fmt.format(v.mean()), va="center", fontsize=8, color=INK2)
                rows.append(dict(medium=lab, metric=col, planner=pl, mean=v.mean(), n=len(v), min=v.min(), max=v.max()))
            ax.set_xlim(-0.6, 1.9)
            ax.set_xticks([0, 1])
            ax.set_xticklabels(["A*", "RRT*"])
            ax.margins(y=0.25)
            ax.set_title(f"{lab}: {title}" if j == 0 else title, loc="left", fontsize=9.5)
    _legend(fig, 0.985)
    fig.text(0.012, 0.006, "Cada ponto é um voo; traço preto = média. Ar: b1_poles e b2_gate em K1 e K2 (A* 1 voo, RRT* 2 seeds por caso). "
             "Água: d1_poles (K1, K2, K5) e d2_gate (K1, K2), RRT* 1 seed. Métricas da verdade do BiguaSim.\n"
             "Eixos não começam em zero. Ar, folga 0,44 m (RRT*, b1_poles K2, seed 1): abaixo dos 0,5 m do raio do veículo, sem evidência de contato.", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.035, 1, 0.965))
    fig.savefig(out / "sitl_metrics.png", dpi=130)
    plt.close(fig)
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(RESULTS / "comparison"))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    sa = stage_a(out)
    ss = sitl(out)
    sa.to_csv(out / "comparison_stage_a.csv", index=False)
    ss.to_csv(out / "comparison_sitl.csv", index=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
