"""Orchestrate T8.2 Stage B: BiguaSim + ArduPilot SITL (GUIDED), one fresh stack per flight.

    python3 stage_b/run_stage_b.py --out results/stage_b --scenarios b1_poles b2_gate \
        --conditions K1 K2 --rrt-seeds 2                     # the campaign
    python3 stage_b/run_stage_b.py --out results/stage_b_smoke --scenarios b1_poles --conditions K1 \
        --planners astar --rrt-seeds 0                        # one flight, to check the setup
    python3 stage_b/run_stage_b.py --out results/stage_b --analyse-only    # rebuild CSV/figures from saved runs

For each flight it starts ArduPilot SITL (aircraft_resources/missions/t2_sitl_run.sh), the BiguaSim runner
(sitl_runner.py) and the executive (sitl_exec.py), then tears everything down. Per flight it saves
<out>/runs/<id>/{exec.json, truth.csv, *.log}; runs_b.csv gets one row per flight.

Metrics come from BiguaSim's ground truth (not the EKF): flown length, minimum clearance and collisions against the
real obstacles, cross-track error against the plan, and the energy from the thrust the plant actually applied.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common_b import HOME_BSIM, MISSIONS_DIR, PLANNING, SCENARIO_DIR, from_bsim, sitl_config  # noqa: E402

from energy import EnergyModel, J_PER_WH  # noqa: E402
from scenario import load_scenario  # noqa: E402

STAGE_B = Path(__file__).resolve().parent
PY = sys.executable
FIELDS = ["run_id", "scenario", "condition", "planner", "resolution", "seed", "status", "detail",
          "t_initial_ms", "n_waypoints", "planned_wh", "exec_wh_thrust", "delta_e_thrust_wh", "exec_wh_kinematic",
          "delta_e_kin_wh", "planned_length_m", "flown_length_m", "mission_time_s", "n_replans", "replan_ms_mean",
          "replan_ms_max", "stops_at_waypoints", "min_clearance_m", "collided", "max_crosstrack_m",
          "mean_speed_mps", "max_speed_mps", "final_error_m"]


# ----------------------------------------------------------------- process control

def _cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
    except OSError:
        return ""


def kill_leftovers() -> None:
    """Kill stray SITL / runner processes (own processes only). pkill is blocked in this sandbox, os.kill is not."""
    # never kill ourselves or any ancestor (e.g. the shell whose command line mentions these names)
    protected, pid = set(), os.getpid()
    while pid > 1 and pid not in protected:
        protected.add(pid)
        try:
            pid = int(Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    for p in Path("/proc").iterdir():
        if not p.name.isdigit() or int(p.name) in protected:
            continue
        c = _cmdline(int(p.name))
        if any(k in c for k in ("arducopter", "ardusub", "rov_runner.py", "rov_exec.py", "sim_vehicle.py", "sitl_runner.py", "sitl_exec.py", "t2_sitl_run.sh",
                                "biguasim_sim_runner", "hover_probe.py", "Linux/Biguasim/Binaries")):
            try:
                os.kill(int(p.name), signal.SIGKILL)
            except OSError:
                pass
    time.sleep(1.0)


def start(cmd: list[str], log: Path, cwd: Path | None = None) -> subprocess.Popen:
    return subprocess.Popen(cmd, stdout=open(log, "w"), stderr=subprocess.STDOUT, cwd=cwd, start_new_session=True)


def stop(p: subprocess.Popen | None) -> None:
    if p is None or p.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 20), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        try:
            os.killpg(p.pid, sig)
        except OSError:
            return
        t0 = time.time()
        while time.time() - t0 < wait:
            if p.poll() is not None:
                return
            time.sleep(0.2)


def wait_log(path: Path, needle: str, timeout: float, proc: subprocess.Popen | None = None) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if path.exists() and needle in path.read_text(errors="ignore"):
            return True
        if proc is not None and proc.poll() is not None:
            return False
        time.sleep(1.0)
    return False


# --------------------------------------------------------------------------- one flight

def run_flight(run_id: str, scenario: str, planner: str, cond: str, seed: int, res: float, out: Path,
               boot_timeout: float, flight_timeout: float, viewport: bool, runner_extra: list[str] | None = None) -> dict | None:
    d = out / "runs" / run_id
    d.mkdir(parents=True, exist_ok=True)
    scen_path = SCENARIO_DIR / f"{scenario}.yaml"
    sitl = runner = None
    try:
        kill_leftovers()
        sitl = start(["bash", str(MISSIONS_DIR / "t2_sitl_run.sh")], d / "sitl.log", cwd=MISSIONS_DIR)
        time.sleep(8.0)
        rcmd = [PY, str(STAGE_B / "sitl_runner.py"), "--scenario", str(scen_path), "--log", str(d / "truth.csv")]
        if viewport:
            rcmd.append("--viewport")
        rcmd += runner_extra or []
        runner = start(rcmd, d / "runner.log")
        if not wait_log(d / "runner.log", "STAGE_B_RUNNER_READY", boot_timeout, runner):
            print(f"  [{run_id}] BiguaSim did not come up (see {d / 'runner.log'})", flush=True)
            return None
        ecmd = [PY, str(STAGE_B / "sitl_exec.py"), "--scenario", str(scen_path), "--planner", planner, "--condition", cond,
                "--seed", str(seed), "--resolution", str(res), "--out", str(d / "exec.json")]
        ex = start(ecmd, d / "exec.log")
        t0 = time.time()
        while ex.poll() is None and time.time() - t0 < flight_timeout:
            time.sleep(1.0)
        if ex.poll() is None:
            stop(ex)
            print(f"  [{run_id}] executive timed out", flush=True)
        return json.loads((d / "exec.json").read_text()) if (d / "exec.json").exists() else None
    finally:
        stop(runner)
        stop(sitl)
        kill_leftovers()


# ------------------------------------------------------------------------------ analysis

def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def analyse(run_dir: Path, ex: dict, cfg, energy: EnergyModel, sc) -> dict:
    row = {k: "" for k in FIELDS}
    row.update(run_id=run_dir.name, scenario=ex["scenario"], condition=ex["condition"], planner=ex["planner"],
               resolution=ex.get("resolution") or "", seed=ex["seed"], status=ex["status"], detail=ex.get("detail", ""),
               t_initial_ms=ex.get("t_initial_ms", ""), n_waypoints=ex.get("n_waypoints", ""),
               planned_wh=ex.get("planned_wh", ""), planned_length_m=ex.get("planned_length_m", ""))
    truth_csv = run_dir / "truth.csv"
    if "wall_start" not in ex or not truth_csv.exists():
        return row
    tr = np.genfromtxt(truth_csv, delimiter=",", names=True)
    t0, t1 = ex["wall_start"], ex.get("wall_end", ex["wall_start"])
    tr = tr[(tr["wall"] >= t0) & (tr["wall"] <= t1)]
    if len(tr) < 5:
        return row
    home = tuple(ex.get("home_bsim", HOME_BSIM))
    loc = np.array([from_bsim(x, y, z, home) for x, y, z in zip(tr["x"], tr["y"], tr["z"])])
    vel = np.stack([tr["vx"], -tr["vy"], tr["vz"]], axis=1)              # north, east, up
    dt = np.diff(tr["wall"], prepend=tr["wall"][0])
    dt[0] = 0.0
    sim_dt = np.diff(tr["sim_t"], prepend=tr["sim_t"][0])
    sim_dt[0] = 0.0

    # thrust-based energy: P(T) with the planner's air curves, T = the thrust BiguaSim applied
    c2, c1, c0 = cfg.get("energy", ex.get("medium", "air"))["coeffs"]
    thrust = tr["thrust_n"]
    e_thrust = float(np.sum((c2 * thrust ** 2 + c1 * thrust + c0) * sim_dt)) / J_PER_WH
    # kinematic energy from ground-truth velocity/acceleration with the planner's own model
    acc = np.gradient(vel, np.maximum(tr["sim_t"], 0), axis=0) if len(tr) > 3 else np.zeros_like(vel)
    e_kin = float(sum(energy.instant_power(loc[i, 2], vel[i], acc[i]) * sim_dt[i] for i in range(len(tr)))) / J_PER_WH

    truth_world = sc.truth_world(0.0)
    clr = truth_world.clearance(loc)
    speed = np.linalg.norm(vel, axis=1)
    hist = ex.get("path_history", [])
    xt = np.nan
    if hist:
        paths = [np.array(h["path"]) for h in hist]
        d_all = np.full(len(loc), np.inf)
        for path in paths:
            for a, b in zip(path[:-1], path[1:]):
                d_all = np.minimum(d_all, _seg_dist(loc, a, b))
        xt = float(np.max(d_all))
    replans = ex.get("replans", [])
    goal = sc.waypoints[-1]
    changes = [r for r in replans if r.get("kind") == "mission_change"]
    if changes:                                   # K5: the goal that counts is the one the mission was changed to
        goal = np.array(changes[-1]["goal"], dtype=float)
    planned = float(ex.get("planned_wh", np.nan))
    row.update(
        exec_wh_thrust=round(e_thrust, 4), delta_e_thrust_wh=round(planned - e_thrust, 4),
        exec_wh_kinematic=round(e_kin, 4), delta_e_kin_wh=round(planned - e_kin, 4),
        flown_length_m=round(float(np.sum(np.linalg.norm(np.diff(loc, axis=0), axis=1))), 2),
        mission_time_s=round(float(tr["sim_t"][-1] - tr["sim_t"][0]), 1),
        n_replans=len(replans), replan_ms_mean=round(float(np.mean([r["ms"] for r in replans])), 1) if replans else "",
        replan_ms_max=round(float(np.max([r["ms"] for r in replans])), 1) if replans else "",
        stops_at_waypoints=ex.get("stopped_at_waypoints", ""), min_clearance_m=round(float(clr.min()), 2),
        collided=int(clr.min() < float(cfg.get("margins", "vehicle_radius"))), max_crosstrack_m=round(xt, 2),
        mean_speed_mps=round(float(speed.mean()), 2), max_speed_mps=round(float(speed.max()), 2),
        final_error_m=round(float(np.linalg.norm(loc[-1] - goal)), 2))
    if row["collided"] and row["status"] == "SUCESSO":
        row["status"] = "COLISAO"
    np.save(run_dir / "flown_local.npy", np.column_stack([tr["sim_t"], loc, speed, thrust]))
    return row


def figures(out: Path, rows: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    figdir = out / "figures"
    figdir.mkdir(exist_ok=True)
    for name in sorted({r["scenario"] for r in rows}):
        sc = load_scenario(SCENARIO_DIR / f"{[p for p in SCENARIO_DIR.glob('*.yaml') if load_scenario(p).name == name][0].name}")
        mine = [r for r in rows if r["scenario"] == name and (out / "runs" / r["run_id"] / "flown_local.npy").exists()]
        if not mine:
            continue
        fig, (ax, az, av) = plt.subplots(1, 3, figsize=(17, 5.5), gridspec_kw={"width_ratios": [0.9, 1.1, 1.1]})
        for o in sc.known + sc.hidden:
            solid = o in sc.known
            kw = dict(color="0.35") if solid else dict(fill=False, ec="0.35", ls=":", lw=1.3)
            if hasattr(o, "radius"):
                ax.add_patch(Circle((o.east, o.north), o.radius, **kw))
                n0, n1, u0, u1 = o.north - o.radius, o.north + o.radius, o.z_min, o.z_max
            else:
                ax.add_patch(Rectangle((o.e_min, o.n_min), o.e_max - o.e_min, o.n_max - o.n_min, **kw))
                n0, n1, u0, u1 = o.n_min, o.n_max, o.u_min, o.u_max
            lo, hi = sc.bounds[2]
            az.add_patch(Rectangle((n0, max(u0, lo - 1)), n1 - n0, min(u1, hi + 1) - max(u0, lo - 1), **kw))
        for r in mine:
            d = out / "runs" / r["run_id"]
            fl = np.load(d / "flown_local.npy")
            ex = json.loads((d / "exec.json").read_text())
            color = "tab:blue" if r["planner"] == "astar" else "tab:orange"
            label = f"{r['planner']} {r['condition']}" + (f" s{r['seed']}" if r["planner"] == "rrt_star" else "")
            ax.plot(fl[:, 2], fl[:, 1], "-", color=color, alpha=0.7, lw=1.4, label=label)
            az.plot(fl[:, 1], fl[:, 3], "-", color=color, alpha=0.7, lw=1.4)
            for h in ex.get("path_history", [])[:1]:
                pth = np.array(h["path"])
                ax.plot(pth[:, 1], pth[:, 0], "o--", color=color, alpha=0.35, ms=3, lw=0.8)
            av.plot(fl[:, 0] - fl[0, 0], fl[:, 4], color=color, alpha=0.7, lw=1.2, label=label)
        for w in sc.waypoints:
            ax.plot(w[1], w[0], "k*", ms=11)
        ax.set_xlim(*sc.bounds[1]); ax.set_ylim(*sc.bounds[0]); ax.set_aspect("equal")
        ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")
        ax.set_title(f"{name}: top view, flown (solid) vs first plan (dashed)")
        ax.legend(fontsize=6, ncol=2)
        az.set_xlim(*sc.bounds[0]); az.set_ylim(sc.bounds[2][0] - 1, sc.bounds[2][1] + 1)
        az.axhspan(sc.bounds[2][0], sc.bounds[2][1], color="tab:green", alpha=0.06)
        az.set_xlabel("north (m)"); az.set_ylabel("z (m): Stage B above home, Stage D water surface = 0"); az.set_title("side view (z vs north); green = allowed band")
        av.set_xlabel("time (s)"); av.set_ylabel("ground speed (m/s)"); av.set_title("speed (BiguaSim ground truth)")
        fig.tight_layout()
        fig.savefig(figdir / f"stage_b_{name}.png", dpi=120)
        plt.close(fig)


def write_notes(out: Path, rows: list[dict]) -> None:
    import pandas as pd

    r = pd.DataFrame(rows)
    for c in ("planned_wh", "exec_wh_thrust", "delta_e_thrust_wh", "flown_length_m", "planned_length_m", "mission_time_s",
              "n_replans", "min_clearance_m", "max_crosstrack_m", "mean_speed_mps", "max_speed_mps", "final_error_m",
              "stops_at_waypoints", "replan_ms_max"):
        r[c] = pd.to_numeric(r[c], errors="coerce")
    r["dE_pct"] = 100.0 * r["delta_e_thrust_wh"] / r["planned_wh"]
    coll = r[r["collided"].astype(str) == "1"]
    lines = [
        "**Como foi rodado.** Cada voo sobe uma pilha nova: ArduCopter SITL (o mesmo `t2_sitl_run.sh` e `t2_biguasim.parm` do T2), "
        "BiguaSim (mundo SkyDive/Bridge, Hydrone = perfil DjiMatrice) e um executivo em Python que fala MAVLink direto com o SITL "
        "(pymavlink, GUIDED, `SET_POSITION_TARGET_LOCAL_NED`). O executivo usa os mesmos planejadores, mapa, modelo de energia "
        "e replanejadores da Etapa A. **Não usa MAVROS nem o `mission_node`.** A percepção de obstáculos continua simulada "
        "(pontos de superfície dentro de 12 m da posição do EKF); os obstáculos existem de verdade no BiguaSim como props.",
        "",
        "**Só a parte aérea**, 4 a 9 m acima do home, no corredor x = 8…48 m, y ≈ 0 do mundo Bridge. A parte subaquática não foi testada em SITL.",
        "",
        "**Métricas** vêm da verdade do BiguaSim (não do EKF). A energia executada usa o empuxo total que a planta aplicou "
        "(`Σ k_eta·ω²`, k_eta = 6,64e-5) nas mesmas curvas de potência do planejamento (coeficientes placeholder; o BiguaSim não tem modelo de potência). "
        "ΔE = planejada − executada, então **ΔE negativo significa que o voo gastou mais que o plano**.",
        "",
        f"**Voos:** {len(r)}; concluídos sem colisão: {int((r['status'] == 'SUCESSO').sum())}; "
        f"folga mínima abaixo do raio do veículo (0,5 m): {len(coll)}.",
        "",
    ]
    for pl, g in r.groupby("planner"):
        lines.append(f"* **{'A*' if pl == 'astar' else 'RRT*'}** ({len(g)} voos): energia planejada {g.planned_wh.mean():.2f} Wh, executada "
                     f"{g.exec_wh_thrust.mean():.2f} Wh (ΔE {g.dE_pct.mean():+.0f} %), comprimento voado {g.flown_length_m.mean():.1f} m "
                     f"(planejado {g.planned_length_m.mean():.1f} m), tempo {g.mission_time_s.mean():.1f} s, folga mínima média {g.min_clearance_m.mean():.2f} m, "
                     f"erro lateral máximo médio {g.max_crosstrack_m.mean():.2f} m, erro final médio {g.final_error_m.mean():.2f} m.")
    for cond, g in r.groupby("condition"):
        lines.append(f"* **{cond}** ({len(g)} voos): {g.n_replans.mean():.1f} replanejamentos por voo, tempo {g.mission_time_s.mean():.1f} s, "
                     f"ΔE {g.dE_pct.mean():+.0f} %" + (f", tempo máximo de replanejamento {g.replan_ms_max.max():.0f} ms." if g.replan_ms_max.notna().any() else ", sem replanejamento."))
    lines += [
        f"* **Velocidade:** média {r.mean_speed_mps.mean():.2f} m/s com picos de {r.max_speed_mps.max():.1f} m/s, acima dos 2 m/s do `WPNAV_SPEED` do T2. "
        f"O plano usa 2,5 m/s, calibrado no primeiro voo (`stage_b_calibration`, que **não** entra na tabela: só os voos seguintes dizem algo sobre ΔE).",
        f"* **Paradas em waypoints (stop-and-go):** {int(r.stops_at_waypoints.sum())} nos {len(r)} voos; nos demais o ArduPilot fez fly-by, com queda de velocidade no waypoint.",
    ]
    if len(coll):
        for _, c in coll.iterrows():
            lines.append(f"* **Folga abaixo do raio do veículo:** `{c.run_id}` ({c.n_replans:.0f} replanejamentos, folga mínima {c.min_clearance_m:.2f} m). "
                         "O obstáculo oculto é revelado só em parte (pontos de superfície dentro do raio), o plano passa por trás do que ainda não foi visto e cada nova "
                         "porção corta o caminho outra vez: houve uma sequência de replanejamentos em poucos segundos com o veículo já dentro da zona de folga, "
                         "e o GUIDED não para de imediato. Não há evidência de contato físico, só de que a margem de segurança foi violada.")
    lines += [
        "",
        "**Rodada anterior (`stage_b_run1_low_floor`, piso de 2 m):** 2 dos 12 voos do RRT\\* ficaram ~70 s parados (velocidade média 0,5 m/s, "
        "energia 3 a 7 Wh) depois de o plano descer a ~3,5 m do home. O ArduPilot registrou `Yaw Imbalance 38–50 %` e o veículo só seguiu depois. "
        "A causa provável é geometria real do Bridge perto do deck (o home fica sobre a ponte), ausente do mapa dos planejadores; não confirmei o objeto. "
        "A campanha desta seção usa piso de 4 m.",
        "",
        "**Ajustes no ambiente que foram necessários** (detalhes no README): (1) o giro de guinada estava invertido para todos os veículos por uma alteração minha "
        "anterior no BiguaSim, o que derrubava o Hydrone; (2) a ponte BiguaSim→ArduPilot manda `[lat, lon, alt]` no campo `position`, que o ArduPilot lê como NED em metros "
        "(o runner do Stage B reescreve o campo); (3) o SITL só arma com giroscópio ≥ 216 Hz, então o runner usa 250 ticks/s.",
    ]
    (out / "NOTES.md").write_text("\n".join(lines) + "\n")


def collect(out: Path) -> list[dict]:
    cfg = sitl_config()
    energy = EnergyModel.from_config(cfg)
    rows = []
    for d in sorted((out / "runs").glob("*")):
        f = d / "exec.json"
        if not f.exists():
            continue
        ex = json.loads(f.read_text())
        sc = load_scenario([p for p in SCENARIO_DIR.glob("*.yaml") if load_scenario(p).name == ex["scenario"]][0])
        rows.append(analyse(d, ex, cfg, energy, sc))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(PLANNING / "results" / "stage_b"))
    ap.add_argument("--scenarios", nargs="*", default=["b1_poles", "b2_gate"])
    ap.add_argument("--conditions", nargs="*", default=["K1", "K2"])
    ap.add_argument("--planners", nargs="*", default=["astar", "rrt_star"], choices=["astar", "rrt_star"])
    ap.add_argument("--rrt-seeds", type=int, default=2)
    ap.add_argument("--astar-resolution", type=float, default=1.0)
    ap.add_argument("--boot-timeout", type=float, default=480.0)
    ap.add_argument("--flight-timeout", type=float, default=600.0)
    ap.add_argument("--viewport", action="store_true")
    ap.add_argument("--analyse-only", action="store_true")
    ap.add_argument("--retries", type=int, default=1, help="extra attempts if the stack fails to come up")
    a = ap.parse_args()

    out = Path(a.out)
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
        with open(out / "runs_b.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            w.writeheader()
            w.writerows(rows)
        figures(out, rows)
        write_notes(out, rows)
        print(f"wrote {out}/runs_b.csv, figures/, NOTES.md ({len(rows)} flights)")


if __name__ == "__main__":
    main()
