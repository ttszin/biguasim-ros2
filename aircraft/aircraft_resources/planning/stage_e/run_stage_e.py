"""T8.2 Stage E: the hybrid air -> water mission in SITL, one plan flown by two vehicles in sequence.

    python3 stage_e/run_stage_e.py --out results/stage_e --planners astar rrt_star [--no-video]

For each planner: (1) plan the WHOLE mission on the hybrid map (stage_e/hybrid.py), (2) fly its AR segment with the Hydrone (ArduCopter SITL,
Stage B stack) to 2 m above the water at the crossing column, (3) model the vertical crossing (not simulated), (4) fly its AGUA segment with the
BlueROV2 (ArduSub SITL, Stage D stack) spawned at that column. Output per planner in <out>/runs/e1_<planner>_{air,water}/ (exec.json, truth.csv, logs),
<out>/e1_<planner>.json (plan + both legs + energy bookkeeping), <out>/videos/e1_<planner>.mp4 (air video + transition card + water video)
and one row per planner in <out>/runs_e.csv.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

STAGE_E = Path(__file__).resolve().parent
PLANNING = STAGE_E.parent
sys.path.insert(0, str(PLANNING))
sys.path.insert(0, str(PLANNING / "stage_b"))
sys.path.insert(0, str(PLANNING / "stage_d"))
sys.path.insert(0, str(STAGE_E))
import hybrid  # noqa: E402
import record_videos as rv  # noqa: E402
import run_stage_b as B  # noqa: E402
import run_stage_d as D  # noqa: E402
from common_b import sitl_config  # noqa: E402
from config import Config  # noqa: E402

from energy import EnergyModel  # noqa: E402
from scenario import load_scenario  # noqa: E402

E1 = STAGE_E / "e1_hybrid.yaml"
BLUE, ORANGE = "255,130,70", "60,140,255"
HOME_OFFSET = [str(v) for v in hybrid.HOME_IN_SURFACE]
ORIGIN = [str(v) for v in hybrid.ORIGIN_BSIM]


def _card(path: Path, lines: list[str]) -> None:
    """Title card 1280x720: first line large, the rest smaller; every line is scaled down until it fits with a margin."""
    img = np.zeros((720, 1280, 3), np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    y = 250
    for k, txt in enumerate(lines):
        size = 1.25 if k == 0 else 0.8
        while cv2.getTextSize(txt, font, size, 2)[0][0] > 1180 and size > 0.3:
            size -= 0.05
        (tw, _), _ = cv2.getTextSize(txt, font, size, 2)
        cv2.putText(img, txt, ((1280 - tw) // 2, y), font, size, (255, 255, 255), 2, cv2.LINE_AA)
        y += 90 if k == 0 else 55
    cv2.imwrite(str(path), img)


def _join(air: Path, water: Path, card_lines: list[str], out: Path) -> None:
    card_png, card_mp4 = out.with_suffix(".card.png"), out.with_suffix(".card.mp4")
    _card(card_png, card_lines)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-t", "4", "-i", str(card_png), "-r", "25", "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", str(card_mp4)], check=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(air), "-i", str(card_mp4), "-i", str(water), "-filter_complex",
                    "[0:v][1:v][2:v]concat=n=3:v=1:a=0[v]", "-map", "[v]", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(out)], check=True)
    card_png.unlink(missing_ok=True)
    card_mp4.unlink(missing_ok=True)


def card_lines(plan: dict) -> list[str]:
    tr = plan["energy_wh_by_medium"].get("TRANSICAO", 0.0)
    return ["TRANSICAO AR -> AGUA", "modelada pelo planejador, NAO simulada em SITL",
            f"descida vertical de {hybrid.HOVER_Z:.0f} m acima da agua a {abs(hybrid.SPAWN_Z):.0f} m de profundidade "
            f"({plan['transition_m']:.0f} m a 0.5 m/s), {tr:.3f} Wh modelados",
            "o Hydrone (ArduCopter) para aqui; o BlueROV2 (ArduSub) assume na mesma coluna"]


def run_pair(planner: str, out: Path, cond: str, seed: int, video: bool, boot_timeout: float, air_timeout: float, water_timeout: float) -> dict:
    pid = f"e1_{planner}"
    rundir = out / "runs"
    legs = out / "legs" / pid
    res = 1.0 if planner == "astar" else 0.0
    color = BLUE if planner == "astar" else ORANGE
    name = "A*" if planner == "astar" else "RRT*"
    plan = hybrid.plan_hybrid(E1, planner, seed, cond, resolution=res or 1.0)
    rec: dict = {"planner": planner, "condition": cond, "seed": seed, "plan": plan}
    print(f"[{pid}] hybrid plan: {plan['status']}"
          + (f", crossing column {np.round(plan['entry'], 1).tolist()}, {plan['energy_wh_total']:.2f} Wh planned" if "entry" in plan else ""), flush=True)
    if "entry" not in plan:
        return rec
    info = hybrid.write_legs(E1, plan, legs)
    rec["legs"] = info
    vids = out / "videos"
    vids.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------ AR: Hydrone
    B.SCENARIO_DIR = legs
    air_id = f"{pid}_air"
    air_mp4 = vids / f"{air_id}.mp4"
    extra = ["--frame-origin-bsim", *ORIGIN]
    if video:
        extra += ["--record", str(air_mp4), "--label", f"AR Hydrone | {name} | plano hibrido (K2)", "--trail-bgr", color, "--cam-preset", "air_high",
                  "--marker-dir", str(rundir / air_id)]
        for m in ("started", "finished"):
            (rundir / air_id / m).unlink(missing_ok=True)
    print(f"[{pid}] air leg (Hydrone) ...", flush=True)
    ex_air = B.run_flight(air_id, "air_leg", planner, cond, seed, res or 1.0, out, boot_timeout, air_timeout, False, runner_extra=extra,
                          exec_extra=["--frame-offset", *HOME_OFFSET, "--frame-origin-bsim", *ORIGIN, "--no-land"])
    rec["air"] = ex_air
    print(f"[{pid}] air leg: {ex_air['status'] if ex_air else 'NO_RESULT'} {(ex_air or {}).get('detail', '')}", flush=True)
    if not ex_air or ex_air["status"] != "SUCESSO":
        return rec

    # ------------------------------------------------------------------ AGUA: BlueROV2 at the crossing column
    D.SCEN = legs
    water_id = f"{pid}_water"
    water_mp4 = vids / f"{water_id}.mp4"
    sp = info["spawn_bsim"]
    extra = ["--spawn", *[str(v) for v in sp]]
    if video:
        extra += ["--record", str(water_mp4), "--label", f"AGUA BlueROV2 | {name} | plano hibrido (K2)", "--trail-bgr", color,
                  "--marker-dir", str(rundir / water_id)]
        for m in ("started", "finished"):
            (rundir / water_id / m).unlink(missing_ok=True)
    sn, se, su = info["spawn"]
    print(f"[{pid}] water leg (BlueROV2) from {np.round(info['spawn'], 1).tolist()} ...", flush=True)
    ex_water = D.run_flight(water_id, "water_leg", planner, cond, seed, res or 1.0, out, boot_timeout, water_timeout, False, runner_extra=extra,
                            exec_extra=["--spawn-n", str(sn), "--spawn-e", str(se), "--spawn-up", str(su)])
    rec["water"] = ex_water
    print(f"[{pid}] water leg: {ex_water['status'] if ex_water else 'NO_RESULT'} {(ex_water or {}).get('detail', '')}", flush=True)

    if video and ex_water and air_mp4.exists() and water_mp4.exists():
        rv.trim(air_mp4)
        rv.trim(water_mp4)
        card = card_lines(plan)
        _join(air_mp4, water_mp4, card, vids / f"{pid}.mp4")
        print(f"[{pid}] video {vids / (pid + '.mp4')}", flush=True)
    return rec


def analyse_pair(rec: dict, out: Path) -> dict:
    """One summary row: the plan (by medium) against what each leg spent, from BiguaSim's ground truth."""
    pid = f"e1_{rec['planner']}"
    plan = rec["plan"]
    row = {"planner": rec["planner"], "condition": rec["condition"], "plan_status": plan["status"]}
    if "entry" not in plan:
        return row
    by = plan["energy_wh_by_medium"]
    row.update(entry_north=round(plan["entry"][0], 2), entry_east=round(plan["entry"][1], 2), planned_wh_air=round(by.get("AR", 0), 3),
               planned_wh_transition=round(by.get("TRANSICAO", 0), 3), planned_wh_water=round(by.get("AGUA", 0), 3),
               planned_wh_total=round(plan["energy_wh_total"], 3), plan_time_ms=round(plan["planning_s"] * 1000, 1),
               planned_length_m=round(plan["path_length_m"], 1))
    legs = out / "legs" / pid
    for key, cfg, sc_file, extra in (("air", sitl_config(), "air_leg.yaml", None), ("water", Config.load(D.CFG), "water_leg.yaml", "water")):
        ex = rec.get(key)
        row[f"{key}_status"] = ex["status"] if ex else "NO_RESULT"
        if not ex:
            continue
        run_dir = out / "runs" / f"{pid}_{key}"
        sc = load_scenario(legs / sc_file)
        if key == "water" and (run_dir / "truth.csv").exists():
            D.add_thrust_column(run_dir / "truth.csv") if "thrust_n" not in open(run_dir / "truth.csv").readline() else None
        r = B.analyse(run_dir, ex, cfg, EnergyModel.from_config(cfg), sc)
        row.update({f"{key}_{k}": r[k] for k in ("exec_wh_thrust", "flown_length_m", "mission_time_s", "n_replans", "min_clearance_m", "collided",
                                                   "max_crosstrack_m", "final_error_m")})
    try:
        row["exec_wh_total_with_modelled_transition"] = round(float(row["air_exec_wh_thrust"]) + float(row["water_exec_wh_thrust"]) + by.get("TRANSICAO", 0), 3)
    except (KeyError, ValueError, TypeError):
        pass
    return row


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(PLANNING / "results" / "stage_e"))
    ap.add_argument("--planners", nargs="*", default=["astar", "rrt_star"], choices=["astar", "rrt_star"])
    ap.add_argument("--condition", default="K2", choices=["K1", "K2"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--boot-timeout", type=float, default=600.0)
    ap.add_argument("--air-timeout", type=float, default=900.0)
    ap.add_argument("--water-timeout", type=float, default=1600.0)
    ap.add_argument("--analyse-only", action="store_true")
    ap.add_argument("--rejoin-videos", action="store_true", help="only rebuild <out>/videos/e1_<planner>.mp4 from the trimmed leg videos")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if a.rejoin_videos:
        for pl in a.planners:
            rec = json.loads((out / f"e1_{pl}.json").read_text())
            v = out / "videos"
            _join(v / f"e1_{pl}_air.mp4", v / f"e1_{pl}_water.mp4", card_lines(rec["plan"]), v / f"e1_{pl}.mp4")
            print("rejoined", v / f"e1_{pl}.mp4")
        return
    rows = []
    for pl in a.planners:
        f = out / f"e1_{pl}.json"
        if a.analyse_only and f.exists():
            rec = json.loads(f.read_text())
        else:
            rec = run_pair(pl, out, a.condition, a.seed, not a.no_video, a.boot_timeout, a.air_timeout, a.water_timeout)
            f.write_text(json.dumps(rec))
        rows.append(analyse_pair(rec, out))
    keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 999)
    with open(out / "runs_e.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print("wrote", out / "runs_e.csv")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
