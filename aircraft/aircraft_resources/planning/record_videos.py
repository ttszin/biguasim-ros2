"""Record one successful SITL flight of each planner in the air (Hydrone) and in the water (BlueROV2), K2 (unmapped pole found in flight).

    python3 record_videos.py --out results/videos [--only air_astar water_rrt_star]

Each flight is a full stack run (SITL + BiguaSim + executive, as in Stage B / Stage D) with the runner's `--record`. Output:
<out>/<name>.mp4 plus <out>/runs/<name>/{exec.json, truth.csv, logs}. A flight that does not end in SUCESSO is kept as <name>_failed_N
and retried (`--tries`). The metrics of the recorded flight go to <out>/videos.csv.
"""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
import sys
from pathlib import Path

PLANNING = Path(__file__).resolve().parent
sys.path.insert(0, str(PLANNING / "stage_b"))
sys.path.insert(0, str(PLANNING / "stage_d"))
import run_stage_b as B  # noqa: E402

B_SCEN = B.SCENARIO_DIR
import run_stage_d as D  # noqa: E402  (sets B.SCENARIO_DIR to the stage_d scenarios; restored per flight below)

BLUE, ORANGE = "255,130,70", "60,140,255"      # BGR
FLIGHTS = {
    "air_astar":       dict(stage="B", scenario="b1_poles", planner="astar", cond="K2", res=1.0, label="AR  Hydrone  |  A*  |  K2 (poste oculto)", color=BLUE),
    "air_rrt_star":    dict(stage="B", scenario="b1_poles", planner="rrt_star", cond="K2", res=0.0, label="AR  Hydrone  |  RRT*  |  K2 (poste oculto)", color=ORANGE),
    "water_astar":     dict(stage="D", scenario="d1_poles", planner="astar", cond="K2", res=1.0, label="AGUA  BlueROV2  |  A*  |  K2 (poste oculto)", color=BLUE),
    "water_rrt_star":  dict(stage="D", scenario="d1_poles", planner="rrt_star", cond="K2", res=0.0, label="AGUA  BlueROV2  |  RRT*  |  K2 (poste oculto)", color=ORANGE),
}


def reconstruct_meta(mp4: Path) -> dict:
    """Fallback when the runner died before writing the meta file: video time is simulated time (frames at 25 Hz from t = 0), so the
    route start/end come from the executive's wall-clock stamps mapped through the truth CSV."""
    import numpy as np
    run = mp4.parent / "runs" / mp4.stem
    ex = json.loads((run / "exec.json").read_text())
    tr = np.genfromtxt(run / "truth.csv", delimiter=",", names=True)
    fps = 25
    sim = lambda wall: float(np.interp(wall, tr["wall"], tr["sim_t"]))   # noqa: E731
    dur = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(mp4)], capture_output=True,
                         text=True, check=True).stdout.strip()
    return {"fps": fps, "frames": int(float(dur) * fps), "start_frame": int(sim(ex["wall_start"]) * fps),
            "finish_frame": int(sim(ex.get("wall_end", ex["wall_start"])) * fps)}


def trim(mp4: Path, lead_s: float = 3.0, tail_s: float = 3.0) -> None:
    """Cut the boot/arming wait and the idle tail: from lead_s before the route starts to tail_s after it ends (frames from the meta file)."""
    meta_f = Path(str(mp4) + ".meta.json")
    meta = json.loads(meta_f.read_text()) if meta_f.exists() else reconstruct_meta(mp4)
    if meta["start_frame"] is None:
        return
    fps = meta["fps"]
    t0 = max(0.0, meta["start_frame"] / fps - lead_s)
    t1 = (meta["finish_frame"] if meta["finish_frame"] is not None else meta["frames"]) / fps + tail_s
    tmp = mp4.with_suffix(".trim.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t0:.2f}", "-to", f"{t1:.2f}", "-i", str(mp4), "-c:v", "libx264",
                    "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(tmp)], check=True)
    tmp.replace(mp4)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(PLANNING / "results" / "videos"))
    ap.add_argument("--only", nargs="*", default=list(FLIGHTS))
    ap.add_argument("--tries", type=int, default=3)
    ap.add_argument("--boot-timeout", type=float, default=600.0)
    ap.add_argument("--flight-timeout", type=float, default=1600.0)
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for name in a.only:
        f = FLIGHTS[name]
        for attempt in range(a.tries):
            run_id = name if attempt == 0 else f"{name}_try{attempt + 1}"
            mp4 = out / f"{run_id}.mp4"
            extra = ["--record", str(mp4), "--label", f["label"], "--trail-bgr", f["color"], "--marker-dir", str(out / "runs" / run_id)]
            for m in ("started", "finished"):
                (out / "runs" / run_id / m).unlink(missing_ok=True)
            print(f"[{name}] attempt {attempt + 1}", flush=True)
            mod, seed = (B, 0) if f["stage"] == "B" else (D, 0)
            B.SCENARIO_DIR = B_SCEN if f["stage"] == "B" else D.SCEN
            ex = mod.run_flight(run_id, f["scenario"], f["planner"], f["cond"], seed, f["res"], out, a.boot_timeout, a.flight_timeout,
                                False, runner_extra=extra)
            status = ex["status"] if ex else "NO_RESULT"
            print(f"[{name}] {status} -> {mp4.name}", flush=True)
            if status == "SUCESSO" and mp4.exists() and mp4.stat().st_size > 10_000:
                if run_id != name:
                    shutil.copytree(out / "runs" / run_id, out / "runs" / name, dirs_exist_ok=True)
                    shutil.copy(mp4, out / f"{name}.mp4")
                trim(out / f"{name}.mp4")
                rows.append((name, ex))
                break
    with open(out / "videos.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["video", "scenario", "condition", "planner", "status", "planned_wh", "planned_length_m", "n_replans", "n_waypoints"])
        for name, ex in rows:
            w.writerow([f"{name}.mp4", ex["scenario"], ex["condition"], ex["planner"], ex["status"], round(ex.get("planned_wh", 0), 3),
                        round(ex.get("planned_length_m", 0), 2), len(ex.get("replans", [])), ex.get("n_waypoints", "")])
    print(f"{len(rows)} of {len(a.only)} videos recorded in {out}")


if __name__ == "__main__":
    main()
