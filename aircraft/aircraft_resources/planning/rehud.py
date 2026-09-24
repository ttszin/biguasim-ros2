"""Redraw the header (label, mission clock, height/depth, speed) of a recorded flight video from the flight's ground truth.

    python3 rehud.py water_gate_astar water_gate_rrt_star ...        # in results/videos

Why: the header is burnt into the frames while recording; when a label turned out too long (it collided with the numbers) this
repaints only the top 46 px, using the truth CSV of the run (runs/<name>/truth.csv) and the video's meta file (frame of the route
start; the video was trimmed to start 3 s before it). Video time equals simulated time (frames at 25 Hz from t = 0), so the error is
below one frame period. The label is the one of record_videos.FLIGHTS.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

PLANNING = Path(__file__).resolve().parent
sys.path.insert(0, str(PLANNING))
sys.path.insert(0, str(PLANNING / "stage_b"))
sys.path.insert(0, str(PLANNING / "stage_d"))
from recorder import draw_hud  # noqa: E402

LEAD_S = 3.0


def rehud(mp4: Path, label: str, kind: str, color_bgr: tuple[int, int, int]) -> None:
    run = mp4.parent / "runs" / mp4.stem
    meta = json.loads(Path(str(mp4) + ".meta.json").read_text())
    fps = meta["fps"]
    t_start = meta["start_frame"] / fps                              # route start, in untrimmed video time (= sim time)
    t0 = max(0.0, t_start - LEAD_S)                                  # where the trimmed video begins
    tr = np.genfromtxt(run / "truth.csv", delimiter=",", names=True)
    cap = cv2.VideoCapture(str(mp4))
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    tmp = mp4.with_suffix(".hud.mp4")
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
                           "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(tmp)],
                          stdin=subprocess.PIPE)
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        sim_t = t0 + i / fps
        z = float(np.interp(sim_t, tr["sim_t"], tr["z"]))
        v = float(np.linalg.norm([np.interp(sim_t, tr["sim_t"], tr[k]) for k in ("vx", "vy", "vz")]))
        draw_hud(frame, label, kind, (sim_t - t_start) if sim_t >= t_start else None, z, v, color_bgr)
        ff.stdin.write(frame.tobytes())
        i += 1
    cap.release()
    ff.stdin.close()
    ff.wait()
    tmp.replace(mp4)
    print(f"{mp4.name}: {i} frames redrawn", flush=True)


def main() -> None:
    import record_videos as rv
    root = PLANNING / "results" / "videos"
    for name in sys.argv[1:]:
        f = rv.FLIGHTS[name]
        kind = "air" if name.startswith("air") else "water"
        rehud(root / f"{name}.mp4", f["label"], kind, tuple(int(x) for x in f["color"].split(",")))


if __name__ == "__main__":
    main()
