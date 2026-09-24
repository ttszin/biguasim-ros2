"""Diagnostic: arm, take off to 5 m in GUIDED, hold, report EKF position vs time. Isolates the SITL/BiguaSim setup
from the planners (used to compare runners when a flight misbehaves).

    python3 stage_b/hover_probe.py [--alt 5] [--hold 20]
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sitl_exec import Sitl, bring_up, local_pos, local_vel  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--alt", type=float, default=5.0)
ap.add_argument("--hold", type=float, default=20.0)
ap.add_argument("--settle", type=float, default=0.0, help="seconds to wait after the heartbeat before arming")
a = ap.parse_args()

s = Sitl("tcp:127.0.0.1:5760")
s.m.wait_heartbeat(timeout=300)
t0 = time.time()
while time.time() - t0 < a.settle:
    s.pump()
    time.sleep(0.1)
bring_up(s, a.alt)
t0 = time.time()
last = 0.0
while time.time() - t0 < a.hold:
    s.pump()
    if s.pos is not None and time.time() - last > 2.0:
        last = time.time()
        print(f"t+{time.time() - t0:5.1f}s EKF pos {[round(v, 2) for v in local_pos(s)]} vel {[round(v, 2) for v in local_vel(s)]} mode {s.mode}", flush=True)
    time.sleep(0.1)
print("PROBE_DONE", flush=True)
