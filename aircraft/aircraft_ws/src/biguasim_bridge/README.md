# biguasim_bridge

ROS2 Humble bridge for the T2 aerial/aquatic transition mission (lab tracker #1449).
Republishes BiguaSim pressure/ROV telemetry onto the ROS2 graph and forwards ROV
waypoint commands back, so the rest of the T2 stack (`autopilot_interface`,
`mission`, MAVROS) can run under Humble instead of the host's native Jazzy
install.

## Why this package exists

The T2 mission (`aircraft_resources/missions/t2_land_test.yaml`) validates
that a hybrid aerial/aquatic drone can detect crossing the water surface (via
a simulated pressure sensor) and switch between `AERIAL_NAV` and
`AQUATIC_NAV`, landing correctly on either water or a platform. It had been
implemented and demoed on video running entirely on the host under ROS2
Jazzy (Ubuntu 24.04). The lab reviewer asked for it to be redone "bringing
ROS2 Humble and using our package" (referencing `hydrone-furg/biguasim-ros2`'s
`biguasim_main`, the project's documented ROS2 bridge).

`biguasim_main` couldn't be used directly: it drives BiguaSim with raw motor
commands and has no ArduPilot integration, while T2 requires ArduPilot as the
authority for the aerial/aquatic mode transition. So instead this package
follows `biguasim_main`'s structural conventions (a small `ament_python`
package, a sensor→encoder dispatch table) without depending on its code.

Two constraints shaped the split:

- **Python ABI**: `rclpy` under Humble is built against Python 3.10 (Ubuntu
  22.04); the `biguasim` package requires Python ≥3.11. They cannot share a
  process under Humble (they happened to on the host only because Jazzy/Ubuntu
  24.04 ships Python 3.12 for both).
- **No Ubuntu 24.04 build of Humble**: the new ROS2 package has to run in a
  container (the existing `aircraft-image`, Ubuntu 22.04 + Humble).

## Architecture

```
HOST (Ubuntu 24.04, native, unchanged)          CONTAINER (aircraft-image, Humble, --network host)
  ArduCopter SITL (JSON FDM :9002, MAVLink :5760)  MAVROS  (tcp://127.0.0.1:5760)
  biguasim_sim_runner.py                           autopilot_interface (/Drone0)
    - ArduBiguaSimRunner loop (unchanged)           biguasim_bridge
    - UDP -> container: pressure_pa,                  - /fcu/external_pressure, /nav_mode,
      rov_position (~50Hz, :9100)                       /bluerov0/local_position
    - UDP <- container: /bluerov0/cmd_pos_yaw           - hysteresis+debounce -> /nav_mode
      (:9101)                                         - /bluerov0/cmd_pos_yaw -> UDP to host
                                                    mission (/Drone0, --conops t2_land_test.yaml)
```

Run order: `t2_sitl_run.sh` → `biguasim_sim_runner.py [--viewport]` → the
Humble container via `t2_aircraft.yml.erb`. `--network host` avoids Docker
networking entirely — MAVROS and the UDP telemetry sockets talk to
`127.0.0.1` exactly as they would on bare host.

## Prerequisites

None of this comes from just cloning the repo — before "How to run" below will
work, you need:

- **BiguaSim installed natively on the host** (not in a container), with a
  Python ≥3.11 interpreter (see "Why this package exists" above for why).
- **ArduPilot built on the host**, specifically the `ArduCopter` SITL binary
  that `t2_sitl_run.sh` launches.
- **A GPU-capable machine.** BiguaSim/Unreal Engine renders real frames for
  the RGBCamera sensor even without `--viewport` (which only toggles whether
  a window is shown) — no GPU, no camera data, no detections.
- **The `aircraft-image` Docker image, built locally** — this repo's
  `docker run` commands assume the image already exists, they don't build it.
  Build (or rebuild, after pulling any change touching `aircraft_ws/src` or
  `aircraft_resources/patches`) with:
  ```bash
  docker build -f tools_and_docs/docker/aircraft.dockerfile -t aircraft-image .
  ```
  This takes a while and pulls in a lot (CUDA base image, MAVROS, YOLO/ONNX,
  etc.) — budget real time for the first build.

## How to run

Three terminals, from the repo root, in order:

```bash
# 1. Host, native — ArduCopter SITL
bash aircraft/aircraft_resources/missions/t2_sitl_run.sh

# 2. Host, native — BiguaSim (add --viewport to see the Unreal Engine window)
python3 aircraft/aircraft_resources/missions/biguasim_sim_runner.py --viewport

# 3. Container — MAVROS + autopilot_interface + biguasim_bridge + mission (Humble)
docker run --rm -it --network host \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  --entrypoint bash aircraft-image \
  -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"
```

Command 3 opens a tmux session with 4 windows (`mavros`, `ardupilot_interface`,
`biguasim_bridge`, `mission`) — switch with `Ctrl-b` + window number. It waits
for MAVROS to report `connected: true` and `system_status: 3` before starting
`mission` (see bug #2 below), so there's nothing to race against.

Which mission conops runs is controlled by the `T2_CONOPS` env var (default
`t2_land_test.yaml`), e.g.:

```bash
docker run --rm -it --network host \
  -e T2_CONOPS=/aas/aircraft_resources/missions/vision_land_test.yaml \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  --entrypoint bash aircraft-image \
  -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"
```

`T2_CONOPS=... docker run ...` (setting it as a plain shell prefix, no `-e`)
looks like it should work but silently doesn't — it only sets the variable in
the host shell running the `docker run` command itself, never inside the
container, so the conops quietly falls back to the `t2_land_test.yaml`
default with no error. `-e T2_CONOPS=...` is required to actually pass it
through (confirmed the hard way — a hover test run kept executing
`t2_land_test.yaml`'s `descend_to_water` step instead).

Available conops in `aircraft_resources/missions/`: `t2_land_test.yaml` (the
original aerial/aquatic transition), plus two rounds of vision testing —
detection first, landing control second (kept as separate steps/missions on
purpose, so a bad detection can't be confused with a bad control loop):

- **Detection only**, no `vision_land` step, regular guided `land` at the end:
  `vision_detect_boat_test.yaml` (BlueBoat target) — takeoff, hover over the
  target, watch `/detections` and `/biguasim/camera/image` (or
  `--show-camera`) to validate detection quality before trusting it to fly
  anything.
- **Hover stability** (no vision target involved): `t2_hover_test.yaml` — see
  "Hover stability and the vertical velocity profile" below.
- **Detection + landing control**: `vision_land_test.yaml` (platform) and
  `vision_land_boat_test.yaml` (BlueBoat) — adds the `vision_land` centering/
  descent step. **Not yet tested live** — see the commit history for status.

The `*boat*` conops need `biguasim_sim_runner.py` started with `--spawn-boat`
(step 2 above) so the BlueBoat actually gets spawned — its own built-in
helipad marking (square deck, circle+cross touchdown mark) is what
`ShapeTargetDetector`'s trained YOLO model detects, no synthetic target
needed. (A `--landing-target` flag used to spawn a synthetic gold sphere as
an alternative/fallback target — removed, since the trained detector only
recognizes the real BlueBoat shape and never matched a sphere anyway.)

Add `--show-camera` to that same command for a live `cv2.imshow` debug window
with detection overlays, independent of the `/biguasim/camera/image` ROS2
topic that `biguasim_bridge` always republishes.

## Precision-landing vision: from classical CV to a custom YOLO detector

`marker_detector.py`'s `ShapeTargetDetector` (used by `biguasim_sim_runner.py`
for the `*boat*` conops above) went through three different approaches before
landing on the one now in the tree.

### 1. Classical CV shape detection — tried, didn't hold up

The first approach avoided any ML: ellipse-fit the BlueBoat's helipad marking
(circle+cross) directly out of contours, using a 4th-harmonic FFT signature
on the contour's radius profile to confirm the cross and reject anything
merely round. Explicitly *not* color-based from the start — the request was
for lighting-independent detection, and material color under BiguaSim's
renderer drifts with lighting far more than silhouette does.

This worked at close range, but water reflections/glints kept producing
false positives that passed the same shape filters as the real marking
(a bright specular patch on water can be circular and even carry enough
internal structure to fool a cross-signature check). Position-history jump
filtering, size-ratio rejection, and confidence-decay prediction (all still
present in `ShapeTargetDetector` today, now wrapping the YOLO output instead)
cut down the false positives but never eliminated them, and detection range
stayed short — the marking's fine detail (the cross) needs enough resolved
pixels to tell apart from noise, which runs out well before the boat itself
would otherwise still be visible. A follow-up attempt replaced the marking
detector with a whole-hull HSV-saturation silhouette detector
(`BoatSilhouetteDetector`, since removed — see git history) to get more
resolvable pixels at range; it worked close in but broke down further out,
where a single Otsu threshold split grabbed the water's own large-scale
brightness gradient instead of the hull, and `adaptiveThreshold` alone was
too noisy from water texture to reliably separate the boat from spurious
candidates.

**Why we didn't keep pushing on classical CV**: every fix targeted a
specific failure mode observed in one test frame and reliably introduced or
re-exposed a different one elsewhere (tightening thresholds to reject a
reflection made real detections drop out at range; loosening a filter to
recover range brought reflections back) — a sign the feature (raw
shape/threshold statistics) doesn't actually separate "boat" from "water
glint" well enough on its own, not that the parameters were merely
untuned.

### 2. Pretrained COCO YOLOv8n — tried, dead on arrival

Before building a custom dataset, we checked whether an existing pretrained
detector could sidestep the reflection problem entirely. Ultralytics'
COCO-pretrained `yolov8n.pt` was tested against a real, easy reference frame
(`template_blueboat.png`, boat filling a large fraction of the frame) at
confidence thresholds down to 0.01 — it detected nothing, of any class. COCO's
"boat" class is trained on eye-level/ground photography; a small twin-hull
robot seen from directly overhead doesn't resemble any COCO training example
closely enough to match, so this wasn't a tuning problem either.

### 3. Custom-trained YOLOv8n — what's in the tree now

Since no existing detector fit this exact camera geometry, we collected and
labeled a purpose-built dataset and trained our own single-class detector:

- **`collect_boat_dataset.py`** — teleports the camera agent to randomized
  positions around the stationary BlueBoat (altitude sampled first, then
  orbit radius capped to a fraction of that altitude so the boat reliably
  stays inside the camera's footprint) and saves each frame plus the exact
  camera/boat 3D pose to `metadata.jsonl`. Holds the boat still via
  `set_physics_state(..., velocity=[0,0,0], angular_velocity=[0,0,0])` every
  tick — `teleport()` alone let it drift/sink over the course of a long
  collection run.
- **`label_boat_dataset.py`** — no visual detection involved: projects the
  boat's *known* 3D position through a calibrated pinhole camera model
  (using the recorded camera pose from `metadata.jsonl`) to compute an exact
  YOLO-format bounding box for every frame. Ground truth by construction,
  not by annotation.
- **`prepare_yolo_dataset.py`** — splits the labeled set into Ultralytics'
  expected `images/{train,val}` + `labels/{train,val}` layout and writes
  `data.yaml`.
- Trained with `yolo detect train data=dataset_yolo/data.yaml
  model=yolov8n.pt epochs=100 imgsz=640 batch=16
  name=blueboat_detector` (500 images, 450 train / 50 val) — held-out
  validation metrics: precision=1.0, recall=0.999, mAP50=0.995,
  mAP50-95=0.913.

`ShapeTargetDetector` now runs this model instead of the classical pipeline,
keeping the exact same `detect(frame) -> [(cx, cy, w, h, class_id,
confidence)]` interface and the same position-history/size-ratio filtering
as before — a drop-in swap, no other file needed to change.

The dataset (`dataset_raw/`, `dataset_yolo/`, ~500MB) and trained weights
(`runs/`) are **not committed** (see `.gitignore`) — regenerate with:

```bash
cd aircraft/aircraft_resources/missions
python3 collect_boat_dataset.py --num-images 500 --seed 42
python3 label_boat_dataset.py --dataset-dir dataset_raw --preview 10   # spot-check dataset_raw/preview/ before trusting the full set
python3 prepare_yolo_dataset.py
yolo detect train data=dataset_yolo/data.yaml model=yolov8n.pt epochs=100 imgsz=640 batch=16 name=blueboat_detector
```

`ShapeTargetDetector`'s `DEFAULT_WEIGHTS_PATH` expects the result at
`runs/detect/blueboat_detector/weights/best.pt` (Ultralytics' own default
output path for that `name=`), so no code changes are needed after
retraining.

### Running the live test

With a trained model at the path above, `--spawn-boat` alone is enough to
see it work (no SITL/mission container needed — this just drives the camera
agent directly and shows detections):

```bash
cd aircraft/aircraft_resources/missions

# Static hover directly over the boat
python3 biguasim_sim_runner.py --viewport --show-camera \
  --spawn-boat --location 33 0 6

# Or a continuous all-around look instead of one fixed angle
python3 biguasim_sim_runner.py --viewport --show-camera \
  --spawn-boat --orbit
```

`--show-camera` opens a `cv2.imshow` window with the live detection overlay
(green box + confidence). For the full mission (takeoff, fly to the boat,
center, land), run `vision_detect_boat_test.yaml` (detection only) or
`vision_land_boat_test.yaml` (detection + landing) via the three-terminal
flow in "How to run" above, with `biguasim_sim_runner.py` started using the
same `--spawn-boat` flags.

## Hover stability and the vertical velocity profile

**Status: blocked on a spurious RTL failsafe, not yet tested end to end.**
The original GPU driver/library mismatch that blocked this (`nvidia-smi`
failing, BiguaSim hanging on launch) is resolved (fixed by a reboot). Two
more issues surfaced while actually trying to run `t2_hover_test.yaml` live:

1. **`docker run -d` needs `-t`, not just `-d`.** `t2_aircraft.yml.erb`'s
   tmuxinator script ends with `tmux -u attach-session` (see its `if [ -z
   "$TMUX" ]` branch) — that call needs a real pty. Without `-t`, it fails
   immediately, PID 1 exits, and Docker kills the whole container within
   seconds (silently, if also using `--rm`) — every service inside (mavros,
   `ardupilot_interface`, `mission`) dies with it, way before anything can
   complete. Use `docker run --rm -d -t ...` (or the already-documented
   `--rm -it` for a real interactive terminal) — never bare `-d`.
2. **`T2_CONOPS=... docker run ...` (no `-e`) silently does nothing.** Fixed
   in "How to run" above — this cost real time here: a "hover test" run kept
   silently executing the default `t2_land_test.yaml` instead (only
   noticeable by reading `mission`'s pane and seeing `descend_to_water`
   instead of `wait`).

With both of those fixed, `t2_hover_test.yaml` reached takeoff and started
toward its waypoint (`Mission step: 1`) — but repeatedly, across multiple
runs, ArduCopter autonomously switched to native `RTL` mode (confirmed via
`/mavros/state`'s `mode` field) before the vehicle made real lateral
progress, paired every time with `"HEARTBEAT timed out"` in the `mavros`
pane's log. One occurrence completed a normal, controlled RTL landing back at
the spawn point (0.4 m/s descent, no crash) before being investigated
further; a later run oscillated between `RTL` and reconnecting every
10-20 seconds without ever reaching AERIAL_NAV's actual waypoint. Confirmed
directly in `ardupilot_interface.cpp` that nothing in it commands RTL outside
its `Land` action's own state machine (never invoked here) — this is a native
ArduCopter failsafe, not our code.

**Root cause, found by reading ArduPilot's own SITL source
(`libraries/AP_HAL_SITL/SITL_cmdline.cpp`)**: ArduCopter's SITL scheduler
expects physics/FDM state updates at `SIM_RATE_HZ` — **400Hz by default**
(Copter's compiled-in assumption) — but BiguaSim only ever delivers ~200Hz
(`biguasim_sim_runner.py`'s `--ticks` default, literally the Unreal Engine
process's own `-TicksPerSec=200`). ArduCopter was always waiting for state
updates twice as fast as BiguaSim can actually produce them, which is exactly
what `Main loop slow (200Hz < 400Hz)` and `Gyro 0 rate 203Hz < loop
rate*1.8 720Hz` were reporting the whole time, on every run, not just the
ones that escalated to a failed heartbeat. Under any additional jitter
(Unreal rendering load, concurrent `docker exec` diagnostic polling during a
run, general host CPU contention) this chronic mismatch was apparently enough
to delay MAVLink heartbeat delivery past ArduCopter's failsafe threshold,
triggering the RTL failsafe over and over.

**Fix, part 1**: `t2_sitl_run.sh` now passes `-A "--rate 200"` to
`sim_vehicle.py`, which sets `SIM_RATE_HZ=200` on the `arducopter` binary
(confirmed in the same SITL_cmdline.cpp — `-r`/`--rate` sets exactly this).
**Tested live — insufficient on its own**: the `Main loop slow` warning kept
appearing even with this set. Reading `AP_Arming.cpp`'s actual check
(`system_checks()`) showed why: it compares against
`AP::scheduler().get_loop_rate_hz()`, which reads a *different*,
independent parameter — `AP_Scheduler.cpp`'s `LOOP_RATE` GROUPINFO, i.e.
**`SCHED_LOOP_RATE`** (default 400, `SCHEDULER_DEFAULT_LOOP_RATE`). `SIM_RATE_HZ`
only governs how fast the SITL backend paces itself; it does NOT change what
the flight-control scheduler itself expects for its own health/arming check.

**Fix, part 2**: added `SCHED_LOOP_RATE` to `t2_biguasim.parm` alongside
`SIM_RATE_HZ` (`t2_sitl_run.sh`'s `-A "--rate ..."`) — both rates now need to
match BiguaSim's real delivery, not just one of them. **Tested live — matching
both to 200 still wasn't enough**: the warning changed from `Main loop slow
(200Hz < 400Hz)` to `Main loop slow (152Hz < 200Hz)` — i.e. 200 was only
BiguaSim's *requested* tick rate (`biguasim_sim_runner.py`'s `--ticks`
default), not what it actually sustains with `--viewport` (full Unreal Engine
rendering) on this machine: the real delivered rate measured live is
**~150-160Hz**.

**Fix, part 3 (current)**: both `SCHED_LOOP_RATE` (`t2_biguasim.parm`) and
`-A "--rate ..."` (`t2_sitl_run.sh`) set to **150** — comfortable margin under
the observed 150-160Hz ceiling, since arming requires actual ≥ 90% of the
configured value. If this machine's real achievable rate turns out to be
different, or `--viewport` is ever dropped (headless likely sustains a higher
real rate, since it skips rendering — not yet measured), both places must be
updated together. `FS_GCS_ENABLE 0` / `FS_THR_ENABLE 0` (added before the
rate-mismatch root cause was found — disable the GCS-heartbeat and RC
failsafes ArduCopter falls back to under a genuinely degraded link) are kept
as a second line of defense, since a real SITL is never going to have a
legitimate RC/GCS-hardware failure to detect anyway.

**Fix, part 4**: `--viewport` tested as a variable — removing it did NOT
change the observed real rate at all (still exactly ~133Hz with or without
Unreal rendering, and stable across very different GPU temperatures, 65°C vs
86°C), ruling out both rendering load and GPU thermal throttling as the
bottleneck. Both `SCHED_LOOP_RATE` and `-A "--rate ..."` set to **120**
(margin under the observed ~133Hz ceiling, whatever its real source — most
likely the Python/PyTorch dynamics step itself or the ArduBridge UDP
round-trip, not yet pinned down further).

With that in place, a run armed successfully in well under a minute — much
faster than every earlier attempt — but still hit `mode: RTL` shortly after
`Mission step: 1`, before completing `wait_to_reach_position`. Investigated
the actual `mission_node.py`/`t2_hover_test.yaml` design at that point:
`wait_to_reach_position` (Mission step 2) isn't itself broken — it's used
successfully in the already-validated `t2_land_test.yaml`, just late in that
mission (its PHASE 5, several minutes into the flight, well after everything
has settled), not immediately after the very first takeoff. `t2_hover_test.yaml`
had it right after takeoff instead, keeping the mission actively
GUIDED-navigating through exactly the fragile early-flight window where
arming itself was already observed to take a while to stabilize. Removed
it — `t2_hover_test.yaml`'s PHASE 2 now mirrors `t2_land_test.yaml`'s own
proven PHASE 2 exactly (`go_to_known_gps_waypoint` + a fixed `wait`, no
polling step), to eliminate that as a variable before assuming the rate fix
alone is (or isn't) sufficient.

**Confirmed live, working end to end**, with rates at 120 and
`wait_to_reach_position` removed from the early mission: armed in well under
a minute, reached `Mission step: 2` (the hover step) without an intervening
RTL, held the full 180s hover, then `land` (its `landing_altitude: 0.0`
→ RTL-based landing sequence) completed normally — armed went back to
`false` back at the spawn point, no crash, no manual intervention.

**One more real bug found and fixed while running the logger**:
`log_hover_stability.py` (run as originally documented, on the host) hung
indefinitely with zero output — `ros2 topic info --verbose
/mavros/local_position/odom` from the host showed `Topic type hash: INVALID`
on both the publisher and subscriber sides, meaning cross-distro DDS
discovery between the host's Jazzy and the container's Humble reports the
topic as "visible" but never actually delivers messages to a host-side
subscriber (confirmed: `ardupilot_interface`, running inside the same Humble
container as the publisher, receives this exact topic fine). The script also
had two independent bugs this exposed: (1) its duration/timeout check was
gated behind receiving the first message, so with no data ever arriving it
hung forever instead of erroring out — fixed by starting the wall-clock timer
unconditionally in `__init__` and checking the timeout regardless of data
receipt; (2) the CSV file was never flushed until close, so there was no way
to check a still-running process's progress on disk — fixed with a `flush()`
after every row. **Run the logger from inside the Humble container**
(`docker cp` it in, then `docker exec` with `source
/opt/ros/humble/setup.bash`), not from the host, until/unless the DDS
type-hash mismatch itself gets root-caused.

**Real baseline result** (180s hover, 20s settle excluded, 1600 samples,
`hover_baseline.csv` in the repo root): horizontal position stddev
**0.000 m**, vertical position stddev **0.199 m**, peak |roll| **0.09°**,
peak |pitch| **0.07°**. Horizontal and attitude are excellent — comfortably
inside the proposed criteria (< 0.3 m, < 3°). Vertical stddev is slightly
above the proposed 0.15 m threshold, but close enough that running
`AUTOTUNE` doesn't look justified from this one baseline — the default
ArduCopter gains plus the already-tuned `MOT_THST_HOVER`/`MOT_THST_EXPO`
produce a stable hover as-is.

### How to test, once SITL is available

**1. Hover baseline** — same three-terminal flow as "How to run" above, with
`t2_hover_test.yaml` as the conops, plus the logger in a 4th terminal:

```bash
# Terminal 1
bash aircraft/aircraft_resources/missions/t2_sitl_run.sh

# Terminal 2
python3 aircraft/aircraft_resources/missions/biguasim_sim_runner.py --viewport

# Terminal 3
docker run --rm -it --network host \
  -e T2_CONOPS=/aas/aircraft_resources/missions/t2_hover_test.yaml \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  --entrypoint bash aircraft-image \
  -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"

# Terminal 4, once takeoff completes (hover lasts 180s) — run the logger
# INSIDE the container, not on the host: cross-distro DDS discovery between
# the host's Jazzy and the container's Humble reports mavros topics as
# visible but never actually delivers messages to a host-side subscriber
# (confirmed live — see the "Real baseline result" note above). Get the
# container ID from `docker ps`, then:
docker cp aircraft/aircraft_resources/missions/log_hover_stability.py <CONTAINER_ID>:/tmp/log_hover_stability.py
docker exec -d <CONTAINER_ID> bash -c "source /opt/ros/humble/setup.bash; cd /tmp && python3 log_hover_stability.py --duration 180 --settle 20 --out /tmp/hover_baseline.csv > /tmp/hover_log.log 2>&1"
# ... wait ~180s, then:
docker exec <CONTAINER_ID> cat /tmp/hover_log.log   # prints the summary
docker cp <CONTAINER_ID>:/tmp/hover_baseline.csv .  # copy the raw CSV out
```

Compare the printed summary against the proposed criteria below; only run
`AUTOTUNE` and re-measure if it doesn't already pass.

**2. Vertical velocity ramp** — ⚠️ **`t2_land_test.yaml` currently has an
unresolved regression, found live while trying to set up this exact A/B
test**: during `descend_to_water` (Mission step 3), even with the unmodified
(no-ramp) mission code, the vehicle flew off at ~14 m/s horizontal airspeed
to 46m north instead of holding the commanded zero horizontal velocity at the
17m waypoint — visually confirmed flying into the bridge structure. Not yet
root-caused; the leading suspect is the BiguaSim GPS-glitch-at-bridge-deck-
level behavior `EK3_GLITCH_RAD`/`EK3_ALT_M_NSE` were originally tuned for
(see their comments above) no longer being adequately filtered under the
`SCHED_LOOP_RATE`/`SIM_RATE_HZ=120` change from this same session (previously
validated at the 400Hz default). **Do not run `t2_land_test.yaml` unattended
until this is root-caused** — it may fly the vehicle into the bridge deck.

Same Terminal 1/2, but Terminal 3 runs the
default conops (`t2_land_test.yaml`, no `T2_CONOPS` override needed) and the
logger (inside the container, same as above) runs through the whole mission
to capture both transitions:

```bash
# Terminal 3
docker run --rm -it --network host \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  --entrypoint bash aircraft-image \
  -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"

# logger, inside the container, --duration 300 --settle 0 --out land_test_with_ramp.csv
```

Confirm the mission still completes exactly as in "Verified" below, then
check `land_test_with_ramp.csv` for the peak roll/pitch right after each
`descend_to_water`/`ascend_from_water` transition — that's what actually
validates whether the ramp helped.

**3. BlueBoat YOLO detector** (already tested live once — see "Precision-landing
vision" above — this is just the reference command):

```bash
python3 aircraft/aircraft_resources/missions/biguasim_sim_runner.py --viewport --show-camera --spawn-boat --orbit
```

### Measuring hover stability

`t2_hover_test.yaml` (takeoff 5m → fly to a real waypoint, north=17 — the
same one `t2_land_test.yaml` already validates — → hover there 180s → land)
existed already (originally just took off and hovered in place, with no
transit) but its stability had only ever been eyeballed, not measured.
`t2_biguasim.parm` only
tunes `MOT_THST_HOVER`/`MOT_THST_EXPO` (motor↔thrust mapping) and EKF/waypoint
tolerances — no attitude-rate or position-loop gain (`ATC_RAT_*`, `PSC_*`) has
ever been tuned for this simulated airframe.

`log_hover_stability.py` (new, `aircraft/aircraft_resources/missions/`) is a
standalone rclpy script — no changes needed to `mission_node.py` or
`ardupilot_interface.cpp` — since `/mavros/local_position/odom` and
`/mavros/imu/data` are already published by default in this stack (neither is
in `apm_pluginlists.yaml`'s denylist). It samples both at 10Hz, writes a CSV,
and prints a summary (horizontal/vertical position stddev, peak roll/pitch)
over the window after an excluded post-takeoff settle period. Run it in its
own terminal alongside `t2_hover_test.yaml`:

```bash
python3 aircraft/aircraft_resources/missions/log_hover_stability.py --duration 180 --settle 20 --out hover_baseline.csv
```

Proposed stability criteria (adjust after seeing real numbers): horizontal
position stddev < 0.3 m, vertical < 0.15 m, peak roll/pitch < 3° after
settling. If the baseline already meets this, no gain tuning is needed — kept
as a documented result rather than tuning gains without evidence they help. If
not, the next step is ArduCopter's built-in `AUTOTUNE` flight mode (the
standard way to find `ATC_RAT_{RLL,PIT,YAW}_*` gains, rather than manual
trial-and-error) run via MAVROS, with the resulting gains copied into
`t2_biguasim.parm` and the baseline re-measured to confirm improvement.

### Vertical velocity profile: build vs configure

`descend_to_water`/`ascend_from_water`/`ascend_to_altitude` (`mission_node.py`)
all drive vertical motion through `SetReposition`'s GPS-free velocity branch
(`ardupilot_interface.cpp`'s `set_reposition_callback`, `altitude > 10.0`
sentinel) — a raw `vel.z` setpoint sent directly over
`/mavros/setpoint_raw/local`, bypassing ArduCopter's native climb/descent
shaping (`PILOT_SPEED_UP/DN`, `WPNAV_ACCEL_Z`) entirely. This is a deliberate
"build" choice, not an oversight: `descend_to_water`/`ascend_from_water` need
to operate without a reliable GPS fix near the water surface, and
`vision_land` needs a continuously-variable, externally-computed velocity (a
P-controller on visual centering error) — neither is possible through native
firmware parameters, which only shape a fixed target under normal
GPS-position-controlled navigation. **Decision: kept "build".** `PILOT_SPEED_UP/DN`/`WPNAV_ACCEL_Z` were not adopted.

What *was* a real gap: the commanded velocity used to jump in a single
instantaneous step (0 → ±2.0 m/s in one `SetReposition` call) — a hard step
input to the position/attitude controller, a plausible source of a pitch/roll
transient right at the transition instant (exactly the kind of event
`log_hover_stability.py` above is built to catch). Added a client-side ramp
(`_ramped_vertical_velocity`/`_vertical_velocity_resend_interval` in
`mission_node.py`): the same three call sites now ramp linearly from 0 to the
±2.0 m/s target over 1.5s (resent every 0.2s during the ramp), then fall back
to the pre-existing 10s keep-alive resend once at full velocity — reuses
`SetReposition`/`_call_service_no_advance` as-is, no service or C++ change.
`descend_to_water` now goes through the same `altitude=15.0` GPS-free velocity
branch as ascend (with a negative `vertical_velocity`) instead of its old
`altitude=-10.0` sentinel — that branch hardcoded `vel.z=-2.0` in C++ with no
variable-velocity field to ramp.

**Validation still needed** (blocked on the GPU issue above): re-run
`t2_land_test.yaml` end to end (must still complete exactly as before — see
"Verified" below) and compare `log_hover_stability.py`'s peak-attitude number
at the transition instant with/without the ramp. If it doesn't measurably
help, the added complexity should come back out rather than being kept on
faith.

## Position-hold stability for other BiguaSim vehicles: BlueBoat (Rover)

**Status: confirmed live, working end to end.** The hover-stability work above
only ever validated the DjiMatrice (ArduCopter). Extending the same
measurement to BiguaSim's other vehicle profiles turned out to need real new
code, not just a re-run: `ardupilot_interface.cpp` had **zero** existing
handling for `MAV_TYPE_GROUND_ROVER`/`MAV_TYPE_SUBMARINE` — every `mav_type_`
branch in the file only ever checked for `1` (fixed-wing/VTOL) or `2`
(Multicopter), several with **no default case**, meaning an unhandled
`mav_type_` silently hung forever instead of failing. Given the size of
supporting all four non-Copter profiles (BlueROV2/BlueROVHeavy/TorpedoAUV on
ArduSub, BlueBoat on Rover — two new firmware types, missing `.parm` files,
no multi-instance-SITL convention), scope was deliberately narrowed to the
simplest one first: **BlueBoat (Rover)** — no vertical axis, no
submersion/buoyancy semantics, just 2D position hold on the water surface.

### What was built

- `t2_sitl_run_blueboat.sh` (new) — launches Rover SITL (`sim_vehicle.py -v
  Rover --model JSON`), mirroring `t2_sitl_run.sh`. No `-A "--rate ..."`
  override, unlike the ArduCopter script: confirmed in
  `ardupilot/libraries/AP_Scheduler/AP_Scheduler.cpp:44-47` that
  `SCHEDULER_DEFAULT_LOOP_RATE` is conditionally 400 for Copter/Heli/ArduSub
  but **50 for Rover** — already comfortably below BiguaSim's real delivery
  rate, so the loop-rate mismatch that broke DjiMatrice arming doesn't apply.
- `t2_biguasim_blueboat.parm` (new) — starts empty, same iterative philosophy
  as `t2_biguasim.parm` originally did (add a parameter only once a live run
  shows a concrete need for it).
- `biguasim_sim_runner_blueboat.py` (new) — standalone script that runs the
  BlueBoat as the actual ArduPilot-bridged vehicle (`control_abstraction:
  "cmd_motor_speeds"`, per `VEHICLE_REGISTRY["BlueBoat"]`), unlike
  `biguasim_sim_runner.py`'s own `blueboat0` agent, which is a decorative,
  non-SITL `cmd_pos_yaw`-controlled landing-target prop. No subclassing
  needed — `ArduBiguaSimRunner`'s base `run()` loop is already fully
  vehicle-agnostic (see its own module docstring in
  `biguasim/src/biguasim/ardubridge/runner.py`).
- `ardupilot_interface.cpp`: a new `mav_type_ == 10` branch in
  `takeoff_handle_accepted`, added *alongside* the existing Multicopter/VTOL
  branches, not replacing them — reuses the `Takeoff` action/message as-is
  (Rover has no real "takeoff", so this branch just arms and enters GUIDED,
  then declares the action complete; `takeoff_altitude` is accepted but
  ignored). Also widened `set_reposition_callback`'s guard to accept
  `mav_type_==10` (reusing `ARMED` as Rover's "ready" state — safe to share
  with the Multicopter path since `mav_type_` is fixed for the life of one
  node/one SITL session) and its existing `GlobalPositionTarget`/GUIDED
  branch, which Rover firmware simply ignores the altitude field of. Added an
  explicit `else` for genuinely unsupported `mav_type_` values, so a future
  untested vehicle type fails loudly instead of hanging.
- `mission_node.py`: `go_to_known_gps_waypoint`'s `DRONE_TYPE` gate widened
  from `'quad'`-only to `('quad', 'rover')`.
- `t2_aircraft.yml.erb`: new `T2_DRONE_TYPE` env var (defaults to `'quad'`,
  same `ENV.fetch` pattern as `T2_CONOPS`), threaded into the mission
  process's `DRONE_TYPE` env var.
- `t2_rover_hold_test.yaml` (new) — `takeoff` (arm+GUIDED, reused) →
  `go_to_known_gps_waypoint` (north=5, small since the water area is
  narrower than the aerial one) → `wait` 180s. No disarm/land step: Rover has
  no land-equivalent FSM path yet, and it isn't needed to measure hold
  stability.
- `log_hover_stability.py` needed **no changes at all** — already fully
  vehicle-agnostic (just reads `/mavros/local_position/odom` +
  `/mavros/imu/data`).

### Three more infrastructure bugs found live (none in the new Rover logic itself)

1. **Rover never reports `MAV_STATE_STANDBY` (3).** Confirmed live:
   `mode: MANUAL`, `system_status: 4` (ACTIVE) from startup, disarmed — Rover
   firmware doesn't have Copter's distinct "standby until armed" state. This
   silently blocked mission start twice: `t2_aircraft.yml.erb`'s own
   `until ... system_status: 3` gate (widened to `system_status: (3|4)`), and
   `takeoff_handle_goal`'s `if (mav_state_ != 3) reject` (widened to also
   accept `mav_type_==10 && mav_state_==4`).
2. **`mav_type_` would never get populated for Rover at all.**
   `ardupilot_interface_printout_callback`'s one-time `VehicleInfoGet` query
   (the *only* place `mav_type_` is ever set, from its initial `-1`) was
   gated on the same `mav_state_ == 3` — same fix, widened to `3 || 4`.
   Without this, every `mav_type_`-gated branch in the file, including the
   new Rover one, would never execute regardless of the other two fixes.
3. **New mission YAML wasn't visible inside the container.** `docker run`
   only had `t2_aircraft.yml.erb` and the two edited ROS2 package `src/`
   dirs volume-mounted — `aircraft_resources/missions/` (where
   `t2_rover_hold_test.yaml` lives) is otherwise baked into the image at
   build time, so a brand-new file there is invisible until the image is
   rebuilt. `mission_node.py` failed to load it, silently fell back to an
   empty mission plan, and immediately logged "Mission Complete" with zero
   steps executed — nothing about this looked like a missing-file error at a
   glance. Fixed by also mounting `aircraft_resources/missions/` for testing
   against source that hasn't been baked into the image yet (see "How to
   test" below).

### Real baseline result

180s hold, 20s settle excluded, 1601 samples
(`blueboat_hold_baseline.csv` in the repo root):

| Metric | Result | Same criteria as DjiMatrice |
|---|---|---|
| Horizontal position stddev | **0.001 m** | < 0.3 m |
| Vertical position stddev | **0.015 m** | < 0.15 m |
| Peak \|roll\| | **0.16°** | < 3° |
| Peak \|pitch\| | **0.15°** | < 3° |

Passes comfortably on all four — vertical stability is noticeably better than
the DjiMatrice's (0.015 m vs 0.199 m), consistent with a boat on a water
surface not needing to fight gravity/thrust balance the way a hovering
multirotor does.

### How to test

Same three-terminal shape as the DjiMatrice tests, but note the extra volume
mounts (source hasn't been rebuilt into the image yet) and the explicit
rebuild step:

```bash
# Terminal 1
bash aircraft/aircraft_resources/missions/t2_sitl_run_blueboat.sh

# Terminal 2
python3 aircraft/aircraft_resources/missions/biguasim_sim_runner_blueboat.py --viewport

# Terminal 3
docker run --rm -it --network host \
  -e T2_CONOPS=/aas/aircraft_resources/missions/t2_rover_hold_test.yaml \
  -e T2_DRONE_TYPE=rover \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  -v $(pwd)/aircraft/aircraft_ws/src/autopilot_interface:/aas/aircraft_ws/src/autopilot_interface \
  -v $(pwd)/aircraft/aircraft_ws/src/mission:/aas/aircraft_ws/src/mission \
  -v $(pwd)/aircraft/aircraft_resources/missions:/aas/aircraft_resources/missions \
  --entrypoint bash aircraft-image \
  -c "source /opt/ros/humble/setup.bash && cd /aas/aircraft_ws && colcon build --packages-select autopilot_interface mission && source install/setup.bash && tmuxinator start -p /aas/t2_aircraft.yml.erb"

# logger, inside the container once armed (docker cp it in first, same as the ramp A/B test method above)
```

Once this Rover work is merged into a fresh image build, the extra
`autopilot_interface`/`mission`/`aircraft_resources/missions` volume mounts
and manual rebuild step are no longer needed — same one-line `docker run`
as the DjiMatrice flow.

### Out of scope for this round

BlueROVHeavy, TorpedoAUV — deferred until BlueROV2 (below) is fully
validated, since BlueROVHeavy shares ArduSub with it and TorpedoAUV uses a
different `control_abstraction` (`cmd_rudders_sterns_motor_speed`) needing
its own investigation.

## Position-hold for BlueROV2 (ArduSub) — code done, SITL live validation NOT achieved

**Status: `ardupilot_interface.cpp`/`mission_node.py`/scripts all implemented
and code-reviewed correct. Live GUIDED-mode position-hold over a real
ArduPilot SITL bridge was never confirmed — every attempt (12+ across two
sessions, including two different `control_abstraction` workarounds) hit
BiguaSim/Unreal Engine hanging or crashing during agent spawn, before or
shortly after MAVROS could exercise GUIDED. This is a BiguaSim engine bug,
not something in this repo. The one thing confirmed live and stable is
BiguaSim's own non-SITL, teleport-driven `bluerov0` prop (`cmd_pos_yaw`,
`biguasim_sim_runner.py`, the same agent already used in earlier T2 work)
holding a fixed position — see "What was actually confirmed" below for
exactly what that does and doesn't prove. See below for the full
investigation, what was ruled out, and what's still needed.**

Investigated first (two Explore-agent passes, file:line-grounded) whether
ArduSub needs a structurally different mechanism than Rover did. Findings,
all confirmed correct by the live testing below: BiguaSim's synthetic GPS
works underwater unconditionally (`bridge.py`'s `build_json_state()` always
includes `"position"`; `include_depth_sensor` only adds a `"pressure"` field
alongside it, never replaces position); ArduSub's own `vehicle_system_status()`
(`GCS_MAVLink_Sub.cpp:58-73`) reports STANDBY(3)→ACTIVE(4) like Copter, not
Rover's always-ACTIVE surprise; `SET_POSITION_TARGET_GLOBAL_INT` in GUIDED
treats `alt` as standard z_up (so a depth target is just a negative altitude,
no sign inversion needed); `ModeGuided::init()` needs no dive sequencing;
`SCHEDULER_DEFAULT_LOOP_RATE` (`AP_Scheduler.cpp:44-47`) is 400 for ArduSub,
same as Copter (unlike Rover's 50).

### What was built

- `t2_sitl_run_bluerov2.sh` (new) — mirrors `t2_sitl_run.sh`'s Copter pattern
  (`-A "--rate 120"`), not Rover's no-fix approach, per the loop-rate finding
  above.
- `t2_biguasim_bluerov2.parm` (new) — `SCHED_LOOP_RATE 120` set proactively
  (not left for live discovery, unlike the Rover case, since the framework
  code already confirmed the need). Also carries `FS_EKF_ACTION 0`/
  `EK3_CHECK_SCALE 200`/`EK3_GLITCH_RAD 5`, the same EKF-health relaxation
  `t2_biguasim.parm` uses for the DjiMatrice — added after finding the GUIDED
  block below, on the reasonable hypothesis that it was the same class of
  BiguaSim-synthetic-GPS-glitch issue. **Confirmed live this did NOT fix the
  blocker** (see below) — kept in the file since it's still a reasonable
  defensive relaxation, but it is not sufficient on its own.
- `biguasim_sim_runner_bluerov2.py` (new) — same `ArduBiguaSimRunner`-direct
  pattern as the BlueBoat runner, spawns at the same validated water-crossing
  point (x=25).
- `ardupilot_interface.cpp`: new `mav_type_ == 12` branch in
  `takeoff_handle_accepted`, structurally identical to the Rover branch (arm
  + GUIDED, no altitude-wait) — **confirmed live this part works**: `mav_type_`
  correctly detected as 12, home-position revalidation runs, force-arm
  succeeds, FSM reaches `ARMED`. Widened `set_reposition_callback`'s guard to
  accept `mav_type_==12` reusing `ARMED`. Also gated the three DjiMatrice-
  specific GPS-free altitude branches (`desired_alt < -1.0`/`< 0.0`/`> 10.0`)
  to `mav_type_ == 2` explicitly — found by inspection, before ever running
  live, that an ungated negative depth target for Sub would have collided
  with the DjiMatrice-specific forced-descent branch instead of the intended
  plain `GlobalPositionTarget`/GUIDED path. **Confirmed live via a direct
  `/set_reposition` service call** (bypassing `mission_node.py`) that the
  dispatch correctly reaches the intended branch with the right values.
- `mission_node.py`: `DRONE_TYPE` gate widened to `('quad', 'rover', 'sub')`.
- `t2_bluerov2_hold_test.yaml` (new) — `takeoff` (arm+GUIDED) →
  `go_to_known_gps_waypoint` (north=5, altitude=-3.0 — a real dive target,
  unlike Rover's ignored altitude field) → `wait` 180s.
- `log_hover_stability.py` — one real change needed here, unlike the Rover
  case: added a fallback subscription to `/mavros/global_position/local`
  (same `nav_msgs/Odometry` shape), used whenever `/mavros/local_position/odom`
  has no data. Found live: ArduSub's MAVROS instance never publishes
  `local_position/odom` at all in this setup (no error, just silence — IMU
  and every GPS-derived topic publish fine), while `global_position/local`
  (also GPS/EKF-derived, republished by MAVROS's `global_position` plugin)
  does. Copter/Rover are unaffected — they already publish `local_position/odom`,
  still preferred whenever available.

### The actual blocker: GUIDED mode is refused, "Guided requires position"

Confirmed live, repeatedly, across two SITL sessions (vehicle submerged at
spawn, and respawned at the surface — same result both times, ruling out
depth/submersion as the variable): the vehicle arms successfully (FSM reaches
`ARMED`, `armed: true` on `/mavros/state`), but every `SET_MODE GUIDED`
request is rejected by the firmware with MAVLink statustext `'Mode change
failed: Guided requires position'`. MAVROS's `SetMode` service still reports
`mode_sent: true` in every case — **that field only confirms the command was
transmitted, not that the firmware accepted it**, which is easy to
misdiagnose (the C++ FSM's `call_service_and_update_fsm` trusts exactly this
field, and reaching `ARMED` doesn't actually prove GUIDED ever really took).

Traced the exact rejection to ArduPilot's own source: `Sub::position_ok()`
(`ArduSub/system.cpp:189`) → `Sub::ekf_position_ok()` requires (once armed)
`AP_AHRS::Status::HORIZ_POS_ABS` and not `CONST_POS_MODE`, both of which
derive from `NavEKF3_core::updateFilterStatus()`
(`AP_NavEKF3_Control.cpp:805`): `horiz_pos_abs = doingNormalGpsNav &&
filterHealthy`, requiring `PV_AidingMode == AID_ABSOLUTE`, which
`setAidingMode()` only sets once `readyToUseGPS()` (`AP_NavEKF3_Control.cpp:590`)
returns true — itself requiring `validOrigin && tiltAlignComplete &&
yawAlignComplete && (delAngBiasLearned || ...) && gpsGoodToAlign &&
gpsDataToFuse`. Confirmed live that `validOrigin` (the "EKF3 IMU0/1 origin
set" statustext) takes an unusually long time to fire for ArduSub — in one
session, ~90s after arming, versus seconds for Copter/Rover — and GUIDED
still failed for several more minutes after that, meaning `validOrigin`
alone isn't the bottleneck; one or more of the other `readyToUseGPS()`
conditions never clears.

**Root-caused via the dataflash log** (`Tools/autotest/logs/*.BIN`,
`mavlogdump.py --types MSG` — far more reliable than the live MAVLink channel
for this investigation, which struggled with request/response traffic all
session, including `/mavros/param/get` and even a direct second pymavlink
connection to the SITL port): every ArduSub session — tried with default
`GPS_TYPE`, then `GPS_TYPE=1` (AUTO), then `GPS_TYPE=100` (the `GPS_TYPE_SITL`
enum value, `AP_GPS.h:114`) — logs `GPS 1: probing for u-blox` (or `SITL`)
**exactly once** and never advances to `EKF3 IMUx is using GPS`, no matter
how long it runs. The equivalent DjiMatrice/Copter session, identical
`--model JSON` setup, logs the same `probing for u-blox` line and completes
in ~75s. So this is a real ArduSub-specific GPS backend-detection bug in
this SITL+BiguaSim environment, not an EKF-tuning problem — confirmed by
elimination: `EK3_SRC1_POSXY` is already GPS (shared `AP_NavEKF_Source.cpp`
framework default, no ArduSub override exists in `ArduSub/Parameters.cpp`),
the EKF-health params above don't change the outcome, and neither does
`GPS_TYPE` 1 vs 100. `AP_GPS.cpp:_detect_instance()`'s backend-selection
switch shows *why* `GPS_TYPE_SITL` requires probing at all rather than
resolving instantly: it isn't in the short list of types
(`MAV`/`UAVCAN*`/`MSP`/`EXTERNAL_AHRS`/`GSOF`) that bypass the generic
serial baud-cycling detection loop — it falls through to that loop just like
`AUTO`, and that loop is what's stuck.

**Workaround implemented** (not a real fix — the actual GPS backend-detection
bug in this environment is still unexplained): `GPS_TYPE_MAV = 14` **is** in
that bypass list — `_detect_instance()` returns an `AP_GPS_MAV` backend for
it immediately, no probing, and that backend just waits for MAVLink
`GPS_INPUT` (#232) messages instead of anything serial-based.
`t2_gps_input_bridge_bluerov2.py` (new) closes the loop: connects directly
to the SITL's MAVLink port, reads this same vehicle's own
`GLOBAL_POSITION_INT` (already confirmed reliable throughout this
investigation), and relays it straight back as a synthetic `GPS_INPUT` with
a plausible fix (`fix_type=3`, 10 satellites, `hdop=1.0`). `t2_biguasim_bluerov2.parm`
now sets `GPS1_TYPE`/`GPS_TYPE` to `14`. Must run this script alongside the
SITL/BiguaSim processes (see its own docstring) — it isn't started
automatically by anything yet.

**Not yet validated live — blocked by a second, separate, unresolved problem:
BiguaSim/Unreal Engine hangs deterministically every time it tries to spawn
the BlueROV2 agent, before ArduPilot/MAVROS ever get a chance to exercise
the GPS fix above.** This is a different bug from the GUIDED-mode one — it
happens earlier in the pipeline, entirely on the BiguaSim/rendering side,
and blocks live confirmation of the (already correctly root-caused and
implemented) GPS workaround.

### The Unreal Engine hang: what was ruled out, methodically

Confirmed reproducible across **nine separate attempts**, spanning two
different sessions (one machine reboot in between): `HolodeckLog.txt`
always stops advancing at the exact same point — right after the agent's
last sensor is added and the *first* shader/graphics PSO for its visual
mesh begins compiling (`LogRHI: Display: Encountered a new graphics PSO:
908536767` is consistently the last or near-last line, byte-for-byte
identical across unrelated sessions). GPU utilization drops to 0% and stays
there; the process either exits (`<defunct>`) or just sits alive holding
GPU memory with zero compute activity, indefinitely. DjiMatrice and
BlueBoat have never hit this in the same environment — the underlying
mechanism they both share (`ArduBiguaSimRunner.build_scenario()` +
`run()`) needed **no code changes at all** to build a scenario for
BlueROV2, so it is very unlikely to be a bug in that shared code path.

Each of the following was tested as a hypothesis and disproved:

1. **The `GPS_TYPE=14` param or `t2_gps_input_bridge_bluerov2.py` itself** —
   disproved by running BiguaSim completely alone, with no bridge script and
   no second MAVLink connection at all. Hung identically. (These two
   processes don't share a port or any IPC with BiguaSim/Unreal anyway —
   this was more a sanity check than a real suspect, but the timing
   correlation with when the hangs started made it worth ruling out
   explicitly.)
2. **The on-screen viewport / rendering window** — disproved by running
   headless (`biguasim_sim_runner_bluerov2.py` without `--viewport`, which
   passes SITL's `-RenderOffScreen` flag). Hung at the identical point, 0%
   GPU utilization sustained for 165s straight before being declared stalled.
3. **Spawn depth / submersion** — disproved across three different values:
   submerged at spawn (z=-1.0), at the surface (z=0.2), and deeper (z=-3.0).
   All three hang identically.
4. **Host memory/resource pressure from many consecutive Unreal launches in
   one long session** — disproved by a full machine reboot (swap dropped
   from 2.9 GiB to 0, GPU memory to 20 MiB) followed by immediately retrying:
   hung on the very first attempt post-reboot, and every attempt after.
5. **A corrupted on-disk shader cache from an earlier `kill -9` interrupting
   a compile mid-write** — the most promising lead, since NVIDIA's driver
   caches compiled shaders (used by both OpenGL and Vulkan, despite the
   name) at `~/.cache/nvidia/GLCache`, and one of its files had a modify
   timestamp matching a crash almost to the second. Disproved by deleting
   that cache entirely and retrying: the driver visibly rebuilt it from
   scratch (grew back to 5.7 MB during the run) and still hung at the exact
   same point.
6. **The `DepthSensor` specifically** — the strongest lead, since it's the
   one sensor BlueROV2 has that BlueBoat/DjiMatrice don't
   (`include_depth_sensor=True` only for BlueROV2/BlueROVHeavy in
   `vehicle.py`'s `VEHICLE_REGISTRY`), and it was always the last sensor
   logged immediately before every hang. Disproved directly: rebuilt the
   scenario with `dataclasses.replace(profile, include_depth_sensor=False)`
   (BiguaSim's own `VehicleProfile` is a plain dataclass, so this doesn't
   touch the shared registry) and reran. `DepthSensor` was confirmed absent
   from the log this time — and it hung anyway, at the **same graphics PSO
   ID** (`908536767`), now immediately after `IMUSensor` instead (the new
   last sensor). This proves the hang isn't about any particular sensor at
   all: it's tied to the first visual-mesh shader compile for this agent,
   which happens right after sensor setup regardless of which sensor was
   last.

### Root-caused: a real segfault in BiguaSim's own `SpawnAgentCommand.cpp`

Two more findings closed this out. First, a housekeeping discovery while
setting up the differential test below: every prior kill of a hung/crashed
BiguaSim session leaked its POSIX shared-memory segments and semaphores
(`/dev/shm/HOLODECK_MEM<uuid>_*`, `/dev/shm/sem.HOLODECK_SEMAPHORE_*`) —
`ArduBiguaSimRunner`'s context-manager cleanup never runs when a process is
`kill -9`'d instead of exiting normally. Seven full sets had accumulated
(harmless in terms of actual space — tmpfs showed 1% used despite `ls -la`
reporting 1 GiB per `command_buffer` file, since they're sparse — but a real
leak regardless, and worth clearing: `rm -f /dev/shm/HOLODECK_MEM*
/dev/shm/sem.HOLODECK_SEMAPHORE_*` after killing any hung session).

Second, and the actual answer: tried a **differential test with
`BlueROVHeavy`** (new diagnostic-only `t2_sitl_run_bluerovheavy.sh` /
`t2_biguasim_bluerovheavy.parm` / `biguasim_sim_runner_bluerovheavy.py`,
same pattern as the BlueROV2 scripts — shares ArduSub and
`include_depth_sensor=True`, differs only in motor count/mapping). It hit
the identical failure. But this run, for the first time, produced a
**real, complete crash log** instead of a silent hang (`HolodeckLog.txt`):

```
Assertion failed: SpawnedAgent [File:.../ClientCommands/Private/SpawnAgentCommand.cpp] [Line: 33]
Signal 11 caught.
Unhandled Exception: SIGSEGV: invalid attempt to write memory at address 0x0000000000000003
```

`SpawnAgentCommand.cpp:33` asserts that the just-spawned agent pointer is
valid; here it isn't, and the code goes on to dereference it anyway,
segfaulting at a near-null address. This is a **genuine bug in BiguaSim's
own compiled engine code** — not a shader/asset issue, not a cache issue,
not anything in this repo or launch parameters, and not specific to
BlueROV2's own asset (BlueROVHeavy hits the exact same assertion). It
explains every earlier observation at once: deterministic (same code path
every time) and immune to every external variable tried (rendering,
viewport, spawn depth, reboot, shader cache — none of them touch agent
spawning logic). It also explains why most attempts looked like a silent
*hang* rather than a crash: `HolodeckLog.txt` shows the engine's own crash
handler trying and failing to launch `CrashReportClient` (`File does not
exist` — that binary is simply missing from this packaged BiguaSim build),
and depending on timing that failed launch attempt sometimes returns
quickly (process exits, `<defunct>`) and sometimes appears to hang before
finishing crash handling (process alive, 0% GPU utilization, holding GPU
memory indefinitely) — two different-looking symptoms of the exact same
underlying segfault.

**Not fixable from this repo, but narrowed down to the actual trigger.**
This is a bug in BiguaSim's shipped engine binary (`SpawnAgentCommand.cpp`,
not part of this repo's source) — but which spawn-time condition trips it
turned out to be findable without engine source access, by differential
testing against `biguasim_sim_runner.py`'s own long-validated `bluerov0`
agent (the decorative, non-SITL prop used in earlier T2 land-test work,
`agent_type: "BlueROV2"`, `control_abstraction: "cmd_pos_yaw"`, driven by
teleport commands, not ArduPilot). That agent spawns and runs fine —
confirmed live, 4+ minutes of stable 47-84% GPU utilization — using the
exact same underlying blueprint. Adding its sensors up one at a time onto
that known-working config (`DynamicsSensor`+`LocationSensor` →
`+VelocitySensor`) still worked fine every step, ruling out **every
sensor**, including `DepthSensor`/`IMUSensor` which were the strongest
suspects earlier. The one remaining difference between that always-working
config and `ArduBiguaSimRunner.build_scenario()`'s always-crashing one is
`control_abstraction`: `"cmd_pos_yaw"` (teleport-driven, works) vs
`"cmd_motor_speeds"` (individual per-motor thrust control, crashes) — the
value `VEHICLE_REGISTRY["BlueROV2"]`/`["BlueROVHeavy"]` both specify, and
the only abstraction that actually lets ArduPilot's PWM output drive a real
vehicle (BlueBoat also uses `cmd_motor_speeds` and works fine, so it's not
that value in isolation either — likely something specific to how
BlueROV2/BlueROVHeavy's particular motor *count/socket* configuration
(6 or 8 individual thrusters) gets wired up during spawn, which
`SpawnAgentCommand.cpp` chokes on for these two vehicles specifically).

This means the underlying blueprint/mesh/sensors are all fine — the crash
is isolated to agent-spawn-time motor-thruster setup for `cmd_motor_speeds`
control specifically on these two vehicles, not anything broader.

### `cmd_vel_yaw` workaround: implemented, reduces but does not eliminate the risk

Tried building a real bridge around this: `t2_bluerovheavy_velbridge_runner.py`
and `t2_bluerov2_velbridge_runner.py` (new) reimplement
`ArduBiguaSimRunner.run()`'s loop, but translate ArduPilot's real PWM output
into a `[vx, vy, vz, yaw_delta_deg]` `cmd_vel_yaw` command instead of sending
it as `cmd_motor_speeds` directly — using each vehicle's own
`motor_mapping`/`motor_signs`/`pwm_converters` (already in `vehicle.py`) to
get individual per-motor thrust, then approximating heave from the vertical
motors and surge from the horizontal ones (sway and yaw aren't
reconstructed — see each script's docstring for the exact reasoning and
known limitations). This is a real bridge, not a fake one:
`cmd_vel_yaw`'s handler in BiguaSim's own `uuv.py` runs a genuine
P-controller (velocity error → desired force → `TM_to_f` → thruster forces
→ motor speeds), so real thrust/mass dynamics are still exercised, just
through an extra control loop stacked on top of ArduSub's own.

**First finding: `BlueROVHeavy` is broken regardless of `control_abstraction`.**
Tested `cmd_pos_yaw` directly against it (the one value otherwise confirmed
always-safe) — same `SpawnAgentCommand.cpp:33` assertion, same SIGSEGV. This
is a different, apparently deeper problem than BlueROV2's (which only
breaks with `cmd_motor_speeds`) — no combination tried gets `BlueROVHeavy`
running. Not investigated further; treat it as blocked independent of
everything above until proven otherwise.

**Second finding: `BlueROV2` + `cmd_vel_yaw` is not reliably stable either.**
An isolated spawn test (bypassing the SITL bridge entirely — just
`ArduBiguaSimRunner` + direct `env.step()` calls) ran cleanly for a full
2000-tick / 10s loop with sustained 67-84% GPU utilization, no crash. But
the very next run, this time through the *real* bridge script
(`t2_bluerov2_velbridge_runner.py`, identical scenario construction, only
difference being `bridge.bind()` + waiting on real ArduPilot PWM instead of
a hardcoded test command), hung at the exact same spawn point for 4+
minutes with 0% GPU utilization and no crash signature — not a repeat of
the `cmd_motor_speeds` failure (that failed on *every* one of 9+ attempts,
100% reproducible), but not reliable either. **Conclusion: `cmd_vel_yaw`
lowers the failure rate for BlueROV2 but does not eliminate whatever
underlying spawn-time instability BiguaSim has** — this looks consistent
with the same class of bug as `SpawnAgentCommand.cpp`'s assertion, just
triggered less deterministically for this abstraction. Retry a few times if
using this workaround; it is not guaranteed to spawn on the first attempt.

### Next steps for whoever picks this back up

1. **Report this to whoever maintains the BiguaSim build** (or get access
   to its editor/source project) — `SpawnAgentCommand.cpp:33`'s
   `SpawnedAgent` assertion failing specifically when spawning BlueROV2 with
   `control_abstraction: "cmd_motor_speeds"` (not the blueprint/mesh, not
   any sensor — `cmd_pos_yaw` with the identical blueprint and sensor set
   works fine), and BlueROVHeavy failing this same assertion with *every*
   `control_abstraction` tried, including `cmd_pos_yaw`. SIGSEGV at address
   `0x3` right after the assertion. This blocks fully reliable live testing
   and isn't fixable from this repo without engine source access.
2. **In the meantime, `t2_bluerov2_velbridge_runner.py`'s `cmd_vel_yaw`
   workaround is the best available path for BlueROV2** — real thrust
   dynamics, just not 100% reliable at spawn (retry on hang/crash; see the
   finding above). Don't bother with BlueROVHeavy until finding #1 above is
   understood — no `control_abstraction` has gotten it running so far.
3. Once BiguaSim/Unreal actually gets one of these vehicles running, the
   GPS/GUIDED fix documented above (`GPS_TYPE=14` +
   `t2_gps_input_bridge_bluerov2.py`) is believed correct and ready to
   validate — it was reasoned through carefully via the dataflash log and
   ArduPilot's own source, but has never actually gotten to run against a
   live vehicle yet.
4. If `GPS_TYPE=14` + the bridge script somehow doesn't clear
   `Sub::position_ok()` either, dig into *why* `GPS_TYPE=1`/`100`'s
   serial-probe detection loop never completes for ArduSub specifically.
   `AP_GPS.cpp`'s `_detect_instance()` (around the baud-cycling logic, just
   above the `GPS_TYPE_SITL` switch case) and whatever SITL-side code is
   supposed to be feeding synthetic uBlox-protocol bytes onto the virtual
   serial port `GPS_TYPE=1`/`AUTO` probes against
   (`libraries/SITL/SIM_GPS.cpp`) are the places to check — compare against
   a working Copter SITL session's `_port[instance]`/UART wiring to see
   what's actually different for the `ardusub` binary.
5. Once both are confirmed working, the rest of the pipeline (the
   `mav_type_==12` reposition dispatch, the mission YAML, the logger's
   `global_position/local` fallback) is already implemented and should just
   work — these are believed to be the only two remaining blockers, not one
   of several.

### What was actually confirmed, given SITL live validation wasn't reachable

After 12+ attempts across two sessions (`cmd_motor_speeds`: 100% failure
rate over 9+ tries; `cmd_vel_yaw`: succeeded once in isolation, then hung
twice more through the real bridge) failed to get a stable enough BiguaSim
session to exercise ArduSub GUIDED position-hold end to end, this was
de-scoped: **BiguaSim's own non-SITL `bluerov0` prop** — the same
decorative, teleport-held agent already used in earlier T2 land-test work
(`biguasim_sim_runner.py`, `control_abstraction: "cmd_pos_yaw"`, held at a
fixed `rov_hold` location every tick, no ArduPilot involved at all) — was
run instead, purely to confirm the BlueROV2 blueprint itself still holds a
commanded position reliably in this environment. Confirmed live and stable.

**This does not validate what the task actually needs.** `cmd_pos_yaw` is
driven directly by BiguaSim's own Python-side position controller, not by
ArduPilot's EKF/GUIDED-mode controller — it proves the blueprint can be
commanded to hold a position, not that ArduSub's own position-hold logic
(the thing `mav_type_==12`/the GPS workaround/the whole rest of this
section exists to test) produces a stabilizing response over a real PWM
bridge. That specific validation — the actual goal of this section — was
**not achieved** in this environment. Whoever picks this back up should
treat BlueROV2/BlueROVHeavy live SITL position-hold as still fully open,
blocked on the BiguaSim spawn-time bug documented above, not as "mostly
done."

None of these were introduced by the Humble port itself — they were latent
issues in `ardupilot_interface.cpp`/`mission_node.py`/the container's MAVROS
config that had simply never been exercised by this exact flow before. Fixed
here because they blocked getting a real BiguaSim/ArduPilot run to complete.

1. **`biguasim_bridge`'s params YAML used the wrong key.** ROS2 requires the
   literal key `ros__parameters` (double underscore); the file had
   `ros_parameters`, so `rcl` refused to parse it and the node died on launch.

2. **Startup race between MAVROS and `mission`.** `t2_aircraft.yml.erb`'s
   `mission` window used to start unconditionally alongside `mavros`. If
   `mission` requested takeoff before MAVROS finished syncing FCU state, the
   takeoff goal was rejected outright. Fixed by gating `mission`'s start on
   `/mavros/state` reporting `connected: true` and `system_status: 3`
   (standby) — mirroring the wait `mavros`'s own window already did.

3. **Wrong MAVLink `tgt_system` on the MAVROS launch.** `t2_aircraft.yml.erb`
   passed `tgt_system:=<DRONE_ID>` (default `0`) to `mavros apm.launch`, but
   ArduCopter SITL's default `SYSID_THISMAV` is `1` (unset in
   `t2_biguasim.parm`). MAVROS was listening for heartbeats from system 0 and
   never saw `connected: true`. Removed the override so MAVROS falls back to
   its correct default of `1`.

4. **`descend_to_water` never resent its velocity setpoint.** Unlike
   `ascend_from_water`/`ascend_to_altitude`, the `descend_to_water` step in
   `mission_node.py` sent its `vel.z=-2` command once and only polled for
   timeout afterwards. ArduPilot's `GUID_TIMEOUT` (15s, see
   `t2_biguasim.parm`) expired mid-descent, the FCU reverted to loiter, and
   the mission timed out waiting for `AQUATIC_NAV`. Fixed by resending the
   command every 10s, matching the sibling actions.

5. **Takeoff completion had no debounce.** `ardupilot_interface.cpp`'s MC
   takeoff-complete check (`alt_ - home_alt_ > 90% of target`) fired on a
   single sample. BiguaSim's synthetic GPS/baro is documented (see
   `t2_biguasim.parm`) to glitch during AHRS init — a single noisy sample
   could satisfy the altitude threshold for one loop tick with the vehicle
   still on the ground, marking takeoff "complete" ~10ms after arming. Fixed
   by requiring the condition to hold for 0.5 consecutive seconds
   (`MC_TAKEOFF_COMPLETED_DEBOUNCE_SEC`).

6. **`home_alt_`/`home_lat_`/`home_lon_` captured before a real GPS fix.**
   The debounce above wasn't enough on its own: `takeoff_handle_goal` saved
   `home_alt_ = alt_` from whatever `/mavros/global_position/global` had
   published *so far* — and MAVROS's first message on that topic can be an
   all-zero placeholder before the FCU has a fix. With `home_alt_ = 0.00`,
   `alt_ - home_alt_` was already ~570m (the real MSL altitude) from the very
   first sample, trivially clearing even a 0.5s debounce. Fixed by
   re-validating `lat_`/`lon_`/`alt_` before requesting GUIDED mode, retrying
   once a second until a real (non-zero, non-NaN) fix arrives.

7. **`setpoint_raw` MAVROS plugin missing from the container's allowlist.**
   `aircraft_resources/patches/apm_pluginlists.yaml` (used by the Humble
   container) is an allowlist that had `setpoint_position`/
   `setpoint_velocity`/`setpoint_accel` but not `setpoint_raw`. All of
   `ardupilot_interface.cpp`'s reposition logic publishes to
   `/mavros/setpoint_raw/{local,global}` specifically (for `type_mask`
   control) — those messages were being silently dropped with no plugin
   subscribed to relay them. This broke every horizontal waypoint move
   (`go_to_known_gps_waypoint`) and every velocity-setpoint maneuver
   (`descend_to_water`, `ascend_from_water`, float-at-surface). Only
   `MAV_CMD_NAV_TAKEOFF` (a `COMMAND_LONG`, routed through the allowlisted
   `command` plugin) worked, which is why takeoff alone looked fine. The
   original host/Jazzy setup used a denylist-based `apm_pluginlists.yaml`
   (only blocking a handful of plugins), so `setpoint_raw` was enabled there
   by default and this had never surfaced. Fixed by adding `setpoint_raw` to
   the allowlist. This affects any ArduPilot mission using
   `ardupilot_interface`'s `/set_reposition` service, not just T2.

8. **Same class of bug as #6, independently, in `mission_node.py`.** Its
   `wait_to_reach_position` step computes distance-to-target from its own
   `self.home_lat`/`self.home_lon`, captured unconditionally from the first
   `/mavros/global_position/global` message (`mavros_global_position_callback`).
   If that first message was the same zero placeholder as in #6, `current_north
   = (lat - 0.0) * 111320` came out as a multi-million-meter garbage value,
   so the drone could physically arrive at the target (confirmed via
   `ardupilot_interface`'s own, correctly-validated position) while `mission`
   never detected arrival and hovered indefinitely. Fixed with the same kind
   of validity check before capturing home.

## Verified

- `colcon build` succeeds cleanly for `biguasim_bridge`,
  `autopilot_interface`, and `mission` inside the Humble container.
- Live UDP telemetry round-trip (`biguasim_bridge` ↔ `biguasim_sim_runner.py`)
  confirmed with synthetic packets before any real BiguaSim run.
- Full `t2_land_test.yaml` run against real BiguaSim + ArduPilot SITL:
  takeoff, fly-to-waypoint, GPS-free descent to water, `AQUATIC_NAV`
  detection, ascend back over the bridge deck, GPS return, and landing on the
  platform all completed successfully, matching the originally recorded demo.
- The custom-trained YOLOv8n BlueBoat detector (see "Precision-landing
  vision" above), confirmed live against the real simulator with
  `--spawn-boat --show-camera`: correctly tracks the
  boat's built-in helipad marking across distance/angle, including on the
  exact water-glint frame that used to trip up the classical CV detector.
- Full `t2_hover_test.yaml` run against real BiguaSim + ArduPilot SITL (see
  "Hover stability and the vertical velocity profile" above): armed in under
  a minute, flew to the waypoint, held a 180s hover (horizontal position
  stddev 0.000 m, vertical 0.199 m, peak roll/pitch under 0.1°), and landed
  cleanly back at the spawn point — no RTL failsafe, no manual intervention.
- Full `t2_rover_hold_test.yaml` run against real BiguaSim + ArduPilot Rover
  SITL (see "Position-hold stability for other BiguaSim vehicles" above): the
  BlueBoat armed, drove to its waypoint, and held a 180s position hold
  (horizontal position stddev 0.001 m, vertical 0.015 m, peak roll/pitch
  under 0.2°) — first confirmation that the ArduCopter-only ROS2 bridge
  (`ardupilot_interface.cpp`) also works for a Rover vehicle.
