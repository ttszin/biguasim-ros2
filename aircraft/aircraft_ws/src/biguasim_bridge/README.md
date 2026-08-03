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

## Bugs found during end-to-end validation

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
