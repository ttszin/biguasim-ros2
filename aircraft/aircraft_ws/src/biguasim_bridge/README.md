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
T2_CONOPS=/aas/aircraft_resources/missions/vision_land_test.yaml \
  docker run --rm -it --network host \
  -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
  --entrypoint bash aircraft-image \
  -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"
```

Available conops in `aircraft_resources/missions/`: `t2_land_test.yaml` (the
original aerial/aquatic transition), plus two rounds of vision testing —
detection first, landing control second (kept as separate steps/missions on
purpose, so a bad detection can't be confused with a bad control loop):

- **Detection only**, no `vision_land` step, regular guided `land` at the end:
  `t2_hover_test.yaml` (platform target) and `vision_detect_boat_test.yaml`
  (BlueBoat target) — just takeoff, hover over the target, watch `/detections`
  and `/biguasim/camera/image` (or `--show-camera`) to validate detection
  quality before trusting it to fly anything.
- **Detection + landing control**: `vision_land_test.yaml` (platform) and
  `vision_land_boat_test.yaml` (BlueBoat) — adds the `vision_land` centering/
  descent step. **Not yet tested live** — see the commit history for status.

The `*boat*` conops need `biguasim_sim_runner.py` started with matching flags
(step 2 above) so the BlueBoat and/or a target actually get spawned:

- `--spawn-boat --landing-target none` — the BlueBoat has its own built-in
  helipad marking (square deck, circle+cross touchdown mark) rendered on the
  model itself, no synthetic target needed on top of it. **Preferred** —
  avoids the "no custom ArUco texture via spawn_prop" limitation entirely,
  since the marking is already baked into the BlueBoat asset.
- `--landing-target boat` (implies `--spawn-boat`) — also drops a synthetic
  gold sphere on the boat, for comparing detection against the built-in mark
  or as a fallback if the built-in one doesn't detect well.

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
  --spawn-boat --landing-target none --location 33 0 6

# Or a continuous all-around look instead of one fixed angle
python3 biguasim_sim_runner.py --viewport --show-camera \
  --spawn-boat --landing-target none --orbit
```

`--show-camera` opens a `cv2.imshow` window with the live detection overlay
(green box + confidence). For the full mission (takeoff, fly to the boat,
center, land), run `vision_detect_boat_test.yaml` (detection only) or
`vision_land_boat_test.yaml` (detection + landing) via the three-terminal
flow in "How to run" above, with `biguasim_sim_runner.py` started using the
same `--spawn-boat --landing-target none` flags.

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
  `--spawn-boat --landing-target none --show-camera`: correctly tracks the
  boat's built-in helipad marking across distance/angle, including on the
  exact water-glint frame that used to trip up the classical CV detector.
