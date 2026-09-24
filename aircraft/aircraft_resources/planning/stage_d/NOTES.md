# Stage D — underwater planners in SITL (BlueROV2 / ArduSub, BiguaSim)

Underwater counterpart of Stage B. Same planners, map, energy model and flight logic (`PlannedFlight`); the vehicle is the
BlueROV2 in ArduSub SITL (GUIDED) inside the SkyDive/Bridge world, water column at x = 25 m (north 0..22, east -9..9,
depth 1.5..7 m; bed measured near -9.7 m).

```
bash stage_d/rov_sitl.sh                        # ArduSub SITL, home altitude 0 (see finding 1)
python3 stage_d/rov_exec.py --scenario stage_d/scenarios/d1_poles.yaml --planner astar --condition K1 --out /tmp/run.json
python3 stage_d/rov_runner.py --scenario stage_d/scenarios/d1_poles.yaml --log /tmp/truth.csv --fix-position
python3 stage_d/run_stage_d.py --out results/stage_d          # whole campaign (starts the three above per flight)
```

Order matters: ArduSub blocks in `accept()` on tcp:5760 until a client is connected, so `rov_exec.py` starts before the runner.
`rov_exec.py --probe` arms, dives to 4 m and crosses 10 m without a planner (vehicle-side check).

## What had to be found before any planned flight could run (2026-09-24)

The BlueROV2 pipeline of T2 could not fly a dive in GUIDED. Five separate causes, found in this order with probes
(`rov_exec.py --probe`, truth CSV from the runner). Each one hid the next.

1. **SITL home altitude 584 m makes the water barometer useless.** `sim_vehicle.py` defaults to CMAC (584 m AMSL).
   `AP_Baro_SITL` feeds `sim_alt` to `SimpleUnderWaterAtmosphere(-sim_alt)` for ArduSub, so the absolute pressure came out
   **negative** (`SCALED_PRESSURE` about -57640 hPa), the EKF ignored it and its depth froze while the ROV sank (probe 1: EKF z stuck at
   -2.03 m while the truth went from -1.9 to -9.8 m, controller pushing down harder the whole way). Fix: `-l lat,lon,0,heading`
   (`rov_sitl.sh`). After it, pressure is about 1000 hPa and rises with depth.
2. **Depth reference.** With home altitude 0 the SITL barometer clamps above the surface, so the JSON `position` down component must be
   `-z` relative to the **water surface**, not relative to the spawn (the ROV floats up 0.2 m before arming and would have gone
   "above the water" for SITL and frozen the EKF depth again). `rov_runner.py --fix-position` sends
   `[north, east, -z_surface]`. (The stock bridge sends `[lat, lon, alt]` there; ArduPilot reads NED metres — see
   `project_biguasim_bridge_findings`.)
3. **Yaw loop signs.** The library default (since 2026-08-30) negates `mt_z` in `uuv.py` and does not negate the gyro yaw rate
   (`flip_gyro_yaw=True`). Measured: with that pair the gyro yaw rate has the **opposite sign to the attitude quaternion's yaw rate**
   (gyro +0.05..0.08 rad/s while the quaternion yaw fell 3-5 deg/s), so the EKF fuses inconsistent yaw information and GUIDED spins the
   vehicle within 60 s of arming (yaw -23 deg at 40 s, then thrusters saturate). With the **original `mt_z` and the ordinary
   GLU->FRD gyro (`flip_gyro_yaw=False`)** heading held within about 2 deg for 60 s hold, and the dive and 220 s of navigation later.
   The opposite pair (`mt_z` negated + gyro un-flipped) explodes (yaw -15 rad/s, breaches the surface). `rov_runner.py` sets
   `BIGUA_MTZ_ORIG=1` (temporary toggle I added in `~/biguasim/.../uuv.py`, default unchanged) and `flip_gyro_yaw=False`;
   `--legacy-yaw` restores the library behaviour. Flipping `mt_x` instead capsizes at once (roll 180 deg), so the roll moment sign is right.
4. **Slow exponential roll instability.** With `ATC_RAT_RLL_I = 0.05` (from `t2_biguasim_bluerov2.parm`) the roll oscillation grows from
   0.1 deg at 100 s to 56 deg at 210 s (peak doubles every ~10 s, 0.16 Hz) and the ROV capsizes; it also happened in ALT_HOLD.
   With `ATC_RAT_RLL_I = 0.01` roll stayed at 0.0-0.1 deg for 220 s. `rov_exec.py` sets it after arming (`ROV_PARAMS`); the parm file
   of T2 was not touched. This is very likely the "late instability" (bug #9) of the T2 notes.
5. **Probe artefact, not a bug:** re-sending the position target every 0.25 s makes ArduSub restart its trajectory from the current
   speed each time and the ROV crept at 0.03 m/s. The planned flights send one target per waypoint and reach the commanded 0.2 m/s
   (`WP_SPD`) with no other change.

Depth estimate bias: the EKF's zero is the depth at boot, and the ROV floats up about 0.3 m before the depth loop engages, so the
EKF depth is about 0.3-0.4 m off the truth. The executive converts with the spawn depth (-2.0 m); the planner's 1.0 m water slack covers it.
All metrics use the BiguaSim truth, not the EKF.

## Files

`rov_sitl.sh`, `rov_runner.py` (props, ground-truth GPS_INPUT on SERIAL1, truth CSV with attitude and thruster commands),
`rov_exec.py` (pymavlink executive, `--probe`), `run_stage_d.py` (orchestrator, analysis via `stage_b/run_stage_b.py`),
`scenarios/` (`d0_probe`, `d1_poles`, `d2_gate`), config `config/planner_rov_sitl.yaml` (water cruise 0.2 m/s, vertical 0.3 m/s).

## Results (2026-09-24, `results/stage_d/`, 10 flights)

D1 (two poles, one unmapped; K1, K2, K5 = mission change at 25 s) and D2 (wall with a 6 m gap, an unmapped block narrows it to 4.5 m; K1, K2),
A* (1 m grid) and RRT* (1 seed), ROV at 0.2 m/s (`WP_SPD`), 17 m routes at 1.9 to 4.5 m depth.

| | A* (5 flights) | RRT* (5 flights) |
|---|---|---|
| status | 5 SUCESSO | 5 SUCESSO |
| collisions | 0 | 0 |
| min clearance to real obstacles (mean) | 1.51 m | 1.86 m |
| max cross-track error vs plan (mean) | 0.84 m | 0.74 m |
| final error to goal (mean) | 0.55 m | 0.44 m |
| flown / planned length | 17.5 m / 17.4 m | 17.3 m / 17.2 m |
| mission time | 87.7 s | 85.8 s |
| replans: K2 (up to 6 per flight) / K5 (1) | max 161 ms / 70 ms | max 20 ms / 3 ms |

* The plan speed (0.2 m/s) is what the ROV flies: ground speed 0.20 m/s in every flight, with short bumps to 0.25 m/s when a new
  waypoint is sent.
* Energy: the executed energy (thrust in the planner's WATER power curve, PLACEHOLDER coefficients) is about 0.48 Wh against 0.26 Wh planned
  (dE about -0.22 Wh, the flight spent about 1.9x the plan). **I did not establish why.** Candidates, none checked: the executed thrust is the
  SUM of the magnitudes of the six thruster forces (opposing thrusters count twice, and the vertical pair holds depth against buoyancy) while the
  plan uses the net force; the curve's 10 W constant term; the 0.25 m/s bumps at new waypoints. Only the relative comparison between planners
  is meaningful until the coefficients are measured.
* One flight (`d2_gate_astar_K2`) went up to z = -1.13 m, 0.37 m above the planner's -1.5 m band (still 0.6 m under the surface): the EKF
  depth is offset from the truth by 0.3-0.4 m (finding above). Nothing was hit. The 1.0 m water slack absorbed it.
* Limits: 17 m routes in a 24 x 18 m box, one water column; the real Bridge underwater geometry (pillars) is not known and was not mapped;
  no crossing of the surface; GPS_INPUT is ground truth (no K4 drift); one RRT* seed per case; `mission_node` has no `sub` version yet.
