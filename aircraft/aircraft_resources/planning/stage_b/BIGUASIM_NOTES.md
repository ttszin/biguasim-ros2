# Changes and findings in BiguaSim (`~/biguasim`, a separate repository)

The Stage B/C tests needed changes to the BiguaSim ↔ ArduPilot bridge. They live in `~/biguasim` (**uncommitted there**, together with other local changes),
so they are described here to keep them traceable and re-appliable. Reasons and evidence are in `planning/README.md` ("Defeitos do ambiente").

## 1. Yaw-rate sign is a per-vehicle option (fix of a regression) — `src/biguasim/ardubridge/`

On 2026-08-30 the BlueROV2 work inverted the yaw rate `r` in `frame.imu_glu_to_frd` **for every vehicle**. That broke the Hydrone (DjiMatrice/ArduCopter): it tumbled.
The extra flip is now opt-in:

```python
# frame.py
def imu_glu_to_frd(imu_data, flip_yaw: bool = False) -> tuple:
    ...
    return [ax, -ay, -az], [p, -q, (r if flip_yaw else -r)]      # flip_yaw=False = the original GLU->FRD conversion

# vehicle.py  (VehicleProfile)
    flip_gyro_yaw: bool = False
# ... and `flip_gyro_yaw=True,` in the BlueROV2 and BlueROVHeavy profiles (the only ones the ArduSub yaw finding was confirmed for)

# bridge.py  (build_json_state)
    accel, gyro = imu_glu_to_frd(agent_state["IMUSensor"], flip_yaw=getattr(self._profile, "flip_gyro_yaw", False))
```

Effect: BlueROV2/BlueROVHeavy behave as before; DjiMatrice, BlueBoat and TorpedoAUV get the original conversion back. The BlueBoat was affected by the regression between 2026-08-30 and 2026-09-23.

## 2. JSON state: `"position"` is sent in the wrong format (NOT fixed in the library)

`bridge.build_json_state` puts `[lat, lon, alt(up)]` in `"position"`. ArduPilot's JSON backend (`libraries/SITL/SIM_JSON.h`) reads `"position"` as **NED metres relative to home**;
global coordinates have their own keys (`latitude`, `longitude`, `altitude`). Read as NED, the horizontal position never changes and the altitude sign is inverted: the EKF believes the
vehicle descends while it climbs, and the controller pushes harder (runaway climb; observed 0 → 34 m while the EKF reported a steady 4 m).
`"pressure"` is not a key of that backend either (ignored).

Workaround used by the tests (does not touch the library): `planning/stage_b/sitl_runner.py` rewrites `js["position"] = [north, east, -up]` (metres from home) before `send_state`.
**The T2 runner (`aircraft_resources/missions/biguasim_sim_runner.py`) uses the stock bridge and probably has the same problem**; its "NED=(33.8,-118.4,13.3)" log line is lat/lon in degrees.

## 3. Arming needs a fast enough gyro backend

ArduPilot refuses to arm with `Gyro rate < loop rate * 1.8`. With `SCHED_LOOP_RATE 120` (`t2_biguasim.parm`) that is 216 Hz, above the 200 ticks/s the T2 runner asks for.
`sitl_runner.py` uses `--ticks 250`.

## 4. Other things worth knowing

* `depth_to_pressure` clamps at 101325 Pa above the surface (the barometer does not follow altitude in air); the EKF height comes from the position sent (see 2).
* The DjiMatrice thrust is `k_eta * omega^2` per rotor (`k_eta = 6.64e-5`, mass 3.8 kg); `sitl_runner.py` logs the total from the motor speeds it receives, which is what the energy figures in Stage B/C use.
* The Unreal engine is a child process named `Holodeck`; killing only the Python runner can leave it holding ~1.5 GB of GPU memory. `stage_b/run_stage_b.py::kill_leftovers` handles it.
