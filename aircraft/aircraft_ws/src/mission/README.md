# mission

`mission_node` executes a mission described in a YAML file (`ros2 run mission mission --conops <file.yaml>`), one step at a
time, through the `ardupilot_interface` / `px4_interface` actions and services. Each step is `{action: <name>, params: {...}}`.
Most actions are documented where they are implemented (`mission_node.py`, `conops_callback`). This file documents the actions
added for **trajectory planning (T8.2)**; the planners themselves live in `aircraft/aircraft_resources/planning/`.

## Planned routes: `plan_route` + `follow_route`

Plan the route with A\* or RRT\* before takeoff, then fly it in GUIDED. Quad only (`DRONE_TYPE=quad`, ArduPilot).

```yaml
steps:
  - action: plan_route              # BEFORE takeoff; blocks while planning (RRT* uses its whole time budget)
    params:
      scenario: /aas/aircraft_resources/planning/stage_b/scenarios/b3_field.yaml
      planner: rrt_star             # astar | rrt_star
      condition: K2                 # K1 known map | K2 unmapped obstacles found in flight | K3 continuous sensing
                                    # | K5 goal changes in flight | K6 low energy (must be refused)
      seed: 0
      resolution: 1.0               # A* grid (m); ignored by rrt_star
      result_file: /results/exec.json
  - action: takeoff
    params: {takeoff_altitude: 5.0} # = the altitude of the scenario's first waypoint
  - action: follow_route
    params: {timeout: 300.0}
  - action: wait
    params: {duration: 3.0}
```

Ready-made examples: `aircraft/aircraft_resources/missions/planned_route_astar.yaml` and `planned_route_rrt_k5.yaml`.

### `plan_route` parameters

| param | default | meaning |
|---|---|---|
| `scenario` | required | scenario YAML (obstacles, bounds, start and goal). Frame: `(north, east, up)` metres **relative to home** |
| `planner` | `astar` | `astar` or `rrt_star` |
| `condition` | `K1` | how the map is fed (see the example above) |
| `seed` | `0` | RRT\* seed (fixed seeds make runs reproducible) |
| `resolution` | `1.0` | A\* grid spacing (m) |
| `config` | `planning/config/planner_sitl.yaml` | planner hyperparameters and energy model; falls back to `planner.yaml` |
| `overrides` | none | dict of dotted config keys, e.g. `{rrt_star.step: 1.5}` |
| `energy_available_wh` | config `battery_wh` | energy budget; the mission is refused if the plan needs more than `available - reserve` |
| `mission_change` | none | `{t: <s after follow_route starts>, goal: [north, east, up]}`: a timed goal change (for tests) |
| `max_flight_s` | `420` | flight time limit |
| `result_file` | `/tmp/route_result.json` | where the JSON record is written (also on failure) |

If planning fails (`SEM_CAMINHO`, `TIMEOUT`, `ENERGIA_INSUFICIENTE`) the step fails and the mission ends (`Mission Failed`) before takeoff.

### `follow_route`

10 Hz loop (`PlannedFlight`, `planning/flight_executive.py`):

* **Vehicle state in:** `/mavros/local_position/pose` and `/mavros/local_position/velocity_local` (ENU, origin = home).
* **Waypoints out:** `SetReposition` service of `ardupilot_interface` (GUIDED, `GlobalPositionTarget`, altitude above home) — the same path as `go_to_known_gps_waypoint`.
  Commands are delivered one service call at a time, the latest one wins, and a command the interface rejects
  (`Another service/action is active`, it answers that while it is still handling the previous request) is retried.
* **Replanning:** when newly sensed obstacles cut the route ahead, the vehicle is told to hold, the route is replanned (CL-RRT keeps its tree;
  D\* Lite reuses its search) and the new route is sent.
* **Waypoint arrival:** fly-by within a radius limited by the waypoint's clearance to obstacles; stop-and-go at sharp turns.
* **New goal at any time:** publish `geometry_msgs/Point` on **`/planner/new_goal`** (`x` = north, `y` = east, `z` = up, metres from home), e.g. a decision from T11:
  `ros2 topic pub --once /planner/new_goal geometry_msgs/msg/Point "{x: 36.0, y: -8.0, z: 6.0}"`.
* Ends with `SUCESSO`, or the mission fails (`ABORTADO` on timeout, `SEM_CAMINHO`/`TIMEOUT` if a replan fails).

Result JSON (`result_file`): `status`, `detail`, `planned_paths`, `path_history` (every route actually installed), `replans`
(`t`, `ms`, `kind` for goal changes), `events`, `trace` (`[wall, t, north, east, up, vn, ve, vu, leg, idx]`), `t_initial_ms`, `planned_wh`, `send_failures`, ...

### Constraints and known limits

* **Altitudes must stay in 0..10 m above home.** For a multicopter (`mav_type == 2`) `ardupilot_interface` treats `SetReposition` altitudes `< 0` and `> 10` as special GPS-free manoeuvres (see `set_reposition_callback`), so they would not be normal position targets.
* **Perception is simulated.** Obstacles come from the scenario file: "known" ones are in the a-priori map, "hidden" ones are inserted as sensor cells when the vehicle gets within the perception radius (K2 12 m, K3 30 m).
  There is no LiDAR/sonar input yet; the map interface is `HybridMap.atualizar_ocupacao(celulas, fonte, timestamp)`.
* **Aerial only.** Underwater legs are planned (medium-aware, vertical transition) but were not flown in SITL: GUIDED underwater needs a valid position estimate (T5/T6).
* The `Land` action of `ardupilot_interface` goes through RTL (climb, return, land), which takes minutes in BiguaSim; the test missions end after `follow_route` instead.
* The planning code is imported from `AAS_PLANNING_PATH` (default `/aas/aircraft_resources/planning`). It needs numpy, scipy and PyYAML (all in `aircraft-image`) and runs on Python 3.10.

## `wait_to_reach_waypoint`

3D arrival check for the same frame as `go_to_known_gps_waypoint`, PX4 and ArduPilot (unlike `wait_to_reach_position`, which only checks north and needs the MAVROS home).
Params: `north`, `east`, `altitude` (m from home), `threshold` (m, default 1.5), `timeout` (s, default 60).

## Running it in SITL

`aircraft/aircraft_resources/planning/stage_c/run_stage_c.py` starts ArduPilot SITL, BiguaSim (with the obstacles as props) and the ROS 2 side in `aircraft-image`
(`stage_c/aircraft_stack.sh`: MAVROS, `ardupilot_interface`, `mission`) and flies a generated mission per test. The host's `mission_node.py` is bind-mounted over the image's
copy (the image installs `mission` with `--symlink-install`), so the image does not need rebuilding. See `planning/README.md` ("Etapa C").
