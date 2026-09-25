"""T8.2 Stage E: one hybrid air-water plan, split by medium, executed as two SITL legs.

    plan = plan_hybrid("stage_e/e1_hybrid.yaml", "astar", seed=0)     # a single plan origin -> underwater target
    write_legs(scenario_yaml, plan, out_dir)                          # air_leg.yaml (Stage B frame) and water_leg.yaml (ROV frame)

The planner plans the WHOLE mission at once (air, vertical crossing, water) on the hybrid map. The plan's AR segment ends over the
crossing column and its AGUA segment starts under it. Real vehicles cannot do both media here (see stage_e/NOTES.md), so the two
segments are flown by two SITL stacks one after the other, joined at the column:

  * AR   : Hydrone (ArduCopter, Stage B stack) from the start to `hover_z` (2 m) above the water at the crossing column;
  * TRANSICAO : the vertical crossing (|z| <= mu). NOT flown: modelled by the planner's transition cost (speed 0.5 m/s, fixed energy);
  * AGUA : BlueROV2 (ArduSub, Stage D stack) spawned at the column at `spawn_z` (-2 m) and flown to the underwater target.

Frames: everything (hybrid plan and both legs) is in the water-surface frame with origin at BiguaSim (25, 0, 0) (north = x - 25,
east = -y, up = z), which is the ROV frame. The Hydrone's MAVLink frame is relative to its home (BiguaSim 8, 0, 13.4) = (-17, 0, 13.4) in the
surface frame; sitl_exec.py --frame-offset does the conversion (the planner needs heights over the water to tell the media apart).
"""

from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import numpy as np
import yaml

PLANNING = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PLANNING))

from config import Config  # noqa: E402
from execution import Mission  # noqa: E402
from interface import split_segments  # noqa: E402
from scenario import _obstacle, load_scenario  # noqa: E402

HOME_IN_SURFACE = np.array([-17.0, 0.0, 13.4])     # the Hydrone's home (BiguaSim 8, 0, 13.4) in the surface frame
ORIGIN_BSIM = (25.0, 0.0, 0.0)                      # BiguaSim position of the surface-frame origin
HOVER_Z = 2.0                                       # the Hydrone stops this high above the water at the crossing column
SPAWN_Z = -2.0                                      # the ROV starts this deep (its tested spawn depth)
HYBRID_CONFIG = PLANNING / "config" / "planner_hybrid_sitl.yaml"


def plan_hybrid(scenario_path: str | Path, planner: str, seed: int = 0, condition: str = "K2", resolution: float = 1.0) -> dict:
    """Plan origin -> underwater target in one go (no-go volumes included) and cut the path by medium."""
    cfg = Config.load(HYBRID_CONFIG)
    sc = load_scenario(scenario_path)
    raw = yaml.safe_load(Path(scenario_path).read_text())
    nogo = [_obstacle(o) for o in raw.get("nogo", [])]
    m = Mission(dataclasses.replace(sc, known=list(sc.known) + nogo), cfg, planner, condition, seed,
                resolution=resolution if planner == "astar" else None)
    ok, status, t_total, t_first = m.initial_plan()
    out = {"planner": planner, "seed": seed, "condition": condition, "status": status, "planning_s": t_total, "first_solution_s": t_first}
    if not ok:
        return out
    path = m.paths[0]
    segs = split_segments(path, m.energy, m.map.mu)
    by = {s.medium: s.energy_wh for s in segs}
    air = next(s for s in segs if s.medium == "AR")
    water = next(s for s in segs if s.medium == "AGUA")
    entry = air.waypoints[-1][:2]                                          # the vertical crossing column (north, east)
    out.update(path=path.tolist(), segments=[{"medium": s.medium, "waypoints": s.waypoints.tolist(), "energy_wh": s.energy_wh} for s in segs],
               energy_wh_by_medium=by, energy_wh_total=float(sum(by.values())), entry=entry.tolist(),
               path_length_m=float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))),
               transition_m=float(abs(HOVER_Z - SPAWN_Z)), air_waypoints=air.waypoints.tolist(), water_waypoints=water.waypoints.tolist())
    return out


def _obstacle_yaml(o, shift: np.ndarray) -> dict:
    """An obstacle of the scenario in another frame (translation only)."""
    if hasattr(o, "radius"):
        return {"type": "cylinder", "north": float(o.north - shift[0]), "east": float(o.east - shift[1]), "radius": float(o.radius),
                "z": [float(o.z_min - shift[2]), float(o.z_max - shift[2])]}
    return {"type": "box", "north": [float(o.n_min - shift[0]), float(o.n_max - shift[0])],
            "east": [float(o.e_min - shift[1]), float(o.e_max - shift[1])], "up": [float(o.u_min - shift[2]), float(o.u_max - shift[2])]}


def write_legs(scenario_path: str | Path, plan: dict, out_dir: str | Path) -> dict:
    """air_leg.yaml and water_leg.yaml, both in the surface frame (the Hydrone's executive adds the home offset). Both carry the whole world's
    obstacles (the props are spawned in each run) so the two videos show the same scene; the no-go volumes are not written."""
    sc = load_scenario(scenario_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    n_e, e_e = plan["entry"]
    start = sc.waypoints[0]
    target = sc.waypoints[-1]

    def obstacles(shift):
        return ([_obstacle_yaml(o, shift) for o in sc.known], [_obstacle_yaml(o, shift) for o in sc.hidden])

    # --- air leg: the SURFACE frame too (the planner's media are defined by height over the water); the executive adds the home offset
    known, hidden = obstacles(np.zeros(3))
    top = np.array([n_e, e_e, HOVER_Z])
    # the leg flies the hybrid plan's own AR waypoints (its energy-optimal descent is diagonal, not vertical: with the model's 1 m/s vertical speed a
    # diagonal at cruise speed costs less), ending on the crossing column at hover height instead of at the band edge z = +mu
    air_wps = [np.asarray(w, dtype=float) for w in plan["air_waypoints"] if w[2] > HOVER_Z + 0.5] + [top]
    air = {"name": "E1_AIR", "phase": "sitl", "description": "Stage E air leg: origin to the crossing column, 2 m above the water",
           "bounds": {"north": [-19.0, 25.0], "east": [-12.0, 12.0], "up": [float(HOVER_Z - 1.0), 22.4]},
           "waypoints": [{"name": f"plano_{k}", "pos": [float(v) for v in w]} for k, w in enumerate(air_wps)],
           "known": known, "hidden": hidden}
    (out / "air_leg.yaml").write_text(yaml.safe_dump(air, sort_keys=False))

    # --- water leg, in the surface frame (= the ROV frame)
    known, hidden = obstacles(np.zeros(3))
    spawn = np.array([n_e, e_e, SPAWN_Z])
    water = {"name": "E1_WATER", "phase": "sitl", "description": "Stage E water leg: from the crossing column to the underwater target",
             "bounds": {"north": [float(min(n_e, target[0]) - 4.0), 24.0], "east": [-9.0, 9.0], "up": [-7.0, -1.5]},
             "waypoints": [{"name": "coluna_de_entrada", "pos": [float(v) for v in spawn]}, {"name": "sub_agua", "pos": [float(v) for v in target]}],
             "known": known, "hidden": hidden}
    (out / "water_leg.yaml").write_text(yaml.safe_dump(water, sort_keys=False))
    return {"hover": top.tolist(), "spawn": spawn.tolist(), "spawn_bsim": [ORIGIN_BSIM[0] + n_e, ORIGIN_BSIM[1] - e_e, SPAWN_Z]}
