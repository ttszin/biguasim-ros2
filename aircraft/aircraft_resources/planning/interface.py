"""Common planner interface (identical for A* and RRT*).

Input  : mission waypoints (origin, ROI, underwater position, return), the map,
         vehicle state with uncertainty, available energy.
Output : pruned waypoints split into legs per medium (AR / TRANSICAO / AGUA),
         estimated energy per leg and in total, and a status:
         SUCESSO, SEM_CAMINHO, TIMEOUT or ENERGIA_INSUFICIENTE.

Energy feasibility (redone at every replanning):
    energy of the remaining legs (the return is the last mission waypoint)
    <= available energy - reserve
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

import astar
import rrt_star
from common import ENERGIA_INSUFICIENTE, SUCESSO, PlanResult
from config import Config
from energy import EnergyBreakdown, J_PER_WH, wh
from medium import Medium, split_by_medium
from postprocess import prune

PLANNERS = {"astar": astar.plan, "rrt_star": rrt_star.plan}


@dataclass
class VehicleState:
    position: np.ndarray               # local frame (north, east, up)
    position_sigma_m: float = 0.0      # 1-sigma position uncertainty
    energy_available_wh: float | None = None  # None -> config battery


@dataclass
class PlanRequest:
    waypoints: list[np.ndarray]        # ordered mission waypoints, local frame; [0] is the start
    hmap: object
    energy: object
    cfg: Config
    planner: str = "rrt_star"
    seed: int = 0
    vehicle: VehicleState | None = None
    options: dict = field(default_factory=dict)  # planner kwargs (resolution, time_budget_s, ...)


@dataclass
class Segment:
    medium: str
    waypoints: np.ndarray
    energy_wh: float


@dataclass
class PlanResponse:
    status: str
    waypoints: np.ndarray                  # pruned, to fly
    segments: list[Segment]                # split by medium
    energy_wh_total: float
    energy_wh_by_medium: dict
    energy_wh_by_leg: list[float]
    time_s: float                          # planning time, all legs + post-processing
    legs: list[PlanResult]
    path_length_m: float = 0.0
    feasible_energy: bool = True
    detail: str = ""


def split_segments(path: np.ndarray, energy, mu: float) -> list[Segment]:
    """Cut a path into runs that stay in one medium; merge consecutive pieces."""
    segs: list[tuple[Medium, list[np.ndarray], float]] = []
    for a, b in zip(path[:-1], path[1:]):
        for pa, pb, medium in split_by_medium(a, b, mu):
            e = energy.segment_energy(pa, pb).total
            if segs and segs[-1][0] is medium:
                segs[-1][1].append(pb)
                segs[-1] = (medium, segs[-1][1], segs[-1][2] + e)
            else:
                segs.append((medium, [pa, pb], e))
        cross = energy.crossing_cost(a, b)
        if cross and segs:
            segs[-1] = (segs[-1][0], segs[-1][1], segs[-1][2] + cross)
    return [Segment(m.value, np.array(w), wh(e)) for m, w, e in segs]


def plan_mission(req: PlanRequest) -> PlanResponse:
    t0 = time.perf_counter()
    plan_fn = PLANNERS[req.planner]
    pts = [np.asarray(p, dtype=float) for p in req.waypoints]
    if req.vehicle is not None:
        pts[0] = np.asarray(req.vehicle.position, dtype=float)
    cfg = req.cfg
    reserve = float(cfg.get("energy", "reserve_wh"))
    available = (req.vehicle.energy_available_wh if req.vehicle and req.vehicle.energy_available_wh is not None
                 else float(cfg.get("energy", "battery_wh")))

    legs: list[PlanResult] = []
    full = [pts[0]]
    for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        r = plan_fn(req.hmap, req.energy, cfg, a, b, seed=req.seed + k, **req.options)
        legs.append(r)
        if not r.success:
            return PlanResponse(r.status, np.empty((0, 3)), [], float("inf"), {}, [], time.perf_counter() - t0, legs,
                                detail=f"leg {k} ({a.tolist()} -> {b.tolist()}): {r.extra.get('reason', r.status)}")
        pruned = prune(r.path, req.hmap, req.energy, min_gain=float(cfg.get("postprocess", "min_gain")))
        full.extend(pruned[1:])
    path = np.array(full)

    by_leg = []
    total = EnergyBreakdown()
    for a, b in zip(path[:-1], path[1:]):
        e = req.energy.segment_energy(a, b)
        e.transition += req.energy.crossing_cost(a, b)
        by_leg.append(wh(e.total))
        total = total + e
    feasible = wh(total.total) <= available - reserve
    return PlanResponse(
        status=SUCESSO if feasible else ENERGIA_INSUFICIENTE,
        waypoints=path,
        segments=split_segments(path, req.energy, req.hmap.mu),
        energy_wh_total=wh(total.total),
        energy_wh_by_medium={"AR": wh(total.air), "AGUA": wh(total.water), "TRANSICAO": wh(total.transition)},
        energy_wh_by_leg=by_leg,
        time_s=time.perf_counter() - t0,
        legs=legs,
        path_length_m=float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))),
        feasible_energy=feasible,
        detail=f"energy {wh(total.total):.2f} Wh vs available {available:.1f} - reserve {reserve:.1f}",
    )
