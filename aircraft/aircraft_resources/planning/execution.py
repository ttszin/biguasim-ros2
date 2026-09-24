"""Stage A of the T8.2 protocol: closed-loop test with a simple path follower.

`run_mission(scenario, cfg, planner, condition, seed)` plans a mission, flies
it with a kinematic follower, feeds the map through `atualizar_ocupacao` the way
the sensors would, replans when the remaining route is cut, and returns every
metric the T8.2 guide asks for. No simulator is needed, so it is fast and
reproducible (seeded); `biguasim_replay.py` replays the resulting trace inside
BiguaSim.

Conditions (one per run):
  K1 map fully known (static obstacles); moving ones arrive as sensor updates
  K2 unmapped obstacles discovered in flight within a limited perception radius
  K3 continuous map updates (LiDAR-like): larger radius, higher rate
  K4 underwater position uncertainty (2 %..20 %)
  K5 mission change during execution (new waypoint decided by T11)
  K6 reduced initial energy -> expects ENERGIA_INSUFICIENTE

The follower:
  * first-order velocity tracking with an acceleration limit;
  * speed by medium/direction from the energy config; inside the transition zone
    only vertical motion is commanded (the same rule the planners obey);
  * hovers (and burns hover power) while a replan runs, so slow planning costs
    time and energy;
  * executed energy = integral of instantaneous power over the trajectory, with
    the SAME thrust->power curves the planner uses. So delta E measures the
    difference between the planned and the flown trajectory (corner rounding,
    accelerations, pauses), NOT the fidelity of the consumption model.
"""

from __future__ import annotations

import math
import time
import tracemalloc
from dataclasses import dataclass, field

import numpy as np

import astar
import rrt_star
from common import ENERGIA_INSUFICIENTE, SEM_CAMINHO, SUCESSO, TIMEOUT
from config import Config
from energy import EnergyModel, J_PER_WH, wh
from hybrid_map import HybridMap
from medium import Medium, classify
from postprocess import prune
from replan import CLRRTReplanner, DStarLiteReplanner
from scenario import Scenario
from world import Box, Cylinder, World

TEMPO_TOTAL_EXCEDIDO = "TEMPO_TOTAL_EXCEDIDO"
COLISAO = "COLISAO"
TRAVADO = "TRAVADO"   # the vehicle stopped making progress (liveness guard)
STALL_S = 20.0

CONDITIONS = {
    "K1": {"desc": "map fully known"},
    "K2": {"desc": "unmapped obstacles discovered in flight", "radius": 12.0, "rate_hz": 2.0, "reveal": "hidden"},
    "K3": {"desc": "continuous map updates (LiDAR-like)", "radius": 30.0, "rate_hz": 5.0, "reveal": "all"},
    "K4": {"desc": "underwater position uncertainty", "uncertainty": 0.10},
    "K5": {"desc": "mission change during execution"},
    "K6": {"desc": "reduced initial energy", "energy_fraction": 0.5},
}
MOVING_RADIUS_M = 40.0      # moving obstacles (WhiteBoat) are seen within this range, in every condition
MOVING_RATE_HZ = 1.0
MOVING_HORIZON_S = 6.0      # how far ahead the shared WhiteBoat motion is swept into the map
MOVING_LATENCY_S = 0.5      # sensing + planning latency assumed when padding moving obstacles
SAMPLE_SPACING_M = 0.35     # sensor point spacing on obstacle surfaces (< margins, so no exploitable gaps)
DT = 0.2
TAU = 0.8                   # velocity lag (s)
A_MAX = 2.0                 # m/s^2
BAND_TRIM_MPS = 0.3         # max horizontal position trim inside the transition band
LOOKAHEAD_M = 2.0           # carrot distance ahead of the projection on the segment
A_BRAKE = 1.0               # m/s^2 used to plan braking before turns/stops (conservative: covers the velocity lag)


# ------------------------------------------------------------ sensor model

def surface_points(o, spacing: float, centre: np.ndarray, radius: float) -> np.ndarray:
    """Points on the surface of an obstacle that lie within `radius` of `centre`."""
    if isinstance(o, Cylinder):
        z0, z1 = max(o.z_min, centre[2] - radius), min(o.z_max, centre[2] + radius)
        if z1 < z0:
            return np.empty((0, 3))
        zs = np.arange(z0, z1 + 1e-9, spacing)
        th = np.linspace(0, 2 * np.pi, max(8, int(np.ceil(2 * np.pi * o.radius / spacing))), endpoint=False)
        zz, tt = np.meshgrid(zs, th, indexing="ij")
        pts = [np.stack([o.north + o.radius * np.cos(tt), o.east + o.radius * np.sin(tt), zz], -1).reshape(-1, 3)]
        for z in (o.z_min, o.z_max):
            if abs(z - centre[2]) <= radius:
                rs = np.arange(0.0, o.radius + 1e-9, spacing)
                for r in rs:
                    n = max(1, int(np.ceil(2 * np.pi * r / spacing)))
                    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
                    pts.append(np.stack([o.north + r * np.cos(a), o.east + r * np.sin(a), np.full(n, z)], -1))
        pts = np.vstack(pts)
    else:
        ns = np.arange(o.n_min, o.n_max + 1e-9, spacing)
        es = np.arange(o.e_min, o.e_max + 1e-9, spacing)
        us = np.arange(o.u_min, o.u_max + 1e-9, spacing)
        faces = []
        for n in (o.n_min, o.n_max):
            e_, u_ = np.meshgrid(es, us, indexing="ij")
            faces.append(np.stack([np.full(e_.shape, n), e_, u_], -1).reshape(-1, 3))
        for e in (o.e_min, o.e_max):
            n_, u_ = np.meshgrid(ns, us, indexing="ij")
            faces.append(np.stack([n_, np.full(n_.shape, e), u_], -1).reshape(-1, 3))
        for u in (o.u_min, o.u_max):
            n_, e_ = np.meshgrid(ns, es, indexing="ij")
            faces.append(np.stack([n_, e_, np.full(n_.shape, u)], -1).reshape(-1, 3))
        pts = np.vstack(faces)
    return pts[np.linalg.norm(pts - centre, axis=1) <= radius]


def obstacle_distance(o, p: np.ndarray) -> float:
    return float(World(bounds=[[-1e9, 1e9]] * 3, cylinders=[o] if isinstance(o, Cylinder) else [],
                       boxes=[o] if isinstance(o, Box) else []).clearance(p[None, :])[0])


# ----------------------------------------------------------------- results

@dataclass
class RunResult:
    scenario: str
    planner: str
    resolution: float | None
    condition: str
    seed: int
    status: str = SUCESSO
    detail: str = ""
    t_initial_ms: float = float("nan")
    t_first_solution_ms: float = float("nan")
    peak_mem_mb: float = float("nan")
    n_replans: int = 0
    replan_ms: list = field(default_factory=list)
    replan_kinds: list = field(default_factory=list)
    realtime_ok: bool = True
    energy_planned_wh: float = float("nan")
    energy_exec_wh: float = float("nan")
    delta_e_wh: float = float("nan")
    energy_by_medium_wh: dict = field(default_factory=dict)
    length_planned_m: float = float("nan")
    length_exec_m: float = float("nan")
    n_waypoints: int = 0
    min_clearance_m: float = float("nan")
    mission_time_s: float = float("nan")
    trace: list = field(default_factory=list)          # (t, north, east, up)
    planned_paths: list = field(default_factory=list)  # first plan per leg (arrays)


# ------------------------------------------------------------------ harness

def _leg_budget(cfg: Config, planner: str) -> float:
    return float(cfg.get("rrt_star", "time_budget_s") if planner == "rrt_star" else cfg.get("astar", "time_budget_s"))


class Mission:
    def __init__(self, sc: Scenario, cfg: Config, planner: str, condition: str, seed: int,
                 resolution: float | None = None, uncertainty: float | None = None, energy_fraction: float | None = None):
        self.sc, self.cfg, self.planner, self.cond, self.seed = sc, cfg, planner, condition, seed
        self.res = resolution
        self.energy = EnergyModel.from_config(cfg)
        self.opts = dict(CONDITIONS[condition])
        if uncertainty is not None:
            self.opts["uncertainty"] = uncertainty
        if energy_fraction is not None:
            self.opts["energy_fraction"] = energy_fraction
        self.vehicle_radius = float(cfg.get("margins", "vehicle_radius"))
        self.reserve = float(cfg.get("energy", "reserve_wh"))
        self.available = float(cfg.get("energy", "battery_wh"))
        self.speed_min = min(cfg.get("energy", "air")["cruise_speed"], cfg.get("energy", "water")["cruise_speed"])

        static_truth = list(sc.known) + list(sc.hidden)
        known = static_truth if condition in ("K1", "K4", "K5", "K6") else list(sc.known)
        w = World(bounds=sc.bounds)
        for o in known:
            (w.cylinders if isinstance(o, Cylinder) else w.boxes).append(o)
        self.map = HybridMap(w, cfg)
        if condition == "K4":
            self.map.set_position_uncertainty(self.opts["uncertainty"], self._gps_fix_point())
        self.truth_static = World(bounds=[[-1e9, 1e9]] * 3)
        for o in static_truth:
            (self.truth_static.cylinders if isinstance(o, Cylinder) else self.truth_static.boxes).append(o)
        self.hidden_revealed: set = set()
        self._moving_cells: dict[str, np.ndarray] = {}
        self.waypoints = [wp.copy() for wp in sc.waypoints]
        self.replanners: list = []
        self.paths: list[np.ndarray] = []

    def _gps_fix_point(self) -> np.ndarray:
        """Last place with a GPS fix before the underwater phase: the waypoint just before
        the first underwater one (or that waypoint itself if the mission starts underwater)."""
        mu = self.map.mu if hasattr(self, "map") else float(self.cfg.get("medium", "mu"))
        for j, wp in enumerate(self.sc.waypoints):
            if wp[2] < -mu:
                return self.sc.waypoints[j - 1] if j > 0 else wp
        return self.sc.waypoints[0]

    # ---- planning ----------------------------------------------------

    def _make_replanner(self, k: int, start, goal):
        if self.planner == "rrt_star":
            return CLRRTReplanner(self.map, self.energy, self.cfg, start, goal, seed=self.seed + 1000 * k)
        return DStarLiteReplanner(self.map, self.energy, self.cfg, start, goal, seed=self.seed + 1000 * k,
                                  resolution=self.res)

    def initial_plan(self) -> tuple[bool, str, float, float]:
        """Plan every leg. Returns (ok, status, total seconds, seconds to first solution)."""
        total, first = 0.0, 0.0
        self.paths, self.replanners, self.leg_initial_s = [], [], []
        for k, (a, b) in enumerate(zip(self.waypoints[:-1], self.waypoints[1:])):
            if self.planner == "astar":
                r = astar.plan(self.map, self.energy, self.cfg, a, b, seed=self.seed + 1000 * k, resolution=self.res)
                t_first = r.time_s
            else:
                rp = self._make_replanner(k, a, b)
                r = rp.initial(improve=True)
                t_first = r.extra.get("first_solution_time") or r.time_s
                # Keep the tree that produced the initial plan: CL-RRT replanning must REUSE it
                # (invalidate the cut branches, re-root, regrow), not plan again from scratch.
                self.replanners.append(rp)
            t0 = time.perf_counter()
            if r.success:
                r.path = prune(r.path, self.map, self.energy, float(self.cfg.get("postprocess", "min_gain")))
            t_post = time.perf_counter() - t0
            total += r.time_s + t_post
            first += t_first + t_post
            self.leg_initial_s.append(r.time_s + t_post)
            if not r.success:
                return False, r.status, total, first
            self.paths.append(r.path)
        return True, SUCESSO, total, first

    def build_replanners(self) -> float:
        """A* family: D* Lite is initialised on the map as it stands (background setup, timed apart)."""
        t0 = time.perf_counter()
        if self.planner == "astar":
            for k, (a, b) in enumerate(zip(self.waypoints[:-1], self.waypoints[1:])):
                rp = self._make_replanner(k, a, b)
                rp.initial()
                self.replanners.append(rp)
        return time.perf_counter() - t0

    def planned_energy_j(self, paths=None) -> float:
        return sum(self.energy.path_energy(p).total for p in (paths if paths is not None else self.paths))

    # ---- sensing -----------------------------------------------------

    def _insert(self, pts: np.ndarray, t: float) -> None:
        if len(pts):
            self.map.atualizar_ocupacao(pts, "lidar", t)

    def sense_static(self, pos, t: float) -> None:
        radius = self.opts.get("radius")
        if radius is None:
            return
        obstacles = self.sc.hidden if self.opts["reveal"] == "hidden" else list(self.sc.known) + list(self.sc.hidden)
        for i, o in enumerate(obstacles):
            if obstacle_distance(o, pos) > radius:
                continue
            pts = surface_points(o, SAMPLE_SPACING_M, pos, radius)
            if len(pts) == 0:
                continue
            keys = {tuple(k) for k in np.floor(pts / self.map.voxel).astype(int).tolist()}
            fresh = keys - self.hidden_revealed
            if fresh:
                self.hidden_revealed |= fresh
                self._insert(pts, t)

    def sense_moving(self, pos, t: float) -> None:
        for m in self.sc.moving:
            box = m.box_at(t)
            visible = obstacle_distance(box, pos) <= MOVING_RADIUS_M
            # The shared WhiteBoat position/velocity (T10) lets the map hold the volume the boat
            # sweeps over the next MOVING_HORIZON_S seconds (plus latency padding), not just where
            # it is now; otherwise the conflict is only seen once the boat is already on the route.
            ahead = m.box_at(t + MOVING_HORIZON_S)
            pad = m.speed_at(t) * MOVING_LATENCY_S
            box = Box(min(box.n_min, ahead.n_min) - pad, max(box.n_max, ahead.n_max) + pad,
                      min(box.e_min, ahead.e_min) - pad, max(box.e_max, ahead.e_max) + pad, box.u_min, box.u_max)
            old = self._moving_cells.pop(m.name, None)
            if old is not None:
                self.map.atualizar_ocupacao(np.hstack([old, np.zeros((len(old), 1))]), "lidar", t)
            if visible:
                pts = surface_points(box, SAMPLE_SPACING_M * 1.5, pos, MOVING_RADIUS_M)
                self._moving_cells[m.name] = pts
                self._insert(pts, t)

    # ---- truth -------------------------------------------------------

    def clearance_and_hit(self, p, t: float) -> tuple[float, bool]:
        c = float(self.truth_static.clearance(p[None, :])[0])
        for m in self.sc.moving:
            c = min(c, obstacle_distance(m.box_at(t), p))
        return c, c < self.vehicle_radius

    # ---- speeds ------------------------------------------------------

    def waypoint_speed(self, paths, leg: int, idx: int, nominal: float) -> tuple[float, bool]:
        """Speed the vehicle should have when it reaches waypoint idx of the current leg, and
        whether it must effectively stop there.

        Straight through: keep nominal; the sharper the turn the slower ((1 + cos turn)/2: 90 deg
        -> half, reversal -> 0). Stop before any segment that touches the transition band after
        a horizontal approach, so the crossing starts vertical (no residual horizontal motion).
        """
        path = paths[leg]
        last_leg = leg == len(paths) - 1
        if idx == len(path) - 1:
            if last_leg:
                return 0.0, True
            nxt = paths[leg + 1]
            b = nxt[1] - nxt[0]
        else:
            b = path[idx + 1] - path[idx]
        a = path[idx] - path[idx - 1]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            return nominal, False
        mu = self.map.mu
        seg_end = path[idx] + b
        touches_band = (min(path[idx][2], seg_end[2]) < mu and max(path[idx][2], seg_end[2]) > -mu) or abs(path[idx][2]) <= mu
        if touches_band and np.linalg.norm(a[:2]) > 0.1:
            return 0.0, True
        cos_turn = float(a @ b) / (na * nb)
        v_turn = nominal * 0.5 * (1.0 + cos_turn)
        # The velocity lag makes the vehicle overshoot a corner by about v * TAU: cap the
        # arrival speed so that overshoot fits inside the waypoint's clearance to obstacles.
        clearance = float(self.map.clearance(path[idx][None, :])[0])
        v_room = max(0.3, (clearance - self.vehicle_radius - 0.3) / TAU)
        return min(v_turn, v_room), False

    def desired_velocity(self, pos, target, v_target: float, origin=None) -> np.ndarray:
        """Line following with a carrot: aim at a point LOOKAHEAD_M ahead of the vehicle's
        projection on the segment origin->target, so cross-track error is pulled back to the
        planned line (planned clearance is measured along that line, not around the waypoint)."""
        d = target - pos
        dist = float(np.linalg.norm(d))
        if dist < 1e-9:
            return np.zeros(3)
        aim = d
        if origin is not None:
            seg = target - origin
            L = float(np.linalg.norm(seg))
            if L > 1e-9:
                t = float(np.clip(((pos - origin) @ seg) / (L * L) + LOOKAHEAD_M / L, 0.0, 1.0))
                aim = origin + t * seg - pos
        na = float(np.linalg.norm(aim))
        u = aim / na if na > 1e-9 else d / dist
        medium = classify(pos[2], self.map.mu)
        if medium is Medium.TRANSITION:
            # planned motion in the band is vertical; the only horizontal command is a slow
            # correction of position error (<= BAND_TRIM_MPS), as a real vehicle would trim drift
            vz = np.sign(d[2]) * self.cfg.get("energy", "transition")["speed"] if abs(d[2]) > 0.05 else 0.0
            trim = d[:2]
            n = float(np.linalg.norm(trim))
            trim = trim * min(1.0, BAND_TRIM_MPS / n) if n > 1e-9 else trim
            if abs(d[2]) > 0.05:
                vz = min(abs(vz), math.sqrt(v_target ** 2 + 2 * A_BRAKE * abs(d[2]))) * np.sign(d[2])
            return np.array([trim[0], trim[1], vz])
        else:
            prm = self.cfg.get("energy", "air" if medium is Medium.AIR else "water")
            speed = prm["vertical_speed"] if abs(u[2]) > 0.9 else prm["cruise_speed"]
        # brake so that speed at the waypoint is v_target (A_BRAKE is conservative: the
        # first-order velocity lag adds stopping distance on top of v^2 / 2a)
        speed = min(speed, math.sqrt(v_target ** 2 + 2 * A_BRAKE * dist))
        return u * speed

    def nominal_speed(self, pos, target) -> float:
        d = target - pos
        n = np.linalg.norm(d)
        medium = classify(pos[2], self.map.mu)
        if medium is Medium.TRANSITION:
            return self.cfg.get("energy", "transition")["speed"]
        prm = self.cfg.get("energy", "air" if medium is Medium.AIR else "water")
        return prm["vertical_speed"] if n > 1e-9 and abs(d[2] / n) > 0.9 else prm["cruise_speed"]

    def accept_radius(self, path, i: int, pos) -> float:
        """Fly-by radius for waypoint i. Cutting a corner by r brings the vehicle up to r closer
        to whatever the waypoint hugs, so r is capped by the waypoint's clearance (known map):
        never eat into the vehicle radius. Tight near the transition band and at the goal."""
        wp = path[i]
        if i == len(path) - 1:
            return 0.4
        if abs(wp[2]) <= self.map.mu + 1.0:
            return 0.3
        clearance = float(self.map.clearance(wp[None, :])[0])
        return float(min(1.5, max(0.3, clearance - self.vehicle_radius - 0.3)))


def run_mission(sc: Scenario, cfg: Config, planner: str, condition: str = "K1", seed: int = 0,
                resolution: float | None = None, uncertainty: float | None = None,
                energy_fraction: float | None = None, measure_memory: bool = False,
                keep_trace: bool = True) -> RunResult:
    m = Mission(sc, cfg, planner, condition, seed, resolution, uncertainty, energy_fraction)
    res = RunResult(sc.name, planner, resolution if planner == "astar" else None, condition, seed)

    if measure_memory:
        tracemalloc.start()
    ok, status, t_init, t_first = m.initial_plan()
    if measure_memory:
        res.peak_mem_mb = tracemalloc.get_traced_memory()[1] / 1e6
        tracemalloc.stop()
    res.t_initial_ms, res.t_first_solution_ms = t_init * 1000, t_first * 1000
    if not ok:
        res.status = status
        res.detail = "initial plan failed"
        return res
    res.planned_paths = [p.copy() for p in m.paths]

    planned_j = m.planned_energy_j()
    res.energy_planned_wh = wh(planned_j)
    res.length_planned_m = float(sum(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)) for p in m.paths))
    res.n_waypoints = int(sum(len(p) for p in m.paths) - (len(m.paths) - 1))
    if condition == "K6":
        m.available = m.reserve + m.opts["energy_fraction"] * wh(planned_j)
    if wh(planned_j) > m.available - m.reserve:
        res.status = ENERGIA_INSUFICIENTE
        res.detail = f"plan needs {wh(planned_j):.2f} Wh, available {m.available:.2f} Wh - reserve {m.reserve:.1f} Wh"
        return res
    setup_s = m.build_replanners()
    res.detail = f"replanner setup {setup_s:.2f}s" if planner == "astar" else ""

    # ---- flight ------------------------------------------------------
    total_traverse = res.length_planned_m / max(m.speed_min, 1e-6)
    time_limit = 3.0 * total_traverse + 60.0
    pos = m.paths[0][0].copy()
    vel = np.zeros(3)
    t, exec_j, exec_len = 0.0, 0.0, 0.0
    by_medium = {"AR": 0.0, "AGUA": 0.0, "TRANSICAO": 0.0}
    min_clear = float("inf")
    leg, idx = 0, 1
    last_static_sense = last_moving_sense = -1e9
    checked_version = m.map.version
    change = sc.mission_change if condition == "K5" else None
    change_done = False
    trace = [(0.0, *pos)] if keep_trace else []
    last_progress_t, last_progress_pos = 0.0, pos.copy()

    def remaining_energy_j(from_pos, k, i):
        j = 0.0
        cur = np.vstack([from_pos[None, :], m.paths[k][i:]]) if i < len(m.paths[k]) else from_pos[None, :]
        j += m.energy.path_energy(cur).total
        for later in m.paths[k + 1:]:
            j += m.energy.path_energy(later).total
        return j

    def pause(seconds: float) -> None:
        nonlocal t, exec_j
        p_hover = m.energy.instant_power(pos[2], np.zeros(3))
        exec_j += p_hover * seconds
        by_medium[classify(pos[2], m.map.mu).value] += p_hover * seconds
        t += seconds

    def record_replan(kind: str, seconds: float, remaining_s: float) -> None:
        res.n_replans += 1
        res.replan_ms.append(seconds * 1000)
        res.replan_kinds.append(kind)
        if seconds > remaining_s:
            res.realtime_ok = False

    def install(path, k):
        """Adopt a replanned path for leg k starting from the current position."""
        path = np.asarray(path, dtype=float)
        if np.linalg.norm(path[0] - pos) > 1e-6:
            path = np.vstack([pos[None, :], path])
        m.paths[k] = prune(path, m.map, m.energy, float(cfg.get("postprocess", "min_gain")))

    def guard_energy() -> str | None:
        if wh(exec_j + remaining_energy_j(pos, leg, idx)) > m.available - m.reserve:
            return ENERGIA_INSUFICIENTE
        return None

    while True:
        # end of mission
        if leg >= len(m.paths):
            break
        if t > time_limit:
            res.status, res.detail = TEMPO_TOTAL_EXCEDIDO, f"{t:.0f}s > {time_limit:.0f}s"
            break

        # sensors
        if condition in ("K2", "K3") and t - last_static_sense >= 1.0 / m.opts["rate_hz"]:
            m.sense_static(pos, t)
            last_static_sense = t
        if m.sc.moving and t - last_moving_sense >= 1.0 / MOVING_RATE_HZ:
            m.sense_moving(pos, t)
            last_moving_sense = t

        # mission change (K5)
        if change and not change_done and t >= change["t"]:
            change_done = True
            k_new = int(change["index"])
            if k_new <= leg:
                res.detail = (res.detail + "; " if res.detail else "") + f"mission change ignored: waypoint {k_new} already reached"
            if k_new > leg:
                m.waypoints[k_new] = np.array(change["goal"], dtype=float)
                for k in (k_new - 1, k_new):
                    if k >= len(m.paths) or k < leg:
                        continue
                    t0 = time.perf_counter()
                    a = pos if k == leg else m.waypoints[k]
                    b = m.waypoints[k + 1]
                    if k == leg:
                        rp = m.replanners[k] if k < len(m.replanners) else None
                        if rp is None:
                            rp = m._make_replanner(k, a, b)
                            r = rp.initial(improve=False) if planner == "rrt_star" else rp.initial()
                            m.replanners.append(rp)
                        else:
                            rp.move_start(pos)
                            r = rp.set_goal(b)
                    else:
                        rp = m._make_replanner(k, a, b)
                        r = rp.initial(improve=False) if planner == "rrt_star" else rp.initial()
                        while len(m.replanners) <= k:
                            m.replanners.append(None)
                        m.replanners[k] = rp
                    if not r.success:
                        res.status, res.detail = r.status, f"mission change, leg {k}"
                        break
                    install(r.path, k) if k == leg else m.paths.__setitem__(k, prune(r.path, m.map, m.energy, float(cfg.get("postprocess", "min_gain"))))
                    elapsed = time.perf_counter() - t0
                    rem_s = np.sum(np.linalg.norm(np.diff(m.paths[k], axis=0), axis=1)) / max(m.speed_min, 1e-6)
                    record_replan("mission_change", elapsed, rem_s)
                    pause(elapsed)
                    if k == leg:
                        idx = 1
                if res.status != SUCESSO:
                    break
                g = guard_energy()
                if g:
                    res.status, res.detail = g, "after mission change"
                    break

        # replan trigger: the remaining route of this leg got cut
        if m.map.version != checked_version:
            checked_version = m.map.version
            remaining = np.vstack([pos[None, :], m.paths[leg][idx:]])
            if not m.map.path_free(remaining):
                t0 = time.perf_counter()
                if m.replanners and leg < len(m.replanners) and m.replanners[leg] is not None:
                    rp = m.replanners[leg]
                else:
                    rp = m._make_replanner(leg, pos, m.waypoints[leg + 1])
                    rp.initial(improve=False) if planner == "rrt_star" else rp.initial()
                    while len(m.replanners) <= leg:
                        m.replanners.append(None)
                    m.replanners[leg] = rp
                rp.move_start(pos)
                r = rp.update()
                if not r.success:
                    record_replan("map", time.perf_counter() - t0, 0.0)
                    res.status, res.detail = r.status, f"replan failed at t={t:.1f}s"
                    break
                install(r.path, leg)
                elapsed = time.perf_counter() - t0
                rem_s = np.sum(np.linalg.norm(np.diff(m.paths[leg], axis=0), axis=1)) / max(m.speed_min, 1e-6)
                record_replan("map", elapsed, rem_s)
                pause(elapsed)
                idx = 1
                g = guard_energy()
                if g:
                    res.status, res.detail = g, f"after replan at t={t:.1f}s"
                    break

        # liveness guard: no real progress for STALL_S of mission time
        if np.linalg.norm(pos - last_progress_pos) > 0.5:
            last_progress_t, last_progress_pos = t, pos.copy()
        elif t - last_progress_t > STALL_S:
            res.status = TRAVADO
            res.detail = (f"no progress for {STALL_S:.0f}s at t={t:.0f}s: pos {np.round(pos, 1).tolist()}, "
                          f"target {np.round(m.paths[leg][idx], 1).tolist()}, leg {leg}, idx {idx}/{len(m.paths[leg])}, "
                          f"vel {np.round(vel, 2).tolist()}")
            break

        # follow
        path = m.paths[leg]
        target = path[idx]
        v_wp, must_stop = m.waypoint_speed(m.paths, leg, idx, m.nominal_speed(pos, target))
        v_des = m.desired_velocity(pos, target, v_wp, origin=path[idx - 1])
        dv = v_des - vel
        step = np.linalg.norm(dv)
        max_dv = A_MAX * DT
        vel_new = vel + dv * min(1.0, DT / TAU) if step * min(1.0, DT / TAU) <= max_dv else vel + dv / step * max_dv
        acc = (vel_new - vel) / DT
        p_w = m.energy.instant_power(pos[2], vel_new, acc)
        new_pos = pos + vel_new * DT
        exec_j += p_w * DT + m.energy.crossing_cost(pos, new_pos)
        by_medium[classify(pos[2], m.map.mu).value] += p_w * DT
        exec_len += float(np.linalg.norm(new_pos - pos))
        pos, vel = new_pos, vel_new
        t += DT
        c, hit = m.clearance_and_hit(pos, t)
        min_clear = min(min_clear, c)
        if keep_trace:
            trace.append((t, *pos))
        if hit:
            res.status, res.detail = COLISAO, f"t={t:.1f}s at {np.round(pos, 1).tolist()}"
            break

        if np.linalg.norm(pos - target) <= m.accept_radius(path, idx, pos) and (not must_stop or np.linalg.norm(vel[:2]) < 0.15):
            idx += 1
            if idx >= len(path):
                leg += 1
                idx = 1
                if leg < len(m.paths):
                    # the next leg may have been cut while flying this one
                    nxt = m.paths[leg]
                    if not m.map.path_free(nxt):
                        checked_version = -1

    res.mission_time_s = t
    res.energy_exec_wh = wh(exec_j)
    res.energy_by_medium_wh = {k: wh(v) for k, v in by_medium.items()}
    res.length_exec_m = exec_len
    res.min_clearance_m = min_clear
    if res.status == SUCESSO:
        res.delta_e_wh = res.energy_planned_wh - res.energy_exec_wh
    res.trace = trace
    return res
