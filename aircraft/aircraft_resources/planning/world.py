"""Obstacle world model shared by the A* and RRT* planners.

Frame: (north, east, up) in metres, origin at the vehicle's home position --
the same frame `go_to_known_gps_waypoint` uses in the mission YAMLs, so a
planned path can be written out as mission waypoints with no conversion.

Obstacles are vertical cylinders (trees, poles) and axis-aligned boxes
(buildings). Both are inflated by `safety_margin` for every collision query,
so planners never need to reason about vehicle size themselves.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class Cylinder:
    north: float
    east: float
    radius: float
    z_min: float
    z_max: float


@dataclass(frozen=True)
class Box:
    n_min: float
    n_max: float
    e_min: float
    e_max: float
    u_min: float
    u_max: float


def _clip_to_interval(p_u: float, d_u: float, lo: float, hi: float) -> tuple[float, float]:
    """Sub-interval of t in [0, 1] where p_u + t*d_u lies in [lo, hi]."""
    if abs(d_u) < 1e-12:
        return (0.0, 1.0) if lo <= p_u <= hi else (1.0, 0.0)
    t0, t1 = (lo - p_u) / d_u, (hi - p_u) / d_u
    if t0 > t1:
        t0, t1 = t1, t0
    return max(0.0, t0), min(1.0, t1)


@dataclass
class World:
    bounds: np.ndarray  # shape (3, 2): [[n_lo, n_hi], [e_lo, e_hi], [u_lo, u_hi]]
    cylinders: list[Cylinder] = field(default_factory=list)
    boxes: list[Box] = field(default_factory=list)
    safety_margin: float = 0.5

    def __post_init__(self) -> None:
        self.bounds = np.asarray(self.bounds, dtype=float)

    # ---- point queries -------------------------------------------------

    def in_bounds(self, p: np.ndarray) -> np.ndarray:
        p = np.asarray(p, dtype=float)
        return np.all((p >= self.bounds[:, 0]) & (p <= self.bounds[:, 1]), axis=-1)

    def points_free(self, pts: np.ndarray, margin: float | None = None) -> np.ndarray:
        """Vectorised collision test for an (N, 3) array; True = free.
        `margin` overrides safety_margin for this query (per-medium clearance)."""
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        free = self.in_bounds(pts)
        m = self.safety_margin if margin is None else margin
        for c in self.cylinders:
            d2 = (pts[:, 0] - c.north) ** 2 + (pts[:, 1] - c.east) ** 2
            inside = (d2 <= (c.radius + m) ** 2) & (pts[:, 2] >= c.z_min - m) & (pts[:, 2] <= c.z_max + m)
            free &= ~inside
        for b in self.boxes:
            inside = (
                (pts[:, 0] >= b.n_min - m) & (pts[:, 0] <= b.n_max + m)
                & (pts[:, 1] >= b.e_min - m) & (pts[:, 1] <= b.e_max + m)
                & (pts[:, 2] >= b.u_min - m) & (pts[:, 2] <= b.u_max + m)
            )
            free &= ~inside
        return free

    def point_free(self, p: Sequence[float], margin: float | None = None) -> bool:
        return bool(self.points_free(np.asarray(p, dtype=float)[None, :], margin)[0])

    # ---- segment queries (exact, not sampled) --------------------------

    def _arrays(self):
        """Cached (cylinders (N,5), boxes (M,6)) arrays for the vectorised tests."""
        key = (len(self.cylinders), len(self.boxes))
        if getattr(self, "_arr_key", None) != key:
            self._cyl = np.array([[c.north, c.east, c.radius, c.z_min, c.z_max] for c in self.cylinders]).reshape(-1, 5)
            self._box = np.array([[b.n_min, b.n_max, b.e_min, b.e_max, b.u_min, b.u_max] for b in self.boxes]).reshape(-1, 6)
            self._arr_key = key
        return self._cyl, self._box

    def segment_free(self, p: np.ndarray, q: np.ndarray, margin: float | None = None) -> bool:
        p = np.asarray(p, dtype=float)
        q = np.asarray(q, dtype=float)
        if not (self.in_bounds(p) and self.in_bounds(q)):
            return False
        d = q - p
        m = self.safety_margin if margin is None else margin
        cyl, box = self._arrays()

        if len(cyl):
            lo, hi = cyl[:, 3] - m, cyl[:, 4] + m
            if abs(d[2]) < 1e-12:
                ok = (lo <= p[2]) & (p[2] <= hi)
                ta, tb = np.zeros(len(cyl)), np.where(ok, 1.0, -1.0)
            else:
                t0, t1 = (lo - p[2]) / d[2], (hi - p[2]) / d[2]
                ta = np.maximum(0.0, np.minimum(t0, t1))
                tb = np.minimum(1.0, np.maximum(t0, t1))
            valid = ta <= tb
            if valid.any():
                rel = cyl[:, :2] - p[:2]
                dd = float(d[:2] @ d[:2])
                t = ta if dd < 1e-12 else np.minimum(np.maximum((rel @ d[:2]) / dd, ta), tb)
                closest = p[:2] + t[:, None] * d[:2]
                hit = valid & (np.sum((closest - cyl[:, :2]) ** 2, axis=1) <= (cyl[:, 2] + m) ** 2)
                if hit.any():
                    return False

        if len(box):
            lo = box[:, [0, 2, 4]] - m
            hi = box[:, [1, 3, 5]] + m
            t_lo, t_hi = np.zeros(len(box)), np.ones(len(box))
            hit = np.ones(len(box), dtype=bool)
            for ax in range(3):
                if abs(d[ax]) < 1e-12:
                    hit &= (lo[:, ax] <= p[ax]) & (p[ax] <= hi[:, ax])
                else:
                    a, b = (lo[:, ax] - p[ax]) / d[ax], (hi[:, ax] - p[ax]) / d[ax]
                    t_lo = np.maximum(t_lo, np.minimum(a, b))
                    t_hi = np.minimum(t_hi, np.maximum(a, b))
            if (hit & (t_lo <= t_hi)).any():
                return False
        return True

    def path_free(self, path: np.ndarray) -> bool:
        path = np.asarray(path, dtype=float)
        return all(self.segment_free(path[i], path[i + 1]) for i in range(len(path) - 1))

    # ---- clearance (distance to true, un-inflated surfaces) -------------

    def clearance(self, pts: np.ndarray) -> np.ndarray:
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        best = np.full(len(pts), np.inf)
        for c in self.cylinders:
            dh = np.maximum(0.0, np.hypot(pts[:, 0] - c.north, pts[:, 1] - c.east) - c.radius)
            dv = np.maximum(0.0, np.maximum(c.z_min - pts[:, 2], pts[:, 2] - c.z_max))
            best = np.minimum(best, np.hypot(dh, dv))
        for b in self.boxes:
            dn = np.maximum(0.0, np.maximum(b.n_min - pts[:, 0], pts[:, 0] - b.n_max))
            de = np.maximum(0.0, np.maximum(b.e_min - pts[:, 1], pts[:, 1] - b.e_max))
            du = np.maximum(0.0, np.maximum(b.u_min - pts[:, 2], pts[:, 2] - b.u_max))
            best = np.minimum(best, np.sqrt(dn**2 + de**2 + du**2))
        return best

    def path_min_clearance(self, path: np.ndarray, step: float = 0.25) -> float:
        path = np.asarray(path, dtype=float)
        samples = [path[0]]
        for a, b in zip(path[:-1], path[1:]):
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
            samples.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
        return float(np.min(self.clearance(np.array(samples))))


@dataclass
class Scenario:
    name: str
    world: World
    start: np.ndarray
    goal: np.ndarray


def load_scenario(path: str | Path) -> Scenario:
    cfg = yaml.safe_load(Path(path).read_text())
    b = cfg["bounds"]
    world = World(
        bounds=[b["north"], b["east"], b["up"]],
        safety_margin=float(cfg.get("safety_margin", 0.5)),
    )
    for o in cfg.get("obstacles", []):
        if o["type"] == "cylinder":
            world.cylinders.append(Cylinder(o["north"], o["east"], o["radius"], *o["z"]))
        elif o["type"] == "box":
            world.boxes.append(Box(*o["north"], *o["east"], *o["up"]))
        else:
            raise ValueError(f"unknown obstacle type {o['type']!r}")
    sc = Scenario(cfg.get("name", Path(path).stem), world, np.array(cfg["start"], float), np.array(cfg["goal"], float))
    for label, pt in (("start", sc.start), ("goal", sc.goal)):
        if not world.point_free(pt):
            raise ValueError(f"scenario {sc.name!r}: {label} {pt.tolist()} is not collision-free")
    return sc
