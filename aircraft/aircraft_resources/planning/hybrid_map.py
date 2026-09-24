"""Hybrid air/water 3D map: the single collision/occupancy interface both
planners use.

Two layers share one local frame (north, east, up) with the surface at z = 0:

* a-priori obstacles: analytic cylinders/boxes (`World`), so a 200 x 200 x 30 m
  area at 0.10 m never has to be stored as a dense grid (~1.2 GB);
* sensed occupancy: a sparse voxel set (hash map keyed by voxel index) fed
  through `atualizar_ocupacao(celulas, fonte, timestamp)`, independent of which
  sensor produced the cells (LiDAR, Hydrone sonar, WhiteBoat sonar, a-priori).

Every query (`segment_free`, `point_free`) applies, per medium, a margin of
vehicle radius + medium slack (larger underwater, plus a position-uncertainty
sphere), and enforces that the transition zone is only crossed vertically.
Segment checks are continuous (exact geometry), not just at the endpoints.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from config import Config
from medium import Medium, classify, split_by_medium, violates_vertical_rule
from world import World

SOURCES = ("a_priori", "lidar", "sonar_hydrone", "sonar_whiteboat")


@dataclass
class VoxelRecord:
    source: str
    timestamp: float


class HybridMap:
    def __init__(self, world: World, cfg: Config):
        self.world = world
        self.mu = float(cfg.get("medium", "mu"))
        mg = cfg.section("margins")
        self.margin_air = float(mg["vehicle_radius"] + mg["air_slack"])
        self.margin_water = float(mg["vehicle_radius"] + mg["water_slack"])
        self.uncertainty_frac = 0.0   # K4: underwater drift as a fraction of distance since the last GPS fix
        self.fix_point: np.ndarray | None = None
        self.voxel = float(cfg.get("map", "voxel_size"))
        self._half_diag = math.sqrt(3.0) * self.voxel / 2.0
        self._cells: dict[tuple[int, int, int], VoxelRecord] = {}
        self._centres = np.empty((0, 3))
        self._tree: cKDTree | None = None
        self._dirty = False
        self.version = 0
        self._added_log: list[tuple[int, np.ndarray]] = []  # (version, centres) for replan invalidation
        self._changed_log: list[tuple[int, np.ndarray]] = []  # cells added OR cleared, for grid replanners

    # ------------------------------------------------------------ occupancy

    def _key(self, p: np.ndarray) -> tuple[int, int, int]:
        return tuple(int(v) for v in np.floor(np.asarray(p, dtype=float) / self.voxel))

    def _centre(self, key: tuple[int, int, int]) -> np.ndarray:
        return (np.asarray(key, dtype=float) + 0.5) * self.voxel

    def atualizar_ocupacao(self, celulas, fonte: str, timestamp: float) -> int:
        """Insert (or clear) occupied cells from any sensor.

        celulas: (N, 3) points [north, east, up] in the local frame, all
        occupied; or (N, 4) with a last column 1 = occupied / 0 = free
        (clears the cell). Returns the number of cells whose state changed.
        """
        if fonte not in SOURCES:
            raise ValueError(f"unknown source {fonte!r}; expected one of {SOURCES}")
        cells = np.atleast_2d(np.asarray(celulas, dtype=float))
        if cells.size == 0:
            return 0
        if cells.shape[1] not in (3, 4):
            raise ValueError("celulas must have shape (N, 3) or (N, 4)")
        changed, added, touched = 0, [], []
        if cells.shape[1] == 3:
            # bulk path (every cell occupied): dedupe voxels vectorised, then one dict pass
            keys = np.unique(np.floor(cells / self.voxel).astype(np.int64), axis=0)
            for k in map(tuple, keys.tolist()):
                if k in self._cells:
                    self._cells[k] = VoxelRecord(fonte, float(timestamp))  # refresh
                else:
                    self._cells[k] = VoxelRecord(fonte, float(timestamp))
                    c = self._centre(k)
                    added.append(c)
                    touched.append(c)
                    changed += 1
        else:
            for row in cells:
                key = self._key(row[:3])
                if bool(row[3]) and key not in self._cells:
                    self._cells[key] = VoxelRecord(fonte, float(timestamp))
                    added.append(self._centre(key))
                    touched.append(self._centre(key))
                    changed += 1
                elif bool(row[3]):
                    self._cells[key] = VoxelRecord(fonte, float(timestamp))  # refresh
                elif key in self._cells:
                    del self._cells[key]
                    touched.append(self._centre(key))
                    changed += 1
        if changed:
            self.version += 1
            self._dirty = True
            if added:
                self._added_log.append((self.version, np.array(added)))
            self._changed_log.append((self.version, np.array(touched)))
        return changed

    update_occupancy = atualizar_ocupacao  # English alias

    def newly_occupied_since(self, version: int) -> np.ndarray:
        """Centres of cells added after `version` (what a replanner must re-check)."""
        parts = [c for v, c in self._added_log if v > version]
        return np.vstack(parts) if parts else np.empty((0, 3))

    def changed_since(self, version: int) -> np.ndarray:
        """Centres of cells added or cleared after `version` (grid replanners)."""
        parts = [c for v, c in self._changed_log if v > version]
        return np.vstack(parts) if parts else np.empty((0, 3))

    def _rebuild(self) -> None:
        if self._dirty:
            self._centres = (np.array([self._centre(k) for k in self._cells]) if self._cells else np.empty((0, 3)))
            self._tree = cKDTree(self._centres) if len(self._centres) else None
            self._dirty = False

    @property
    def n_cells(self) -> int:
        return len(self._cells)

    def memory_bytes(self) -> int:
        """Rough footprint of the sparse layer (key tuple + record per cell)."""
        return len(self._cells) * (3 * 8 + 56 + 48 + 24)

    # -------------------------------------------------------------- margins

    def set_position_uncertainty(self, fraction: float, fix_point) -> None:
        """K4: underwater position uncertainty. The radius of the uncertainty sphere at a
        point is `fraction` x its distance to `fix_point` (the last place with a GPS
        fix), added to the underwater margin -- it grows the farther the vehicle is
        from where it last knew where it was. Air keeps its normal margin."""
        self.uncertainty_frac = float(fraction)
        self.fix_point = None if fix_point is None else np.asarray(fix_point, dtype=float)

    def _uncertainty_at(self, pts: np.ndarray) -> np.ndarray:
        if self.uncertainty_frac == 0.0 or self.fix_point is None:
            return np.zeros(len(pts))
        return self.uncertainty_frac * np.linalg.norm(pts - self.fix_point, axis=1)

    def margin_for(self, medium: Medium, *points) -> float:
        """Margin for a piece in `medium`; underwater it is evaluated at the farthest
        of the given points from the GPS fix (conservative for a whole segment)."""
        if medium is Medium.AIR:
            return self.margin_air
        extra = float(np.max(self._uncertainty_at(np.array(points)))) if points else 0.0
        return self.margin_water + extra  # water and transition (conservative)

    def margin_array(self, pts: np.ndarray) -> np.ndarray:
        """Per-point margin for an (N, 3) array."""
        return np.where(pts[:, 2] > self.mu, self.margin_air, self.margin_water + self._uncertainty_at(pts))

    def medium_at(self, z: float) -> Medium:
        return classify(z, self.mu)

    # ----------------------------------------------------------- collision

    def _sparse_point_free(self, p: np.ndarray, margin: float) -> bool:
        self._rebuild()
        if self._tree is None:
            return True
        return not self._tree.query_ball_point(p, margin + self._half_diag)

    def _sparse_segment_free(self, a: np.ndarray, b: np.ndarray, margin: float) -> bool:
        self._rebuild()
        if self._tree is None:
            return True
        d = b - a
        idx = self._tree.query_ball_point(0.5 * (a + b), 0.5 * float(np.linalg.norm(d)) + margin + self._half_diag)
        if not idx:
            return True
        c = self._centres[idx]
        dd = float(d @ d)
        t = np.clip(((c - a) @ d) / dd, 0.0, 1.0) if dd > 1e-12 else np.zeros(len(c))
        dist = np.linalg.norm(c - (a + t[:, None] * d), axis=1)
        return not bool(np.any(dist <= margin + self._half_diag))

    def point_free(self, p) -> bool:
        p = np.asarray(p, dtype=float)
        m = self.margin_for(self.medium_at(p[2]), p)
        return self.world.point_free(p, m) and self._sparse_point_free(p, m)

    def segment_free(self, p, q) -> bool:
        p = np.asarray(p, dtype=float)
        q = np.asarray(q, dtype=float)
        if violates_vertical_rule(p, q, self.mu):
            return False
        pieces = split_by_medium(p, q, self.mu)
        for a, b, medium in pieces:
            m = self.margin_for(medium, a, b)
            if not self.world.segment_free(a, b, m) or not self._sparse_segment_free(a, b, m):
                return False
        # the pieces cover both endpoints unless the segment is degenerate (a point)
        return bool(pieces) or self.point_free(p)

    def points_free(self, pts: np.ndarray) -> np.ndarray:
        """Vectorised point test, per-medium (and per-position) margins; True = free."""
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        m_pt = self.margin_array(pts)
        free = np.ones(len(pts), dtype=bool)
        # World.points_free takes one margin per call: group points into 0.25 m margin buckets
        # (rounded UP, so the test is never less conservative than the exact margin).
        buckets = np.ceil(m_pt / 0.25) * 0.25
        for b in np.unique(buckets):
            mask = buckets == b
            free[mask] = self.world.points_free(pts[mask], float(b))
        self._rebuild()
        if self._tree is not None:
            d, _ = self._tree.query(pts, k=1, distance_upper_bound=float(m_pt.max() + self._half_diag))
            free &= ~(d <= m_pt + self._half_diag)
        return free

    @property
    def max_margin(self) -> float:
        """Largest margin anywhere in the map (bounds the search radius for cut edges)."""
        extra = 0.0
        if self.uncertainty_frac and self.fix_point is not None:
            lo, hi = self.bounds[:, 0], self.bounds[:, 1]
            corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
            extra = float(self._uncertainty_at(corners).max())
        return max(self.margin_air, self.margin_water + extra)

    def path_free(self, path) -> bool:
        path = np.asarray(path, dtype=float)
        return all(self.segment_free(path[i], path[i + 1]) for i in range(len(path) - 1))

    def in_bounds(self, p) -> bool:
        return bool(self.world.in_bounds(np.asarray(p, dtype=float)))

    @property
    def bounds(self) -> np.ndarray:
        return self.world.bounds

    # ------------------------------------------------------------ clearance

    def clearance(self, pts: np.ndarray) -> np.ndarray:
        """Distance to the nearest true (un-inflated) obstacle surface."""
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        best = self.world.clearance(pts)
        self._rebuild()
        if self._tree is not None:
            d, _ = self._tree.query(pts)
            best = np.minimum(best, np.maximum(0.0, d - self.voxel / 2.0))
        return best

    def path_min_clearance(self, path: np.ndarray, step: float = 0.25) -> float:
        path = np.asarray(path, dtype=float)
        samples = [path[0]]
        for a, b in zip(path[:-1], path[1:]):
            n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
            samples.extend(a + (b - a) * (k / n) for k in range(1, n + 1))
        return float(np.min(self.clearance(np.array(samples))))


# ------------------------------------------------------------------ GPS input

def gps_to_local(lat: float, lon: float, alt: float, home: tuple[float, float, float]) -> np.ndarray:
    """(lat deg, lon deg, alt m) -> (north, east, up) metres from `home`
    (equirectangular; error < 1 cm over the few hundred metres of a mission).
    Done once, at mission entry, so planners only ever see the local frame."""
    h_lat, h_lon, h_alt = home
    north = math.radians(lat - h_lat) * 6378137.0
    east = math.radians(lon - h_lon) * 6378137.0 * math.cos(math.radians(h_lat))
    return np.array([north, east, alt - h_alt])
