"""3D grid A* on the hybrid map (deterministic baseline).

* Local grid built per plan from the map bounds (not one dense global grid).
  Occupancy and clearance are rasterised once, slab by slab, with the per-medium
  margins; edges are 26-connected and cost the energy of the segment.
* Edges that touch the transition zone are only allowed vertically.
* Diagonal moves check the cells they cut across (no corner cutting), and edges
  near obstacles are confirmed with the exact continuous segment test.
* Heuristic = Euclidean distance x the smallest energy-per-metre of the model
  (admissible), scaled by `heuristic_weight` (>1 = weighted A*).
* Grid jitter (A*_r, Zammit & van Kampen 2022): the grid origin is shifted by a
  random offset in [0, resolution/2) on every plan, which cuts the spread of
  path lengths at a tiny average cost. Seeded, so runs are reproducible.
"""

from __future__ import annotations

import heapq
import itertools
import time

import numpy as np

from common import PlanResult, SUCESSO, SEM_CAMINHO, TIMEOUT, fail
from config import Config
from medium import violates_vertical_rule

_MOVES = [m for m in itertools.product((-1, 0, 1), repeat=3) if m != (0, 0, 0)]
_MOVE_LEN = {m: float(np.linalg.norm(m)) for m in _MOVES}


def _corner_cells(move):
    axes = [k for k, v in enumerate(move) if v != 0]
    cells = []
    for mask in itertools.product((0, 1), repeat=len(axes)):
        if sum(mask) in (0, len(axes)):
            continue
        c = [0, 0, 0]
        for use, ax in zip(mask, axes):
            if use:
                c[ax] = move[ax]
        cells.append(tuple(c))
    return cells


_CORNERS = {m: _corner_cells(m) for m in _MOVES}


class Grid:
    def __init__(self, hmap, resolution: float, offset: np.ndarray):
        self.res = resolution
        self.origin = hmap.bounds[:, 0] + offset
        span = hmap.bounds[:, 1] - self.origin
        self.shape = tuple(int(np.floor(s / resolution)) + 1 for s in span)
        nx, ny, nz = self.shape
        self.free = np.zeros(self.shape, dtype=bool)
        self.clear = np.full(self.shape, np.inf, dtype=np.float32)
        yy, zz = np.meshgrid(np.arange(ny), np.arange(nz), indexing="ij")
        for i in range(nx):  # slab by slab: bounded memory even for fine grids
            pts = self.origin + np.stack([np.full(yy.shape, i), yy, zz], axis=-1).reshape(-1, 3) * resolution
            self.free[i] = hmap.points_free(pts).reshape(ny, nz)
            self.clear[i] = hmap.clearance(pts).reshape(ny, nz)

    def refresh(self, hmap, centres: np.ndarray) -> list[tuple[int, int, int]]:
        """Re-evaluate occupancy around changed map cells; returns the grid
        indices whose free/occupied state flipped."""
        if len(centres) == 0:
            return []
        reach = hmap.max_margin + hmap.voxel * 2.0 + self.res
        lo = np.floor((centres.min(axis=0) - reach - self.origin) / self.res).astype(int)
        hi = np.ceil((centres.max(axis=0) + reach - self.origin) / self.res).astype(int)
        lo = np.maximum(lo, 0)
        hi = np.minimum(hi, np.array(self.shape) - 1)
        if np.any(hi < lo):
            return []
        ii = np.stack(np.meshgrid(*[np.arange(lo[k], hi[k] + 1) for k in range(3)], indexing="ij"), axis=-1).reshape(-1, 3)
        new_free = hmap.points_free(self.to_world(ii))
        old_free = self.free[ii[:, 0], ii[:, 1], ii[:, 2]]
        flipped = ii[new_free != old_free]
        self.free[ii[:, 0], ii[:, 1], ii[:, 2]] = new_free
        return [tuple(int(v) for v in c) for c in flipped]

    def to_world(self, idx) -> np.ndarray:
        return self.origin + np.asarray(idx, dtype=float) * self.res

    def to_index(self, p) -> tuple[int, int, int]:
        return tuple(int(round(v)) for v in (np.asarray(p, dtype=float) - self.origin) / self.res)

    def is_free(self, i) -> bool:
        return (0 <= i[0] < self.shape[0] and 0 <= i[1] < self.shape[1] and 0 <= i[2] < self.shape[2]
                and bool(self.free[i]))

    @property
    def nbytes(self) -> int:
        return int(self.free.nbytes + self.clear.nbytes)


def _nearest_free(grid: Grid, i, radius: int = 2):
    best, best_d = None, np.inf
    for d in itertools.product(range(-radius, radius + 1), repeat=3):
        c = (i[0] + d[0], i[1] + d[1], i[2] + d[2])
        if grid.is_free(c) and sum(x * x for x in d) < best_d:
            best, best_d = c, sum(x * x for x in d)
    return best


def plan(hmap, energy, cfg: Config, start, goal, seed: int | None = 0, resolution: float | None = None,
         heuristic_weight: float | None = None, time_budget_s: float | None = None,
         jitter: bool | None = None, exact_edges: bool | None = None) -> PlanResult:
    """exact_edges=False (default): edges are validated on the raster (cell +
    corner cells); the returned path is then checked exactly and, if that fails,
    the search is repeated with the exact continuous test on every edge."""
    a = cfg.section("astar")
    exact = bool(a.get("exact_edges", False) if exact_edges is None else exact_edges)
    res = float(a["resolution"] if resolution is None else resolution)
    w = float(a["heuristic_weight"] if heuristic_weight is None else heuristic_weight)
    budget = float(a["time_budget_s"] if time_budget_s is None else time_budget_s)
    use_jitter = bool(a["grid_jitter"] if jitter is None else jitter)
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    t0 = time.perf_counter()

    if not (hmap.point_free(start) and hmap.point_free(goal)):
        return fail(SEM_CAMINHO, time.perf_counter() - t0, 0, reason="start or goal not collision-free")

    rng = np.random.default_rng(seed)
    offset = rng.uniform(0.0, res / 2.0, size=3) if use_jitter else np.zeros(3)
    grid = Grid(hmap, res, offset)
    s = _nearest_free(grid, grid.to_index(start))
    g = _nearest_free(grid, grid.to_index(goal))
    info = {"grid_shape": grid.shape, "resolution": res, "grid_bytes": grid.nbytes, "offset": offset.tolist()}
    if s is None or g is None:
        return fail(SEM_CAMINHO, time.perf_counter() - t0, 0, reason="no free grid cell near start/goal", **info)

    goal_w = grid.to_world(g)
    e_min = energy.min_energy_per_metre_in(hmap.bounds[2, 0], hmap.bounds[2, 1])
    mu = hmap.mu
    margin_max = hmap.max_margin

    def h(i) -> float:
        return w * e_min * float(np.linalg.norm(grid.to_world(i) - goal_w))

    # Edge energy only depends on (z-layer, move): cache it.
    edge_cache: dict[tuple[int, tuple[int, int, int]], float | None] = {}

    def edge_cost(cur, m) -> float | None:
        key = (cur[2], m)
        if key not in edge_cache:
            p = np.array([0.0, 0.0, grid.origin[2] + cur[2] * res])
            q = p + np.array(m, dtype=float) * res
            edge_cache[key] = None if violates_vertical_rule(p, q, mu) else energy.edge_cost(p, q)
        return edge_cache[key]

    tie = itertools.count()
    open_heap = [(h(s), next(tie), s)]
    g_cost = {s: 0.0}
    parent: dict = {}
    closed: set = set()
    expanded = 0

    while open_heap:
        if expanded % 256 == 0 and time.perf_counter() - t0 > budget:
            return fail(TIMEOUT, time.perf_counter() - t0, expanded, reason="time budget exceeded", **info)
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == g:
            cells = [cur]
            while cells[-1] in parent:
                cells.append(parent[cells[-1]])
            path = grid.to_world(np.array(cells[::-1]))
            # Land exactly on the requested endpoints when the connection is valid.
            if hmap.segment_free(start, path[1] if len(path) > 1 else path[0]) or len(path) == 1:
                path[0] = start
            if hmap.segment_free(path[-2] if len(path) > 1 else path[-1], goal) or len(path) == 1:
                path[-1] = goal
            if not exact and not hmap.path_free(path):
                retry = plan(hmap, energy, cfg, start, goal, seed=seed, resolution=res, heuristic_weight=w,
                             time_budget_s=max(0.0, budget - (time.perf_counter() - t0)), jitter=use_jitter,
                             exact_edges=True)
                retry.time_s = time.perf_counter() - t0
                retry.extra["exact_retry"] = True
                return retry
            cost = float(sum(energy.edge_cost(p, q) for p, q in zip(path[:-1], path[1:])))
            info.update(expanded=expanded, peak_open=len(open_heap), g_size=len(g_cost), grid_cost=g_cost[g])
            return PlanResult(SUCESSO, path, cost, time.perf_counter() - t0, expanded, info)
        for m in _MOVES:
            nxt = (cur[0] + m[0], cur[1] + m[1], cur[2] + m[2])
            if nxt in closed or not grid.is_free(nxt):
                continue
            if any(not grid.is_free((cur[0] + c[0], cur[1] + c[1], cur[2] + c[2])) for c in _CORNERS[m]):
                continue
            c = edge_cost(cur, m)
            if c is None:
                continue
            length = _MOVE_LEN[m] * res
            if exact and min(grid.clear[cur], grid.clear[nxt]) - length / 2.0 <= margin_max:  # could graze something
                if not hmap.segment_free(grid.to_world(cur), grid.to_world(nxt)):
                    continue
            ng = g_cost[cur] + c
            if ng < g_cost.get(nxt, float("inf")):
                g_cost[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(open_heap, (ng + h(nxt), next(tie), nxt))

    return fail(SEM_CAMINHO, time.perf_counter() - t0, expanded, reason="no path exists on this grid", **info)
