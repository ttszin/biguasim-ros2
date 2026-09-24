"""Continuous replanning, one implementation per planner family.

* CLRRTReplanner  (RRT*)  : keeps the tree between map updates (CL-RRT style),
  drops the branches the new obstacles cut and only grows again when the
  goal branch died.
* DStarLiteReplanner (A*) : reuses the previous backward search (D* Lite) and
  only re-expands what the changed cells affect. Re-running A* from scratch on
  every detection is not viable in real time.

Both expose:  initial(budget_s) -> PlanResult,  update(budget_s) -> PlanResult
where `update` reads what changed in the HybridMap since the last call and
`time_s`/`effort` of the returned result refer to that call only (replanning
time, the metric compared against the real-time limit).
"""

from __future__ import annotations

import heapq
import itertools
import time

import numpy as np

from astar import Grid, _CORNERS, _MOVES, _nearest_free
from common import PlanResult, SEM_CAMINHO, SUCESSO, TIMEOUT, fail
from config import Config
from medium import violates_vertical_rule
from rrt_star import RRTStar


# Replanning is bounded by the time budget only (the config max_iter belongs to the initial plan:
# a re-rooted tree is small and needs more iterations than that to grow back).
NO_ITER_CAP = 10**9


class CLRRTReplanner:
    def __init__(self, hmap, energy, cfg: Config, start, goal, seed: int = 0):
        self.map = hmap
        self.rrt = RRTStar(hmap, energy, cfg, start, goal, seed=seed)
        self.budget = float(cfg.get("rrt_star", "time_budget_s"))

    def initial(self, budget_s: float | None = None, improve: bool = False) -> PlanResult:
        self.rrt.grow(time_budget_s=budget_s or self.budget, stop_on_first=not improve)
        return self.rrt.result()

    def update(self, budget_s: float | None = None) -> PlanResult:
        t0 = time.perf_counter()
        removed = self.rrt.invalidate()
        it0 = self.rrt.iterations
        if self.rrt.best_path() is None:
            self.rrt.grow(max_iter=NO_ITER_CAP, time_budget_s=budget_s or self.budget, stop_on_first=True)
        res = self.rrt.result()
        res.time_s = time.perf_counter() - t0
        res.effort = self.rrt.iterations - it0
        res.extra["nodes_removed"] = removed
        return res

    def move_start(self, position) -> None:
        """Vehicle advanced: re-root the tree at a nearby node the vehicle can
        actually reach in a straight, collision-free line."""
        pos = np.asarray(position, float)
        live = np.flatnonzero(self.rrt.alive[: self.rrt.n])
        order = live[np.argsort(np.linalg.norm(self.rrt.nodes[live] - pos, axis=1))]
        pick = next((int(i) for i in order[:12] if self.map.segment_free(pos, self.rrt.nodes[i])), int(order[0]))
        self.rrt.reroot(pick)

    def set_goal(self, goal, budget_s: float | None = None) -> PlanResult:
        """Mission change (K5): keep the tree, drop the old goal node and try to
        reconnect the new goal to it; grow only if no node reaches it."""
        t0 = time.perf_counter()
        r = self.rrt
        if r.goal_idx >= 0:
            r._kill_subtree(r.goal_idx)
        r.goal = np.asarray(goal, dtype=float)
        r.history = []
        r.first_solution_iter = None
        it0 = r.iterations
        live = np.flatnonzero(r.alive[: r.n])
        near = live[np.linalg.norm(r.nodes[live] - r.goal, axis=1) <= r.goal_radius]
        for i in near:
            r._connect_goal_from(int(i))
        if r.best_path() is None:
            r.grow(max_iter=NO_ITER_CAP, time_budget_s=budget_s or self.budget, stop_on_first=True)
        res = r.result()
        res.time_s, res.effort = time.perf_counter() - t0, r.iterations - it0
        return res


class DStarLiteReplanner:
    def __init__(self, hmap, energy, cfg: Config, start, goal, seed: int = 0,
                 resolution: float | None = None, heuristic_weight: float | None = None):
        a = cfg.section("astar")
        self.map, self.energy = hmap, energy
        self.res = float(a["resolution"] if resolution is None else resolution)
        self.w = float(a["heuristic_weight"] if heuristic_weight is None else heuristic_weight)
        self.budget = float(a["time_budget_s"])
        rng = np.random.default_rng(seed)
        offset = rng.uniform(0.0, self.res / 2.0, size=3) if a["grid_jitter"] else np.zeros(3)
        self.grid = Grid(hmap, self.res, offset)
        self.start_w, self.goal_w = np.asarray(start, float), np.asarray(goal, float)
        self.s = _nearest_free(self.grid, self.grid.to_index(start))
        self.g_cell = _nearest_free(self.grid, self.grid.to_index(goal))
        self.e_min = energy.min_energy_per_metre_in(hmap.bounds[2, 0], hmap.bounds[2, 1])
        self.mu = hmap.mu
        self._edge_cache: dict = {}
        self._version = hmap.version
        self.km = 0.0
        self._last_start = self.s
        self.g: dict = {}
        self.rhs: dict = {}
        self._heap: list = []
        self._key_of: dict = {}
        self._tie = itertools.count()
        self._ready = self.s is not None and self.g_cell is not None
        self.expanded_total = 0
        if self._ready:
            self.rhs[self.g_cell] = 0.0
            self._push(self.g_cell)

    # ---- graph ---------------------------------------------------------

    def _h(self, a, b) -> float:
        return self.w * self.e_min * float(np.linalg.norm((np.array(a) - np.array(b)) * self.res))

    def _cost(self, a, b) -> float:
        if not (self.grid.is_free(a) and self.grid.is_free(b)):
            return float("inf")
        m = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        if any(not self.grid.is_free((a[0] + c[0], a[1] + c[1], a[2] + c[2])) for c in _CORNERS[m]):
            return float("inf")
        key = (a[2], m)
        if key not in self._edge_cache:
            p = np.array([0.0, 0.0, self.grid.origin[2] + a[2] * self.res])
            q = p + np.array(m, float) * self.res
            self._edge_cache[key] = float("inf") if violates_vertical_rule(p, q, self.mu) else self.energy.edge_cost(p, q)
        return self._edge_cache[key]

    def _neigh(self, u):
        for m in _MOVES:
            yield (u[0] + m[0], u[1] + m[1], u[2] + m[2])

    # ---- D* Lite -------------------------------------------------------

    def _key(self, u):
        k2 = min(self.g.get(u, np.inf), self.rhs.get(u, np.inf))
        return (k2 + self._h(self.s, u) + self.km, k2)

    def _push(self, u) -> None:
        k = self._key(u)
        self._key_of[u] = k
        heapq.heappush(self._heap, (k[0], k[1], next(self._tie), u))

    def _update_vertex(self, u) -> None:
        if u != self.g_cell:
            best = np.inf
            for v in self._neigh(u):
                c = self._cost(u, v)
                if c < np.inf:
                    best = min(best, c + self.g.get(v, np.inf))
            self.rhs[u] = best
        self._key_of.pop(u, None)
        if self.g.get(u, np.inf) != self.rhs.get(u, np.inf):
            self._push(u)

    def _top_key(self):
        while self._heap:
            k1, k2, _, u = self._heap[0]
            if self._key_of.get(u) == (k1, k2):
                return (k1, k2)
            heapq.heappop(self._heap)
        return (np.inf, np.inf)

    def _compute(self, t0: float, budget: float) -> tuple[int, bool]:
        expanded, ok = 0, True
        while True:
            top = self._top_key()
            if not (top < self._key(self.s) or self.rhs.get(self.s, np.inf) != self.g.get(self.s, np.inf)):
                break
            if expanded % 128 == 0 and time.perf_counter() - t0 > budget:
                ok = False
                break
            k1, k2, _, u = heapq.heappop(self._heap)
            k_new = self._key(u)
            if (k1, k2) < k_new:
                self._key_of[u] = k_new
                heapq.heappush(self._heap, (k_new[0], k_new[1], next(self._tie), u))
                continue
            self._key_of.pop(u, None)
            expanded += 1
            gu = self.g.get(u, np.inf)
            if gu > self.rhs.get(u, np.inf):
                # over-consistent: g drops to rhs; predecessors can only improve, so relax them
                # incrementally (26 edge costs) instead of re-scanning all their successors.
                self.g[u] = gu = self.rhs[u]
                for p in self._neigh(u):
                    if p == self.g_cell:
                        continue
                    c = self._cost(p, u)
                    if c + gu < self.rhs.get(p, np.inf):
                        self.rhs[p] = c + gu
                        self._key_of.pop(p, None)
                        if self.g.get(p, np.inf) != self.rhs[p]:
                            self._push(p)
            else:
                # under-consistent: only predecessors whose best route went through u need a full re-scan
                self.g[u] = np.inf
                for p in self._neigh(u):
                    if p != self.g_cell and self.rhs.get(p, np.inf) == self._cost(p, u) + gu:
                        self._update_vertex(p)
                self._update_vertex(u)
        return expanded, ok

    # ---- output --------------------------------------------------------

    def _extract(self) -> np.ndarray | None:
        if self.g.get(self.s, np.inf) == np.inf:
            return None
        cells, cur = [self.s], self.s
        for _ in range(200000):
            if cur == self.g_cell:
                break
            best, best_v = np.inf, None
            for v in self._neigh(cur):
                c = self._cost(cur, v) + self.g.get(v, np.inf)
                if c < best:
                    best, best_v = c, v
            if best_v is None or best == np.inf:
                return None
            cells.append(best_v)
            cur = best_v
        else:
            return None
        path = self.grid.to_world(np.array(cells))
        path[0], path[-1] = self.start_w, self.goal_w
        return path

    def _result(self, t0: float, expanded: int, ok: bool) -> PlanResult:
        extra = {"grid_shape": self.grid.shape, "resolution": self.res, "expanded_total": self.expanded_total}
        elapsed = time.perf_counter() - t0
        if not ok:
            return fail(TIMEOUT, elapsed, expanded, reason="time budget exceeded", **extra)
        path = self._extract()
        if path is None:
            return fail(SEM_CAMINHO, elapsed, expanded, reason="no path on the grid", **extra)
        extra["path_valid"] = bool(self.map.path_free(path))
        cost = float(sum(self.energy.edge_cost(a, b) for a, b in zip(path[:-1], path[1:])))
        return PlanResult(SUCESSO, path, cost, elapsed, expanded, extra)

    def initial(self, budget_s: float | None = None) -> PlanResult:
        t0 = time.perf_counter()
        if not self._ready:
            return fail(SEM_CAMINHO, 0.0, 0, reason="no free grid cell near start/goal")
        expanded, ok = self._compute(t0, budget_s or self.budget)
        self.expanded_total += expanded
        return self._result(t0, expanded, ok)

    def update(self, budget_s: float | None = None) -> PlanResult:
        t0 = time.perf_counter()
        if not self._ready:
            return fail(SEM_CAMINHO, 0.0, 0, reason="no free grid cell near start/goal")
        centres = self.map.changed_since(self._version)
        self._version = self.map.version
        for cell in self.grid.refresh(self.map, centres):
            self._update_vertex(cell)
            for v in self._neigh(cell):
                self._update_vertex(v)
        expanded, ok = self._compute(t0, budget_s or self.budget)
        self.expanded_total += expanded
        return self._result(t0, expanded, ok)

    def set_goal(self, goal, budget_s: float | None = None) -> PlanResult:
        """Mission change (K5): D* Lite cannot reuse the search for a new goal,
        so it restarts the search on the same grid (the grid itself is kept)."""
        t0 = time.perf_counter()
        self.goal_w = np.asarray(goal, dtype=float)
        self.g_cell = _nearest_free(self.grid, self.grid.to_index(goal))
        self.g, self.rhs, self._heap, self._key_of = {}, {}, [], {}
        self.km, self._last_start = 0.0, self.s
        self._ready = self.s is not None and self.g_cell is not None
        if not self._ready:
            return fail(SEM_CAMINHO, 0.0, 0, reason="no free grid cell near the new goal")
        self.rhs[self.g_cell] = 0.0
        self._push(self.g_cell)
        # Bring the grid up to date with map changes seen since the last call; the
        # search restarts from scratch anyway, so the flipped cells need no vertex updates.
        self.grid.refresh(self.map, self.map.changed_since(self._version))
        self._version = self.map.version
        expanded, ok = self._compute(t0, budget_s or self.budget)
        self.expanded_total += expanded
        return self._result(t0, expanded, ok)

    def move_start(self, position) -> None:
        new = _nearest_free(self.grid, self.grid.to_index(position))
        if new is not None and new != self.s:
            self.km += self._h(self._last_start, new)
            self._last_start = new
            self.s = new
            self.start_w = np.asarray(position, float)
