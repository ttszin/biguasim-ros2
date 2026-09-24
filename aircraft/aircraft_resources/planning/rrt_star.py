"""RRT* on the hybrid map, with energy cost and a persistent tree.

Core loop (per the T8.2 guide):
  1. sample a point in free space, biased toward the goal;
  2. find the nearest node and extend by at most `step`;
  3. if the extension would cross the transition zone non-vertically, replace
     it by a vertical extension (the zone is only ever crossed vertically);
  4. continuous collision check with the margin of the corresponding medium;
  5. choose, among neighbours inside the rewiring radius, the parent with the
     least accumulated energy;
  6. rewire neighbours that become cheaper through the new node.

The tree persists between calls (`grow` can be called again after the map
changes): `invalidate` drops the branches the new obstacles cut, and
`best_path` always reads the best route available -- CL-RRT style continuous
replanning instead of planning from scratch.

Cost is directed energy (J), so climbing/descending is not symmetric.
"""

from __future__ import annotations

import time

import numpy as np
from scipy.spatial import cKDTree

from common import PlanResult, SUCESSO, TIMEOUT, SEM_CAMINHO, fail
from config import Config
from medium import violates_vertical_rule


class RRTStar:
    def __init__(self, hmap, energy, cfg: Config, start, goal, seed: int | None = 0):
        p = cfg.section("rrt_star")
        self.map, self.energy = hmap, energy
        self.step = float(p["step"])
        self.goal_bias = float(p["goal_bias"])
        self.goal_radius = float(p["goal_radius"])
        self.max_radius = float(p["rewire_radius"])
        self.max_iter = int(p["max_iter"])
        self.time_budget = float(p["time_budget_s"])
        limit_wh = cfg.section("energy").get("max_segment_wh")
        self.max_segment_j = None if limit_wh is None else float(limit_wh) * 3600.0
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.start = np.asarray(start, dtype=float)
        self.goal = np.asarray(goal, dtype=float)
        self.lo, self.hi = hmap.bounds[:, 0], hmap.bounds[:, 1]
        vol = float(np.prod(self.hi - self.lo))
        self.gamma = 2.0 * (1.5 * vol / (4.0 / 3.0 * np.pi)) ** (1.0 / 3.0) * 1.1

        self._cap = 1024
        self.nodes = np.empty((self._cap, 3))
        self.cost = np.empty(self._cap)
        self.parent = np.full(self._cap, -1, dtype=int)
        self.alive = np.zeros(self._cap, dtype=bool)
        self.children: list[list[int]] = [[] for _ in range(self._cap)]
        self.n = 1
        self.root = 0
        self.nodes[0], self.cost[0], self.alive[0] = self.start, 0.0, True
        self.goal_idx = -1
        self.iterations = 0
        self.history: list[tuple[int, float, float]] = []  # (iteration, seconds, best energy J)
        self.first_solution_iter: int | None = None
        self.first_solution_time: float | None = None
        self._elapsed = 0.0
        self._map_version = hmap.version
        self._trapped = False

    # ------------------------------------------------------------ storage

    def _grow_arrays(self) -> None:
        old = self._cap
        self._cap *= 2
        self.nodes = np.vstack([self.nodes, np.empty((old, 3))])
        self.cost = np.concatenate([self.cost, np.empty(old)])
        self.parent = np.concatenate([self.parent, np.full(old, -1, dtype=int)])
        self.alive = np.concatenate([self.alive, np.zeros(old, dtype=bool)])
        self.children.extend([] for _ in range(old))

    def _add(self, pos, parent, cost) -> int:
        if self.n >= self._cap:
            self._grow_arrays()
        i = self.n
        self.nodes[i], self.parent[i], self.cost[i], self.alive[i] = pos, parent, cost, True
        self.children[i] = []
        self.children[parent].append(i)
        self.n += 1
        return i

    def _propagate(self, idx: int, delta: float) -> None:
        stack = [idx]
        while stack:
            k = stack.pop()
            self.cost[k] += delta
            stack.extend(self.children[k])

    # -------------------------------------------------------------- edges

    def _edge_energy(self, a, b):
        """Cheap part of an edge test: energy cost, or None if the vertical rule
        or the per-segment energy limit already rules it out (no collision check)."""
        if violates_vertical_rule(a, b, self.map.mu):
            return None
        c = self.energy.edge_cost(a, b)
        if self.max_segment_j is not None and c > self.max_segment_j:
            return None
        return c

    def _seg_ok(self, i: int, b) -> bool:
        """Collision part of the edge node_i -> b. If the root itself sits inside the
        inflated zone of an obstacle (a moving obstacle swept over the vehicle), the
        only acceptable edges from it are those that end in free space AND move
        away from the obstacle: the escape rule."""
        a = self.nodes[i]
        if i == self.root and self._trapped:
            return (self.map.point_free(b)
                    and float(self.map.clearance(np.asarray(b)[None, :])[0])
                    >= float(self.map.clearance(a[None, :])[0]) - 1e-9)
        return self.map.segment_free(a, b)

    def _edge(self, a, b):
        """Directed edge cost a->b, or None if invalid (collision, vertical rule, energy limit)."""
        c = self._edge_energy(a, b)
        if c is None or not self.map.segment_free(a, b):
            return None
        return c

    def _steer(self, near: np.ndarray, sample: np.ndarray) -> np.ndarray | None:
        d = float(np.linalg.norm(sample - near))
        if d < 1e-9:
            return None
        new = near + (sample - near) * min(1.0, self.step / d)
        if violates_vertical_rule(near, new, self.map.mu):
            dz = sample[2] - near[2]
            if abs(dz) < 1e-9:
                return None
            new = near + np.array([0.0, 0.0, np.sign(dz) * min(self.step, abs(dz))])
        return new

    # -------------------------------------------------------------- goal

    def _connect_goal_from(self, idx: int) -> None:
        d = float(np.linalg.norm(self.goal - self.nodes[idx]))
        if d > self.goal_radius:
            return
        c = self._edge(self.nodes[idx], self.goal)
        if c is None:
            return
        new_cost = self.cost[idx] + c
        if self.goal_idx < 0:
            self.goal_idx = self._add(self.goal, idx, new_cost)
        elif new_cost < self.cost[self.goal_idx] - 1e-9:
            self.children[self.parent[self.goal_idx]].remove(self.goal_idx)
            self.parent[self.goal_idx] = idx
            self.children[idx].append(self.goal_idx)
            self._propagate(self.goal_idx, new_cost - self.cost[self.goal_idx])

    # -------------------------------------------------------------- growth

    def grow(self, max_iter: int | None = None, time_budget_s: float | None = None,
             stop_on_first: bool = False) -> None:
        """Run RRT* iterations until the iteration or time budget is spent."""
        max_iter = self.max_iter if max_iter is None else max_iter
        budget = self.time_budget if time_budget_s is None else time_budget_s
        t0 = time.perf_counter()
        t_base = self._elapsed
        self._trapped = not self.map.point_free(self.nodes[self.root])
        for _ in range(max_iter):
            if time.perf_counter() - t0 > budget:
                break
            self.iterations += 1
            it = self.iterations
            sample = self.goal if self.rng.random() < self.goal_bias else self.rng.uniform(self.lo, self.hi)
            live = np.flatnonzero(self.alive[: self.n])
            dists = np.linalg.norm(self.nodes[live] - sample, axis=1)
            near_i = int(live[np.argmin(dists)])
            new = self._steer(self.nodes[near_i], sample)
            if new is None or not self.map.point_free(new):
                continue
            c_near = self._edge_energy(self.nodes[near_i], new)
            if c_near is None or not self._seg_ok(near_i, new):
                continue

            radius = max(self.step, min(self.max_radius,
                         self.gamma * (np.log(len(live) + 1) / (len(live) + 1)) ** (1.0 / 3.0)))
            dn = np.linalg.norm(self.nodes[live] - new, axis=1)
            neighbours = live[dn <= radius]

            prev_goal = self.cost[self.goal_idx] if self.goal_idx >= 0 else np.inf
            # Parent choice: rank candidate parents by accumulated energy (cheap),
            # then run the collision check only down the ranking until one passes.
            best_parent, best_cost = near_i, self.cost[near_i] + c_near
            ranked = []
            for j in neighbours:
                if j == near_i:
                    continue
                ce = self._edge_energy(self.nodes[j], new)
                if ce is not None and self.cost[j] + ce < best_cost - 1e-12:
                    ranked.append((self.cost[j] + ce, int(j)))
            for total, j in sorted(ranked):
                if self._seg_ok(j, new):
                    best_parent, best_cost = j, total
                    break

            new_idx = self._add(new, best_parent, best_cost)

            for j in neighbours:
                if j == best_parent or j == self.root or j == new_idx:
                    continue
                ce = self._edge_energy(new, self.nodes[j])
                if ce is None or best_cost + ce >= self.cost[j] - 1e-12:
                    continue
                if self.map.segment_free(new, self.nodes[j]):
                    self.children[self.parent[j]].remove(int(j))
                    self.parent[j] = new_idx
                    self.children[new_idx].append(int(j))
                    self._propagate(int(j), best_cost + ce - self.cost[j])

            self._connect_goal_from(new_idx)
            if self.goal_idx >= 0 and self.cost[self.goal_idx] < prev_goal - 1e-9:
                now = t_base + (time.perf_counter() - t0)
                self.history.append((it, now, float(self.cost[self.goal_idx])))
                if self.first_solution_iter is None:
                    self.first_solution_iter, self.first_solution_time = it, now
                if stop_on_first and prev_goal == np.inf:  # goal (re)reached during this call
                    break
        self._elapsed = t_base + (time.perf_counter() - t0)

    # ------------------------------------------------------------ replanning

    def invalidate(self) -> int:
        """Drop branches cut by obstacles added since the last call. Returns
        the number of nodes removed. The goal is forgotten if its branch died."""
        new_cells = self.map.newly_occupied_since(self._map_version)
        self._map_version = self.map.version
        if len(new_cells) == 0:
            return 0
        tree = cKDTree(new_cells)
        reach = self.map.max_margin + self.map.voxel * 1.8
        removed = 0
        for i in np.flatnonzero(self.alive[: self.n]):
            i = int(i)
            if i == self.root or not self.alive[i]:
                continue
            a, b = self.nodes[self.parent[i]], self.nodes[i]
            half = 0.5 * float(np.linalg.norm(b - a))
            if tree.query(0.5 * (a + b))[0] > half + reach:
                continue  # nowhere near the new cells: cannot have been cut
            if not self.map.segment_free(a, b):
                removed += self._kill_subtree(i)
        return removed

    def _kill_subtree(self, idx: int) -> int:
        p = self.parent[idx]
        if p >= 0 and idx in self.children[p]:
            self.children[p].remove(idx)
        stack, count = [idx], 0
        while stack:
            k = stack.pop()
            stack.extend(self.children[k])
            self.children[k] = []
            self.alive[k] = False
            if k == self.goal_idx:
                self.goal_idx = -1
            count += 1
        return count

    def reroot(self, idx: int) -> None:
        """Make an existing node the new root (vehicle moved along the path);
        everything not below it is discarded and costs are re-based."""
        keep, stack = [], [idx]
        while stack:
            k = stack.pop()
            keep.append(k)
            stack.extend(self.children[k])
        offset = self.cost[idx]
        self.alive[: self.n] = False
        for k in keep:
            self.alive[k] = True
            self.cost[k] -= offset
        self.parent[idx] = -1
        self.root = idx
        if self.goal_idx >= 0 and not self.alive[self.goal_idx]:
            self.goal_idx = -1

    # --------------------------------------------------------------- output

    def best_path(self) -> np.ndarray | None:
        if self.goal_idx < 0 or not self.alive[self.goal_idx]:
            return None
        seq = [self.goal_idx]
        while self.parent[seq[-1]] >= 0:
            seq.append(int(self.parent[seq[-1]]))
        return self.nodes[seq[::-1]].copy()

    def result(self) -> PlanResult:
        path = self.best_path()
        extra = {"tree_size": int(self.alive[: self.n].sum()), "history": list(self.history),
                 "first_solution_iter": self.first_solution_iter,
                 "first_solution_time": self.first_solution_time, "seed": self.seed}
        if path is None:
            # Sampling never proves that no path exists: a miss is TIMEOUT.
            return PlanResult(TIMEOUT, np.empty((0, 3)), float("inf"), self._elapsed, self.iterations, extra)
        return PlanResult(SUCESSO, path, float(self.cost[self.goal_idx]), self._elapsed, self.iterations, extra)


def plan(hmap, energy, cfg: Config, start, goal, seed: int | None = 0, max_iter: int | None = None,
         time_budget_s: float | None = None, stop_on_first: bool = False) -> PlanResult:
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    if not (hmap.point_free(start) and hmap.point_free(goal)):
        return fail(SEM_CAMINHO, 0.0, 0, reason="start or goal not collision-free")
    rrt = RRTStar(hmap, energy, cfg, start, goal, seed=seed)
    rrt.grow(max_iter=max_iter, time_budget_s=time_budget_s, stop_on_first=stop_on_first)
    return rrt.result()
