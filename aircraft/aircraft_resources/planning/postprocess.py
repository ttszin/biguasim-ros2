"""Post-processing shared by both planners: line-of-sight pruning.

Fewer waypoints mean fewer stops and restarts of ArduPilot's GUIDED mode. A
pass jumps from each waypoint to the farthest one still directly reachable
*and* not more expensive in energy than the sub-path it replaces. Passes repeat
until one reduces the cost by less than `min_gain` (1 % by default).
"""

from __future__ import annotations

import numpy as np

from common import path_cost


def _pass(path: np.ndarray, hmap, energy) -> np.ndarray:
    out = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        best = i + 1
        for j in range(n - 1, i + 1, -1):
            if not hmap.segment_free(path[i], path[j]):
                continue
            direct = energy.edge_cost(path[i], path[j])
            via = sum(energy.edge_cost(path[k], path[k + 1]) for k in range(i, j))
            if direct <= via + 1e-9:
                best = j
                break
        out.append(path[best])
        i = best
    return np.array(out)


def prune(path: np.ndarray, hmap, energy, min_gain: float = 0.01, max_passes: int = 10) -> np.ndarray:
    path = np.asarray(path, dtype=float)
    if len(path) < 3:
        return path
    cost = path_cost(path, energy)
    for _ in range(max_passes):
        new = _pass(path, hmap, energy)
        new_cost = path_cost(new, energy)
        gain = (cost - new_cost) / cost if cost > 0 else 0.0
        if new_cost <= cost + 1e-9:
            path, cost = new, new_cost
        if gain < min_gain:
            break
    return path
