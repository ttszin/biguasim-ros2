"""Result type and helpers shared by both planners.

Both planners return the same PlanResult, so benchmarking, post-processing
and the mission interface never need to know which one produced it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

SUCESSO = "SUCESSO"
SEM_CAMINHO = "SEM_CAMINHO"
TIMEOUT = "TIMEOUT"
ENERGIA_INSUFICIENTE = "ENERGIA_INSUFICIENTE"


@dataclass
class PlanResult:
    status: str
    path: np.ndarray  # (N, 3) north/east/up, empty unless status == SUCESSO
    cost: float  # planner cost in joules (energy), inf when there is no path
    time_s: float
    # Planner-specific effort: expanded nodes for A*, iterations for RRT*.
    effort: int
    extra: dict = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return self.status == SUCESSO

    @property
    def length(self) -> float:
        return path_length(self.path) if self.success else float("inf")


def fail(status: str, time_s: float, effort: int, **extra) -> PlanResult:
    return PlanResult(status, np.empty((0, 3)), float("inf"), time_s, effort, dict(extra))


def path_length(path: np.ndarray) -> float:
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def path_cost(path: np.ndarray, energy) -> float:
    return float(sum(energy.edge_cost(a, b) for a, b in zip(path[:-1], path[1:])))
