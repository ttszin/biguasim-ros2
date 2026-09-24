"""Medium classification for the hybrid air/water map.

Surface is z = 0. With mu the half-height of the transition zone:
    z >  mu       -> AIR
    -mu <= z <= mu -> TRANSITION
    z < -mu       -> WATER
"""

from __future__ import annotations

from enum import Enum


class Medium(str, Enum):
    AIR = "AR"
    TRANSITION = "TRANSICAO"
    WATER = "AGUA"


def classify(z: float, mu: float) -> Medium:
    if z > mu:
        return Medium.AIR
    if z < -mu:
        return Medium.WATER
    return Medium.TRANSITION


def split_by_medium(p, q, mu: float):
    """Split segment p->q at z = +mu and z = -mu.

    Returns a list of (a, b, medium), in order, where every piece lies in a
    single medium (decided by its midpoint). Pieces have positive length only.
    """
    import numpy as np

    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    dz = q[2] - p[2]
    ts = [0.0, 1.0]
    if abs(dz) > 1e-12:
        for plane in (mu, -mu):
            t = (plane - p[2]) / dz
            if 0.0 < t < 1.0:
                ts.append(t)
    ts = sorted(set(ts))
    pieces = []
    for t0, t1 in zip(ts[:-1], ts[1:]):
        if t1 - t0 <= 1e-12:
            continue
        a, b = p + (q - p) * t0, p + (q - p) * t1
        pieces.append((a, b, classify(0.5 * (a[2] + b[2]), mu)))
    return pieces


def violates_vertical_rule(p, q, mu: float, horizontal_tol: float = 1e-6) -> bool:
    """True if p->q enters the transition band without being vertical (invalid edge).

    Rule from the T8.2 guide: the transition zone is only ever crossed
    vertically. A segment whose z-range overlaps the open band (-mu, mu) --
    or that starts/ends inside the closed band -- must have (almost) zero
    horizontal displacement.
    """
    import numpy as np

    p = np.asarray(p, dtype=float)
    q = np.asarray(q, dtype=float)
    zlo, zhi = min(p[2], q[2]), max(p[2], q[2])
    touches = (zlo < mu and zhi > -mu) or (abs(p[2]) <= mu) or (abs(q[2]) <= mu)
    if not touches:
        return False
    return float(np.linalg.norm(q[:2] - p[:2])) > horizontal_tol
