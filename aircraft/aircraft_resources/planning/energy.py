"""Energy cost model (per medium) for the hybrid planners.

Segment cost is estimated energy, not distance. For each piece of a segment in
one medium the vehicle flies at that medium's cruise (or vertical) speed; the
required thrust balances weight/buoyancy plus quadratic drag, and electrical
power comes from a 2nd-degree polynomial in thrust (Pinheiro et al., IROS 2024):

    F = w_net * z_hat + k * |v| * v          (w_net = m*g in air, net weight in water)
    T = |F|
    P = c2*T^2 + c1*T + c0
    E = P * length / speed

The transition zone is charged separately (power * time at the transition speed,
plus a fixed cost per air<->water crossing) and reported in its own bucket.
All energies here are in joules; use `wh()` to report watt-hours.

The coefficients live in config/planner.yaml and are PLACEHOLDERS until real
ones are supplied -- see the note there.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from config import Config
from medium import Medium, split_by_medium

G = 9.80665
J_PER_WH = 3600.0


def wh(joules: float) -> float:
    return joules / J_PER_WH


@dataclass
class EnergyBreakdown:
    air: float = 0.0
    water: float = 0.0
    transition: float = 0.0

    @property
    def total(self) -> float:
        return self.air + self.water + self.transition

    def __add__(self, other: "EnergyBreakdown") -> "EnergyBreakdown":
        return EnergyBreakdown(self.air + other.air, self.water + other.water, self.transition + other.transition)


@dataclass
class EnergyModel:
    mu: float
    air: dict
    water: dict
    transition: dict
    _e_min_per_m: float = field(default=0.0, init=False)

    @classmethod
    def from_config(cls, cfg: Config) -> "EnergyModel":
        e = cfg.section("energy")
        m = cls(cfg.get("medium", "mu"), e["air"], e["water"], e["transition"])
        m._e_min_per_m = m._compute_min_energy_per_metre()
        return m

    # ---- per-medium physics -------------------------------------------

    def _thrust(self, medium: Medium, v_vec: np.ndarray) -> float:
        speed = float(np.linalg.norm(v_vec))
        if medium is Medium.AIR:
            w = self.air["mass_kg"] * G
            k = self.air["drag_area_cd"]
        else:
            w = self.water["net_weight_n"]
            k = self.water["drag_area_cd"]
        force = np.array([0.0, 0.0, w]) + k * speed * v_vec  # drag opposes motion; thrust must cancel it
        return float(np.linalg.norm(force))

    def _speed_for(self, medium: Medium, direction: np.ndarray) -> float:
        prm = self.air if medium is Medium.AIR else self.water
        vertical = abs(direction[2]) > 0.9
        return prm["vertical_speed"] if vertical else prm["cruise_speed"]

    def _piece_energy(self, medium: Medium, a: np.ndarray, b: np.ndarray) -> float:
        d = b - a
        length = float(np.linalg.norm(d))
        if length < 1e-12:
            return 0.0
        direction = d / length
        if medium is Medium.TRANSITION:
            return self.transition["power_w"] * length / self.transition["speed"]
        speed = self._speed_for(medium, direction)
        c2, c1, c0 = (self.air if medium is Medium.AIR else self.water)["coeffs"]
        thrust = self._thrust(medium, direction * speed)
        return (c2 * thrust**2 + c1 * thrust + c0) * length / speed

    # ---- instantaneous power (used to integrate executed energy) --------

    def instant_power(self, z: float, v_vec, a_vec=None) -> float:
        """Electrical power (W) at height z for velocity v and acceleration a.
        Same thrust->power curves as the planner; the acceleration term m*a is
        what makes executed energy differ from the steady-state plan."""
        from medium import classify

        v_vec = np.asarray(v_vec, dtype=float)
        medium = classify(z, self.mu)
        if medium is Medium.TRANSITION:
            return float(self.transition["power_w"])
        prm = self.air if medium is Medium.AIR else self.water
        w = prm["mass_kg"] * G if medium is Medium.AIR else prm["net_weight_n"]
        force = np.array([0.0, 0.0, w]) + prm["drag_area_cd"] * float(np.linalg.norm(v_vec)) * v_vec
        if a_vec is not None:
            force = force + prm.get("mass_kg", self.air["mass_kg"]) * np.asarray(a_vec, dtype=float)
        c2, c1, c0 = prm["coeffs"]
        t = float(np.linalg.norm(force))
        return c2 * t * t + c1 * t + c0

    # ---- public API -----------------------------------------------------

    def segment_energy(self, p, q) -> EnergyBreakdown:
        """Energy (J) of the straight segment p->q, split by medium."""
        out = EnergyBreakdown()
        pieces = split_by_medium(p, q, self.mu)
        for a, b, medium in pieces:
            e = self._piece_energy(medium, a, b)
            if medium is Medium.AIR:
                out.air += e
            elif medium is Medium.WATER:
                out.water += e
            else:
                out.transition += e
        return out

    def edge_cost(self, p, q) -> float:
        """Planner edge cost (J): segment energy + air<->water crossing charge."""
        return self.segment_energy(p, q).total + self.crossing_cost(p, q)

    def crossing_cost(self, p, q) -> float:
        """Fixed air<->water crossing charge (J) for the segment p->q.

        Half of `fixed_wh` is charged each time the path crosses a transition
        plane (z = +mu, z = -mu), using the same closed/open convention as
        `classify` (z > mu is air; z >= -mu is transition-or-above). A full
        crossing therefore always costs `fixed_wh`, even if a waypoint falls
        inside the band, and the charge is additive over split segments.
        """
        half = 0.5 * self.transition["fixed_wh"] * J_PER_WH
        n = int((p[2] > self.mu) != (q[2] > self.mu)) + int((p[2] >= -self.mu) != (q[2] >= -self.mu))
        return half * n

    def path_energy(self, path) -> EnergyBreakdown:
        total = EnergyBreakdown()
        for a, b in zip(path[:-1], path[1:]):
            total = total + self.segment_energy(a, b)
            total.transition += self.crossing_cost(a, b)
        return total

    def _compute_min_energy_per_metre(self, media=(Medium.AIR, Medium.WATER, Medium.TRANSITION)) -> float:
        """Lower bound on J/m over all directions (26 unit directions) for the
        given media, with a small safety factor: admissible A* heuristic scale
        and RRT* prune bound."""
        import itertools

        dirs = [np.array(m, dtype=float) for m in itertools.product((-1, 0, 1), repeat=3) if m != (0, 0, 0)]
        best = np.inf
        for medium in media:
            if medium is Medium.TRANSITION:
                best = min(best, self.transition["power_w"] / self.transition["speed"])
                continue
            for d in dirs:
                best = min(best, self._piece_energy(medium, np.zeros(3), d / np.linalg.norm(d)))
        return float(best) * 0.999

    def min_energy_per_metre_in(self, z_lo: float, z_hi: float) -> float:
        """Same bound, restricted to the media a map with heights [z_lo, z_hi]
        actually contains (much tighter for an air-only or water-only map)."""
        media = []
        if z_hi > self.mu:
            media.append(Medium.AIR)
        if z_lo < -self.mu:
            media.append(Medium.WATER)
        if z_lo <= self.mu and z_hi >= -self.mu:
            media.append(Medium.TRANSITION)
        return self._compute_min_energy_per_metre(tuple(media))

    @property
    def min_energy_per_metre(self) -> float:
        return self._e_min_per_m


class DistanceModel:
    """Same interface as EnergyModel, but cost = Euclidean length (sanity check
    and ablation: shows what the energy model changes)."""

    mu = 0.0
    min_energy_per_metre = 1.0

    def min_energy_per_metre_in(self, z_lo, z_hi) -> float:
        return 1.0

    def segment_energy(self, p, q) -> EnergyBreakdown:
        return EnergyBreakdown(air=float(np.linalg.norm(np.asarray(q) - np.asarray(p))))

    def crossing_cost(self, p, q) -> float:
        return 0.0

    def edge_cost(self, p, q) -> float:
        return float(np.linalg.norm(np.asarray(q) - np.asarray(p)))

    def path_energy(self, path) -> EnergyBreakdown:
        return EnergyBreakdown(air=float(np.sum(np.linalg.norm(np.diff(np.asarray(path), axis=0), axis=1))))
