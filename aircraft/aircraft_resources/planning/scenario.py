"""T8.2 mission scenarios (C1..C6) and their obstacle model.

A scenario file describes ONE mission in the local frame (north, east, up) with
z = 0 at the water surface (the BiguaSim convention). BiguaSim world coordinates
map as  north = x,  east = -y,  up = z  (see to_biguasim()).

Obstacles come in three flavours, which is what lets the same file serve every
test condition (K1..K6):
  * known    : in the a-priori map from the start;
  * hidden   : real, but NOT in the a-priori map -- they are only discovered
               when the vehicle gets within the perception radius (K2/K3);
  * moving   : follow a timed path (the WhiteBoat on the surface, C2): each is a
               box whose centre is given at timestamps, linearly interpolated.

`waypoints` are the ordered mission points (origin, ROI, underwater position,
return). `mission_change` (K5) injects a new goal at a given time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from world import Box, Cylinder, World


@dataclass
class MovingBox:
    name: str
    size: tuple[float, float, float]            # north, east, up extent (m)
    track: list[tuple[float, tuple[float, float, float]]]  # (t, centre)

    def centre_at(self, t: float) -> np.ndarray:
        ts = [p[0] for p in self.track]
        cs = np.array([p[1] for p in self.track], dtype=float)
        return np.array([np.interp(t, ts, cs[:, k]) for k in range(3)])

    def speed_at(self, t: float, dt: float = 0.5) -> float:
        return float(np.linalg.norm(self.centre_at(t + dt) - self.centre_at(t))) / dt

    def box_at(self, t: float) -> Box:
        c, s = self.centre_at(t), np.array(self.size) / 2.0
        return Box(c[0] - s[0], c[0] + s[0], c[1] - s[1], c[1] + s[1], c[2] - s[2], c[2] + s[2])


@dataclass
class Scenario:
    name: str
    phase: str
    description: str
    bounds: np.ndarray
    waypoints: list[np.ndarray]
    waypoint_names: list[str]
    known: list = field(default_factory=list)     # Cylinder | Box
    hidden: list = field(default_factory=list)
    moving: list[MovingBox] = field(default_factory=list)
    mission_change: dict | None = None            # {"t": s, "goal": [n, e, u]}
    underwater_reference_m: float = 30.0          # worst-case underwater distance (K4 uncertainty radius)
    safety_note: str = ""

    @property
    def start(self) -> np.ndarray:
        return self.waypoints[0]

    @property
    def goal(self) -> np.ndarray:
        return self.waypoints[-1]

    def _world(self, obstacles) -> World:
        w = World(bounds=self.bounds)
        for o in obstacles:
            (w.cylinders if isinstance(o, Cylinder) else w.boxes).append(o)
        return w

    def known_world(self) -> World:
        """A-priori map (K1 baseline): known obstacles only (moving ones are added by the harness)."""
        return self._world(self.known)

    def truth_world(self, t: float = 0.0) -> World:
        """Everything that physically exists at time t: used to detect collisions and to feed perception."""
        return self._world(list(self.known) + list(self.hidden) + [m.box_at(t) for m in self.moving])


def _obstacle(o: dict):
    if o["type"] == "cylinder":
        return Cylinder(o["north"], o["east"], o["radius"], *o["z"])
    if o["type"] == "box":
        return Box(*o["north"], *o["east"], *o["up"])
    raise ValueError(f"unknown obstacle type {o['type']!r}")


def load_scenario(path: str | Path) -> Scenario:
    cfg = yaml.safe_load(Path(path).read_text())
    b = cfg["bounds"]
    wps = cfg["waypoints"]
    sc = Scenario(
        name=cfg["name"], phase=cfg.get("phase", ""), description=cfg.get("description", ""),
        bounds=np.array([b["north"], b["east"], b["up"]], dtype=float),
        waypoints=[np.array(w["pos"], dtype=float) for w in wps],
        waypoint_names=[w["name"] for w in wps],
        known=[_obstacle(o) for o in cfg.get("known", [])],
        hidden=[_obstacle(o) for o in cfg.get("hidden", [])],
        moving=[MovingBox(m["name"], tuple(m["size"]), [(float(t), tuple(c)) for t, c in m["track"]])
                for m in cfg.get("moving", [])],
        mission_change=cfg.get("mission_change"),
        underwater_reference_m=float(cfg.get("underwater_reference_m", 30.0)),
    )
    return sc


def to_biguasim(p, home_xy=(0.0, 0.0)) -> list[float]:
    """Planner frame (north, east, up) -> BiguaSim world [x, y, z] (x forward, y left, z up)."""
    return [float(p[0]) + home_xy[0], -float(p[1]) + home_xy[1], float(p[2])]


def from_biguasim(p, home_xy=(0.0, 0.0)) -> np.ndarray:
    return np.array([p[0] - home_xy[0], -(p[1] - home_xy[1]), p[2]], dtype=float)
