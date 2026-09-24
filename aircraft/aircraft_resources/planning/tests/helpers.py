from pathlib import Path

import numpy as np

from config import Config
from energy import EnergyModel
from hybrid_map import HybridMap
from medium import violates_vertical_rule
from world import Box, Cylinder, World, load_scenario

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


def make(world: World, **overrides):
    cfg = Config.load().override(**overrides) if overrides else Config.load()
    return HybridMap(world, cfg), EnergyModel.from_config(cfg), cfg


def air_world(x=30.0, y=20.0, z=10.0):
    return World(bounds=[[-1, x], [-y / 2, y / 2], [0.5, z]])


def hybrid_world(obstacles=True):
    """Air above z=0, water below; a pillar in the air and one underwater."""
    w = World(bounds=[[-1, 30], [-10, 10], [-8, 8]], safety_margin=0.0)
    if obstacles:
        w.cylinders.append(Cylinder(10, 0, 1.0, 0.0, 8.0))     # airborne mast
        w.boxes.append(Box(18, 20, -4, 10, -8, -0.6))           # underwater wall with a gap at east>10
    return w


def assert_vertical_only_in_band(path, mu):
    for a, b in zip(path[:-1], path[1:]):
        assert not violates_vertical_rule(a, b, mu), f"non-vertical transition segment {a} -> {b}"
