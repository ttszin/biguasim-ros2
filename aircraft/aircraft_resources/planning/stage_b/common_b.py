"""Shared constants for Stage B (BiguaSim + ArduPilot SITL in GUIDED)."""

from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANNING = HERE.parent
sys.path.insert(0, str(PLANNING))

from config import Config  # noqa: E402

# Spawn (= ArduPilot home) in BiguaSim world coordinates, confirmed in T2.
HOME_BSIM = (8.0, 0.0, 13.4)
MISSIONS_DIR = PLANNING.parent / "missions"
SCENARIO_DIR = HERE / "scenarios"


def to_bsim(north: float, east: float, up: float) -> list[float]:
    """Home-relative (north, east, up) -> BiguaSim world [x, y, z] (x forward = north, y left = -east)."""
    return [HOME_BSIM[0] + north, HOME_BSIM[1] - east, HOME_BSIM[2] + up]


def from_bsim(x: float, y: float, z: float) -> tuple[float, float, float]:
    return x - HOME_BSIM[0], -(y - HOME_BSIM[1]), z - HOME_BSIM[2]


def sitl_config() -> Config:
    """Planner config for the real vehicle in SITL: config/planner_sitl.yaml (the same file the mission_node's
    'plan_route' action loads by default). The speed the vehicle actually flies under GUIDED is not WPNAV_SPEED: the
    first SITL flight (b1_poles, A*, K1) averaged 2.46 m/s with peaks near 5 m/s, so the plan uses 2.5 m/s. That
    flight is a CALIBRATION flight (results/stage_b_calibration): only flights after it say anything about delta E."""
    return Config.load(PLANNING / "config" / "planner_sitl.yaml")
