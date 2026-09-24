from pathlib import Path

import numpy as np
import pytest

from common import ENERGIA_INSUFICIENTE, SUCESSO
from config import Config
from energy import EnergyModel
from execution import COLISAO, run_mission, surface_points
from helpers import hybrid_world, make
from scenario import from_biguasim, load_scenario, to_biguasim
from world import Cylinder

SC = Path(__file__).resolve().parents[1] / "scenarios"
FAST = Config.load().override(**{"rrt_star.time_budget_s": 1.5, "rrt_star.max_iter": 3000})


def scen(name):
    return load_scenario(SC / name)


# ------------------------------------------------------------- energy detail

def test_crossing_charge_is_additive_over_split_segments():
    _, en, _ = make(hybrid_world(False))
    a, b, c = np.array([0, 0, 4.0]), np.array([0, 0, 0.0]), np.array([0, 0, -4.0])   # b inside the band
    assert en.crossing_cost(a, b) + en.crossing_cost(b, c) == pytest.approx(en.crossing_cost(a, c))
    assert en.crossing_cost(a, c) == pytest.approx(0.05 * 3600)
    d = np.array([0, 0, 0.5])                                                          # exactly on +mu
    assert en.crossing_cost(a, d) + en.crossing_cost(d, c) == pytest.approx(en.crossing_cost(a, c))


def test_instant_power_reduces_to_planned_power_at_constant_velocity():
    _, en, _ = make(hybrid_world(False))
    p = np.array([0, 0, 5.0])
    v = np.array([3.0, 0, 0])
    e = en.segment_energy(p, p + np.array([30.0, 0, 0])).total
    assert en.instant_power(5.0, v) * 10.0 == pytest.approx(e, rel=1e-6)      # 30 m at 3 m/s
    assert en.instant_power(5.0, v, np.array([1.0, 0, 0])) > en.instant_power(5.0, v)


# ---------------------------------------------------------------- coordinates

def test_biguasim_frame_roundtrip():
    p = np.array([12.0, 7.0, -3.0])
    assert np.allclose(from_biguasim(to_biguasim(p)), p)
    assert to_biguasim(p)[1] == -7.0        # east = -y (BiguaSim y is "left")


def test_surface_points_lie_on_the_surface_and_within_range():
    cyl = Cylinder(10, 0, 1.0, 0.0, 10.0)
    pts = surface_points(cyl, 0.3, np.array([10.0, 5.0, 5.0]), 6.0)
    assert len(pts) > 100
    side = pts[(pts[:, 2] > 0.01) & (pts[:, 2] < 9.99)]
    assert np.allclose(np.hypot(side[:, 0] - 10, side[:, 1]), 1.0, atol=1e-6)
    assert np.all(np.linalg.norm(pts - np.array([10.0, 5.0, 5.0]), axis=1) <= 6.0 + 1e-9)


# ------------------------------------------------------------------ missions

@pytest.mark.parametrize("planner", ["astar", "rrt_star"])
def test_c3_known_map_flies_without_replanning_or_collision(planner):
    r = run_mission(scen("c3_buoy_close.yaml"), FAST, planner, "K1", seed=0, resolution=1.0)
    assert r.status == SUCESSO and r.n_replans == 0
    assert r.min_clearance_m > 0.5
    assert np.linalg.norm(np.array(r.trace[-1][1:]) - scen("c3_buoy_close.yaml").goal) < 1.0


@pytest.mark.parametrize("planner", ["astar", "rrt_star"])
def test_hidden_obstacle_is_discovered_and_avoided_under_k2(planner):
    r = run_mission(scen("c3_buoy_close.yaml"), FAST, planner, "K2", seed=0, resolution=1.0)
    assert r.status == SUCESSO
    assert r.n_replans >= 1 and set(r.replan_kinds) == {"map"}
    assert r.min_clearance_m > 0.5


def test_k1_already_knows_the_hidden_obstacle():
    """K1 = fully known map: the hidden pole is on the a-priori map, so no replanning is needed."""
    r = run_mission(scen("c3_buoy_close.yaml"), FAST, "astar", "K1", seed=0, resolution=1.0)
    assert r.n_replans == 0


def test_k6_low_energy_reports_energia_insuficiente_and_does_not_fly():
    r = run_mission(scen("c3_buoy_close.yaml"), FAST, "astar", "K6", seed=0, resolution=1.0)
    assert r.status == ENERGIA_INSUFICIENTE and len(r.trace) == 0


def test_k4_uncertainty_inflates_underwater_plan_only():
    sc = scen("c5_underwater.yaml")
    a = run_mission(sc, FAST, "astar", "K1", seed=0, resolution=1.0)
    b = run_mission(sc, FAST, "astar", "K4", seed=0, resolution=1.0, uncertainty=0.20)
    assert a.status == b.status == SUCESSO
    assert b.min_clearance_m > a.min_clearance_m          # larger margin -> flies farther from obstacles


def test_k5_mission_change_replans_with_set_goal():
    sc = scen("c2_moving_boat.yaml")
    r = run_mission(sc, FAST, "rrt_star", "K5", seed=0)
    assert r.status == SUCESSO and "mission_change" in r.replan_kinds
    assert np.linalg.norm(np.array(r.trace[-1][1:]) - np.array(sc.mission_change["goal"])) < 1.5   # went to the NEW goal


def test_transition_is_crossed_vertically_in_flight():
    sc = scen("c4_transition.yaml")
    r = run_mission(sc, FAST, "astar", "K1", seed=0, resolution=1.0)
    assert r.status == SUCESSO
    tr = np.array(r.trace)
    band = np.abs(tr[:, 3]) <= 0.5
    horiz = np.linalg.norm(np.diff(tr[:, 1:3], axis=0), axis=1)
    assert horiz[band[1:] & band[:-1]].max(initial=0.0) < 0.07      # only a slow (<=0.3 m/s) trim inside the band


def test_executed_energy_is_close_to_planned_on_a_quiet_mission():
    r = run_mission(scen("c1_long_air.yaml"), FAST, "astar", "K1", seed=0, resolution=2.0)
    assert r.status == SUCESSO
    assert abs(r.delta_e_wh) / r.energy_planned_wh < 0.10


def test_rrt_star_replanner_keeps_the_initial_tree_for_reuse():
    """CL-RRT must replan on the tree built for the initial plan, not from scratch."""
    from execution import Mission
    m = Mission(scen("c3_buoy_close.yaml"), FAST, "rrt_star", "K2", 0)
    ok, status, _, _ = m.initial_plan()
    assert ok and len(m.replanners) == len(m.paths)
    rrt = m.replanners[0].rrt
    assert int(rrt.alive[: rrt.n].sum()) > 100          # the tree that produced the plan is still there
    assert np.allclose(m.replanners[0].rrt.goal, m.waypoints[1])
