import numpy as np
import pytest

import astar
import rrt_star
from common import SEM_CAMINHO, SUCESSO, TIMEOUT, path_cost, path_length
from helpers import SCENARIOS, assert_vertical_only_in_band, hybrid_world, make
from postprocess import prune
from world import Box, World, load_scenario

HYB_START, HYB_GOAL = np.array([0.0, 0.0, 4.0]), np.array([26.0, 6.0, -4.0])


def _scenario(name, **ov):
    sc = load_scenario(SCENARIOS / f"{name}.yaml")
    hm, en, cfg = make(sc.world, **ov)
    return sc, hm, en, cfg


# ------------------------------------------------------------------- A*

def test_astar_open_air_is_near_straight_line_cost():
    hm, en, cfg = make(World(bounds=[[0, 20], [0, 20], [1, 12]]), **{"astar.grid_jitter": False})
    start, goal = np.array([1.0, 1.0, 3.0]), np.array([15.0, 9.0, 6.0])
    r = astar.plan(hm, en, cfg, start, goal, resolution=1.0)
    assert r.status == SUCESSO
    assert np.allclose(r.path[0], start) and np.allclose(r.path[-1], goal)
    straight = en.edge_cost(start, goal)
    # 26-connected grids cannot follow arbitrary directions: a modest excess over the straight line is expected
    assert straight * 0.999 <= r.cost <= 1.20 * straight


@pytest.mark.parametrize("name", ["orchard", "urban", "narrow_gap"])
def test_astar_collision_free_in_bundled_scenarios(name):
    sc, hm, en, cfg = _scenario(name)
    r = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=1.0)
    assert r.status == SUCESSO, r.extra
    assert hm.path_free(r.path)


def test_astar_heuristic_is_admissible_matches_dijkstra():
    """weight 1 must find the same optimal cost as blind Dijkstra (weight 0), with fewer expansions."""
    for name, res in (("narrow_gap", 1.0), ("orchard", 2.0)):  # the 1.5 m free gap needs a 1 m grid
        sc, hm, en, cfg = _scenario(name, **{"astar.grid_jitter": False})
        a = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=res, heuristic_weight=1.0)
        d = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=res, heuristic_weight=0.0)
        # compare on-grid cost: the final path cost also includes snapping the ends to the exact start/goal
        assert a.extra["grid_cost"] == pytest.approx(d.extra["grid_cost"], rel=1e-9), name
        assert a.effort < d.effort, name


def test_astar_hybrid_heuristic_is_admissible():
    hm, en, cfg = make(hybrid_world(), **{"astar.grid_jitter": False})
    a = astar.plan(hm, en, cfg, HYB_START, HYB_GOAL, resolution=2.0, heuristic_weight=1.0)
    d = astar.plan(hm, en, cfg, HYB_START, HYB_GOAL, resolution=2.0, heuristic_weight=0.0)
    assert a.success and a.extra["grid_cost"] == pytest.approx(d.extra["grid_cost"], rel=1e-9)


def test_astar_is_deterministic_and_jitter_is_seeded():
    sc, hm, en, cfg = _scenario("narrow_gap")
    a = astar.plan(hm, en, cfg, sc.start, sc.goal, seed=5, resolution=1.0)
    b = astar.plan(hm, en, cfg, sc.start, sc.goal, seed=5, resolution=1.0)
    c = astar.plan(hm, en, cfg, sc.start, sc.goal, seed=6, resolution=1.0)
    assert np.array_equal(a.path, b.path)
    assert a.extra["offset"] != c.extra["offset"]


def test_astar_reports_sem_caminho_when_wall_blocks_everything():
    w = World(bounds=[[0, 20], [0, 20], [1, 10]], boxes=[Box(9, 11, -1, 21, 0, 11)])
    hm, en, cfg = make(w)
    r = astar.plan(hm, en, cfg, np.array([2.0, 10, 5]), np.array([18.0, 10, 5]), resolution=1.0)
    assert r.status == SEM_CAMINHO and np.isinf(r.cost)


def test_astar_reports_timeout():
    sc, hm, en, cfg = _scenario("urban")
    r = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=0.5, time_budget_s=0.001)
    assert r.status == TIMEOUT


def test_astar_air_to_water_crosses_vertically_and_avoids_obstacles():
    hm, en, cfg = make(hybrid_world())
    r = astar.plan(hm, en, cfg, HYB_START, HYB_GOAL, resolution=1.0)
    assert r.status == SUCESSO, r.extra
    assert hm.path_free(r.path)
    assert_vertical_only_in_band(r.path, hm.mu)
    assert r.path[0][2] > hm.mu and r.path[-1][2] < -hm.mu


def test_astar_heavier_weight_expands_no_more_nodes():
    sc, hm, en, cfg = _scenario("orchard", **{"astar.grid_jitter": False})
    opt = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=1.0, heuristic_weight=1.0)
    fast = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=1.0, heuristic_weight=2.0)
    assert fast.effort <= opt.effort
    assert opt.extra["grid_cost"] <= fast.extra["grid_cost"] + 1e-6
    assert fast.extra["grid_cost"] <= 2.0 * opt.extra["grid_cost"] + 1e-6


# ------------------------------------------------------------------ RRT*

@pytest.mark.parametrize("name", ["orchard", "urban", "narrow_gap"])
def test_rrt_star_collision_free_in_bundled_scenarios(name):
    sc, hm, en, cfg = _scenario(name)
    r = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=1, max_iter=6000, time_budget_s=60)
    assert r.status == SUCESSO
    assert hm.path_free(r.path)
    assert np.allclose(r.path[0], sc.start) and np.allclose(r.path[-1], sc.goal)
    assert r.cost == pytest.approx(path_cost(r.path, en), rel=1e-6)


def test_rrt_star_is_deterministic_for_a_seed():
    sc, hm, en, cfg = _scenario("narrow_gap")
    a = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=7, max_iter=1200, time_budget_s=60)
    b = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=7, max_iter=1200, time_budget_s=60)
    assert a.status == SUCESSO and np.array_equal(a.path, b.path)


def test_rrt_star_history_strictly_improves_and_matches_final_cost():
    sc, hm, en, cfg = _scenario("narrow_gap")
    r = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=2, max_iter=2500, time_budget_s=60)
    costs = [c for _, _, c in r.extra["history"]]
    assert costs and all(b < a for a, b in zip(costs, costs[1:]))
    assert costs[-1] == pytest.approx(r.cost)


def test_rrt_star_more_iterations_never_hurt():
    sc, hm, en, cfg = _scenario("narrow_gap")
    short = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=3, max_iter=800, time_budget_s=60)
    long = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=3, max_iter=2400, time_budget_s=60)
    assert short.success and long.success and long.cost <= short.cost + 1e-6


def test_rrt_star_air_to_water_crosses_vertically():
    hm, en, cfg = make(hybrid_world())
    r = rrt_star.plan(hm, en, cfg, HYB_START, HYB_GOAL, seed=0, max_iter=6000, time_budget_s=60)
    assert r.status == SUCESSO
    assert hm.path_free(r.path)
    assert_vertical_only_in_band(r.path, hm.mu)


def test_rrt_star_miss_is_timeout_not_sem_caminho():
    w = World(bounds=[[0, 20], [0, 20], [1, 10]], boxes=[Box(9, 11, -1, 21, 0, 11)])
    hm, en, cfg = make(w)
    r = rrt_star.plan(hm, en, cfg, np.array([2.0, 10, 5]), np.array([18.0, 10, 5]), seed=0, max_iter=300, time_budget_s=30)
    assert r.status == TIMEOUT and np.isinf(r.cost)


def test_rrt_star_time_budget_is_respected():
    sc, hm, en, cfg = _scenario("orchard")
    r = rrt_star.plan(hm, en, cfg, sc.start, sc.goal, seed=0, max_iter=10**6, time_budget_s=0.5)
    assert r.time_s < 1.5


# ---------------------------------------------------------------- pruning

@pytest.mark.parametrize("planner", ["astar", "rrt"])
def test_prune_never_costs_more_and_stays_valid(planner):
    hm, en, cfg = make(hybrid_world())
    if planner == "astar":
        r = astar.plan(hm, en, cfg, HYB_START, HYB_GOAL, resolution=1.0)
    else:
        r = rrt_star.plan(hm, en, cfg, HYB_START, HYB_GOAL, seed=4, max_iter=4000, time_budget_s=60)
    assert r.success
    pruned = prune(r.path, hm, en)
    assert path_cost(pruned, en) <= path_cost(r.path, en) + 1e-6
    assert len(pruned) <= len(r.path)
    assert hm.path_free(pruned) and np.allclose(pruned[0], r.path[0]) and np.allclose(pruned[-1], r.path[-1])
    assert_vertical_only_in_band(pruned, hm.mu)


def test_prune_stops_on_marginal_gain():
    hm, en, cfg = make(World(bounds=[[0, 30], [0, 10], [1, 10]]))
    path = np.array([[0, 0, 5.0], [10, 0.01, 5.0], [20, 0.0, 5.0]])
    assert len(prune(path, hm, en, min_gain=0.01)) == 2
