import numpy as np
import pytest

import astar
from common import SEM_CAMINHO, SUCESSO, TIMEOUT
from helpers import SCENARIOS, make
from replan import CLRRTReplanner, DStarLiteReplanner
from world import Box, World, load_scenario


def block(hm, centre, half=0.6, step=0.1, t=1.0):
    n = int(half / step)
    cells = [[centre[0] + i * step, centre[1] + j * step, centre[2] + k * step]
             for i in range(-n, n + 1) for j in range(-n, n + 1) for k in range(-n, n + 1)]
    hm.atualizar_ocupacao(cells, "lidar", t)
    return cells


def setup(name="narrow_gap"):
    sc = load_scenario(SCENARIOS / f"{name}.yaml")
    hm, en, cfg = make(sc.world, **{"astar.grid_jitter": False})
    return sc, hm, en, cfg


# ---------------------------------------------------------------- D* Lite

def test_dstar_initial_matches_astar_cost():
    sc, hm, en, cfg = setup()
    d = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal).initial()
    a = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=1.0, jitter=False)
    assert d.status == SUCESSO and d.cost == pytest.approx(a.cost, rel=0.01)


def test_dstar_replans_around_new_obstacle_and_matches_scratch():
    sc, hm, en, cfg = setup()
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    r0 = rp.initial()
    block(hm, r0.path[len(r0.path) // 2])
    r1 = rp.update()
    assert r1.status == SUCESSO and hm.path_free(r1.path) and r1.extra["path_valid"]
    assert r1.cost >= r0.cost - 1e-6                      # an obstacle can only make it worse
    scratch = astar.plan(hm, en, cfg, sc.start, sc.goal, resolution=1.0, jitter=False)
    assert r1.cost == pytest.approx(scratch.cost, rel=0.02)


def test_dstar_recovers_when_obstacle_is_cleared():
    sc, hm, en, cfg = setup()
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    r0 = rp.initial()
    cells = block(hm, r0.path[len(r0.path) // 2])
    rp.update()
    hm.atualizar_ocupacao(np.array([c + [0] for c in cells]), "lidar", 2.0)  # clear them again
    r2 = rp.update()
    assert r2.status == SUCESSO and r2.cost == pytest.approx(r0.cost, rel=1e-6)


def test_dstar_update_without_changes_is_a_noop():
    sc, hm, en, cfg = setup()
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    rp.initial()
    r = rp.update()
    assert r.status == SUCESSO and r.effort == 0


def test_dstar_reuses_search_when_obstacle_is_off_the_path():
    sc, hm, en, cfg = setup("orchard")
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    r0 = rp.initial()
    block(hm, [r0.path[-1][0] - 2, r0.path[-1][1] + 12, r0.path[-1][2]])
    r1 = rp.update()
    assert r1.status == SUCESSO and r1.effort < r0.effort / 3     # far cheaper than the initial search


def test_dstar_sem_caminho_when_sealed():
    sc, hm, en, cfg = setup("narrow_gap")
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    rp.initial()
    for e in np.arange(-5.0, 25.0, 0.5):                       # wall across the whole map, all heights
        for z in np.arange(0.5, 8.0, 0.5):
            hm.atualizar_ocupacao([[13.0, e, z]], "lidar", 1.0)
    r = rp.update()
    assert r.status == SEM_CAMINHO


def test_dstar_move_start_keeps_a_valid_path():
    sc, hm, en, cfg = setup()
    rp = DStarLiteReplanner(hm, en, cfg, sc.start, sc.goal)
    r0 = rp.initial()
    rp.move_start(r0.path[len(r0.path) // 3])
    r1 = rp.update()
    assert r1.status == SUCESSO and np.allclose(r1.path[0], r0.path[len(r0.path) // 3])
    assert r1.cost < r0.cost


# ----------------------------------------------------------------- CL-RRT

def test_clrrt_keeps_tree_when_map_change_is_irrelevant():
    sc, hm, en, cfg = setup()
    rp = CLRRTReplanner(hm, en, cfg, sc.start, sc.goal, seed=1)
    r0 = rp.initial()
    assert r0.status == SUCESSO
    block(hm, [26.0, -4.0, 6.0])                                # nowhere near the route
    r1 = rp.update()
    assert r1.status == SUCESSO and r1.extra["nodes_removed"] == 0 and r1.effort == 0
    assert np.array_equal(r0.path, r1.path)


def test_clrrt_prunes_and_regrows_around_new_obstacle():
    sc, hm, en, cfg = setup()
    rp = CLRRTReplanner(hm, en, cfg, sc.start, sc.goal, seed=1)
    r0 = rp.initial(improve=False)
    tree_before = r0.extra["tree_size"]
    block(hm, r0.path[len(r0.path) // 2])
    r1 = rp.update(budget_s=30)
    assert r1.extra["nodes_removed"] > 0
    assert r1.status == SUCESSO and hm.path_free(r1.path)
    assert r1.extra["tree_size"] < tree_before + r1.effort     # it did not start from an empty tree


def test_clrrt_is_faster_than_planning_from_scratch_for_small_changes():
    sc, hm, en, cfg = setup()
    rp = CLRRTReplanner(hm, en, cfg, sc.start, sc.goal, seed=2)
    rp.initial(budget_s=10, improve=True)
    r0 = rp.rrt.result()
    block(hm, r0.path[len(r0.path) // 2], half=0.3)
    r1 = rp.update(budget_s=30)
    assert r1.status == SUCESSO and hm.path_free(r1.path)
    assert r1.time_s < r0.time_s


def test_clrrt_reroot_rebases_costs():
    sc, hm, en, cfg = setup()
    rp = CLRRTReplanner(hm, en, cfg, sc.start, sc.goal, seed=1)
    r0 = rp.initial()
    target = r0.path[len(r0.path) // 2]
    rp.move_start(target)
    r1 = rp.rrt.result()
    assert r1.status == SUCESSO and np.allclose(r1.path[0], target) and r1.cost < r0.cost


def test_clrrt_timeout_when_sealed():
    w = World(bounds=[[0, 20], [0, 20], [1, 10]])
    hm, en, cfg = make(w)
    rp = CLRRTReplanner(hm, en, cfg, np.array([2.0, 10, 5]), np.array([18.0, 10, 5]), seed=0)
    rp.initial(budget_s=5)
    for e in np.arange(0.0, 20.5, 0.5):
        for z in np.arange(1.0, 10.5, 0.5):
            hm.atualizar_ocupacao([[10.0, e, z]], "lidar", 1.0)
    r = rp.update(budget_s=2)
    assert r.status == TIMEOUT                                    # sampling never proves "no path"
