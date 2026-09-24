import numpy as np
import pytest

from common import path_cost
from config import Config
from energy import EnergyModel, wh
from helpers import SCENARIOS, air_world, hybrid_world, make
from hybrid_map import HybridMap, gps_to_local
from medium import Medium, classify, split_by_medium, violates_vertical_rule
from world import Box, Cylinder, World, load_scenario

MU = 0.5


# ------------------------------------------------------------------ medium

def test_classify_boundaries():
    assert classify(0.51, MU) is Medium.AIR
    assert classify(0.5, MU) is Medium.TRANSITION
    assert classify(0.0, MU) is Medium.TRANSITION
    assert classify(-0.5, MU) is Medium.TRANSITION
    assert classify(-0.51, MU) is Medium.WATER


def test_split_by_medium_pieces_cover_segment_and_are_single_medium():
    pieces = split_by_medium([0, 0, 3], [0, 0, -3], MU)
    assert [m for _, _, m in pieces] == [Medium.AIR, Medium.TRANSITION, Medium.WATER]
    assert np.allclose(pieces[0][0], [0, 0, 3]) and np.allclose(pieces[-1][1], [0, 0, -3])
    for (_, b, _), (a, _, _) in zip(pieces[:-1], pieces[1:]):
        assert np.allclose(b, a)


def test_vertical_rule():
    assert violates_vertical_rule([0, 0, 3], [4, 0, -3], MU)         # diagonal through the band
    assert not violates_vertical_rule([0, 0, 3], [0, 0, -3], MU)     # vertical is fine
    assert not violates_vertical_rule([0, 0, 3], [9, 0, 3], MU)      # horizontal in air
    assert not violates_vertical_rule([0, 0, -3], [9, 0, -3], MU)    # horizontal in water
    assert violates_vertical_rule([0, 0, 0.2], [5, 0, 0.2], MU)      # horizontal inside the band
    assert not violates_vertical_rule([0, 0, 0.5 + 1e-3], [5, 0, 0.5 + 1e-3], MU)


# ------------------------------------------------------------------ energy

def test_energy_is_positive_and_scales_with_length():
    _, en, _ = make(air_world())
    e1 = en.segment_energy(np.array([0, 0, 5.0]), np.array([10, 0, 5.0])).total
    e2 = en.segment_energy(np.array([0, 0, 5.0]), np.array([20, 0, 5.0])).total
    assert e1 > 0 and e2 == pytest.approx(2 * e1)


def test_energy_split_by_medium_and_separate_transition_bucket():
    _, en, _ = make(hybrid_world(False))
    e = en.segment_energy(np.array([0, 0, 4.0]), np.array([0, 0, -4.0]))
    assert e.air > 0 and e.water > 0 and e.transition > 0
    assert e.total == pytest.approx(e.air + e.water + e.transition)
    assert en.crossing_cost(np.array([0, 0, 4.0]), np.array([0, 0, -4.0])) == pytest.approx(0.05 * 3600)
    assert en.crossing_cost(np.array([0, 0, 4.0]), np.array([9, 0, 4.0])) == 0.0


def test_energy_differs_from_distance():
    """Same length, different medium -> different cost (that is the point of the model)."""
    _, en, _ = make(hybrid_world(False))
    air = en.edge_cost(np.array([0, 0, 4.0]), np.array([10, 0, 4.0]))
    water = en.edge_cost(np.array([0, 0, -4.0]), np.array([10, 0, -4.0]))
    assert air != pytest.approx(water, rel=0.05)


def test_min_energy_per_metre_is_a_true_lower_bound():
    _, en, _ = make(hybrid_world(False))
    rng = np.random.default_rng(0)
    for _ in range(300):
        p = rng.uniform([-1, -5, -7], [29, 5, 7])
        q = p + rng.normal(size=3) * 3
        d = float(np.linalg.norm(q - p))
        assert en.edge_cost(p, q) >= en.min_energy_per_metre * d - 1e-6


def test_path_energy_matches_sum_of_edges():
    _, en, _ = make(hybrid_world(False))
    path = np.array([[0, 0, 4.0], [10, 0, 4.0], [10, 0, -4.0], [20, 0, -4.0]])
    assert en.path_energy(path).total == pytest.approx(path_cost(path, en))


def test_config_override_and_unknown_key():
    cfg = Config.load()
    assert cfg.override(**{"rrt_star.step": 1.25}).get("rrt_star", "step") == 1.25
    assert cfg.get("rrt_star", "step") != 1.25  # original untouched
    with pytest.raises(KeyError):
        cfg.override(**{"rrt_star.not_a_param": 1})


# --------------------------------------------------------------------- map

def test_margin_is_larger_in_water():
    hm, _, _ = make(hybrid_world(False))
    assert hm.margin_for(Medium.WATER) > hm.margin_for(Medium.AIR)


def test_a_priori_obstacle_uses_the_medium_margin():
    w = World(bounds=[[-1, 30], [-10, 10], [-8, 8]], cylinders=[Cylinder(10, 0, 1.0, 0.0, 8.0),
                                                              Cylinder(10, 5, 1.0, -8.0, -0.6)], safety_margin=0.0)
    hm, _, _ = make(w)  # air margin 1.0, water margin 1.5
    # 1.2 m off the mast axis+radius: 1.0 + 0.2 -> inside the 1.0 air margin? distance to surface = 0.2 -> collides
    assert not hm.point_free([10, 1.2, 3.0])
    assert hm.point_free([10, 2.2, 3.0])                       # 1.2 m from surface > 1.0
    # same geometry underwater: 1.2 m from surface < 1.5 water margin -> collides
    assert not hm.point_free([10, 5 + 2.2, -3.0])
    assert hm.point_free([10, 5 + 2.6, -3.0])


def test_segment_check_is_continuous_across_media():
    hm, _, _ = make(hybrid_world())
    assert not hm.segment_free([10, -6, 4.0], [10, 6, 4.0])     # runs into the mast
    assert hm.segment_free([10, 3.5, 4.0], [10, 9, 4.0])
    assert not hm.segment_free([0, 0, 4.0], [10, 0, -4.0])      # diagonal through the transition zone


def test_atualizar_ocupacao_blocks_and_clears():
    hm, _, _ = make(air_world())
    assert hm.segment_free([2, 0, 5], [25, 0, 5])
    n = hm.atualizar_ocupacao([[10.0, 0.0, 5.0]], "lidar", 1.0)
    assert n == 1 and hm.n_cells == 1
    assert not hm.segment_free([2, 0, 5], [25, 0, 5])
    assert not hm.point_free([10.0, 0.5, 5.0])
    hm.atualizar_ocupacao([[10.0, 0.0, 5.0, 0]], "lidar", 2.0)   # free the cell
    assert hm.n_cells == 0 and hm.segment_free([2, 0, 5], [25, 0, 5])


def test_atualizar_ocupacao_validates_source_and_shape():
    hm, _, _ = make(air_world())
    with pytest.raises(ValueError):
        hm.atualizar_ocupacao([[1, 1, 1]], "radar", 0.0)
    with pytest.raises(ValueError):
        hm.atualizar_ocupacao([[1, 1]], "lidar", 0.0)
    assert hm.atualizar_ocupacao(np.empty((0, 3)), "lidar", 0.0) == 0


def test_same_cells_from_different_sources_give_the_same_map():
    a, _, _ = make(air_world())
    b, _, _ = make(air_world())
    cells = [[10.0, 1.0, 5.0], [10.0, 1.1, 5.0]]
    a.atualizar_ocupacao(cells, "lidar", 0.0)
    b.atualizar_ocupacao(cells, "sonar_hydrone", 0.0)
    for seg in (([2, 1, 5], [25, 1, 5]), ([2, 6, 5], [25, 6, 5])):
        assert a.segment_free(*seg) == b.segment_free(*seg)


def test_newly_occupied_since_tracks_versions():
    hm, _, _ = make(air_world())
    v0 = hm.version
    hm.atualizar_ocupacao([[5.0, 0.0, 5.0]], "lidar", 0.0)
    v1 = hm.version
    hm.atualizar_ocupacao([[8.0, 0.0, 5.0]], "lidar", 1.0)
    assert len(hm.newly_occupied_since(v0)) == 2 and len(hm.newly_occupied_since(v1)) == 1


def test_position_uncertainty_grows_with_distance_from_the_gps_fix_and_only_underwater():
    hm, _, _ = make(World(bounds=[[-1, 60], [-10, 10], [-8, 8]], cylinders=[Cylinder(10, 3, 0.5, -8, 8),
                                                                              Cylinder(50, 3, 0.5, -8, 8)], safety_margin=0.0))
    near_water, far_water = [10, 5.4, -4.0], [50, 5.4, -4.0]
    near_air, far_air = [10, 5.4, 4.0], [50, 5.4, 4.0]
    base = (hm.point_free(near_water), hm.point_free(far_water), hm.point_free(near_air), hm.point_free(far_air))
    assert all(base)
    hm.set_position_uncertainty(0.10, [0, 0, -3.0])     # fix at the origin: radius = 10 % of the distance
    assert not hm.point_free(far_water)                  # 50 m away -> ~5 m extra margin blocks it
    assert hm.point_free(near_air) and hm.point_free(far_air)       # air is unaffected
    assert hm.margin_for(Medium.WATER, np.array([0, 0, -3.0])) == pytest.approx(hm.margin_water, abs=1e-9)
    assert hm.margin_for(Medium.WATER, np.array([40, 0, -3.0])) > hm.margin_for(Medium.WATER, np.array([10, 0, -3.0]))


def test_vectorised_points_free_agrees_with_scalar():
    hm, _, _ = make(hybrid_world())
    hm.atualizar_ocupacao([[5.0, 5.0, 3.0], [22.0, 8.0, -5.0]], "lidar", 0.0)
    rng = np.random.default_rng(1)
    pts = rng.uniform([-1, -10, -8], [30, 10, 8], size=(400, 3))
    vec = hm.points_free(pts)
    assert all(vec[i] == hm.point_free(pts[i]) for i in range(len(pts)))


def test_gps_to_local_roundtrip_scale():
    home = (-32.0, -52.0, 5.0)
    p = gps_to_local(-32.0 + 1e-4, -52.0, 12.0, home)
    assert p[0] == pytest.approx(11.13, abs=0.05) and abs(p[1]) < 1e-6 and p[2] == pytest.approx(7.0)


def test_bundled_scenarios_still_load():
    for name in ("orchard", "urban", "narrow_gap"):
        sc = load_scenario(SCENARIOS / f"{name}.yaml")
        hm, _, _ = make(sc.world)
        assert hm.point_free(sc.start) and hm.point_free(sc.goal)
