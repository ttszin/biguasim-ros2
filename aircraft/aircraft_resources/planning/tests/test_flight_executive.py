"""PlannedFlight against a simulated vehicle (same interface the mission_node and the pymavlink executive use)."""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

PLANNING = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PLANNING / "stage_b"))

from common import ENERGIA_INSUFICIENTE, SUCESSO  # noqa: E402
from common_b import sitl_config  # noqa: E402
from flight_executive import PlannedFlight  # noqa: E402
from scenario import load_scenario  # noqa: E402

SCEN = PLANNING / "stage_b" / "scenarios"
CFG = sitl_config().override(**{"rrt_star.time_budget_s": 1.5, "rrt_star.max_iter": 3000})


class SimVehicle:
    """Point mass tracking the last goto with a speed limit and an acceleration limit; hold = stay."""

    def __init__(self, start, vmax=2.5, amax=2.0):
        self.pos, self.vel, self.target = np.array(start, float), np.zeros(3), np.array(start, float)
        self.vmax, self.amax = vmax, amax

    def apply(self, cmd):
        if cmd is not None:
            self.target = np.array(cmd.target if cmd.kind == "goto" else self.pos, float)

    def advance(self, dt):
        d = self.target - self.pos
        dist = float(np.linalg.norm(d))
        v_des = np.zeros(3) if dist < 1e-6 else d / dist * min(self.vmax, np.sqrt(2 * 1.0 * dist))
        dv = v_des - self.vel
        n = float(np.linalg.norm(dv))
        self.vel = self.vel + (dv if n <= self.amax * dt else dv / n * self.amax * dt)
        self.pos = self.pos + self.vel * dt


def fly(scen, planner, cond, seed=0, mission_change=None, external_goal=None, max_t=150.0, **kw):
    sc = load_scenario(SCEN / scen)
    flight = PlannedFlight(sc, CFG, planner, cond, seed, resolution=1.0, mission_change=mission_change, **kw)
    ok, status, detail = flight.plan()
    if not ok:
        return flight, sc, None
    veh = SimVehicle([0.0, 0.0, flight.takeoff_altitude])
    veh.apply(flight.begin(veh.pos, 0.0, 0.0))
    truth = sc.truth_world(0.0)
    min_clear, t, dt = np.inf, 0.0, 0.1
    while not flight.done and t < max_t:
        veh.advance(dt)
        t += dt
        min_clear = min(min_clear, float(truth.clearance(veh.pos[None, :])[0]))
        if external_goal is not None and abs(t - external_goal[0]) < dt / 2:
            flight.set_goal(external_goal[1])
        veh.apply(flight.step(veh.pos, veh.vel, t, t))
    return flight, sc, {"min_clear": min_clear, "pos": veh.pos, "t": t}


@pytest.mark.parametrize("planner", ["astar", "rrt_star"])
def test_k1_reaches_the_goal_without_replanning(planner):
    flight, sc, out = fly("b1_poles.yaml", planner, "K1")
    assert flight.status == SUCESSO
    assert np.linalg.norm(out["pos"] - sc.goal) < 1.0
    assert out["min_clear"] > 0.5
    assert len(flight.result()["replans"]) == 0


@pytest.mark.parametrize("planner", ["astar", "rrt_star"])
def test_k2_discovers_the_hidden_pole_and_replans(planner):
    flight, sc, out = fly("b1_poles.yaml", planner, "K2")
    res = flight.result()
    assert flight.status == SUCESSO and len(res["replans"]) >= 1
    assert out["min_clear"] > 0.5
    assert len(res["path_history"]) == 1 + len(res["replans"])


@pytest.mark.parametrize("planner", ["astar", "rrt_star"])
def test_timed_mission_change_goes_to_the_new_goal(planner):
    new_goal = [30.0, -6.0, 6.0]
    flight, sc, out = fly("b1_poles.yaml", planner, "K1", mission_change={"t": 5.0, "goal": new_goal})
    res = flight.result()
    assert flight.status == SUCESSO
    assert np.linalg.norm(out["pos"] - np.array(new_goal)) < 1.0
    assert [r.get("kind") for r in res["replans"]] == ["mission_change"]
    assert out["min_clear"] > 0.5


def test_external_goal_from_a_topic_is_applied():
    new_goal = [30.0, -6.0, 6.0]
    flight, sc, out = fly("b1_poles.yaml", "rrt_star", "K1", external_goal=(4.0, new_goal))
    assert flight.status == SUCESSO and np.linalg.norm(out["pos"] - np.array(new_goal)) < 1.0


def test_k6_low_energy_is_rejected_before_takeoff():
    flight, sc, out = fly("b1_poles.yaml", "astar", "K6")
    assert out is None and flight.status == ENERGIA_INSUFICIENTE and flight.done


def test_explicit_energy_budget_below_the_plan_is_rejected():
    flight, sc, out = fly("b1_poles.yaml", "astar", "K1", energy_available_wh=5.0)   # 5 - 10 Wh reserve < plan
    assert out is None and flight.status == ENERGIA_INSUFICIENTE


def test_stop_and_go_at_a_sharp_gate_turn_still_finishes():
    flight, sc, out = fly("b2_gate.yaml", "astar", "K2")
    assert flight.status == SUCESSO and out["min_clear"] > 0.5


def test_result_is_json_serialisable_and_has_what_the_analysis_needs():
    flight, sc, out = fly("b1_poles.yaml", "astar", "K2")
    res = json.loads(json.dumps(flight.result()))
    for key in ("scenario", "planner", "condition", "seed", "status", "t_initial_ms", "planned_wh", "planned_length_m",
                "n_waypoints", "path_history", "replans", "events", "trace", "wall_start", "wall_end", "stopped_at_waypoints"):
        assert key in res, key
    assert len(res["trace"][0]) == 10


def test_a_flight_cannot_outlive_its_time_limit():
    sc = load_scenario(SCEN / "b1_poles.yaml")
    flight = PlannedFlight(sc, CFG, "astar", "K1", 0, resolution=1.0, max_flight_s=3.0)
    assert flight.plan()[0]
    flight.begin([0, 0, 5.0], 0.0, 0.0)
    cmd = None
    for k in range(60):
        cmd = flight.step([0, 0, 5.0], [0, 0, 0], 0.1 * k, 0.1 * k)
    assert flight.done and flight.status == "ABORTADO" and "time limit" in flight.detail
