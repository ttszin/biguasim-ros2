"""Transport-independent flight executive for the planners (used by the mission_node and by the pymavlink executive).

`PlannedFlight` owns everything that is NOT I/O: the a-priori map and planners, simulated perception, replanning
when the route ahead is cut, mission change (a new goal), the energy feasibility check, waypoint arrival logic
(fly-by vs stop-and-go) and the result record. The caller supplies the vehicle: it feeds position and velocity in
and executes the returned commands (`goto`/`hold`). Two callers exist today:

  * aircraft_ws/src/mission/mission/mission_node.py   (ROS 2 Humble: MAVROS local pose -> SetReposition in GUIDED)
  * planning/stage_b/sitl_exec.py                     (pymavlink -> SET_POSITION_TARGET_LOCAL_NED)

so the same logic flies through both and can be compared. Frame: (north, east, up) metres RELATIVE TO HOME.

Typical use:
    flight = PlannedFlight(scenario, cfg, "astar", "K2")
    flight.plan()                                  # before takeoff; may return ENERGIA_INSUFICIENTE
    cmd = flight.begin(pos, wall_now, t_sim)       # first waypoint
    loop: cmd = flight.step(pos, vel, t_sim, wall_now)   # None = nothing to do
    flight.result()                                # JSON-serialisable record

Python 3.10 compatible (the aircraft image is Humble / Python 3.10).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from common import ENERGIA_INSUFICIENTE, SUCESSO
from energy import wh
from execution import Mission
from postprocess import prune

STOP_SPEED = 0.25          # m/s: "stopped" for stop-and-go waypoints
MAX_FLIGHT_S = 420.0


@dataclass
class Command:
    kind: str                      # "goto" | "hold"
    target: np.ndarray             # north, east, up (relative to home)
    reason: str = ""


class PlannedFlight:
    def __init__(self, scenario, cfg, planner: str, condition: str = "K1", seed: int = 0, resolution: float | None = None,
                 mission_change: dict | None = None, energy_available_wh: float | None = None,
                 max_flight_s: float = MAX_FLIGHT_S):
        self.sc, self.cfg, self.planner, self.condition, self.seed = scenario, cfg, planner, condition, seed
        self.resolution = resolution if planner == "astar" else None
        self.m = Mission(scenario, cfg, planner, condition, seed, resolution=self.resolution)
        self.mission_change = mission_change
        self.max_flight_s = max_flight_s
        self.reserve_wh = float(cfg.get("energy", "reserve_wh"))
        self.available_wh = float(energy_available_wh if energy_available_wh is not None else cfg.get("energy", "battery_wh"))
        self.status = "ABORTADO"
        self.detail = ""
        self.done = False
        self.leg, self.idx = 0, 1
        self._checked_version = self.m.map.version
        self._pending_goal: np.ndarray | None = None
        self._change_done = False
        self._t0_sim: float | None = None
        self._stopped_at_wp = 0
        self.rec: dict = {
            "scenario": scenario.name, "planner": planner, "condition": condition, "seed": seed,
            "resolution": self.resolution, "replans": [], "events": [], "trace": [], "path_history": [],
        }

    # ------------------------------------------------------------------ planning

    def plan(self) -> tuple[bool, str, str]:
        """Initial plan of every leg. Returns (ok, status, detail). Call before takeoff."""
        ok, status, t_init, t_first = self.m.initial_plan()
        self.rec.update(t_initial_ms=t_init * 1000, t_first_solution_ms=t_first * 1000)
        if not ok:
            self.status, self.detail = status, "initial plan failed"
            self.rec.update(status=self.status, detail=self.detail)
            return False, self.status, self.detail
        if self.planner == "astar":
            self.m.build_replanners()
        planned_j = self.m.planned_energy_j()
        planned_wh = wh(planned_j)
        if self.condition == "K6":
            self.available_wh = self.reserve_wh + float(self.m.opts["energy_fraction"]) * planned_wh
        self.rec.update(
            planned_paths=[p.tolist() for p in self.m.paths], planned_wh=planned_wh,
            planned_length_m=float(sum(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)) for p in self.m.paths)),
            n_waypoints=int(sum(len(p) for p in self.m.paths) - (len(self.m.paths) - 1)),
            available_wh=self.available_wh)
        self.rec["path_history"].append({"t": 0.0, "path": self.m.paths[0].tolist()})
        if planned_wh > self.available_wh - self.reserve_wh:
            self.status = ENERGIA_INSUFICIENTE
            self.detail = (f"plan needs {planned_wh:.2f} Wh, available {self.available_wh:.2f} Wh - reserve {self.reserve_wh:.1f} Wh")
            self.done = True
            self.rec.update(status=self.status, detail=self.detail)
            return False, self.status, self.detail
        return True, SUCESSO, f"{self.rec['n_waypoints']} waypoints, {planned_wh:.3f} Wh, initial {t_init * 1000:.0f} ms"

    @property
    def takeoff_altitude(self) -> float:
        return float(self.m.paths[0][0][2])

    # ------------------------------------------------------------------- flying

    def begin(self, pos, wall_now: float, t_sim: float) -> Command:
        self.rec["hover_start"] = np.asarray(pos, dtype=float).tolist()
        self.rec["wall_start"] = wall_now
        self._t0_sim = t_sim
        self.leg, self.idx = 0, 1
        self._checked_version = self.m.map.version
        return Command("goto", self._target(), "start")

    def set_goal(self, goal) -> None:
        """Mission change from outside (T11): applied on the next step."""
        self._pending_goal = np.asarray(goal, dtype=float)

    def _target(self) -> np.ndarray:
        return self.m.paths[self.leg][self.idx]

    def _stop_needed(self, pos) -> bool:
        nominal = self.m.nominal_speed(pos, self._target())
        v_wp, must = self.m.waypoint_speed(self.m.paths, self.leg, self.idx, nominal)
        return bool(must or v_wp < 0.5 * nominal)

    def _install(self, path, pos) -> None:
        path = np.asarray(path, dtype=float)
        if np.linalg.norm(path[0] - pos) > 1e-6:
            path = np.vstack([pos[None, :], path])
        self.m.paths[self.leg] = prune(path, self.m.map, self.m.energy, float(self.cfg.get("postprocess", "min_gain")))
        self.idx = 1

    def _finish(self, status: str, detail: str, wall_now: float, pos) -> None:
        self.status, self.detail, self.done = status, detail, True
        self.rec.update(status=status, detail=detail, wall_end=wall_now, final_position=np.asarray(pos).tolist(),
                        stopped_at_waypoints=self._stopped_at_wp)

    def step(self, pos, vel, t_sim: float, wall_now: float) -> Command | None:
        """One control cycle. Returns a Command when the vehicle has to be told something, else None."""
        if self.done:
            return None
        pos = np.asarray(pos, dtype=float)
        vel = np.asarray(vel, dtype=float)
        t = t_sim - (self._t0_sim if self._t0_sim is not None else t_sim)
        self.rec["trace"].append([wall_now, t_sim, *pos.tolist(), *vel.tolist(), self.leg, self.idx])
        if t > self.max_flight_s:
            self._finish("ABORTADO", "flight time limit", wall_now, pos)
            return None

        if self.condition in ("K2", "K3"):
            self.m.sense_static(pos, t)

        # mission change: an external goal (topic) or the timed one from the parameters
        goal = self._pending_goal
        if goal is None and self.mission_change and not self._change_done and t >= float(self.mission_change["t"]):
            goal = np.asarray(self.mission_change["goal"], dtype=float)
        if goal is not None:
            self._pending_goal = None
            self._change_done = True
            return self._change_goal(goal, pos, t, wall_now)

        # replan when the remaining route was cut by newly sensed cells
        if self.m.map.version != self._checked_version:
            self._checked_version = self.m.map.version
            remaining = np.vstack([pos[None, :], self.m.paths[self.leg][self.idx:]])
            if not self.m.map.path_free(remaining):
                return self._replan(pos, t, wall_now)

        target = self._target()
        dist = float(np.linalg.norm(pos - target))
        speed = float(np.linalg.norm(vel))
        need_stop = self._stop_needed(pos)
        last = self.leg == len(self.m.paths) - 1 and self.idx == len(self.m.paths[self.leg]) - 1
        radius = self.m.accept_radius(self.m.paths[self.leg], self.idx, pos)
        if dist <= radius and (not (need_stop or last) or speed < STOP_SPEED):
            if need_stop and not last:
                self._stopped_at_wp += 1
            self.rec["events"].append(["waypoint", wall_now, t, self.leg, self.idx, dist, speed, bool(need_stop or last)])
            self.idx += 1
            if self.idx >= len(self.m.paths[self.leg]):
                self.leg += 1
                self.idx = 1
            if self.leg >= len(self.m.paths):
                self._finish(SUCESSO, "", wall_now, pos)
                return None
            return Command("goto", self._target(), "next waypoint")
        return None

    def _replan(self, pos, t: float, wall_now: float) -> Command | None:
        t0 = time.perf_counter()
        rp = self.m.replanners[self.leg]
        rp.move_start(pos)
        r = rp.update()
        if not r.success:
            self._finish(r.status, f"replan failed at t={t:.1f}s", wall_now, pos)
            return Command("hold", pos, "replan failed")
        self._install(r.path, pos)
        ms = (time.perf_counter() - t0) * 1000
        self.rec["path_history"].append({"t": t, "path": self.m.paths[self.leg].tolist()})
        self.rec["replans"].append({"t": t, "ms": ms, "waypoints": len(self.m.paths[self.leg]), "pos": pos.tolist()})
        self.rec["events"].append(["replan", wall_now, t])
        return Command("goto", self._target(), "replan")

    def _change_goal(self, goal, pos, t: float, wall_now: float) -> Command | None:
        """New final goal (mission change): reuse the planner state (CL-RRT keeps its tree, D* Lite restarts the search)."""
        t0 = time.perf_counter()
        if self.leg != len(self.m.paths) - 1:
            self._finish("ABORTADO", "mission change is only supported on the last leg", wall_now, pos)
            return Command("hold", pos, "mission change unsupported")
        rp = self.m.replanners[self.leg]
        rp.move_start(pos)
        r = rp.set_goal(goal)
        if not r.success:
            self._finish(r.status, f"mission change failed at t={t:.1f}s", wall_now, pos)
            return Command("hold", pos, "mission change failed")
        self.m.waypoints[-1] = goal
        self._install(r.path, pos)
        ms = (time.perf_counter() - t0) * 1000
        self.rec["path_history"].append({"t": t, "path": self.m.paths[self.leg].tolist()})
        self.rec["replans"].append({"t": t, "ms": ms, "waypoints": len(self.m.paths[self.leg]), "pos": pos.tolist(),
                                    "kind": "mission_change", "goal": goal.tolist()})
        self.rec["events"].append(["mission_change", wall_now, t])
        return Command("goto", self._target(), "mission change")

    # ------------------------------------------------------------------- result

    def abort(self, detail: str, wall_now: float, pos=(0.0, 0.0, 0.0)) -> None:
        if not self.done:
            self._finish("ABORTADO", detail, wall_now, pos)

    def result(self) -> dict:
        self.rec.setdefault("status", self.status)
        self.rec.setdefault("detail", self.detail)
        return self.rec
