"""
T2 Direct Runner — controle de posição via BiguaSim cmd_pos_yaw, sem ArduPilot/MAVROS.

Fluxo da missão:
  WARMUP → TAKEOFF → FLY_TO_WATER → HOVER → DESCEND → ASCEND → HOVER_EXIT → RETURN_HOME → DONE

Início (só 2 terminais):
  1. python3 t2_direct_runner.py [--viewport] [--water-x 25] [--water-y 0] [--descent-z -2]
  2. python3 validador_t2.py

Calibração:
  bash t2_direct_run.sh --viewport --calibrate --water-x 25 --water-y 0
"""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import String

import biguasim
from biguasim.ardubridge.frame import depth_to_pressure

ATM_PRESSURE = 101325.0  # Pa — pressão atmosférica padrão


# ---------------------------------------------------------------------------
# Estados da missão
# ---------------------------------------------------------------------------

class S:
    WARMUP       = "WARMUP"
    TAKEOFF      = "TAKEOFF"
    FLY_TO_WATER = "FLY_TO_WATER"
    HOVER        = "HOVER"          # paira acima da água antes de descer
    DESCEND      = "DESCEND"
    ASCEND       = "ASCEND"
    HOVER_EXIT   = "HOVER_EXIT"     # paira acima da água após sair
    RETURN_HOME  = "RETURN_HOME"
    DONE         = "DONE"
    CALIBRATE    = "CALIBRATE"


# ---------------------------------------------------------------------------
# Runner principal
# ---------------------------------------------------------------------------

class T2DirectRunner:

    WARMUP_FRAMES  = 300    # frames iniciais parado para estabilizar
    REACH_XY_TOL   = 1.0   # metros — tolerância XY
    REACH_Z_TOL    = 1.0   # metros — tolerância Z
    HOVER_SECS     = 3.0   # segundos de hover antes/depois de descer
    NAV_TIMEOUT_S  = 60.0  # timeout aguardando transição de nav_mode

    def __init__(
        self,
        spawn: list,
        cruise_z: float,
        water_xy: list,
        water_z: float,
        descent_z: float,
        speed: float = 3.0,
        speed_z: float = 3.0,
        ticks: int = 100,
        show_viewport: bool = False,
        calibrate: bool = False,
    ):
        self._spawn     = np.array(spawn, dtype=float)
        self._cruise_z  = cruise_z
        self._water_xy  = np.array(water_xy, dtype=float)
        self._water_z   = water_z
        self._descent_z = descent_z
        self._speed_z   = speed_z
        self._ticks     = ticks
        self._calibrate = calibrate

        self._state     = S.CALIBRATE if calibrate else S.WARMUP
        self._warmup_n  = 0
        self._nav_mode  = "AERIAL_NAV"
        self._nav_t0    = None
        self._hover_t0  = None
        self._agent     = "hydrone0"

        # debug: tempo da última linha impressa no DESCEND
        self._last_debug_t = 0.0

        self._patch_gains(speed)
        scenario = self._build_scenario(ticks)
        self._env = biguasim.make(scenario_cfg=scenario, show_viewport=show_viewport)

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("t2_direct_runner")
        self._press_pub = self._node.create_publisher(FluidPressure, "/fcu/external_pressure", 10)
        self._node.create_subscription(String, "/nav_mode", self._nav_cb, 10)
        threading.Thread(target=lambda: rclpy.spin(self._node), daemon=True).start()

    # ------------------------------------------------------------------
    # Gains
    # ------------------------------------------------------------------

    @staticmethod
    def _patch_gains(speed: float) -> None:
        from biguasim.dynamics.agents import DjiMatrice
        # Só escala ganhos de posição — atitude fica nos valores originais para não causar deriva XY
        DjiMatrice._params["kp_pos"] = 0.05 * speed
        DjiMatrice._params["kd_pos"] = 0.01 * speed
        DjiMatrice._params["kp_att"] = 0.1
        DjiMatrice._params["kd_att"] = 0.01
        print(f"[T2] kp_pos={DjiMatrice._params['kp_pos']:.3f}  kp_att=0.1 (fixo)  (speed={speed}x)")

    # ------------------------------------------------------------------
    # Scenario
    # ------------------------------------------------------------------

    def _build_scenario(self, ticks: int) -> dict:
        sensors = [
            {"sensor_type": "DynamicsSensor", "socket": "COM",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "LocationSensor", "socket": "COM",
             "configuration": {"Sigma": 0}},
            {"sensor_type": "VelocitySensor", "socket": "COM"},
            {"sensor_type": "IMUSensor", "socket": "IMUSocket", "Hz": ticks,
             "configuration": {"AccelSigma": 0.0, "AngVelSigma": 0.0,
                               "AccelBiasSigma": 0.0, "AngVelBiasSigma": 0.0,
                               "ReturnBias": False}},
            {"sensor_type": "DepthSensor", "socket": "DepthSocket", "Hz": ticks,
             "configuration": {"Sigma": 0.0}},
        ]
        return {
            "package_name": "SkyDive",
            "world": "Bridge",
            "main_agent": self._agent,
            "ticks_per_sec": ticks,
            "frames_per_sec": False,
            "octree_min": 0.02,
            "octree_max": 5.0,
            "agents": [{
                "agent_name": self._agent,
                "agent_type": "DjiMatrice",
                "control_abstraction": "cmd_pos_yaw",
                "location": self._spawn.tolist(),
                "rotation": [0.0, 0.0, 0.0],
                "dynamics": {"batch_size": 1},
                "sensors": sensors,
            }],
        }

    # ------------------------------------------------------------------
    # Callbacks / helpers
    # ------------------------------------------------------------------

    def _nav_cb(self, msg: String) -> None:
        self._nav_mode = msg.data

    def _pos(self, state: dict) -> np.ndarray | None:
        loc = state.get("LocationSensor")
        return np.array(loc, dtype=float) if loc is not None else None

    def _depth_and_pressure(self, state: dict) -> tuple[float | None, float | None]:
        raw = state.get("DepthSensor")
        if raw is None:
            return None, None
        z_up = float(raw[0]) if hasattr(raw, "__len__") else float(raw)
        return z_up, depth_to_pressure(z_up)

    def _publish_pressure(self, state: dict) -> float | None:
        depth, pressure = self._depth_and_pressure(state)
        if pressure is None:
            return None
        msg = FluidPressure()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.fluid_pressure = pressure
        self._press_pub.publish(msg)
        return pressure

    def _z_cmd(self, current_z: float, target_z: float) -> float:
        """Virtual Z amplificado pelo speed_z — equivale a kp_pos maior só no eixo Z."""
        error = target_z - current_z
        if abs(error) <= self.REACH_Z_TOL:
            return target_z
        return current_z + error * self._speed_z

    def _reached(self, current: np.ndarray, target: np.ndarray) -> bool:
        return (float(np.linalg.norm(current[:2] - target[:2])) < self.REACH_XY_TOL
                and abs(current[2] - target[2]) < self.REACH_Z_TOL)

    def _hover_done(self) -> bool:
        if self._hover_t0 is None:
            self._hover_t0 = time.monotonic()
        return (time.monotonic() - self._hover_t0) >= self.HOVER_SECS

    def _nav_timeout(self) -> bool:
        return self._nav_t0 is not None and (time.monotonic() - self._nav_t0) > self.NAV_TIMEOUT_S

    def _transition(self, new_state: str, msg: str) -> None:
        self._state = new_state
        self._hover_t0 = None
        print(f"[T2] {msg} → {new_state}")

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------

    def run(self) -> None:
        spawn_cmd = self._spawn.tolist() + [0.0]
        cmd = spawn_cmd

        with self._env:
            self._env.step(cmd)
            self._env._agent.teleport(location=np.array(self._spawn, dtype=np.float32))
            raw = self._env.step(cmd)

            print(f"\n[T2] spawn={self._spawn}  cruise_z={self._cruise_z}")
            print(f"[T2] água xy={self._water_xy}  surface_z={self._water_z}  descent_z={self._descent_z}")
            print(f"[T2] estado inicial: {self._state}  (Ctrl-C para parar)\n")

            try:
                while self._state != S.DONE:
                    state = raw[self._agent][0]
                    pos   = self._pos(state)
                    pres  = self._publish_pressure(state)

                    # ---------- CALIBRATE ----------
                    if self._state == S.CALIBRATE:
                        self._warmup_n += 1
                        if self._warmup_n < self.WARMUP_FRAMES:
                            cmd = spawn_cmd
                        else:
                            cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                            if pos is not None and self._warmup_n % self._ticks == 0:
                                depth, _ = self._depth_and_pressure(state)
                                depth_str = f"{depth:.4f}" if depth is not None else "N/A"
                                print(f"[CALIBRATE] pos={pos.round(2)}  depth={depth_str}m")

                    # ---------- WARMUP ----------
                    elif self._state == S.WARMUP:
                        cmd = spawn_cmd
                        self._warmup_n += 1
                        if self._warmup_n >= self.WARMUP_FRAMES:
                            self._transition(S.TAKEOFF, f"WARMUP completo ({self.WARMUP_FRAMES} frames)")

                    # ---------- TAKEOFF: sobe reto no ponto de spawn, sem boost de Z ----------
                    elif self._state == S.TAKEOFF:
                        target = np.array([self._spawn[0], self._spawn[1], self._cruise_z])
                        cmd    = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._transition(S.FLY_TO_WATER, f"altitude {pos[2]:.1f}m atingida")

                    # ---------- FLY_TO_WATER: voa em direção à água ----------
                    elif self._state == S.FLY_TO_WATER:
                        target = np.array([self._water_xy[0], self._water_xy[1], self._cruise_z])
                        cmd    = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._transition(S.HOVER, f"sobre a água em {pos.round(2)}")

                    # ---------- HOVER: paira antes de descer ----------
                    elif self._state == S.HOVER:
                        cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                        if self._hover_done():
                            self._transition(S.DESCEND, f"hover concluído ({self.HOVER_SECS}s)")

                    # ---------- DESCEND: desce reto para dentro da água ----------
                    elif self._state == S.DESCEND:
                        z_v = self._z_cmd(pos[2], self._descent_z) if pos is not None else self._descent_z
                        cmd = [self._water_xy[0], self._water_xy[1], z_v, 0.0]

                        # --- DEBUG: imprime a cada 1s ---
                        now = time.monotonic()
                        if now - self._last_debug_t >= 1.0:
                            self._last_debug_t = now
                            depth, pressure = self._depth_and_pressure(state)
                            pos_str   = pos.round(2) if pos is not None else "?"
                            depth_str = f"{depth:.4f}m" if depth is not None else "N/A"
                            pres_str  = f"{pressure:.1f}Pa" if pressure is not None else "N/A"
                            above_atm = f"+{pressure - ATM_PRESSURE:.1f}Pa" if pressure is not None else ""
                            print(
                                f"[DESCEND] pos={pos_str}  depth={depth_str}"
                                f"  pressure={pres_str} ({above_atm})"
                                f"  nav={self._nav_mode}"
                            )
                            if depth is not None and depth <= 0:
                                print(f"[DESCEND] AVISO: depth={depth:.4f} ≤ 0 — drone ainda fora da água.")
                            if "DepthSensor" not in state:
                                print("[DESCEND] AVISO: DepthSensor ausente no agent_state — verifique o cenário.")

                        if self._nav_mode == "AQUATIC_NAV":
                            self._nav_t0 = time.monotonic()
                            self._transition(S.ASCEND, f"AQUATIC_NAV em pos={pos.round(2) if pos is not None else '?'}")
                        elif pos is not None and self._reached(pos, np.array([self._water_xy[0], self._water_xy[1], self._descent_z])):
                            print(f"[T2] AVISO: atingiu descent_z={self._descent_z:.1f} sem AQUATIC_NAV.")
                            print("[T2]        Verifique water-z e descent-z — a água pode estar em z diferente.")

                    # ---------- ASCEND: sobe reto para fora da água ----------
                    elif self._state == S.ASCEND:
                        z_v = self._z_cmd(pos[2], self._cruise_z) if pos is not None else self._cruise_z
                        cmd = [self._water_xy[0], self._water_xy[1], z_v, 0.0]
                        if self._nav_mode == "AERIAL_NAV":
                            self._transition(S.HOVER_EXIT, f"AERIAL_NAV em pos={pos.round(2) if pos is not None else '?'}")
                        elif self._nav_timeout():
                            print("[T2] TIMEOUT aguardando AERIAL_NAV — abortando.")
                            break

                    # ---------- HOVER_EXIT: paira após sair da água ----------
                    elif self._state == S.HOVER_EXIT:
                        cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                        if self._hover_done():
                            self._transition(S.RETURN_HOME, f"hover pós-água concluído ({self.HOVER_SECS}s)")

                    # ---------- RETURN_HOME: volta ao ponto de spawn, sem boost de Z ----------
                    elif self._state == S.RETURN_HOME:
                        target = np.array([self._spawn[0], self._spawn[1], self._cruise_z])
                        cmd    = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._state = S.DONE
                            print(f"[T2] De volta ao home em {pos.round(2)}")
                            print("[T2] Missão T2 CONCLUÍDA com sucesso!")

                    raw = self._env.step(cmd)

            except KeyboardInterrupt:
                print("\n[T2] Interrompido pelo usuário.")
            finally:
                self._node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="T2 Direct Runner — BiguaSim cmd_pos_yaw")
    parser.add_argument("--viewport", action="store_true")
    parser.add_argument("--ticks",    type=int,   default=100)
    parser.add_argument("--speed",    type=float, default=3.0,
                        help="Multiplicador de ganho de posição XY (default: 3.0)")
    parser.add_argument("--speed-z",  type=float, default=3.0,
                        help="Multiplicador de velocidade vertical (default: 3.0)")
    parser.add_argument("--spawn", nargs=3, type=float, default=[8.0, 0.0, 13.4],
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--cruise-z", type=float, default=28.0,
                        help="Z de cruzeiro no mundo NWU (default: 28.0)")
    parser.add_argument("--water-x",   type=float, default=25.0)
    parser.add_argument("--water-y",   type=float, default=0.0)
    parser.add_argument("--water-z",   type=float, default=0.0,
                        help="Z da superfície da água (default: 0.0)")
    parser.add_argument("--descent-z", type=float, default=-2.0,
                        help="Z alvo abaixo da água (default: -2.0)")
    parser.add_argument("--calibrate", action="store_true",
                        help="Voa até water-xy e printa posição+depth para calibrar")

    args = parser.parse_args()

    runner = T2DirectRunner(
        spawn=args.spawn,
        cruise_z=args.cruise_z,
        water_xy=[args.water_x, args.water_y],
        water_z=args.water_z,
        descent_z=args.descent_z,
        speed=args.speed,
        speed_z=args.speed_z,
        ticks=args.ticks,
        show_viewport=args.viewport,
        calibrate=args.calibrate,
    )
    runner.run()


if __name__ == "__main__":
    main()
