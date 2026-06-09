"""
T2 Direct Runner com BlueROV2 — controle de posição via BiguaSim cmd_pos_yaw, sem ArduPilot/MAVROS.

Fluxo da missão:
  WARMUP → TAKEOFF → FLY_TO_WATER → HOVER → DESCEND →
  ROV_TOUR (drone paira na borda, ROV controlado via ROS2) →
  ASCEND → HOVER_EXIT → RETURN_HOME → DONE

  • DESCEND : drone toca a água → AQUATIC_NAV → controle passa ao ROV
  • ROV_TOUR : drone paira em water_z; ROV recebe comandos de /bluerov0/cmd_pos_yaw
               e sinaliza fim via /bluerov0/tour_done
  • ASCEND   : drone sobe → AERIAL_NAV → controle volta ao UAV

Tópicos publicados:
  /hydrone0/local_position  [geometry_msgs/Point]  — posição do drone (BiguaSim)
  /bluerov0/local_position  [geometry_msgs/Point]  — posição do ROV  (BiguaSim)

Tópicos consumidos:
  /nav_mode                 [std_msgs/String]       — AERIAL_NAV / AQUATIC_NAV
  /bluerov0/cmd_pos_yaw     [geometry_msgs/Point]   — setpoint de posição do ROV
  /bluerov0/tour_done       [std_msgs/Bool]         — sinaliza fim do tour (rov_tour_node)

Uso:
  python3 t2_direct_runner.py [--viewport] [--water-x 25] [--water-y 0] [--descent-z -2]
  bash t2_direct_run.sh --viewport --calibrate --water-x 25 --water-y 0
"""

from __future__ import annotations

import argparse
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, String

import biguasim
from biguasim.ardubridge.frame import depth_to_pressure

ATM_PRESSURE = 101325.0


# ---------------------------------------------------------------------------
# Estados da missão
# ---------------------------------------------------------------------------

class S:
    WARMUP       = "WARMUP"
    TAKEOFF      = "TAKEOFF"
    FLY_TO_WATER = "FLY_TO_WATER"
    HOVER        = "HOVER"
    DESCEND      = "DESCEND"
    ROV_TOUR     = "ROV_TOUR"      # drone paira na borda, ROV controlado via ROS2
    ASCEND       = "ASCEND"
    HOVER_EXIT   = "HOVER_EXIT"
    RETURN_HOME  = "RETURN_HOME"
    LAND         = "LAND"
    DONE         = "DONE"
    CALIBRATE    = "CALIBRATE"


# ---------------------------------------------------------------------------
# Runner principal
# ---------------------------------------------------------------------------

class T2DirectRunner:

    WARMUP_FRAMES  = 300
    REACH_XY_TOL   = 1.0
    REACH_Z_TOL    = 1.0
    HOVER_SECS     = 3.0
    NAV_TIMEOUT_S  = 60.0
    TOUR_TIMEOUT_S = 300.0  # aborta ROV_TOUR se tour_done não chegar a tempo

    def __init__(
        self,
        spawn: list,
        cruise_z: float,
        water_xy: list,
        water_z: float,
        descent_z: float,
        rov_depth: float = -0.5,
        speed: float = 3.0,
        speed_z: float = 3.0,
        ticks: int = 100,
        show_viewport: bool = False,
        calibrate: bool = False,
    ):
        self._spawn      = np.array(spawn, dtype=float)
        self._cruise_z   = cruise_z
        self._water_xy   = np.array(water_xy, dtype=float)
        self._water_z    = water_z
        self._descent_z  = descent_z
        self._speed_z    = speed_z
        self._ticks      = ticks
        self._calibrate  = calibrate

        # ROV spawna logo abaixo da superfície (rov_depth é negativo em NWU)
        self._rov_spawn = np.array([water_xy[0], water_xy[1], rov_depth], dtype=float)

        self._state     = S.CALIBRATE if calibrate else S.WARMUP
        self._warmup_n  = 0
        self._nav_mode  = "AERIAL_NAV"
        self._nav_t0    = None
        self._hover_t0  = None
        self._agent     = "hydrone0"
        self._rov_agent = "bluerov0"

        # Controle ROS2 do ROV
        self._rov_cmd_ros: list | None = None   # setpoint recebido via ROS2
        self._tour_done_flag = False             # sinalizado por /bluerov0/tour_done
        self._rov_tour_t0: float | None = None  # timestamp de entrada em ROV_TOUR

        self._last_debug_t = 0.0

        self._patch_gains(speed)
        scenario = self._build_scenario(ticks)
        self._env = biguasim.make(scenario_cfg=scenario, show_viewport=show_viewport)

        if not rclpy.ok():
            rclpy.init()
        self._node = Node("t2_direct_runner")

        # Publishers
        self._press_pub     = self._node.create_publisher(FluidPressure, "/fcu/external_pressure", 10)
        self._drone_pos_pub = self._node.create_publisher(Point, "/hydrone0/local_position", 10)
        self._rov_pos_pub   = self._node.create_publisher(Point, "/bluerov0/local_position", 10)

        # Subscribers
        self._node.create_subscription(String, "/nav_mode",              self._nav_cb,      10)
        self._node.create_subscription(Point,  "/bluerov0/cmd_pos_yaw",  self._rov_cmd_cb,  10)
        self._node.create_subscription(Bool,   "/bluerov0/tour_done",    self._tour_done_cb, 10)

        threading.Thread(target=lambda: rclpy.spin(self._node), daemon=True).start()

    # ------------------------------------------------------------------
    # Gains
    # ------------------------------------------------------------------

    @staticmethod
    def _patch_gains(speed: float) -> None:
        from biguasim.dynamics.agents import DjiMatrice, BlueROV2
        DjiMatrice._params["kp_pos"] = 0.05 * speed
        DjiMatrice._params["kd_pos"] = 0.01 * speed
        DjiMatrice._params["kp_att"] = 0.1
        DjiMatrice._params["kd_att"] = 0.01
        # kp_pos=0.25 → amortecimento crítico: ζ = sqrt(k_vxy/kp_pos)/2 = sqrt(1/0.25)/2 = 1
        # Mesma força inicial do original (v_des clamped a 1.5 m/s para erro ≥ 6m = k_vxy/kp_pos*1.5/4)
        # Desaceleração proporcional suave para erro < 6m → sem overshoot/oscilação
        BlueROV2._params["kp_pos"] = 0.25
        print(f"[T2] kp_pos={DjiMatrice._params['kp_pos']:.3f}  kp_att=0.1 (fixo)  (speed={speed}x)")
        print(f"[T2] BlueROV2 kp_pos=0.25 (amortecimento crítico)")

    # ------------------------------------------------------------------
    # Scenario
    # ------------------------------------------------------------------

    def _build_scenario(self, ticks: int) -> dict:
        drone_sensors = [
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
        rov_sensors = [
            {"sensor_type": "DynamicsSensor", "socket": "COM",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "LocationSensor", "socket": "COM",
             "configuration": {"Sigma": 0}},
        ]
        return {
            "package_name": "SkyDive",
            "world": "Bridge",
            "main_agent": self._agent,
            "ticks_per_sec": ticks,
            "frames_per_sec": False,
            "octree_min": 0.02,
            "octree_max": 5.0,
            "agents": [
                {
                    "agent_name": self._agent,
                    "agent_type": "DjiMatrice",
                    "control_abstraction": "cmd_pos_yaw",
                    "location": self._spawn.tolist(),
                    "rotation": [0.0, 0.0, 0.0],
                    "dynamics": {"batch_size": 1},
                    "sensors": drone_sensors,
                },
                {
                    "agent_name": self._rov_agent,
                    "agent_type": "BlueROV2",
                    "control_abstraction": "cmd_pos_yaw",
                    "location": self._rov_spawn.tolist(),
                    "rotation": [0.0, 0.0, 0.0],
                    "dynamics": {"batch_size": 1},
                    "sensors": rov_sensors,
                },
            ],
        }

    # ------------------------------------------------------------------
    # Callbacks ROS2
    # ------------------------------------------------------------------

    def _nav_cb(self, msg: String) -> None:
        self._nav_mode = msg.data

    def _rov_cmd_cb(self, msg: Point) -> None:
        if self._rov_cmd_ros is None:
            print(f"[T2] /bluerov0/cmd_pos_yaw recebido pela primeira vez: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]")
        self._rov_cmd_ros = [msg.x, msg.y, msg.z, 0.0]

    def _tour_done_cb(self, msg: Bool) -> None:
        if msg.data:
            self._tour_done_flag = True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pos(self, state: dict) -> np.ndarray | None:
        loc = state.get("LocationSensor")
        return np.array(loc, dtype=float) if loc is not None else None

    def _depth_and_pressure(self, state: dict) -> tuple[float | None, float | None]:
        raw = state.get("DepthSensor")
        if raw is None:
            return None, None
        z_up = float(raw[0]) if hasattr(raw, "__len__") else float(raw)
        return z_up, depth_to_pressure(z_up)

    def _publish_pressure(self, state: dict) -> None:
        depth, pressure = self._depth_and_pressure(state)
        if pressure is None:
            return
        msg = FluidPressure()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.fluid_pressure = pressure
        self._press_pub.publish(msg)

    def _publish_positions(self, pos: np.ndarray | None, rov_pos: np.ndarray | None) -> None:
        if pos is not None:
            pt = Point()
            pt.x, pt.y, pt.z = float(pos[0]), float(pos[1]), float(pos[2])
            self._drone_pos_pub.publish(pt)
        if rov_pos is not None:
            pt = Point()
            pt.x, pt.y, pt.z = float(rov_pos[0]), float(rov_pos[1]), float(rov_pos[2])
            self._rov_pos_pub.publish(pt)

    def _z_cmd(self, current_z: float, target_z: float) -> float:
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
        rov_hold  = self._rov_spawn.tolist() + [0.0]

        drone_cmd = spawn_cmd
        rov_cmd   = rov_hold

        with self._env:
            self._env.step({self._agent: drone_cmd, self._rov_agent: rov_cmd})
            self._env._agent.teleport(location=np.array(self._spawn, dtype=np.float32))
            raw = self._env.step({self._agent: drone_cmd, self._rov_agent: rov_cmd})

            print(f"\n[T2] spawn={self._spawn}  cruise_z={self._cruise_z}")
            print(f"[T2] água xy={self._water_xy}  surface_z={self._water_z}  descent_z={self._descent_z}")
            print(f"[T2] ROV spawn={self._rov_spawn}")
            print(f"[T2] ROS2: /bluerov0/cmd_pos_yaw  /bluerov0/tour_done  /hydrone0/local_position  /bluerov0/local_position")
            print(f"[T2] estado inicial: {self._state}  (Ctrl-C para parar)\n")

            try:
                while self._state != S.DONE:
                    drone_state = raw[self._agent][0]
                    rov_state   = raw[self._rov_agent][0]
                    pos         = self._pos(drone_state)
                    rov_pos     = self._pos(rov_state)

                    self._publish_pressure(drone_state)
                    self._publish_positions(pos, rov_pos)

                    # ---------- CALIBRATE ----------
                    if self._state == S.CALIBRATE:
                        self._warmup_n += 1
                        if self._warmup_n < self.WARMUP_FRAMES:
                            drone_cmd = spawn_cmd
                        else:
                            drone_cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                            if pos is not None and self._warmup_n % self._ticks == 0:
                                depth, _ = self._depth_and_pressure(drone_state)
                                depth_str = f"{depth:.4f}" if depth is not None else "N/A"
                                print(f"[CALIBRATE] pos={pos.round(2)}  depth={depth_str}m")

                    # ---------- WARMUP ----------
                    elif self._state == S.WARMUP:
                        drone_cmd = spawn_cmd
                        self._warmup_n += 1
                        if self._warmup_n >= self.WARMUP_FRAMES:
                            self._transition(S.TAKEOFF, f"WARMUP completo ({self.WARMUP_FRAMES} frames)")

                    # ---------- TAKEOFF ----------
                    elif self._state == S.TAKEOFF:
                        target = np.array([self._spawn[0], self._spawn[1], self._cruise_z])
                        drone_cmd = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._transition(S.FLY_TO_WATER, f"altitude {pos[2]:.1f}m atingida")

                    # ---------- FLY_TO_WATER ----------
                    elif self._state == S.FLY_TO_WATER:
                        target = np.array([self._water_xy[0], self._water_xy[1], self._cruise_z])
                        drone_cmd = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._transition(S.HOVER, f"sobre a água em {pos.round(2)}")

                    # ---------- HOVER: paira antes de descer ----------
                    elif self._state == S.HOVER:
                        drone_cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                        if self._hover_done():
                            self._transition(S.DESCEND, f"hover concluído ({self.HOVER_SECS}s)")

                    # ---------- DESCEND: desce até a água ----------
                    elif self._state == S.DESCEND:
                        z_v = self._z_cmd(pos[2], self._descent_z) if pos is not None else self._descent_z
                        drone_cmd = [self._water_xy[0], self._water_xy[1], z_v, 0.0]

                        now = time.monotonic()
                        if now - self._last_debug_t >= 1.0:
                            self._last_debug_t = now
                            depth, pressure = self._depth_and_pressure(drone_state)
                            pos_str   = pos.round(2) if pos is not None else "?"
                            depth_str = f"{depth:.4f}m" if depth is not None else "N/A"
                            pres_str  = f"{pressure:.1f}Pa" if pressure is not None else "N/A"
                            above_atm = f"+{pressure - ATM_PRESSURE:.1f}Pa" if pressure is not None else ""
                            print(
                                f"[DESCEND] pos={pos_str}  depth={depth_str}"
                                f"  pressure={pres_str} ({above_atm})"
                                f"  nav={self._nav_mode}"
                            )
                            if depth is not None and depth >= 0:
                                print(f"[DESCEND] AVISO: depth={depth:.4f} ≥ 0 — drone ainda fora da água.")
                            if "DepthSensor" not in drone_state:
                                print("[DESCEND] AVISO: DepthSensor ausente no agent_state — verifique o cenário.")

                        if self._nav_mode == "AQUATIC_NAV":
                            self._tour_done_flag = False
                            self._rov_tour_t0 = time.monotonic()
                            self._transition(
                                S.ROV_TOUR,
                                f"AQUATIC_NAV → aguardando tour ROS2 em pos={pos.round(2) if pos is not None else '?'}",
                            )
                        elif pos is not None and self._reached(pos, np.array([self._water_xy[0], self._water_xy[1], self._descent_z])):
                            print(f"[T2] AVISO: atingiu descent_z={self._descent_z:.1f} sem AQUATIC_NAV.")
                            print("[T2]        Verifique water-z e descent-z — a água pode estar em z diferente.")

                    # ---------- ROV_TOUR: drone paira, ROV controlado via ROS2 ----------
                    elif self._state == S.ROV_TOUR:
                        # Drone paira na superfície da água
                        drone_cmd = [self._water_xy[0], self._water_xy[1], self._water_z, 0.0]

                        # ROV usa setpoint recebido via ROS2; mantém posição enquanto aguarda primeiro cmd
                        rov_cmd = self._rov_cmd_ros if self._rov_cmd_ros is not None else rov_hold

                        now = time.monotonic()
                        if now - self._last_debug_t >= 1.0:
                            self._last_debug_t = now
                            rov_pos_str = rov_pos.round(2).tolist() if rov_pos is not None else "N/A"
                            rov_cmd_str = [round(v, 2) for v in rov_cmd]
                            ros_str = [round(v, 2) for v in self._rov_cmd_ros] if self._rov_cmd_ros else "None"
                            print(f"[ROV_TOUR] pos={rov_pos_str}  cmd={rov_cmd_str}  ros_cmd={ros_str}")

                        if self._tour_done_flag:
                            rov_cmd = rov_hold
                            self._nav_t0 = time.monotonic()
                            self._transition(S.ASCEND, "tour ROV concluído → drone sobe")
                        elif (self._rov_tour_t0 is not None
                              and time.monotonic() - self._rov_tour_t0 > self.TOUR_TIMEOUT_S):
                            print("[T2] TIMEOUT aguardando /bluerov0/tour_done — abortando.")
                            break

                    # ---------- ASCEND: sobe para fora da água ----------
                    elif self._state == S.ASCEND:
                        z_v = self._z_cmd(pos[2], self._cruise_z) if pos is not None else self._cruise_z
                        drone_cmd = [self._water_xy[0], self._water_xy[1], z_v, 0.0]
                        if self._nav_mode == "AERIAL_NAV":
                            self._transition(S.HOVER_EXIT, f"AERIAL_NAV em pos={pos.round(2) if pos is not None else '?'}")
                        elif self._nav_timeout():
                            print("[T2] TIMEOUT aguardando AERIAL_NAV — abortando.")
                            break

                    # ---------- HOVER_EXIT: paira após sair da água ----------
                    elif self._state == S.HOVER_EXIT:
                        drone_cmd = [self._water_xy[0], self._water_xy[1], self._cruise_z, 0.0]
                        if self._hover_done():
                            self._transition(S.RETURN_HOME, f"hover pós-água concluído ({self.HOVER_SECS}s)")

                    # ---------- RETURN_HOME: volta ao spawn ----------
                    elif self._state == S.RETURN_HOME:
                        target = np.array([self._spawn[0], self._spawn[1], self._cruise_z])
                        drone_cmd = target.tolist() + [0.0]
                        if pos is not None and self._reached(pos, target):
                            self._transition(S.LAND, f"de volta ao home em {pos.round(2)}")

                    # ---------- LAND: desce até o ponto de spawn ----------
                    elif self._state == S.LAND:
                        z_v = self._z_cmd(pos[2], self._spawn[2]) if pos is not None else self._spawn[2]
                        drone_cmd = [self._spawn[0], self._spawn[1], z_v, 0.0]
                        if pos is not None and abs(pos[2] - self._spawn[2]) < self.REACH_Z_TOL:
                            self._state = S.DONE
                            print(f"[T2] Pousou em {pos.round(2)}")
                            print("[T2] Missão T2 CONCLUÍDA com sucesso!")

                    raw = self._env.step({self._agent: drone_cmd, self._rov_agent: rov_cmd})

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
    parser = argparse.ArgumentParser(description="T2 Direct Runner com BlueROV2 — BiguaSim cmd_pos_yaw")
    parser.add_argument("--viewport",   action="store_true")
    parser.add_argument("--ticks",      type=int,   default=100)
    parser.add_argument("--speed",      type=float, default=3.0,
                        help="Multiplicador de ganho de posição XY do drone (default: 3.0)")
    parser.add_argument("--speed-z",    type=float, default=3.0,
                        help="Multiplicador de velocidade vertical do drone (default: 3.0)")
    parser.add_argument("--spawn",      nargs=3,    type=float, default=[8.0, 0.0, 13.4],
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--cruise-z",   type=float, default=28.0,
                        help="Z de cruzeiro no mundo NWU (default: 28.0)")
    parser.add_argument("--water-x",    type=float, default=25.0)
    parser.add_argument("--water-y",    type=float, default=0.0)
    parser.add_argument("--water-z",    type=float, default=0.0,
                        help="Z da superfície — drone paira aqui durante ROV_TOUR (default: 0.0)")
    parser.add_argument("--descent-z",  type=float, default=-2.0,
                        help="Z alvo do drone abaixo da água (default: -2.0)")
    parser.add_argument("--rov-depth",  type=float, default=-0.5,
                        help="Z de operação do ROV — deve ser > floor do mundo (default: -0.5)")
    parser.add_argument("--calibrate",  action="store_true",
                        help="Voa até water-xy e printa posição+depth para calibrar")

    args = parser.parse_args()

    runner = T2DirectRunner(
        spawn=args.spawn,
        cruise_z=args.cruise_z,
        water_xy=[args.water_x, args.water_y],
        water_z=args.water_z,
        descent_z=args.descent_z,
        rov_depth=args.rov_depth,
        speed=args.speed,
        speed_z=args.speed_z,
        ticks=args.ticks,
        show_viewport=args.viewport,
        calibrate=args.calibrate,
    )
    runner.run()


if __name__ == "__main__":
    main()
