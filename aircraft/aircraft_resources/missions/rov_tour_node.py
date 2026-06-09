"""
ROV Tour Node — controla o BlueROV2 via ROS2 durante a fase ROV_TOUR.

Aguarda AQUATIC_NAV, navega pelos waypoints do tour e sinaliza conclusão.

Tópicos consumidos:
  /nav_mode                [std_msgs/String]      — detecta AQUATIC_NAV para iniciar
  /bluerov0/local_position [geometry_msgs/Point]  — posição atual do ROV

Tópicos publicados:
  /bluerov0/cmd_pos_yaw    [geometry_msgs/Point]  — setpoint de posição para o ROV
  /bluerov0/tour_done      [std_msgs/Bool]        — True quando o tour é concluído

Uso:
  python3 rov_tour_node.py [--water-x 25] [--water-y 0] [--descent-z -2] [--radius 8]
  source /opt/ros/jazzy/setup.bash && python3 rov_tour_node.py --radius 6

Teste manual (substitui este nó):
  ros2 topic pub /bluerov0/cmd_pos_yaw geometry_msgs/msg/Point "{x: 33.0, y: 0.0, z: -2.0}"
  ros2 topic pub /bluerov0/tour_done   std_msgs/msg/Bool        "data: true"
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, String


class RovTourNode(Node):

    REACH_TOL = 1.5   # metros — waypoint considerado atingido
    CMD_HZ    = 10.0  # frequência de publicação do setpoint

    def __init__(self, water_xy: list, rov_depth: float, radius: float) -> None:
        super().__init__("rov_tour_node")

        wx, wy = water_xy
        # Tour em L: leste → nordeste → retorno
        self._waypoints: list[np.ndarray] = [
            np.array([wx + radius, wy,           rov_depth]),
            np.array([wx + radius, wy + radius,  rov_depth]),
            np.array([wx,          wy,            rov_depth]),   # retorno ao ponto de entrada
        ]
        self._wp_idx = 0
        self._rov_pos: np.ndarray | None = None
        self._active = False
        self._done = False

        # Publishers
        self._cmd_pub  = self.create_publisher(Point, "/bluerov0/cmd_pos_yaw", 10)
        self._done_pub = self.create_publisher(Bool,  "/bluerov0/tour_done",   10)

        # Subscribers
        self.create_subscription(String, "/nav_mode",                self._nav_cb,  10)
        self.create_subscription(Point,  "/bluerov0/local_position", self._pos_cb,  10)

        self.create_timer(1.0 / self.CMD_HZ, self._tick)

        self.get_logger().info(
            f"ROV Tour Node iniciado.\n"
            f"  rov_depth={rov_depth}  waypoints: {[wp.round(2).tolist() for wp in self._waypoints]}\n"
            f"  Aguardando AQUATIC_NAV em /nav_mode..."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _nav_cb(self, msg: String) -> None:
        if msg.data == "AQUATIC_NAV" and not self._active and not self._done:
            self.get_logger().info("AQUATIC_NAV detectado → iniciando tour do ROV.")
            self._active = True

    def _pos_cb(self, msg: Point) -> None:
        self._rov_pos = np.array([msg.x, msg.y, msg.z])

    # ------------------------------------------------------------------
    # Tick (10 Hz)
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if not self._active or self._done:
            return

        if self._wp_idx >= len(self._waypoints):
            self._done = True
            self._active = False
            done_msg = Bool()
            done_msg.data = True
            self._done_pub.publish(done_msg)
            self.get_logger().info("Tour concluído — /bluerov0/tour_done publicado.")
            return

        wp = self._waypoints[self._wp_idx]

        # Publica setpoint do waypoint atual
        cmd = Point()
        cmd.x, cmd.y, cmd.z = float(wp[0]), float(wp[1]), float(wp[2])
        self._cmd_pub.publish(cmd)

        # Verifica se chegou
        if self._rov_pos is not None:
            dist_xy = float(np.linalg.norm(self._rov_pos[:2] - wp[:2]))
            dist_z  = abs(float(self._rov_pos[2]) - float(wp[2]))
            if dist_xy < self.REACH_TOL and dist_z < self.REACH_TOL:
                self.get_logger().info(
                    f"WP {self._wp_idx + 1}/{len(self._waypoints)} atingido: {wp.round(2)}"
                )
                self._wp_idx += 1


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="ROV Tour Node — controla BlueROV2 via ROS2")
    parser.add_argument("--water-x",   type=float, default=25.0)
    parser.add_argument("--water-y",   type=float, default=0.0)
    parser.add_argument("--rov-depth", type=float, default=-0.5,
                        help="Z de operação do ROV em NWU (negativo = abaixo da superfície, default: -0.5)")
    parser.add_argument("--radius",    type=float, default=8.0,
                        help="Raio do tour em metros (default: 8.0)")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RovTourNode(
        water_xy=[cli_args.water_x, cli_args.water_y],
        rov_depth=cli_args.rov_depth,
        radius=cli_args.radius,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
