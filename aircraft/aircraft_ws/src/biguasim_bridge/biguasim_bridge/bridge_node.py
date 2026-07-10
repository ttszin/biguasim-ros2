"""BiguaSim T2 bridge node — Humble/ROS2 side of the pressure/nav_mode bridge.

Receives UDP telemetry (pressure, ROV position) from biguasim_sim_runner.py
(a separate, ROS2-free process — BiguaSim requires Python >= 3.11, which
cannot share a process with ROS2 Humble's rclpy, built against Python 3.10).
Republishes that telemetry on the ROS2 graph, runs the AERIAL_NAV/
AQUATIC_NAV hysteresis+debounce state machine, and forwards
/bluerov0/cmd_pos_yaw back to the simulation over UDP.
"""

from __future__ import annotations

import json
import socket
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import String

from biguasim_bridge.encoders import TELEMETRY_ENCODERS


class BiguaSimBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('biguasim_bridge')

        self.declare_parameter('telemetry_port', 9100)
        self.declare_parameter('rov_cmd_port', 9101)
        self.declare_parameter('sim_host', '127.0.0.1')
        self.declare_parameter('pressure_enter_pa', 102500.0)  # ~12 cm depth
        self.declare_parameter('pressure_exit_pa', 101500.0)   # ~1.7 cm depth
        self.declare_parameter('debounce_secs', 0.5)

        self._pressure_enter = self.get_parameter('pressure_enter_pa').value
        self._pressure_exit = self.get_parameter('pressure_exit_pa').value
        self._debounce_secs = self.get_parameter('debounce_secs').value

        self.pressure_pub = self.create_publisher(FluidPressure, '/fcu/external_pressure', 10)
        self.nav_mode_pub = self.create_publisher(String, '/nav_mode', 10)
        self.rov_pos_pub = self.create_publisher(Point, '/bluerov0/local_position', 10)
        self.create_subscription(Point, '/bluerov0/cmd_pos_yaw', self._rov_cmd_cb, 10)

        self._nav_mode = 'AERIAL_NAV'
        self._candidate_mode: str | None = None
        self._candidate_since: float | None = None
        self.nav_mode_pub.publish(String(data=self._nav_mode))

        sim_host = self.get_parameter('sim_host').value
        rov_cmd_port = self.get_parameter('rov_cmd_port').value
        self._rov_cmd_addr = (sim_host, rov_cmd_port)
        self._rov_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        telemetry_port = self.get_parameter('telemetry_port').value
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._telemetry_sock.bind(('0.0.0.0', telemetry_port))
        self._telemetry_sock.setblocking(False)

        self.create_timer(0.005, self._poll_udp)
        self.get_logger().info(
            f'BiguaSim bridge ready -> listening on 0.0.0.0:{telemetry_port}, '
            f'forwarding ROV commands to {self._rov_cmd_addr}'
        )

    def _rov_cmd_cb(self, msg: Point) -> None:
        payload = json.dumps({'x': msg.x, 'y': msg.y, 'z': msg.z}).encode('utf-8')
        self._rov_cmd_sock.sendto(payload, self._rov_cmd_addr)

    def _poll_udp(self) -> None:
        while True:
            try:
                data, _ = self._telemetry_sock.recvfrom(1024)
            except BlockingIOError:
                return
            try:
                payload = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            for key, encoder in TELEMETRY_ENCODERS.items():
                if key in payload:
                    encoder(self, payload)

    def on_pressure(self, pressure: float) -> None:
        """Hysteresis + debounce pressure -> nav_mode, publishes /nav_mode on transitions."""
        now = time.monotonic()

        if self._nav_mode == 'AERIAL_NAV' and pressure > self._pressure_enter:
            target = 'AQUATIC_NAV'
        elif self._nav_mode == 'AQUATIC_NAV' and pressure < self._pressure_exit:
            target = 'AERIAL_NAV'
        else:
            self._candidate_mode = None
            self._candidate_since = None
            return

        if self._candidate_mode != target:
            self._candidate_mode = target
            self._candidate_since = now
            return

        if (now - self._candidate_since) >= self._debounce_secs:
            self._nav_mode = target
            self._candidate_mode = None
            self._candidate_since = None
            self.nav_mode_pub.publish(String(data=self._nav_mode))
            self.get_logger().info(
                f'[nav_mode] transition -> {self._nav_mode} (pressure={pressure:.0f} Pa)'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BiguaSimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
