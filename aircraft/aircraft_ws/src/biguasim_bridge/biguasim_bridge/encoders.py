"""Telemetry payload -> ROS2 message encoders for the BiguaSim T2 bridge.

Small dispatch-by-field-name registry, structurally mirroring the
sensor_type -> encoder registry used by the lab's official biguasim_main
ROS2 package (sensor_data_encode.py), adapted to the reduced UDP telemetry
payload sent by biguasim_sim_runner.py instead of raw BiguaSim sensor data.
"""

from __future__ import annotations

from geometry_msgs.msg import Point
from sensor_msgs.msg import FluidPressure


def encode_pressure(node, payload: dict) -> None:
    msg = FluidPressure()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.fluid_pressure = float(payload["pressure_pa"])
    node.pressure_pub.publish(msg)
    node.on_pressure(msg.fluid_pressure)


def encode_rov_position(node, payload: dict) -> None:
    x, y, z = payload["rov_position"]
    msg = Point(x=float(x), y=float(y), z=float(z))
    node.rov_pos_pub.publish(msg)


# Maps a UDP telemetry payload field to the encoder responsible for it.
# A payload datagram may contain zero or more of these fields per tick.
TELEMETRY_ENCODERS = {
    "pressure_pa": encode_pressure,
    "rov_position": encode_rov_position,
}
