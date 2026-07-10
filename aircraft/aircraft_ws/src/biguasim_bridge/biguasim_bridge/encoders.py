"""Telemetry payload -> ROS2 message encoders for the BiguaSim T2 bridge.

Small dispatch-by-field-name registry, structurally mirroring the
sensor_type -> encoder registry used by the lab's official biguasim_main
ROS2 package (sensor_data_encode.py), adapted to the reduced UDP telemetry
payload sent by biguasim_sim_runner.py instead of raw BiguaSim sensor data.
"""

from __future__ import annotations

from geometry_msgs.msg import Point
from sensor_msgs.msg import FluidPressure
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)


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


def encode_marker_detections(node, payload: dict) -> None:
    """Republishes biguasim_sim_runner.py's marker/target detections on /detections.

    Same shape as aircraft_ws/src/yolo_py's Detection2DArray convention (azimuth/
    elevation in degrees, packed into results[0].pose.pose.position.x/y) so anything
    written against that format elsewhere can consume this transparently.
    """
    detection_array = Detection2DArray()
    detection_array.header.stamp = node.get_clock().now().to_msg()
    detection_array.header.frame_id = "camera_frame"

    for det in payload["marker_detections"]:
        hypothesis = ObjectHypothesis()
        hypothesis.class_id = str(det["class_id"])
        hypothesis.score = float(det["confidence"])

        result = ObjectHypothesisWithPose()
        result.hypothesis = hypothesis
        result.pose.pose.position.x = float(det["azimuth_deg"])
        result.pose.pose.position.y = float(det["elevation_deg"])

        detection = Detection2D()
        detection.bbox = BoundingBox2D()
        detection.id = hypothesis.class_id
        detection.results.append(result)
        detection_array.detections.append(detection)

    node.detections_pub.publish(detection_array)


# Maps a UDP telemetry payload field to the encoder responsible for it.
# A payload datagram may contain zero or more of these fields per tick.
TELEMETRY_ENCODERS = {
    "pressure_pa": encode_pressure,
    "rov_position": encode_rov_position,
    "marker_detections": encode_marker_detections,
}
