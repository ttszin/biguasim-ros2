"""
BiguaSim simulation runner for T2 hybrid transition validation.

Replaces: Gazebo simulation + hydro_sensor_bridge.py

Runs the SkyDive/Bridge world (real water physics), reads the DepthSensor,
and streams pressure/ROV telemetry over UDP to the biguasim_bridge ROS2
node (Humble), which republishes /fcu/external_pressure, /nav_mode, and
/bluerov0/local_position, and mirrors ROV waypoint commands back here.

This script has NO ROS2 dependency by design: BiguaSim requires Python
>= 3.11, while ROS2 Humble's rclpy is built against Python 3.10 (Ubuntu
22.04) and cannot be imported in the same process. See aircraft_ws/src/
biguasim_bridge for the ROS2 side of this bridge.

Start order:
  1. bash t2_sitl_run.sh                                 (ArduCopter SITL)
  2. python3 biguasim_sim_runner.py [--viewport]          (this script)
  3. docker run --network host \
       -v $(pwd)/aircraft/t2_aircraft.yml.erb:/aas/t2_aircraft.yml.erb \
       --entrypoint bash aircraft-image \
       -c "tmuxinator start -p /aas/t2_aircraft.yml.erb"  (MAVROS +
     autopilot_interface + mission + biguasim_bridge, all ROS2 Humble —
     see aircraft/t2_aircraft.yml.erb)

ArduPilot receives pressure through the JSON SITL state (handled by
ArduBiguaSimRunner). The biguasim_bridge ROS2 node receives the same
pressure over UDP on --telemetry-port.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time

import cv2
import numpy as np

from dataclasses import replace

from biguasim.ardubridge import ArduBiguaSimRunner, VehicleProfile
from biguasim.ardubridge.frame import depth_to_pressure
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

from marker_detector import ArucoDetector, ColorTargetDetector

# Perfil padrão do DjiMatrice (motor_mapping correto via sitl-test) com DepthSensor habilitado.
HYDRONE_HYBRID = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)

# Cap telemetry UDP sends to ~50 Hz regardless of the (much faster) physics tick rate.
TELEMETRY_HZ = 50.0

# Marker/target detection (OpenCV, on the RGBCamera frame) is throttled separately —
# it's much more expensive per-call than reading a scalar sensor.
DETECTION_HZ = 10.0

# Camera resolution — kept low to bound OpenCV cost per detection tick on CPU.
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_DFOV_DEG = 100.0  # BiguaSim's RGBCamera exposes no intrinsics; assumed like sensor_config.yaml's default.

# JPEG-frame UDP stream to biguasim_bridge, for republishing as a real ROS2 Image topic.
CAMERA_STREAM_HZ = 10.0


class BiguaSimT2Runner(ArduBiguaSimRunner):
    """ArduBiguaSimRunner + UDP telemetry for the biguasim_bridge ROS2 node.

    The parent class already converts DepthSensor → pressure and sends it to
    ArduPilot via JSON SITL. This subclass taps into the same agent_state to
    forward pressure/ROV-position telemetry over UDP to a separate ROS2
    (Humble) process, and receives ROV waypoint commands back the same way.
    """

    def __init__(self, profile: VehicleProfile, scenario: dict,
                 spawn_location: list | None = None,
                 rov_agent: str = "bluerov0",
                 rov_hold: list | None = None,
                 bridge_host: str = "127.0.0.1",
                 telemetry_port: int = 9100,
                 rov_cmd_port: int = 9101,
                 landing_target_location: list | None = None,
                 boat_agent: str | None = None,
                 boat_hold: list | None = None,
                 show_camera: bool = False,
                 camera_stream_port: int = 9103,
                 **kwargs) -> None:
        super().__init__(profile, scenario, **kwargs)
        self._spawn_location = spawn_location
        self._rov_agent_name = rov_agent
        self._rov_hold = (rov_hold or [25.0, 0.0, -0.5]) + [0.0]
        self._rov_cmd_ros: list | None = None
        self._landing_target_location = landing_target_location
        # Stationary BlueBoat landing platform (no ArduPilot/SITL of its own — driven
        # directly via cmd_pos_yaw, same non-SITL pattern as the bluerov0 agent).
        self._boat_agent_name = boat_agent
        self._boat_hold = (boat_hold + [0.0]) if boat_hold is not None else None

        self._telemetry_addr = (bridge_host, telemetry_port)
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # JPEG-compressed RGBCamera frames, sent to biguasim_bridge for republishing
        # as a real sensor_msgs/Image on /biguasim/camera/image (e.g. rqt_image_view).
        # Separate UDP channel from telemetry — the JSON socket only ever carries
        # small text payloads, an image (even compressed) is a different animal.
        self._camera_stream_addr = (bridge_host, camera_stream_port)
        self._camera_stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._last_camera_stream_send = 0.0

        self._rov_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rov_cmd_sock.bind(("0.0.0.0", rov_cmd_port))
        self._rov_cmd_thread = threading.Thread(target=self._rov_cmd_loop, daemon=True)
        self._rov_cmd_thread.start()

        self._last_telemetry_send = 0.0

        # Marker/target detection (ArUco + color blob), run on the RGBCamera frame.
        self._marker_detector_aruco = ArucoDetector()
        self._marker_detector_color = ColorTargetDetector()
        self._last_detection_run = 0.0
        self._show_camera = show_camera
        # Pinhole approximation (same formula as yolo_node.py) from the assumed DFOV,
        # since BiguaSim's RGBCamera exposes no real camera-matrix calibration.
        diag_pixels = math.sqrt(CAMERA_WIDTH ** 2 + CAMERA_HEIGHT ** 2)
        self._camera_fx = diag_pixels / (2 * math.tan(math.radians(CAMERA_DFOV_DEG) / 2))
        self._camera_fy = self._camera_fx

        print(
            f"BiguaSim T2 bridge ready -> UDP telemetry to {self._telemetry_addr}, "
            f"ROV cmd listening on 0.0.0.0:{rov_cmd_port}"
        )

    def _rov_cmd_loop(self) -> None:
        while True:
            try:
                data, _ = self._rov_cmd_sock.recvfrom(1024)
            except OSError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
                self._rov_cmd_ros = [payload["x"], payload["y"], payload["z"], 0.0]
            except (ValueError, KeyError):
                continue

    def _send_telemetry(self, pressure: float | None, rov_pos: list | None, sim_time: float,
                         marker_detections: list | None = None) -> None:
        now = time.monotonic()
        if now - self._last_telemetry_send < (1.0 / TELEMETRY_HZ):
            return
        self._last_telemetry_send = now

        payload: dict = {"stamp": sim_time}
        if pressure is not None:
            payload["pressure_pa"] = pressure
        if rov_pos is not None:
            payload["rov_position"] = rov_pos
        if marker_detections:
            payload["marker_detections"] = marker_detections

        self._telemetry_sock.sendto(json.dumps(payload).encode("utf-8"), self._telemetry_addr)

    def _detect_markers(self, agent_state: dict) -> list:
        """Runs ArUco + color-blob detection on the agent's RGBCamera frame.

        Throttled independently of telemetry (much more expensive per call).
        Returns a list of {class_id, azimuth_deg, elevation_deg, confidence} dicts,
        in the same azimuth/elevation convention used by yolo_node.py (pinhole
        approximation from a diagonal FOV — BiguaSim's RGBCamera has no exposed
        intrinsics/calibration, so this stays an approximation too).
        """
        now = time.monotonic()
        if now - self._last_detection_run < (1.0 / DETECTION_HZ):
            return []
        self._last_detection_run = now

        frame_rgba = agent_state.get("RGBCamera")
        if frame_rgba is None:
            return []
        frame_bgr = cv2.cvtColor(np.asarray(frame_rgba, dtype=np.uint8), cv2.COLOR_RGBA2BGR)
        h, w = frame_bgr.shape[:2]
        w_half, h_half = w * 0.5, h * 0.5

        self._stream_camera_frame(frame_bgr)

        raw = self._marker_detector_aruco.detect(frame_bgr) + self._marker_detector_color.detect(frame_bgr)
        detections = []
        for cx, cy, box_w, box_h, class_id, confidence in raw:
            dx = cx - w_half
            dy = h_half - cy
            azimuth_deg = math.degrees(math.atan(dx / self._camera_fx))
            elevation_deg = math.degrees(math.atan(dy / self._camera_fy))
            detections.append({
                "class_id": class_id,
                "azimuth_deg": azimuth_deg,
                "elevation_deg": elevation_deg,
                "confidence": confidence,
            })

        if self._show_camera:
            self._draw_debug_window(frame_bgr, raw)

        return detections

    def _stream_camera_frame(self, frame_bgr) -> None:
        """Sends a JPEG-compressed frame to biguasim_bridge for republishing as
        sensor_msgs/Image on /biguasim/camera/image (e.g. for rqt_image_view).
        Throttled to CAMERA_STREAM_HZ, independent of the detection cadence.
        """
        now = time.monotonic()
        if now - self._last_camera_stream_send < (1.0 / CAMERA_STREAM_HZ):
            return
        self._last_camera_stream_send = now

        ok, jpeg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        self._camera_stream_sock.sendto(jpeg.tobytes(), self._camera_stream_addr)

    def _draw_debug_window(self, frame_bgr, raw_detections: list) -> None:
        """Live cv2.imshow of the downward RGBCamera feed with detection overlays.

        Runs in the same host process (no ROS2/cv_bridge/Image-topic round trip
        needed, unlike the old biguasim_bridge_runner.py's /biguasim/camera/image).
        """
        display = frame_bgr.copy()
        for cx, cy, box_w, box_h, class_id, confidence in raw_detections:
            x1, y1 = int(cx - box_w / 2), int(cy - box_h / 2)
            x2, y2 = int(cx + box_w / 2), int(cy + box_h / 2)
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(display, f"{class_id} {confidence:.2f}", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        h, w = display.shape[:2]
        cv2.drawMarker(display, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 12, 1)
        cv2.imshow("BiguaSim RGBCamera (vision_land)", display)
        cv2.waitKey(1)

    def _step_cmds(self, motor_cmds: list, rov_cmd: list) -> dict:
        cmds = {self._agent_name: motor_cmds, self._rov_agent_name: rov_cmd}
        if self._boat_agent_name is not None:
            cmds[self._boat_agent_name] = self._boat_hold
        return cmds

    def run(self) -> None:
        bridge = self._bridge
        env = self._env
        agent = self._agent_name
        dt = self._dt

        rov = self._rov_agent_name

        bridge.bind()
        motor_cmds = [0.0] * self._profile.num_motors
        rov_cmd = self._rov_hold
        env.step(self._step_cmds(motor_cmds, rov_cmd))

        if self._landing_target_location is not None:
            # Testable-today color target for vision_land (spawn_prop only supports
            # basic shapes/materials — no way to apply a custom ArUco texture via
            # the Python API; that would need placing a marker manually in the
            # Unreal Editor). "gold" gives strong contrast against the platform.
            env.spawn_prop("box", location=self._landing_target_location, scale=0.5,
                            material="gold", tag="landing_target")

        if self._spawn_location is not None:
            env._agent.teleport(location=np.array(self._spawn_location, dtype=np.float32))
            env.step(self._step_cmds(motor_cmds, rov_cmd))

        raw = env.step(self._step_cmds(motor_cmds, rov_cmd))
        agent_state = raw[agent][0]
        sim_time = 0.0

        print(f"Running BiguaSim T2 bridge on '{agent}' + ROV '{rov}' (Ctrl-C to stop)...")
        try:
            while True:
                frame, pwm = bridge.receive_pwm()
                if frame is not None:
                    motor_cmds = bridge.pwm_to_motor_cmds(pwm, frame)

                rov_cmd = self._rov_cmd_ros if self._rov_cmd_ros is not None else self._rov_hold
                raw = env.step(self._step_cmds(motor_cmds, rov_cmd))
                agent_state = raw[agent][0]
                rov_state = raw[rov][0]
                sim_time += dt

                json_state = bridge.build_json_state(agent_state, sim_time)
                bridge.send_state(json_state)

                if json_state is not None and frame is not None and frame % 400 == 0:
                    pos = json_state["position"]
                    # pos: NED (north, east, down) relative to GPS origin
                    # BiguaSim x ≈ spawn_x + pos[0] (NED north)
                    # BiguaSim z ≈ spawn_z - pos[2] (NED down → up)
                    spawn_x = (self._spawn_location or [8.0])[0]
                    spawn_z = (self._spawn_location or [0.0, 0.0, 13.4])[2]
                    bsim_x = spawn_x + pos[0]
                    bsim_z = spawn_z - pos[2]
                    print(
                        f"  t={sim_time:.1f}s  NED=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})"
                        f"  bsim_x={bsim_x:.1f}  bsim_z={bsim_z:.1f}"
                        f"  motors={[f'{m:.0f}' for m in motor_cmds]}"
                    )

                pressure = None
                if "DepthSensor" in agent_state:
                    depth_val = agent_state["DepthSensor"]
                    z_up = float(depth_val[0]) if hasattr(depth_val, "__len__") else float(depth_val)
                    pressure = depth_to_pressure(z_up)

                rov_pos = None
                rov_loc = rov_state.get("LocationSensor")
                if rov_loc is not None:
                    rov_pos = [float(rov_loc[0]), float(rov_loc[1]), float(rov_loc[2])]

                marker_detections = self._detect_markers(agent_state)

                self._send_telemetry(pressure, rov_pos, sim_time, marker_detections)

        except KeyboardInterrupt:
            print("Bridge stopped.")
        finally:
            bridge.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="BiguaSim T2 Simulation Runner")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport")
    parser.add_argument("--port", type=int, default=9002, help="ArduPilot SITL UDP port")
    parser.add_argument("--ticks", type=int, default=200, help="Simulation ticks per second")
    parser.add_argument(
        "--location", nargs=3, type=float, default=[8.0, 0.0, 13.4],
        metavar=("X", "Y", "Z"),
        help="Agent start location in Biguasim NWU metres (default: 8 0 13.4)",
    )
    parser.add_argument(
        "--bridge-host", default="127.0.0.1",
        help="Host running the biguasim_bridge ROS2 node",
    )
    parser.add_argument(
        "--telemetry-port", type=int, default=9100,
        help="UDP port to send pressure/ROV telemetry to",
    )
    parser.add_argument(
        "--rov-cmd-port", type=int, default=9101,
        help="UDP port to listen for ROV waypoint commands on",
    )
    parser.add_argument(
        "--camera-stream-port", type=int, default=9103,
        help="UDP port to send JPEG-compressed RGBCamera frames to, for "
             "biguasim_bridge to republish as /biguasim/camera/image.",
    )
    parser.add_argument(
        "--show-camera", action="store_true",
        help="Open a cv2.imshow window with the downward RGBCamera feed and "
             "detection overlays (ArUco/color-target bounding boxes + image center).",
    )
    parser.add_argument(
        "--landing-target", choices=["platform", "boat", "none"], default="platform",
        help="Where to spawn the vision_land color target: on the takeoff platform "
             "(default, matches vision_land_test.yaml), on a stationary BlueBoat out "
             "on the water (matches vision_land_boat_test.yaml), or nowhere (--landing-"
             "target none, e.g. when a real ArUco marker was placed manually in the "
             "Unreal Editor instead).",
    )
    args = parser.parse_args()

    from biguasim.dynamics.agents import BlueROV2
    BlueROV2._params["kp_pos"] = 0.25

    rov_location = [25.0, 0.0, -0.5]

    scenario = ArduBiguaSimRunner.build_scenario(
        HYDRONE_HYBRID,
        package_name="SkyDive",
        world="Bridge",
        agent_name="hydrone0",
        location=args.location,
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )

    # RGBCamera for marker/target detection (vision_land). VehicleProfile has no
    # generic "extra sensors" hook (unlike include_depth_sensor), so append directly
    # to the built scenario dict, same as the bluerov0 agent's sensors below.
    # rotation points the camera straight down (nadir) so image dx/dy map to a
    # horizontal (east/north-ish) offset to the target below, not azimuth/elevation
    # from a forward-facing camera — needed for vision_land's centering math to make
    # sense. pitch=+90 -> looking down; confirmed empirically in an earlier T2
    # iteration (see git history, biguasim_bridge_runner.py @ 49eb37b).
    scenario["agents"][0]["sensors"].append({
        "sensor_type": "RGBCamera",
        "socket": "CameraSocket",
        "rotation": [0.0, 90.0, 0.0],
        "configuration": {"CaptureWidth": CAMERA_WIDTH, "CaptureHeight": CAMERA_HEIGHT},
    })

    # Landing target for vision_land. "platform": on top of the takeoff/landing
    # platform, i.e. right at the agent's spawn location (small z offset to sit
    # above the deck) — matches vision_land_test.yaml. "boat": on a stationary
    # BlueBoat out on the water — matches vision_land_boat_test.yaml. Reuses the
    # exact [25.0, 0.0, ...] location already validated by t2_land_test.yaml's
    # north=17/east=0 waypoint (bsim_x = spawn_x(8) + 17 = 25) and by bluerov0's
    # own spawn — avoids guessing BiguaSim's NWU east/west (y) sign convention.
    BOAT_LOCATION = [25.0, 0.0, 0.0]  # surface (z=0); bluerov0 floats at z=-0.5, half a metre below
    boat_agent = None
    if args.landing_target == "platform":
        landing_target_location = [args.location[0], args.location[1], args.location[2] + 0.1]
    elif args.landing_target == "boat":
        landing_target_location = [BOAT_LOCATION[0], BOAT_LOCATION[1], BOAT_LOCATION[2] + 0.4]  # above the ~0.376m-tall deck
        boat_agent = "blueboat0"
    else:
        landing_target_location = None

    scenario["agents"].append({
        "agent_name": "bluerov0",
        "agent_type": "BlueROV2",
        "control_abstraction": "cmd_pos_yaw",
        "location": rov_location,
        "rotation": [0.0, 0.0, 0.0],
        "dynamics": {"batch_size": 1},
        "sensors": [
            {"sensor_type": "DynamicsSensor", "socket": "COM",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "LocationSensor", "socket": "COM", "configuration": {"Sigma": 0}},
        ],
    })

    if boat_agent is not None:
        # Stationary landing platform: no ArduPilot/Rover SITL of its own, driven
        # directly via cmd_pos_yaw (same non-SITL pattern as bluerov0) and held at
        # a fixed position/yaw the whole run — no patrol/motion logic in this round.
        scenario["agents"].append({
            "agent_name": boat_agent,
            "agent_type": "BlueBoat",
            "control_abstraction": "cmd_pos_yaw",
            "location": BOAT_LOCATION,
            "rotation": [0.0, 0.0, 0.0],
            "dynamics": {"batch_size": 1},
            "sensors": [
                {"sensor_type": "DynamicsSensor", "socket": "COM",
                 "configuration": {"UseCOM": True, "UseRPY": False}},
                {"sensor_type": "LocationSensor", "socket": "COM", "configuration": {"Sigma": 0}},
            ],
        })

    with BiguaSimT2Runner(
        HYDRONE_HYBRID,
        scenario,
        spawn_location=args.location,
        rov_agent="bluerov0",
        rov_hold=rov_location,
        bridge_host=args.bridge_host,
        telemetry_port=args.telemetry_port,
        rov_cmd_port=args.rov_cmd_port,
        landing_target_location=landing_target_location,
        boat_agent=boat_agent,
        boat_hold=BOAT_LOCATION if boat_agent is not None else None,
        show_camera=args.show_camera,
        camera_stream_port=args.camera_stream_port,
        port=args.port,
        show_viewport=args.viewport,
    ) as runner:
        runner.run()


if __name__ == "__main__":
    main()
