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

from marker_detector import ArucoDetector, ShapeTargetDetector

# Perfil padrão do DjiMatrice (motor_mapping correto via sitl-test) com DepthSensor habilitado.
HYDRONE_HYBRID = replace(VEHICLE_REGISTRY["DjiMatrice"], include_depth_sensor=True)

# Cap telemetry UDP sends to ~50 Hz regardless of the (much faster) physics tick rate.
TELEMETRY_HZ = 50.0

# Marker/target detection (OpenCV, on the RGBCamera frame) is throttled separately —
# it's much more expensive per-call than reading a scalar sensor.
DETECTION_HZ = 10.0

# Camera resolution/FOV: FOV matches the DJI Zenmuse H20T's wide camera (the
# gimbal payload this rig is meant to represent) — 82.9 degree diagonal FOV,
# per DJI's published spec, independent of resolution (same physical lens
# angle regardless of how many pixels sample it). Resolution is 1280x720, NOT
# the H20T's true 4K (3840x2160) capture: confirmed live that BiguaSim
# renders this sensor at the full physics tick rate, not just when a
# detection tick actually samples it (10Hz) — native 4K made the whole
# simulation visibly laggy, and 1920x1080 (a quarter of 4K) plus a much lower
# physics tick rate (--ticks 50, down from the 200 default — cuts redundant
# renders 4x on its own, independent of resolution, since the camera was
# rendering ~190 unused frames/sec for every 10 actually read) still wasn't
# enough. 1280x720 is roughly a third of 1920x1080's pixel count, while still
# double the linear resolution of the earlier placeholder (320x240, then
# 640x480, matching sensor_config.yaml's unrelated Gazebo/PX4-pipeline
# default, DFOV=100 — neither tied to any real camera or chosen for this
# reason) that couldn't resolve the helipad marking (clean ellipse fit +
# 4th-harmonic cross signature) beyond a short range.
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_DFOV_DEG = 82.9

# Running the full multi-mask (8 threshold variants) contour-based detection
# pipeline on the full native frame every DETECTION_HZ tick is still more
# than needed — detection instead runs on a downscaled copy (see
# _detect_markers), with the resulting pixel coordinates scaled back up to
# the native resolution before computing azimuth/elevation, so the published
# detections still reflect the real camera's true intrinsics/FOV. 640 is
# exactly half of the native 1280x720 width and matches the original,
# already-tested processing resolution from before any of this — no change
# in detection-side CPU cost from this round, only less native-resolution
# rendering cost.
DETECTION_PROCESS_WIDTH = 640

# JPEG-frame UDP stream to biguasim_bridge, for republishing as a real ROS2 Image topic.
CAMERA_STREAM_HZ = 10.0

# Debug: periodically overwrite these two files with the latest camera frame (raw,
# and with detection overlays) so a real frame can be inspected without a display.
DEBUG_FRAME_SAVE_INTERVAL_SEC = 2.0
DEBUG_FRAME_RAW_PATH = "/tmp/biguasim_camera_debug_raw.png"
DEBUG_FRAME_ANNOTATED_PATH = "/tmp/biguasim_camera_debug_annotated.png"

# Separate from the periodic save above: overwritten every time ANY detection
# fires (true or false positive), un-throttled by DEBUG_FRAME_SAVE_INTERVAL_SEC.
# A false positive is usually a brief, intermittent event that the 2-second
# periodic save has no reason to land on — this exists so an intermittent
# false-positive frame (e.g. a water reflection momentarily read as the
# target) can actually be inspected after the fact instead of only ever
# catching whatever frame happened to be current at a save tick.
DEBUG_FRAME_LAST_DETECTION_PATH = "/tmp/biguasim_camera_debug_last_detection.png"
# Unannotated companion of the above, saved at the exact same tick — the
# annotated one has a green box/label burned into the pixels, which would
# corrupt a contour re-run; this one is the actual re-runnable evidence.
DEBUG_FRAME_LAST_DETECTION_RAW_PATH = "/tmp/biguasim_camera_debug_last_detection_raw.png"

# Saved once, un-throttled, exactly on the frame where detection first drops
# from "detecting" to "nothing" — the moment right after a real target's own
# candidate stops passing the filters (e.g. because a nearby reflection
# disrupted its thresholding/contour), as opposed to the false-positive
# capture above (a wrong detection firing). Different symptom, different
# frame needed to diagnose it.
DEBUG_FRAME_DROPOUT_RAW_PATH = "/tmp/biguasim_camera_debug_dropout_raw.png"

# JSONL, one line per detection tick (~DETECTION_HZ), truncated at the start of
# each run: every tick's outcome (detections found, or empty), for reviewing
# a whole run's detection behavior after the fact instead of only single
# frame snapshots.
DETECTION_LOG_PATH = "/tmp/biguasim_detections_log.jsonl"


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
                 flyby: tuple | None = None,
                 orbit: tuple | None = None,
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
        # (start_x, end_x, y, z, duration_sec) or None. No SITL/ArduPilot is
        # driving the agent's motors in this standalone-detection-testing mode
        # (motor_cmds stays at whatever pwm_to_motor_cmds last produced, which
        # is all-zero with no SITL connected) — same non-physics-driven
        # teleport technique already used once at spawn (see run()), just
        # repeated every tick along an interpolated path instead of a single
        # static point, to sweep the camera across a range of distances/
        # angles to a FIXED target (the boat) in one continuous run instead
        # of needing a separate process restart per --location tested.
        self._flyby = flyby
        # (center_x, center_y, radius, z, period_sec) or None — same
        # non-physics teleport technique as _flyby, but circling at a FIXED
        # altitude around a point instead of a one-way sweep: hovers (never
        # falls — z is forced every tick, not left to gravity with no SITL
        # holding it) while slowly orbiting a target for a continuous visual
        # look from all sides, instead of one straight pass.
        self._orbit = orbit

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

        # Marker/target detection (ArUco + circle+cross helipad marking), run
        # on the RGBCamera frame. A whole-boat silhouette detector
        # (BoatSilhouetteDetector, still defined in marker_detector.py) was
        # tried as a replacement, meant to resolve at longer range than the
        # marking's own fine detail — worked well close, but at longer range
        # a single Otsu split grabbed a huge, wrong region (the water's own
        # large-scale gradient, not the boat) and adaptiveThreshold alone was
        # too noisy from water texture to reliably tell the real boat apart
        # from dozens of spurious candidates. Reverted to the marking
        # detector's already-validated (if shorter-range) behavior.
        self._marker_detector_aruco = ArucoDetector()
        self._marker_detector_shape = ShapeTargetDetector()
        self._last_detection_run = 0.0
        self._show_camera = show_camera
        self._last_debug_frame_save = 0.0
        self._had_detection_last_run = False
        self._last_flyby_print = -999.0
        # "w" (not "a"): truncate at the start of each run, so the log only
        # ever holds the current run's ticks, not old runs appended forever.
        self._detection_log = open(DETECTION_LOG_PATH, "w")
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

        frame_raw = agent_state.get("RGBCamera")
        if frame_raw is None:
            return []
        # sensors.py's docstring claims RGBA channel order, but that produced a
        # strong yellow/warm tint on a water scene (which should read blue) —
        # the classic symptom of a R<->B swap, suggesting the array is actually
        # already BGRA (same doc/code mismatch pattern found earlier with
        # usv.Catamaran's cmd_pos_yaw). Drop alpha without reordering instead
        # of RGBA2BGR's R<->B swap; revert to COLOR_RGBA2BGR if colors end up
        # wrong the other way.
        frame_bgr = cv2.cvtColor(np.asarray(frame_raw, dtype=np.uint8), cv2.COLOR_BGRA2BGR)
        h, w = frame_bgr.shape[:2]
        w_half, h_half = w * 0.5, h * 0.5

        # Detection runs on a downscaled copy (see DETECTION_PROCESS_WIDTH's
        # comment) — far cheaper than the full native 4K frame — then results
        # are scaled back up to native-resolution pixel coordinates so
        # azimuth/elevation (computed below from the native camera_fx/fy) and
        # the debug/stream frames (drawn on the native-resolution frame_bgr)
        # stay consistent with the real camera's actual intrinsics.
        detect_scale = w / float(DETECTION_PROCESS_WIDTH)
        detect_w = DETECTION_PROCESS_WIDTH
        detect_h = max(1, round(h / detect_scale))
        frame_detect = cv2.resize(frame_bgr, (detect_w, detect_h), interpolation=cv2.INTER_AREA)

        self._stream_camera_frame(frame_detect)

        raw_small = self._marker_detector_aruco.detect(frame_detect) + self._marker_detector_shape.detect(frame_detect)
        raw = [(cx * detect_scale, cy * detect_scale, box_w * detect_scale, box_h * detect_scale, class_id, confidence)
               for cx, cy, box_w, box_h, class_id, confidence in raw_small]
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
            # The downscaled frame_detect/raw_small (not the native 3840x2160
            # frame_bgr/raw) — a native-res cv2.imshow window would be far
            # bigger than most screens, and the box/text drawing sizes below
            # are calibrated for a few-hundred-px-wide image.
            self._draw_debug_window(frame_detect, raw_small)

        self._save_debug_frames(frame_bgr, raw)
        self._log_detection_tick(raw)

        return detections

    def _log_detection_tick(self, raw_detections: list) -> None:
        """Appends one JSONL line per detection tick to DETECTION_LOG_PATH —
        what was accepted this tick (possibly nothing), plus *why* nothing was
        accepted when that's the case (no shape-valid candidate this frame vs.
        one that was rejected by ShapeTargetDetector's position-history jump
        filter — see ShapeTargetDetector.last_rejected) — so a full run's
        detection behavior can be reviewed afterward, not just single frames.
        """
        entry = {
            "t": round(time.time(), 3),
            "detections": [
                {"class_id": class_id, "cx": round(cx, 1), "cy": round(cy, 1),
                 "w": round(box_w, 1), "h": round(box_h, 1), "confidence": round(confidence, 3)}
                for cx, cy, box_w, box_h, class_id, confidence in raw_detections
            ],
        }
        rejected = getattr(self._marker_detector_shape, "last_rejected", None)
        if rejected is not None:
            entry["shape_rejected"] = rejected
        self._detection_log.write(json.dumps(entry) + "\n")
        self._detection_log.flush()

    @staticmethod
    def _annotate_detections(frame_bgr, raw_detections: list):
        annotated = frame_bgr.copy()
        for cx, cy, box_w, box_h, class_id, confidence in raw_detections:
            x1, y1 = int(cx - box_w / 2), int(cy - box_h / 2)
            x2, y2 = int(cx + box_w / 2), int(cy + box_h / 2)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{class_id} {confidence:.2f}", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return annotated

    def _save_debug_frames(self, frame_bgr, raw_detections: list) -> None:
        """Periodically overwrites two fixed files with the latest camera frame
        (raw + with detection overlays), so a real frame can be inspected/shared
        without needing a display — this runs regardless of --show-camera.

        Separately (un-throttled, whenever raw_detections is non-empty),
        overwrites DEBUG_FRAME_LAST_DETECTION_PATH — a false positive is
        usually brief and intermittent, easy to miss on the 2-second periodic
        save, so this always holds the most recent detection (true or false)
        for inspection right after it happens.

        Also un-throttled: the instant raw_detections goes from non-empty to
        empty (a real target that WAS being tracked suddenly isn't), the
        current frame is saved to DEBUG_FRAME_DROPOUT_RAW_PATH — a different
        symptom from a false positive (nothing fires at all, instead of the
        wrong thing firing), so it needs the frame from a different moment.
        """
        if raw_detections:
            cv2.imwrite(DEBUG_FRAME_LAST_DETECTION_PATH, self._annotate_detections(frame_bgr, raw_detections))
            cv2.imwrite(DEBUG_FRAME_LAST_DETECTION_RAW_PATH, frame_bgr)
        elif self._had_detection_last_run:
            cv2.imwrite(DEBUG_FRAME_DROPOUT_RAW_PATH, frame_bgr)
        self._had_detection_last_run = bool(raw_detections)

        now = time.monotonic()
        if now - self._last_debug_frame_save < DEBUG_FRAME_SAVE_INTERVAL_SEC:
            return
        self._last_debug_frame_save = now
        cv2.imwrite(DEBUG_FRAME_RAW_PATH, frame_bgr)
        cv2.imwrite(DEBUG_FRAME_ANNOTATED_PATH, self._annotate_detections(frame_bgr, raw_detections))

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

    def _update_flyby(self, sim_time: float) -> None:
        """Moves the agent along a straight line (start_x, y, z) -> (end_x, y,
        z) as sim_time goes from 0 to duration_sec, then holds at the end
        point — sweeps the downward camera across a continuous range of
        distances/angles to a fixed target (e.g. the boat) in one run,
        instead of needing one process restart per fixed --location tested.

        Passes rotation=[0,0,0] to teleport() alongside location every tick
        (not just location, which is all the single static spawn-point
        teleport elsewhere in this file ever needed): teleport() with
        rotation=None leaves rotation as whatever physics last computed,
        which could let the agent tumble between ticks even at this tiny
        per-tick displacement, tilting the nadir-mounted camera away from
        level. Forcing rotation=[0,0,0] every tick keeps the camera level the
        whole sweep.

        NOT using set_physics_state (location+rotation+velocity+angular
        velocity all at once) — tried first, but confirmed live it doesn't
        actually move the agent when called every tick like this (the
        RGBCamera sensor stayed on the same frame as the untouched spawn
        point the whole run, despite the intended interpolated x/y/z being
        computed correctly on the Python side every tick); plain teleport()
        is the proven-reliable API here (already used successfully for the
        one-time spawn placement elsewhere in this file), so stick to it
        even without velocity control.
        """
        start_x, end_x, y, z, duration_sec = self._flyby
        frac = min(max(sim_time / duration_sec, 0.0), 1.0)
        x = start_x + (end_x - start_x) * frac
        self._env._agent.teleport(
            location=np.array([x, y, z], dtype=np.float32),
            rotation=np.array([0.0, 0.0, 0.0], dtype=np.float32),
        )
        # Throttled diagnostic print: makes it immediately obvious in the
        # terminal whether the sweep is actually progressing, without needing
        # a debug frame — confirmed live that a run can look "stuck at spawn"
        # and this is the fastest way to tell whether _update_flyby itself
        # is/isn't moving the agent, vs. some other cause (rendering, camera).
        if sim_time - self._last_flyby_print >= 1.0:
            self._last_flyby_print = sim_time
            print(f"  [flyby] t={sim_time:.1f}s frac={frac:.2f} -> x={x:.1f} y={y:.1f} z={z:.1f}")

    def _update_orbit(self, sim_time: float) -> None:
        """Circles the agent at a FIXED altitude around (center_x, center_y)
        at the given radius, one full lap every period_sec, forever (no
        start/end — unlike _flyby, this doesn't stop). Same teleport(location,
        rotation=[0,0,0]) technique as _flyby (see its docstring for why not
        set_physics_state) — holds altitude every tick instead of leaving z to
        gravity with no SITL/motors actually holding it, so the agent can't
        fall, while slowly circling for a continuous look at a target (e.g.
        the boat) from every angle instead of one straight pass or a single
        static viewpoint.
        """
        center_x, center_y, radius, z, period_sec = self._orbit
        angle = 2.0 * math.pi * (sim_time / period_sec)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        self._env._agent.teleport(
            location=np.array([x, y, z], dtype=np.float32),
            rotation=np.array([0.0, 0.0, 0.0], dtype=np.float32),
        )
        if sim_time - self._last_flyby_print >= 1.0:
            self._last_flyby_print = sim_time
            print(f"  [orbit] t={sim_time:.1f}s angle={math.degrees(angle):.0f} deg -> x={x:.1f} y={y:.1f} z={z:.1f}")

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
            # Synthetic vision_land target for the --landing-target
            # platform/boat paths (spawn_prop only supports basic shapes/
            # materials — no way to apply a custom ArUco texture via the
            # Python API; that would need placing a marker manually in the
            # Unreal Editor). NOT detected by BoatSilhouetteDetector (which
            # looks for the real BlueBoat hull's saturation signature, not a
            # synthetic sphere) — this path matters only for --landing-target
            # platform/boat, not the current --landing-target none --spawn-boat
            # whole-hull testing.
            env.spawn_prop("sphere", location=self._landing_target_location, scale=0.5,
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
                if self._flyby is not None:
                    self._update_flyby(sim_time)
                if self._orbit is not None:
                    self._update_orbit(sim_time)
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
            self._detection_log.close()


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
        help="Where to spawn a synthetic (sphere) vision_land target: on the takeoff "
             "platform (default, matches vision_land_test.yaml), on the BlueBoat out on "
             "the water (matches vision_land_boat_test.yaml), or nowhere (--landing-target "
             "none — e.g. when relying on the BlueBoat's own built-in helipad marking, or "
             "a real ArUco marker placed manually in the Unreal Editor, instead).",
    )
    parser.add_argument(
        "--spawn-boat", action="store_true",
        help="Spawn the stationary BlueBoat out on the water, independent of "
             "--landing-target (the BlueBoat model has its own built-in helipad "
             "marking — no synthetic target needed on top of it). Implied by "
             "--landing-target boat.",
    )
    parser.add_argument(
        "--boat-z", type=float, default=0.2,
        help="BlueBoat SPAWN height only (BiguaSim z, up-positive) — usv.Catamaran's "
             "cmd_pos_yaw control law (see biguasim source, dynamics/usv.py) never "
             "reads/controls the z axis, only x/y/yaw; once simulation starts, "
             "height is governed entirely by real buoyancy physics, not this flag. "
             "NOT yet empirically confirmed: z=0 fully submerged the boat, z=0.5 "
             "spawned it clear of the water and it fell/bounced settling in; 0.2 is "
             "a first guess at splitting the difference to minimize spawn-splash "
             "oscillation. Some initial bobbing before it damps out is expected "
             "regardless of this value — it's real buoyancy physics, not a bug.",
    )
    parser.add_argument(
        "--flyby", action="store_true",
        help="Sweep the agent on a straight-line teleported path (no SITL/mission "
             "needed, same non-physics teleport technique as the single spawn "
             "point below) from (--flyby-start-x, --flyby-y, --flyby-z) to "
             "(--flyby-end-x, --flyby-y, --flyby-z) over --flyby-duration seconds, "
             "then holds at the end point — passes over/near a fixed target (e.g. "
             "--spawn-boat's BOAT_LOCATION, x=33) at a continuous range of "
             "distances/angles in one run, instead of one process restart per "
             "--location tested. Overrides --location's ongoing position (still "
             "used for the initial spawn/teleport before the sweep starts).",
    )
    parser.add_argument(
        "--flyby-start-x", type=float, default=28.0,
        help="5m before the boat (x=33) by default — was 13.0 (20m before), "
             "which took 10 of the 20 default sweep seconds just to reach the "
             "boat's vicinity at all. Closer start reaches it sooner.",
    )
    parser.add_argument("--flyby-end-x", type=float, default=43.0)
    parser.add_argument(
        "--flyby-y", type=float, default=0.0,
        help="Fixed y (BiguaSim NWU) for the whole flyby sweep. 0.0 passes "
             "directly over the boat's own y=0 — straight overhead at the "
             "midpoint, oblique approaching/departing either side. A nonzero "
             "value keeps the boat off to one side the whole time (never nadir).",
    )
    parser.add_argument("--flyby-z", type=float, default=6.0, help="Fixed altitude for the flyby sweep.")
    parser.add_argument("--flyby-duration", type=float, default=20.0, help="Sweep duration in seconds.")
    parser.add_argument(
        "--orbit", action="store_true",
        help="Circle the agent at a FIXED altitude (--orbit-z) around "
             "(--orbit-center-x, --orbit-center-y) at --orbit-radius, one lap "
             "every --orbit-duration seconds, indefinitely (no SITL/mission "
             "needed — same non-physics teleport technique as --flyby, but "
             "circling forever instead of a one-way pass). Holds altitude "
             "every tick, so the agent can't fall the way it would with no "
             "SITL/motors actually holding it — a continuous, hovering "
             "all-around look at a fixed target (e.g. --spawn-boat's "
             "BOAT_LOCATION, x=33) instead of one straight pass. Mutually "
             "exclusive with --flyby in practice (both drive position every "
             "tick; --flyby takes priority if both are set).",
    )
    parser.add_argument("--orbit-center-x", type=float, default=33.0, help="Orbit center x (default: the boat's x).")
    parser.add_argument("--orbit-center-y", type=float, default=0.0, help="Orbit center y (default: the boat's y).")
    parser.add_argument(
        "--orbit-radius", type=float, default=4.0,
        help="Orbit radius in meters. The camera looks straight down (nadir), "
             "not at the orbit center, so the target is only ever in view if "
             "the radius stays within the downward camera's own footprint at "
             "--orbit-z: roughly radius < 0.85 * orbit_z for the H20T's 82.9 "
             "degree DFOV (confirmed live: the default radius=8 at z=6 kept "
             "the boat outside the frame for the whole orbit — 4.0 at the "
             "default z=6 stays comfortably inside that footprint instead).",
    )
    parser.add_argument("--orbit-z", type=float, default=6.0, help="Fixed altitude for the orbit.")
    parser.add_argument("--orbit-duration", type=float, default=30.0, help="Seconds per full lap.")
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
    # BlueBoat out on the water — matches vision_land_boat_test.yaml.
    # x=33 (8m further north than bluerov0's x=25, same y=0) so the two agents
    # don't spawn on top of each other; stays on the already-validated north
    # axis (bsim_x = spawn_x(8) + north) instead of guessing BiguaSim's NWU
    # east/west (y) sign convention. Matching mission waypoint: north=25.
    BOAT_LOCATION = [33.0, 0.0, args.boat_z]  # bluerov0 floats separately at [25.0, 0.0, -0.5]
    # Spawning the boat is decoupled from the synthetic target: the BlueBoat model
    # has its own built-in helipad marking (square deck, circle+cross touchdown
    # mark) — use --spawn-boat --landing-target none to rely on that instead of
    # also placing a sphere on top of it.
    boat_agent = "blueboat0" if (args.spawn_boat or args.landing_target == "boat") else None
    if args.landing_target == "platform":
        landing_target_location = [args.location[0], args.location[1], args.location[2] + 0.1]
    elif args.landing_target == "boat":
        landing_target_location = [BOAT_LOCATION[0], BOAT_LOCATION[1], BOAT_LOCATION[2] + 0.4]  # above the ~0.376m-tall deck
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
        flyby=((args.flyby_start_x, args.flyby_end_x, args.flyby_y, args.flyby_z, args.flyby_duration)
               if args.flyby else None),
        orbit=((args.orbit_center_x, args.orbit_center_y, args.orbit_radius, args.orbit_z, args.orbit_duration)
               if args.orbit else None),
        port=args.port,
        show_viewport=args.viewport,
    ) as runner:
        runner.run()


if __name__ == "__main__":
    main()
