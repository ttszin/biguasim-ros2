"""Video recording of a BiguaSim flight (T8.2): chase camera attached to the vehicle, HUD, persistent trail, H.264 mp4.

Used by stage_b/sitl_runner.py and stage_d/rov_runner.py through `--record out.mp4`. The camera is a BiguaSim RGBCamera on
the vehicle's COM socket (the engine's `move_viewport` did not move the ViewportCapture in this BiguaSim build, so a
body-fixed camera is used; it follows the vehicle's heading and tilt). At `Hz` the state dict carries the frame under the
sensor name; on the other ticks the key is absent, so the cost is paid only on captured ticks (the step rate drops from about
115 to 70 steps/s at 1280x720, 25 Hz; SITL runs lockstep on simulated time, so this only makes the flight take longer).

Camera convention (BiguaSim, checked on frames): rotation [roll, pitch, yaw] in degrees, POSITIVE pitch looks DOWN.

    rec = Recorder("out.mp4", label="A*  K2  B1", ticks_per_sec=250, kind="air", trail_color=(70, 130, 255))
    rec.add_sensor(scenario)                 # before creating the runner
    ...
    rec.frame(env, state_of_agent)           # every tick; writes when a frame arrived
    rec.close()
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:                               # pragma: no cover
    cv2 = None

SENSOR = "Chase"
FPS = 25

# camera offset in the body frame (x forward, y left, z up) and pitch, per kind of vehicle
CAMERAS = {
    "air": dict(location=[-7.0, 0.0, 3.0], rotation=[0.0, 25.0, 0.0]),
    "water": dict(location=[-3.0, 0.0, 0.7], rotation=[0.0, 6.0, 0.0]),
    # high oblique view for scenes with tall walls/blocks: a chase camera 7 m behind the vehicle ends up INSIDE a block when the
    # route bends around it (frames full of texture); 15 m up it stays above obstacles that stand up to 12 m over the home height
    "air_high": dict(location=[-5.0, 0.0, 15.0], rotation=[0.0, 55.0, 0.0], trail_thickness=8.0),
}


def draw_hud(img: np.ndarray, label: str, kind: str, t: float | None, z: float, speed: float, trail_color) -> None:
    """Header bar (label, mission clock, height/depth, speed) and the trail legend, drawn in place on a BGR frame."""
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.rectangle(img, (0, 0), (w, 46), (0, 0, 0), -1)
    cv2.putText(img, label, (14, 31), font, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    vert = f"altura {z:5.1f} m" if kind == "air" else f"profundidade {-z:4.1f} m"
    txt = (f"t {t:5.1f} s   " if t is not None else "") + f"{vert}   vel {speed:4.2f} m/s"
    (tw, _), _ = cv2.getTextSize(txt, font, 0.72, 2)
    cv2.putText(img, txt, (w - tw - 14, 31), font, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.rectangle(img, (14, 56), (44, 70), trail_color, -1)
    cv2.putText(img, "trajetoria voada", (52, 70), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


class Recorder:
    def __init__(self, path: str, label: str, ticks_per_sec: int, kind: str = "air", trail_color=(255, 130, 70),
                 width: int = 1280, height: int = 720, cam: dict | None = None, marker_dir: str | None = None):
        if cv2 is None:
            raise RuntimeError("opencv is needed to record")
        self.path, self.label, self.kind = path, label, kind
        self.ticks_per_sec, self.w, self.h = ticks_per_sec, width, height
        self.cam = dict(cam or CAMERAS[kind])
        self.trail_thickness = float(self.cam.pop("trail_thickness", 3.0))
        self.trail_color = trail_color                       # BGR for the overlay legend, RGB below for the engine
        self.ffmpeg: subprocess.Popen | None = None
        self.n = 0
        self._last_pos: np.ndarray | None = None
        self._t0: float | None = None
        # the executive touches <marker_dir>/started when the planned route begins and <marker_dir>/finished when it ends; the HUD clock
        # counts from `started`, and the frame numbers of both are saved to <mp4>.meta.json so the video can be trimmed
        self.marker_dir = Path(marker_dir) if marker_dir else None
        self.t_start: float | None = None
        self.f_start: int | None = None
        self.f_finish: int | None = None

    def add_sensor(self, scenario: dict) -> None:
        scenario["agents"][0]["sensors"].append({
            "sensor_type": "RGBCamera", "sensor_name": SENSOR, "socket": "COM", "location": list(self.cam["location"]),
            "rotation": list(self.cam["rotation"]), "Hz": FPS,
            "configuration": {"CaptureWidth": self.w, "CaptureHeight": self.h}})

    def _open(self) -> None:
        self.ffmpeg = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{self.w}x{self.h}", "-r", str(FPS),
             "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p",
             "-movflags", "frag_keyframe+empty_moov", self.path], stdin=subprocess.PIPE,
            start_new_session=True)      # own session: the SIGINT that stops the runner must not kill ffmpeg before it flushes

    def _hud(self, img: np.ndarray, t: float | None, z: float, speed: float) -> None:
        draw_hud(img, self.label, self.kind, t, z, speed, self.trail_color)

    def frame(self, env, state: dict, sim_t: float) -> None:
        """Call every tick with the agent state dict: draws the trail (every tick with a frame) and writes the frame."""
        img = state.get(SENSOR)
        if img is None:
            return
        loc = np.asarray(state["LocationSensor"], dtype=float)[:3]
        vel = np.asarray(state["VelocitySensor"], dtype=float)[:3]
        if self._t0 is None:
            self._t0 = sim_t
        if self._last_pos is not None and np.linalg.norm(loc - self._last_pos) > 0.02:
            rgb = [int(self.trail_color[2]), int(self.trail_color[1]), int(self.trail_color[0])]
            env.draw_line(self._last_pos.tolist(), loc.tolist(), color=rgb, thickness=self.trail_thickness, lifetime=0)
        if self._last_pos is None or np.linalg.norm(loc - self._last_pos) > 0.02:
            self._last_pos = loc
        frame = np.ascontiguousarray(np.asarray(img)[:, :, :3])           # BiguaSim returns BGRA (cv2 wrote correct colours)
        if self.marker_dir is not None:
            if self.t_start is None and (self.marker_dir / "started").exists():
                self.t_start, self.f_start = sim_t, self.n
            if self.f_finish is None and self.t_start is not None and (self.marker_dir / "finished").exists():
                self.f_finish = self.n
        self._hud(frame, (sim_t - self.t_start) if self.t_start is not None else None, float(loc[2]), float(np.linalg.norm(vel)))
        if self.ffmpeg is None:
            self._open()
        self.ffmpeg.stdin.write(frame.tobytes())
        self.n += 1
        if self.n % 100 == 0 or self.f_start == self.n - 1 or self.f_finish == self.n - 1:      # the runner is usually killed, not closed
            self._write_meta()

    def _write_meta(self) -> None:
        Path(self.path + ".meta.json").write_text(json.dumps({"fps": FPS, "frames": self.n, "start_frame": self.f_start,
                                                              "finish_frame": self.f_finish}))

    def close(self) -> None:
        if self.ffmpeg is not None:
            try:
                self.ffmpeg.stdin.close()
                self.ffmpeg.wait(timeout=60)
            except Exception:                                              # noqa: BLE001
                self.ffmpeg.kill()
            self.ffmpeg = None
        Path(self.path + ".meta.json").write_text(json.dumps({"fps": FPS, "frames": self.n, "start_frame": self.f_start,
                                                              "finish_frame": self.f_finish}))
