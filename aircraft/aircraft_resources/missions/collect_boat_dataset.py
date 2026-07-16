"""Collects RGBCamera images of the BiguaSim BlueBoat from many random
positions (distance, angle, altitude, yaw) around it, for building a YOLO
training dataset.

No SITL/ArduPilot/mission needed — positions the camera drone directly via
teleport(), the same non-physics technique already validated in
biguasim_sim_runner.py's --flyby/--orbit modes (teleport(location, rotation)
each shot; NOT set_physics_state, which was found live not to actually move
the agent when called repeatedly).

This script collects raw images AND a metadata.jsonl (one line per image:
camera position/yaw, the boat's actual sensor-read position, and camera
intrinsics) — label_boat_dataset.py uses that to compute each image's boat
bounding box geometrically (known 3D poses projected through the pinhole
camera model), instead of needing manual annotation.

Usage:
    python3 collect_boat_dataset.py --num-images 500 --show-camera --viewport
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random

import cv2
import numpy as np

from biguasim.ardubridge import ArduBiguaSimRunner, VehicleProfile
from biguasim.ardubridge.vehicle import VEHICLE_REGISTRY

# Same profile as biguasim_sim_runner.py, minus the depth sensor (not needed
# for image collection — DjiMatrice default is fine).
HYDRONE_HYBRID = VEHICLE_REGISTRY["DjiMatrice"]

# Same resolution/FOV as biguasim_sim_runner.py — matches the DJI Zenmuse
# H20T wide camera's real 82.9 degree diagonal FOV (resolution itself isn't
# the H20T's true 4K; kept at 1280x720, the value already confirmed not to
# lag the simulation — see biguasim_sim_runner.py's own comment history).
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_DFOV_DEG = 82.9

# Same convention as biguasim_sim_runner.py: the boat's fixed world position,
# offset from the camera-agent's own spawn area so they don't overlap.
BOAT_LOCATION = [33.0, 0.0, 0.6]

# Same pinhole approximation as biguasim_sim_runner.py's azimuth/elevation
# math, used in reverse by label_boat_dataset.py to PROJECT the boat's known
# world position into pixel coordinates (instead of converting a detected
# pixel position back to azimuth/elevation).
_DIAG_PIXELS = math.sqrt(CAMERA_WIDTH ** 2 + CAMERA_HEIGHT ** 2)
CAMERA_FX = _DIAG_PIXELS / (2 * math.tan(math.radians(CAMERA_DFOV_DEG) / 2))
CAMERA_FY = CAMERA_FX

DEFAULT_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_raw")

# Max radius, as a fraction of the shot's own altitude, for the boat to stay
# comfortably inside the frame — the camera looks straight down, so a radius
# independent of altitude put the boat outside the frame far too often
# (confirmed live: sampling radius and altitude independently over their
# full ranges left many images with no boat at all). Using
# label_boat_dataset.py's empirically CALIBRATED focal length (706px, not
# the assumed-DFOV one above — see its own comment for why), the boat's
# center reaches the frame's vertical edge at radius/altitude ~= 0.51
# (half_height / fx = 360/706); 0.4 stays under that with margin so the
# boat's own ~1m body, not just its center point, usually stays in frame too.
MAX_RADIUS_ALTITUDE_RATIO = 0.4

# env.agents' real dict key for the boat, confirmed live — NOT the plain
# "blueboat0" agent_name used everywhere else (scenario dict, env.step()'s
# command dict, raw[...] state lookups): environments.py appends "-id{j}"
# per-instance internally (j=0 for the only instance here), and env.agents
# is the one place that shows up, needed to call .teleport() directly on the
# boat's own agent object (env.step()'s command dict doesn't take a
# teleport-style position, only the cmd_pos_yaw control values).
BOAT_AGENT_KEY = "blueboat0-id0"


class BoatDatasetCollector(ArduBiguaSimRunner):
    """Teleports the camera agent to random positions around the boat and
    saves one RGBCamera frame per position — see module docstring.
    """

    def __init__(self, profile: VehicleProfile, scenario: dict,
                 output_dir: str, num_images: int,
                 min_radius: float, max_radius: float,
                 min_altitude: float, max_altitude: float,
                 settle_ticks: int, boat_location: list,
                 show_camera: bool = False,
                 **kwargs) -> None:
        super().__init__(profile, scenario, **kwargs)
        self._output_dir = output_dir
        self._num_images = num_images
        self._min_radius = min_radius
        self._max_radius = max_radius
        self._min_altitude = min_altitude
        self._max_altitude = max_altitude
        self._settle_ticks = settle_ticks
        self._boat_location = boat_location
        self._show_camera = show_camera

    def _hold_boat(self) -> None:
        """Forces the boat back to its fixed spawn position/level rotation
        AND zeroes its velocity/angular velocity every call.

        cmd_pos_yaw (boat_cmd in collect()) never actually controls z —
        confirmed earlier this session by reading usv.Catamaran's source —
        so the boat's height is governed entirely by its own buoyancy physics
        with nothing holding it at the intended surface height.

        set_physics_state, not teleport() (which only takes location/
        rotation, not velocity): confirmed live that teleport() alone still
        let the boat sink over a run (slower than with no correction at all,
        but still drifting down tick by tick) — it resets POSITION each call
        but leaves whatever downward VELOCITY the physics sim built up in
        between calls untouched, so the boat keeps accelerating down between
        teleports even though each teleport itself resets where it starts
        from. Explicitly zeroing velocity too (set_physics_state) fixed it —
        confirmed rock-solid at the exact commanded height across 80
        consecutive ticks in a standalone test. (This is the opposite
        finding from the camera/drone agent, where set_physics_state was
        tried first and found NOT to actually move it when called every
        tick — different vehicle dynamics models, not a contradiction.)
        """
        self._env.agents[BOAT_AGENT_KEY].set_physics_state(
            location=np.array(self._boat_location, dtype=np.float32),
            rotation=np.array([0.0, 0.0, 0.0], dtype=np.float32),
            velocity=np.array([0.0, 0.0, 0.0], dtype=np.float32),
            angular_velocity=np.array([0.0, 0.0, 0.0], dtype=np.float32),
        )

    def collect(self) -> None:
        env = self._env
        agent = self._agent_name
        os.makedirs(self._output_dir, exist_ok=True)
        metadata_path = os.path.join(self._output_dir, "metadata.jsonl")

        motor_cmds = [0.0] * self._profile.num_motors
        boat_cmd = self._boat_location + [0.0]
        self._hold_boat()
        env.step({agent: motor_cmds, "blueboat0": boat_cmd})

        print(f"Collecting {self._num_images} images into '{self._output_dir}'...")
        saved = 0
        with open(metadata_path, "w") as metadata_file:
            for i in range(self._num_images):
                # Altitude first, then cap the radius to a fraction of THIS
                # shot's altitude (see MAX_RADIUS_ALTITUDE_RATIO) — keeps the
                # boat in-frame far more often than sampling radius and
                # altitude independently did. Uniform over a disk (not just
                # uniform radius) within that cap so shots aren't artificially
                # bunched up near the boat — sqrt of a uniform [0,1) sample
                # gives a uniform-area distribution over the annulus between
                # min_radius and the cap.
                altitude = random.uniform(self._min_altitude, self._max_altitude)
                radius_cap = max(self._min_radius, min(self._max_radius, altitude * MAX_RADIUS_ALTITUDE_RATIO))
                frac = random.random()
                radius = math.sqrt(frac * (radius_cap ** 2 - self._min_radius ** 2) + self._min_radius ** 2)
                azimuth = random.uniform(0.0, 2.0 * math.pi)
                yaw = random.uniform(0.0, 360.0)

                x = self._boat_location[0] + radius * math.cos(azimuth)
                y = self._boat_location[1] + radius * math.sin(azimuth)
                z = altitude

                env._agent.teleport(
                    location=np.array([x, y, z], dtype=np.float32),
                    rotation=np.array([0.0, 0.0, yaw], dtype=np.float32),
                )
                # A few settle ticks per shot: not strictly required for the
                # camera (teleport + rotation reset was confirmed live to
                # take effect the very next tick in --flyby/--orbit), but
                # cheap insurance against a half-rendered frame. Re-holds the
                # boat every tick too (see _hold_boat's docstring) — without
                # this every tick, not just once at the start, it keeps
                # sinking.
                for _ in range(self._settle_ticks):
                    self._hold_boat()
                    raw = env.step({agent: motor_cmds, "blueboat0": boat_cmd})

                agent_state = raw[agent][0]
                frame_raw = agent_state.get("RGBCamera")
                if frame_raw is None:
                    print(f"  [{i}] no RGBCamera data, skipping")
                    continue
                # Same BGRA (not RGBA, despite sensors.py's docstring) handling
                # as biguasim_sim_runner.py's _detect_markers — see its comment
                # for the yellow-tint symptom that pinned this down.
                frame_bgr = cv2.cvtColor(np.asarray(frame_raw, dtype=np.uint8), cv2.COLOR_BGRA2BGR)

                filename = f"boat_{i:04d}.png"
                out_path = os.path.join(self._output_dir, filename)
                cv2.imwrite(out_path, frame_bgr)
                saved += 1

                # The boat's ACTUAL position (buoyancy can bob/drift it a bit
                # from its nominal spawn) — label_boat_dataset.py projects
                # this, not the nominal self._boat_location, for accurate
                # boxes.
                boat_state = raw.get("blueboat0", [{}])[0]
                boat_loc = boat_state.get("LocationSensor")
                boat_xyz = ([float(boat_loc[0]), float(boat_loc[1]), float(boat_loc[2])]
                            if boat_loc is not None else list(self._boat_location))

                metadata_file.write(json.dumps({
                    "file": filename,
                    "cam_x": x, "cam_y": y, "cam_z": z, "cam_yaw_deg": yaw,
                    "boat_x": boat_xyz[0], "boat_y": boat_xyz[1], "boat_z": boat_xyz[2],
                    "width": CAMERA_WIDTH, "height": CAMERA_HEIGHT,
                    "fx": CAMERA_FX, "fy": CAMERA_FY,
                }) + "\n")
                metadata_file.flush()

                if self._show_camera:
                    cv2.imshow("collect_boat_dataset", frame_bgr)
                    cv2.waitKey(1)

                if (i + 1) % 25 == 0 or i == self._num_images - 1:
                    print(f"  [{i + 1}/{self._num_images}] r={radius:.1f} az={math.degrees(azimuth):.0f} "
                          f"alt={altitude:.1f} yaw={yaw:.0f} -> {out_path}")

        print(f"Done: {saved}/{self._num_images} images saved to '{self._output_dir}'. "
              f"Metadata: '{metadata_path}'.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect BlueBoat images for YOLO training")
    parser.add_argument("--num-images", type=int, default=500)
    parser.add_argument(
        "--output-dir", default=DEFAULT_OUTPUT_DIR,
        help="Absolute by default (next to this script) — was a bare relative "
             "'dataset_raw' before, which silently wrote wherever the script "
             "happened to be RUN FROM (confirmed live: ended up at the repo "
             "root, not next to this script, when launched from elsewhere).",
    )
    parser.add_argument("--min-radius", type=float, default=1.5, help="Min horizontal distance from the boat (m).")
    parser.add_argument("--max-radius", type=float, default=15.0, help="Max horizontal distance from the boat (m).")
    parser.add_argument("--min-altitude", type=float, default=2.0, help="Min camera altitude (m, BiguaSim world z).")
    parser.add_argument("--max-altitude", type=float, default=15.0, help="Max camera altitude (m, BiguaSim world z).")
    parser.add_argument("--boat-z", type=float, default=0.6, help="Boat spawn height (see biguasim_sim_runner.py's own flag for caveats — spawn only, buoyancy governs it afterward).")
    parser.add_argument("--settle-ticks", type=int, default=3, help="Physics ticks to step after each teleport before capturing.")
    parser.add_argument("--ticks", type=int, default=50, help="Simulation ticks per second.")
    parser.add_argument("--viewport", action="store_true", help="Show Unreal Engine viewport.")
    parser.add_argument("--show-camera", action="store_true", help="Show a live cv2.imshow preview of captured frames.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed, for reproducible shot lists.")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    boat_location = [BOAT_LOCATION[0], BOAT_LOCATION[1], args.boat_z]

    scenario = ArduBiguaSimRunner.build_scenario(
        HYDRONE_HYBRID,
        package_name="SkyDive",
        world="Bridge",
        agent_name="hydrone0",
        location=[boat_location[0], boat_location[1], args.max_altitude],
        rotation=[0.0, 0.0, 0.0],
        ticks_per_sec=args.ticks,
    )
    # Nadir RGBCamera — same rotation as biguasim_sim_runner.py's, confirmed
    # empirically correct there (pitch=+90 -> looking straight down).
    scenario["agents"][0]["sensors"].append({
        "sensor_type": "RGBCamera",
        "socket": "CameraSocket",
        "rotation": [0.0, 90.0, 0.0],
        "configuration": {"CaptureWidth": CAMERA_WIDTH, "CaptureHeight": CAMERA_HEIGHT},
    })

    # Stationary boat, no SITL of its own — same pattern as
    # biguasim_sim_runner.py's --spawn-boat.
    scenario["agents"].append({
        "agent_name": "blueboat0",
        "agent_type": "BlueBoat",
        "control_abstraction": "cmd_pos_yaw",
        "location": boat_location,
        "rotation": [0.0, 0.0, 0.0],
        "dynamics": {"batch_size": 1},
        "sensors": [
            {"sensor_type": "DynamicsSensor", "socket": "COM",
             "configuration": {"UseCOM": True, "UseRPY": False}},
            {"sensor_type": "LocationSensor", "socket": "COM", "configuration": {"Sigma": 0}},
        ],
    })

    with BoatDatasetCollector(
        HYDRONE_HYBRID,
        scenario,
        output_dir=args.output_dir,
        num_images=args.num_images,
        min_radius=args.min_radius,
        max_radius=args.max_radius,
        min_altitude=args.min_altitude,
        max_altitude=args.max_altitude,
        settle_ticks=args.settle_ticks,
        boat_location=boat_location,
        show_camera=args.show_camera,
        show_viewport=args.viewport,
    ) as collector:
        collector.collect()


if __name__ == "__main__":
    main()
