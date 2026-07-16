"""Generates YOLO-format bounding-box labels for a dataset collected by
collect_boat_dataset.py, using the known camera/boat 3D poses in its
metadata.jsonl — no visual detection involved, the box is computed by
projecting the boat's known world position (and an assumed real-world
"radius") through the pinhole camera model, using the exact camera pose at
capture time.

The projection formula (rotation convention + axis mapping) was calibrated
against two real captured frames from a manual test batch: measured the
helipad marking's actual pixel centroid, compared against several candidate
sign/axis conventions for how camera yaw rotates the world-frame offset into
image space, and kept the one that matched (see _project_to_pixels).
Residual error was ~15-20% of the offset magnitude in that calibration check
— good enough for training-data boxes (some slack is normal/expected for
object detection), not pixel-exact ground truth.

Usage:
    python3 label_boat_dataset.py --dataset-dir dataset_raw
    python3 label_boat_dataset.py --dataset-dir dataset_raw --preview 10
"""

from __future__ import annotations

import argparse
import json
import math
import os

import cv2

# The boat's real-world footprint is approximated as a circle of this radius
# (not its true elongated catamaran shape — a circle keeps the projection
# simple and orientation-independent) — calibrated by measuring the boat's
# actual apparent pixel width in a real captured frame at a known altitude
# and back-solving for the real-world size that would produce it (~0.85m).
# Was 1.0m (extra slack so the box comfortably contains the boat rather than
# clipping it) — tightened to the measured value directly after visual
# review showed the boxes had visible margin on the sides.
BOAT_RADIUS_M = 0.85

# YOLO class index for the boat. Single-class dataset, so this is always 0;
# kept as a constant in case a second class gets added later.
BOAT_CLASS_ID = 0

# Overrides metadata.jsonl's own stored fx/fy (computed from the ASSUMED DFOV
# of 82.9 degrees, the DJI Zenmuse H20T's published spec — see
# collect_boat_dataset.py). Calibrated instead against 3 real captured
# frames: measured the helipad marking's actual pixel offset from center,
# compared against this script's projection formula run at fx=1, and solved
# for the fx that matches. Got 678-733 across the 3 (x-axis measurements,
# more reliable than y — small y offsets make the same ratio noisy), average
# ~706 — this is CAMERA_DFOV_DEG=82.9's assumed fx (831) overestimating by
# ~18%, i.e. BiguaSim's RGBCamera's real rendered FOV is wider than 82.9
# degrees (a ~92 degree DFOV would produce fx~706). Same doc-vs-actual-
# rendering mismatch pattern found earlier this session elsewhere (BGRA vs
# RGBA channel order, camera rotation sign) — not yet carried over to
# biguasim_sim_runner.py's own azimuth/elevation math, which still assumes
# 82.9 degrees; that's live, already-validated code, changing it needs its
# own separate confirmation before touching it.
CALIBRATED_FX = 706.0
CALIBRATED_FY = 706.0


def _project_to_pixels(cam_x: float, cam_y: float, cam_z: float, cam_yaw_deg: float,
                        boat_x: float, boat_y: float, boat_z: float,
                        fx: float, fy: float, width: int, height: int):
    """Projects the boat's world position to (px, py) pixel coordinates,
    None if the boat is behind/level with the camera (h <= 0).

    Camera looks straight down (nadir) with body roll/pitch always 0 in this
    dataset — only yaw varies. World offset (dx, dy) is rotated by -yaw into
    the camera/body frame, then scaled by focal-length-over-height (the same
    pinhole relationship biguasim_sim_runner.py's azimuth/elevation math
    uses, run in reverse: there, a pixel offset becomes an angle; here, a
    known world offset becomes a pixel offset).
    """
    h = cam_z - boat_z
    if h <= 0.01:
        return None
    dx = boat_x - cam_x
    dy = boat_y - cam_y
    yaw = math.radians(cam_yaw_deg)
    local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
    local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
    # Calibrated axis mapping (see module docstring): image-x <- -local_y,
    # image-y <- -local_x.
    px = width / 2.0 + (-fx * local_y / h)
    py = height / 2.0 + (-fy * local_x / h)
    return px, py, h


def compute_box(entry: dict):
    """Returns (cx, cy, w, h_box) in pixel coordinates for one metadata
    entry, or None if the boat is entirely out of frame.
    """
    width, height = entry["width"], entry["height"]
    # CALIBRATED_FX/FY, not entry["fx"]/entry["fy"] (see CALIBRATED_FX's
    # comment) — the metadata's stored values assumed an uncalibrated DFOV.
    projected = _project_to_pixels(
        entry["cam_x"], entry["cam_y"], entry["cam_z"], entry["cam_yaw_deg"],
        entry["boat_x"], entry["boat_y"], entry["boat_z"],
        CALIBRATED_FX, CALIBRATED_FY, width, height,
    )
    if projected is None:
        return None
    px, py, cam_height = projected
    # The boat's real-world radius projects to a pixel radius the same way
    # any other length at this distance/height does (small-angle pinhole
    # approximation — reasonable here since the boat's own size is small
    # relative to the camera height in all but the closest shots).
    radius_px = CALIBRATED_FX * BOAT_RADIUS_M / cam_height

    x1, y1 = px - radius_px, py - radius_px
    x2, y2 = px + radius_px, py + radius_px
    # Clip to the frame — a boat straddling the edge should still get a
    # (clipped) box; one entirely outside gets none.
    x1c, y1c = max(0.0, x1), max(0.0, y1)
    x2c, y2c = min(float(width), x2), min(float(height), y2)
    if x2c <= x1c or y2c <= y1c:
        return None
    return (x1c + x2c) / 2.0, (y1c + y2c) / 2.0, x2c - x1c, y2c - y1c


def main() -> None:
    parser = argparse.ArgumentParser(description="Geometrically label a collect_boat_dataset.py dataset for YOLO")
    parser.add_argument("--dataset-dir", default=None,
                         help="Defaults to 'dataset_raw' next to this script.")
    parser.add_argument("--preview", type=int, default=0,
                         help="Also save N annotated preview images (dataset_dir/preview/) to visually sanity-check the boxes before trusting the whole dataset.")
    args = parser.parse_args()

    dataset_dir = args.dataset_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_raw")
    metadata_path = os.path.join(dataset_dir, "metadata.jsonl")
    if not os.path.isfile(metadata_path):
        raise SystemExit(f"No metadata.jsonl in '{dataset_dir}' — was this dataset collected with "
                          f"collect_boat_dataset.py (not an older version that didn't save poses)?")

    entries = [json.loads(line) for line in open(metadata_path)]
    preview_dir = os.path.join(dataset_dir, "preview")
    if args.preview > 0:
        os.makedirs(preview_dir, exist_ok=True)

    with_box, empty = 0, 0
    for i, entry in enumerate(entries):
        box = compute_box(entry)
        width, height = entry["width"], entry["height"]
        label_path = os.path.join(dataset_dir, os.path.splitext(entry["file"])[0] + ".txt")

        if box is None:
            # Empty label file: valid YOLO format for a negative/background
            # example (no object of the trained class present) — NOT the
            # same as no label file at all, which most YOLO loaders instead
            # treat as "unlabeled, skip this image".
            open(label_path, "w").close()
            empty += 1
        else:
            cx, cy, box_w, box_h = box
            with open(label_path, "w") as f:
                f.write(f"{BOAT_CLASS_ID} {cx / width:.6f} {cy / height:.6f} "
                        f"{box_w / width:.6f} {box_h / height:.6f}\n")
            with_box += 1

        if args.preview and i < args.preview:
            img_path = os.path.join(dataset_dir, entry["file"])
            img = cv2.imread(img_path)
            if img is not None:
                if box is not None:
                    cx, cy, box_w, box_h = box
                    x1, y1 = int(cx - box_w / 2), int(cy - box_h / 2)
                    x2, y2 = int(cx + box_w / 2), int(cy + box_h / 2)
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.imwrite(os.path.join(preview_dir, entry["file"]), img)

    print(f"Labeled {len(entries)} images: {with_box} with a boat box, {empty} empty (boat out of frame).")
    if args.preview:
        print(f"Preview images (with boxes drawn) saved to '{preview_dir}' — check these before training on the "
              f"full set; the projection is a calibrated approximation, not exact ground truth.")


if __name__ == "__main__":
    main()
