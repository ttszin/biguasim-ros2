"""Classical OpenCV detectors for precision-landing targets.

Both detectors expose the same detect(frame) -> list[(cx, cy, w, h, class_id,
confidence)] interface as yolo_node.py's do_yolo(), in pixel coordinates, so
their output can be merged into the same Detection2DArray published on
/detections.
"""

import cv2
import numpy as np


class ArucoDetector:
    """Detects ArUco fiducial markers, one detection per marker id found."""

    def __init__(self, dictionary_name: str = "DICT_7X7_50"):
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        # OpenCV >=4.7 (main-branch ArUco) has the class-based API; older
        # builds (e.g. Ubuntu 22.04's apt python3-opencv, ~4.5.x) only have
        # the free-function API. Support both so this doesn't silently break
        # depending on which OpenCV ends up installed in the container.
        if hasattr(cv2.aruco, "ArucoDetector"):
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            parameters = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            self._legacy = False
        else:
            self._dictionary = cv2.aruco.Dictionary_get(dictionary_id)
            self._parameters = cv2.aruco.DetectorParameters_create()
            self._legacy = True

    def detect(self, frame) -> list:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._legacy:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._parameters)
        else:
            corners, ids, _ = self._detector.detectMarkers(gray)

        detections = []
        if ids is None:
            return detections
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners.reshape(4, 2)
            x_min, y_min = pts.min(axis=0)
            x_max, y_max = pts.max(axis=0)
            cx = float((x_min + x_max) / 2.0)
            cy = float((y_min + y_max) / 2.0)
            w = float(x_max - x_min)
            h = float(y_max - y_min)
            detections.append((cx, cy, w, h, f"aruco_{int(marker_id)}", 1.0))
        return detections


class ShapeTargetDetector:
    """Detects a circular target by shape, not color — robust to lighting changes.

    Pairs with spawn_prop("sphere", ...) in biguasim_sim_runner.py: viewed from
    a nadir (straight-down) camera, a sphere always projects a circle regardless
    of orientation or scene lighting, unlike a color threshold (which drifts with
    illumination/material shading) or a box (whose silhouette changes with yaw).
    Uses Otsu's threshold (auto-calibrates to the frame's own brightness split
    instead of a fixed color range) + contour circularity, so it keeps working
    whether the sphere renders bright-on-dark or dark-on-bright.
    """

    def __init__(self, min_area_px: int = 150, max_area_fraction: float = 0.5,
                 min_circularity: float = 0.85, class_id: str = "shape_target"):
        # min_circularity=0.85 is deliberately above a square/rectangle's ceiling
        # (4*pi*A/P^2 = pi/4 ~= 0.785) — otherwise background/frame-spanning
        # rectangular contours pass the filter and (being large) win the area-based
        # selection below over the actual, smaller, more-circular target.
        self.min_area_px = min_area_px
        self.max_area_fraction = max_area_fraction
        self.min_circularity = min_circularity
        self.class_id = class_id

    def detect(self, frame) -> list:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        max_area = self.max_area_fraction * frame.shape[0] * frame.shape[1]
        # THRESH_OTSU picks its own cutoff from the frame's histogram (lighting-
        # invariant); try both polarities since we don't know if the sphere is
        # brighter or darker than its background in a given shot.
        _, mask_bright = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, mask_dark = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        best = None
        best_score = 0.0
        for mask in (mask_bright, mask_dark):
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area_px or area > max_area:
                    continue
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0:
                    continue
                circularity = 4 * np.pi * area / (perimeter ** 2)  # 1.0 = perfect circle
                if circularity < self.min_circularity:
                    continue
                if area > best_score:
                    best_score = area
                    best = (contour, circularity)

        if best is None:
            return []

        contour, circularity = best
        x, y, w, h = cv2.boundingRect(contour)
        cx = float(x + w / 2.0)
        cy = float(y + h / 2.0)
        return [(cx, cy, float(w), float(h), self.class_id, float(circularity))]
