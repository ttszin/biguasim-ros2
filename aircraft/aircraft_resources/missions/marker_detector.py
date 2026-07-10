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


class ColorTargetDetector:
    """Detects the largest blob within an HSV color range (default: orange/red)."""

    def __init__(self, hsv_lower=(0, 120, 100), hsv_upper=(15, 255, 255),
                 min_area_px: int = 150, class_id: str = "color_target"):
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.min_area_px = min_area_px
        self.class_id = class_id

    def detect(self, frame) -> list:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_area_px:
            return []

        x, y, w, h = cv2.boundingRect(largest)
        cx = float(x + w / 2.0)
        cy = float(y + h / 2.0)
        frame_area = frame.shape[0] * frame.shape[1]
        confidence = float(min(area / (frame_area * 0.5), 1.0))
        return [(cx, cy, float(w), float(h), self.class_id, confidence)]
