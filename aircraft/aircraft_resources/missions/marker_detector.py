"""YOLOv8-based detector for precision-landing and tracking targets.

Exposes the same detect(frame) -> list[(cx, cy, w, h, class_id, confidence)]
interface to seamlessly integrate with biguasim_sim_runner.py.
"""

import os

import cv2
import numpy as np
from ultralytics import YOLO

# Weights from training our own single-class ("blueboat") YOLOv8n model on
# collect_boat_dataset.py + label_boat_dataset.py's geometrically-labeled
# dataset (500 images, mAP50=0.995, mAP50-95=0.913 on the held-out val
# split) — NOT the plain COCO-pretrained yolov8n.pt used earlier, which
# missed the boat entirely in several real test frames (its "boat" class was
# trained on eye-level/ground photos, not this aerial top-down view).
# Absolute path (not a bare relative "runs/..."): a bare relative default
# broke collect_boat_dataset.py's own dataset dir once already this session
# by resolving against whatever directory the script happened to be RUN
# FROM instead of where this file lives.
DEFAULT_WEIGHTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "runs", "detect", "blueboat_detector", "weights", "best.pt",
)


class ArucoDetector:
    """Detects ArUco fiducial markers, one detection per marker id found."""

    def __init__(self, dictionary_name: str = "DICT_7X7_50"):
        dictionary_id = getattr(cv2.aruco, dictionary_name)
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
    """Detects the BlueBoat using our own fine-tuned YOLOv8n model (trained
    on collect_boat_dataset.py + label_boat_dataset.py's geometrically
    -labeled dataset — see DEFAULT_WEIGHTS_PATH), not the plain COCO
    -pretrained one: the COCO "boat" class (trained on eye-level/ground
    photos) missed this small twin-hull robot from directly overhead
    entirely in several real test frames, even at conf_threshold as low as
    0.01 — confirmed dead on arrival for this aerial top-down use case.

    A seamless drop-in replacement for the earlier classic shape detector —
    keeps its exact same history/jump/size-ratio filtering and confidence
    -decay prediction logic (see _pick_candidate/_predict_detection) — so
    biguasim_sim_runner.py's logging and consumption code didn't need to
    change at all.
    """

    def __init__(self, weights_path: str = DEFAULT_WEIGHTS_PATH, conf_threshold: float = 0.25,
                 class_id: str = "shape_target"):
        self.model = YOLO(weights_path)
        self.conf_threshold = conf_threshold
        self.class_id = class_id

        # Our own dataset is single-class ("blueboat" = index 0 in
        # dataset_yolo/data.yaml) — NOT COCO's 80-class index 8. Using the
        # COCO index here would silently match nothing (or the wrong class,
        # if ever run against a COCO-pretrained checkpoint by mistake).
        self.BOAT_CLASS_ID = 0

        self.last_rejected = 0
        self._frame_index = 0
        self._real_pos_b = None  
        self._last_real_size = None  
        self._last_position = None
        self._frames_since_last_detection = 0
        self._confirmed = False
        self._confirm_streak = 0
        self._tentative_position = None
        
        # Parâmetros herdados do seu orquestrador original para filtros de histórico (se o runner usar)
        self.max_jump_px = 120.0
        self.max_history_gap_frames = 20
        self.min_confirm_streak = 2
        self.max_size_ratio = 2.5
        self._last_confirmed_size = None

        print(f"[INFO] YOLOv8 Pretrained Detector inicializado com sucesso (Pesos: {weights_path}).")

    def detect(self, frame) -> list:
        self.last_rejected = 0
        self._frame_index += 1
        
        # Roda a predição na imagem (com verbose=False para não sujar o terminal do ROS)
        results = self.model(frame, verbose=False)[0]
        
        candidates = []
        
        # Varre as caixas delimitadoras encontradas pela rede neural
        for box in results.boxes:
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            
            # Filtra apenas detecções da classe barco acima do limite de confiança
            if cls == self.BOAT_CLASS_ID and conf >= self.conf_threshold:
                # Converte coordenadas da caixa (xyxy) para o formato do runner (cx, cy, w, h)
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                w = float(x2 - x1)
                h = float(y2 - y1)
                cx = float(x1 + w / 2.0)
                cy = float(y1 + h / 2.0)
                
                # Armazena usando o score de confiança para ordenação do pick_candidate
                candidates.append((conf * 100.0, int(x1), int(y1), int(w), int(h)))
            else:
                self.last_rejected += 1

        # Utiliza o seu gerenciador de histórico robusto original para escolher o candidato correto
        chosen = self._pick_candidate(candidates)

        if chosen is not None:
            best_score, x, y, w, h = chosen
            cx = float(x + w / 2.0)
            cy = float(y + h / 2.0)
            self._real_pos_b = (self._frame_index, cx, cy)
            self._last_real_size = (float(w), float(h))
            confidence = float(best_score / 100.0)
            return [(cx, cy, float(w), float(h), self.class_id, confidence)]

        return self._predict_detection()

    def _predict_detection(self) -> list:
        if self._real_pos_b is None or self._frames_since_last_detection > self.max_history_gap_frames:
            return []
        _, cx, cy = self._real_pos_b
        w, h = self._last_real_size
        confidence = max(0.05, 0.3 - 0.03 * self._frames_since_last_detection)
        return [(cx, cy, w, h, self.class_id, confidence)]

    def _pick_candidate(self, candidates: list):
        history_fresh = (self._last_position is not None
                          and self._frames_since_last_detection <= self.max_history_gap_frames)

        if not history_fresh:
            self._confirmed = False
            self._last_confirmed_size = None

        if self._confirmed:
            chosen = None
            if candidates:
                lx, ly = self._last_position
                nearest = min(candidates, key=lambda c: (c[1] + c[3] / 2.0 - lx) ** 2 + (c[2] + c[4] / 2.0 - ly) ** 2)
                _, x, y, w, h = nearest
                dist = ((x + w / 2.0 - lx) ** 2 + (y + h / 2.0 - ly) ** 2) ** 0.5
                if dist > self.max_jump_px:
                    self.last_rejected = {
                        "reason": "jump", "distance_px": round(dist, 1),
                        "max_jump_px": self.max_jump_px,
                        "candidate_xy": (round(x + w / 2.0, 1), round(y + h / 2.0, 1)),
                        "last_xy": (round(lx, 1), round(ly, 1)),
                    }
                elif self._last_confirmed_size is not None:
                    lw, lh = self._last_confirmed_size
                    size_ratio = (w * h) / max(lw * lh, 1e-6)
                    if size_ratio > self.max_size_ratio or size_ratio < 1.0 / self.max_size_ratio:
                        self.last_rejected = {
                            "reason": "size", "size_ratio": round(size_ratio, 2),
                            "max_size_ratio": self.max_size_ratio,
                            "candidate_wh": (round(w, 1), round(h, 1)),
                            "last_wh": (round(lw, 1), round(lh, 1)),
                        }
                    else:
                        chosen = nearest
                else:
                    chosen = nearest
            if chosen is None:
                self._frames_since_last_detection += 1
                return None
            self._frames_since_last_detection = 0
            _, x, y, w, h = chosen
            self._last_position = (x + w / 2.0, y + h / 2.0)
            self._last_confirmed_size = (w, h)
            return chosen

        if not candidates:
            self._frames_since_last_detection += 1
            self._confirm_streak = 0
            self._tentative_position = None
            return None

        best = max(candidates, key=lambda c: c[0])
        _, x, y, w, h = best
        cx, cy = x + w / 2.0, y + h / 2.0
        if self._tentative_position is not None:
            tx, ty = self._tentative_position
            close = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5 <= self.max_jump_px
            self._confirm_streak = self._confirm_streak + 1 if close else 1
        else:
            self._confirm_streak = 1
        self._tentative_position = (cx, cy)

        if self._confirm_streak < self.min_confirm_streak:
            self._frames_since_last_detection += 1
            return None

        self._confirmed = True
        self._frames_since_last_detection = 0
        self._last_position = (cx, cy)
        self._last_confirmed_size = (w, h)
        return best