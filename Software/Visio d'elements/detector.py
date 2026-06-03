"""
visio/detector.py — Detecció de globus amb YOLOv8
==================================================
Responsabilitats:
  - Obrir i gestionar la càmera CSI / V4L2
  - Executar inferència YOLOv8 per frame
  - Seleccionar el globus objectiu (el més gran = el més proper)
  - Calcular l'offset tilt per compensar la distància càmera-canó
  - Dibuixar la finestra de depuració (opcional)

Ús:
    from visio.detector import Detector
    det = Detector(cfg)
    frame, objectiu = det.llegir_frame()   # objectiu = (x1,y1,x2,y2,area) o None
    det.alliberar()
"""

import sys
import math
import logging
from pathlib import Path
from typing import Optional, Tuple

log = logging.getLogger("robot.visio")


def _install(pkgs):
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", *pkgs, "-q"])


try:
    import cv2
except ImportError:
    _install(["opencv-python-headless"]); import cv2

try:
    import numpy as np
except ImportError:
    _install(["numpy"]); import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    _install(["ultralytics"]); from ultralytics import YOLO


# ---------------------------------------------------------------------------
# Tipus d'objectiu
# ---------------------------------------------------------------------------
Objectiu = Optional[Tuple[int, int, int, int, float]]   # x1,y1,x2,y2,area


# ---------------------------------------------------------------------------
# Cerca del model
# ---------------------------------------------------------------------------
def trobar_model(model_path: str) -> Path:
    if model_path:
        p = Path(model_path)
        if p.exists():
            return p
        log.error(f"Model no trobat: {p}")
        sys.exit(1)
    candidates = [
        Path("runs/train_run/weights/best.pt"),
        *sorted(Path(".").rglob("best.pt")),
    ]
    for c in candidates:
        if c.exists():
            log.info(f"Model trobat: {c}")
            return c
    log.error("No s'ha trobat cap model .pt. Entrena'n un primer.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Obertura de càmera
# ---------------------------------------------------------------------------
def _obrir_camera(cfg):
    pipelines = [
        (
            f"libcamerasrc ! video/x-raw,width={cfg.frame_width},"
            f"height={cfg.frame_height},framerate={cfg.fps_target}/1 ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=1",
            cv2.CAP_GSTREAMER,
        ),
        (cfg.camera_index, cv2.CAP_V4L2),
        (cfg.camera_index, cv2.CAP_ANY),
    ]
    for src, backend in pipelines:
        try:
            cap = cv2.VideoCapture(src, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.frame_height)
                cap.set(cv2.CAP_PROP_FPS, cfg.fps_target)
                ret, _ = cap.read()
                if ret:
                    label = src if isinstance(src, str) else f"index {src}"
                    log.info(f"Càmera oberta: {label}")
                    return cap
                cap.release()
        except Exception:
            continue
    log.error("No s'ha pogut obrir la càmera.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Selecció objectiu
# ---------------------------------------------------------------------------
def _seleccionar_objectiu(boxes) -> Objectiu:
    if boxes is None or len(boxes) == 0:
        return None
    millor, millor_area = None, -1.0
    for box in boxes:
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
        area = float((x2 - x1) * (y2 - y1))
        if area > millor_area:
            millor_area = area
            millor = (x1, y1, x2, y2, area)
    return millor


# ---------------------------------------------------------------------------
# Offset tilt per compensar distància càmera-canó
# ---------------------------------------------------------------------------
def calcular_offset_tilt(area_px: float, frame_w: int, frame_h: int, cfg) -> float:
    """
    Estima la distància al globus per la seva mida aparent i retorna
    l'angle de compensació entre càmera i canó.

    Supòsits:
      - Diàmetre real del globus: ~25 cm (radi 0.125 m)
      - FOV horitzontal Camera Module v2: 62°
    """
    if area_px <= 0:
        return cfg.tilt_canon_offset_deg
    radi_px = math.sqrt(area_px / math.pi)
    focal_px = (frame_w / 2.0) / math.tan(math.radians(62.0) / 2.0)
    dist_m = max(0.5, (0.125 * focal_px) / max(radi_px, 1.0))
    offset_deg = math.degrees(math.atan(cfg.canon_offset_cm / 100.0 / dist_m))
    return offset_deg


# ---------------------------------------------------------------------------
# Preview de depuració
# ---------------------------------------------------------------------------
def dibuixar_preview(frame, state, objectiu: Objectiu, cfg) -> None:
    h, w = frame.shape[:2]
    cx_f, cy_f = w // 2, h // 2

    # Creu central
    cv2.line(frame, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (0, 255, 255), 1)
    cv2.line(frame, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (0, 255, 255), 1)

    if objectiu:
        x1, y1, x2, y2, area = objectiu
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        color = (0, 255, 0) if state.on_target else (0, 140, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.line(frame, (cx_f, cy_f), (cx, cy), (100, 100, 255), 1)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (300, 140), (15, 15, 25), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    estat_txt = {
        "EXPLORANT":  ("EXPLORANT",  (200, 200,   0)),
        "APROXIMANT": ("APROXIMANT", (200, 140,   0)),
        "APUNTANT":   ("APUNTANT",   (  0, 200, 200)),
        "DISPARAT":   ("DISPARAT!",  (  0,   0, 255)),
    }.get(state.fase, (state.fase, (200, 200, 200)))

    lines = [
        (f"FPS:   {state.fps:.1f}",                          (200, 200, 200)),
        (f"PAN:   {state.pan_deg:.1f}°",                     (200, 200, 200)),
        (f"TILT:  {state.tilt_deg:.1f}°",                    (200, 200, 200)),
        (f"Globus: {'SI' if state.target_detected else 'NO'}", (200, 200, 200)),
        (f"Estat: {estat_txt[0]}",                            estat_txt[1]),
    ]
    for i, (txt, color) in enumerate(lines):
        cv2.putText(frame, txt, (8, 20 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# Classe principal Detector
# ---------------------------------------------------------------------------
class Detector:
    """
    Encapsula càmera + model YOLO.

    Exemple d'ús:
        det = Detector(cfg)
        frame, obj = det.llegir_frame()
        # obj: (x1,y1,x2,y2,area) o None
        det.alliberar()
    """

    def __init__(self, cfg):
        self._cfg = cfg

        # Càrrega silenciosa del model
        import io, contextlib, logging as _log
        from ultralytics.utils import LOGGER as _UL
        _UL.setLevel(_log.WARNING)

        model_path = trobar_model(cfg.model_path)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self._model = YOLO(str(model_path))
        for line in buf.getvalue().splitlines():
            if "Model summary" in line:
                log.info(line.strip())
                break

        self._cap = _obrir_camera(cfg)
        self.w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cx = self.w // 2
        self.cy = self.h // 2
        log.info(f"Resolució: {self.w}x{self.h} | Centre: ({self.cx},{self.cy})")

    # ------------------------------------------------------------------
    def llegir_frame(self) -> Tuple[Optional[np.ndarray], Objectiu]:
        """
        Llegeix un frame, executa YOLO i retorna (frame, objectiu).
        Retorna (None, None) si no hi ha frame disponible.
        """
        ret, frame = self._cap.read()
        if not ret:
            log.warning("Frame perdut.")
            return None, None

        resultats = self._model.predict(
            frame,
            conf=self._cfg.conf_threshold,
            iou=self._cfg.iou_threshold,
            verbose=False,
            imgsz=320,
        )
        objectiu = _seleccionar_objectiu(resultats[0].boxes)
        return frame, objectiu

    # ------------------------------------------------------------------
    def alliberar(self):
        """Allibera la càmera."""
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        log.info("Càmera alliberada.")
