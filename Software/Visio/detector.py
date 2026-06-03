"""
visio/detector.py — Detecció de globus amb model ONNX (onnxruntime)
====================================================================
Substitueix ultralytics per onnxruntime, que és lleuger i funciona
perfectament a la Raspberry Pi 4.

El model ha d'estar exportat en format ONNX des de YOLOv8:
    yolo export model=best.pt format=onnx imgsz=320 simplify=True

Responsabilitats:
  - Obrir i gestionar la càmera CSI / V4L2
  - Pre-processar el frame per al model (resize, normalitzar, transpose)
  - Executar inferència ONNX
  - Post-processar les sortides (decode bboxes + NMS)
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
    import onnxruntime as ort
except ImportError:
    _install(["onnxruntime"]); import onnxruntime as ort


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
        Path("runs/train_run/weights/best.onnx"),
        *sorted(Path(".").rglob("best.onnx")),
    ]
    for c in candidates:
        if c.exists():
            log.info(f"Model trobat: {c}")
            return c
    log.error(
        "No s'ha trobat cap model .onnx.\n"
        "Exporta'l amb: yolo export model=best.pt format=onnx imgsz=320 simplify=True"
    )
    sys.exit(1)


# ---------------------------------------------------------------------------
# Obertura de càmera (idèntica al codi original)
# ---------------------------------------------------------------------------
def obrir_camera(cfg):
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
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  cfg.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.frame_height)
                cap.set(cv2.CAP_PROP_FPS,          cfg.fps_target)
                ret, _ = cap.read()
                if ret:
                    label = src if isinstance(src, str) else f"index {src}"
                    log.info(f"Càmera oberta: {label}")
                    return cap
                cap.release()
        except Exception:
            continue
    log.error("No s'ha pogut obrir la càmera CSI.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Pre-processat: frame BGR → tensor ONNX [1, 3, H, W] float32 [0..1]
# ---------------------------------------------------------------------------
def _preprocessar(frame: np.ndarray, input_size: Tuple[int, int]) -> np.ndarray:
    """
    Redimensiona amb letterboxing i normalitza.
    Retorna tensor (1, 3, H, W) float32 en [0, 1].
    """
    ih, iw = frame.shape[:2]
    th, tw = input_size  # target height, width

    # Escala uniforme (letterbox)
    scale = min(tw / iw, th / ih)
    nw, nh = int(iw * scale), int(ih * scale)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

    # Padding gris (114) per arribar a la mida del model
    canvas = np.full((th, tw, 3), 114, dtype=np.uint8)
    pad_top  = (th - nh) // 2
    pad_left = (tw - nw) // 2
    canvas[pad_top:pad_top + nh, pad_left:pad_left + nw] = resized

    # BGR → RGB, HWC → CHW, uint8 → float32 [0..1]
    img = canvas[:, :, ::-1].astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))          # (3, H, W)
    img = np.expand_dims(img, axis=0)            # (1, 3, H, W)
    return img, scale, pad_top, pad_left


# ---------------------------------------------------------------------------
# Post-processat: sortida ONNX → llista de deteccions
# ---------------------------------------------------------------------------
def _postprocessar(
    output: np.ndarray,
    scale: float,
    pad_top: int,
    pad_left: int,
    conf_threshold: float,
    iou_threshold: float,
    orig_w: int,
    orig_h: int,
) -> list:
    """
    Decodifica la sortida del model ONNX YOLOv8.

    Format de sortida YOLOv8 ONNX (exportat sense NMS):
      shape: (1, 4 + num_classes, num_anchors)   → transposem a (num_anchors, 4+nc)

    Format de sortida amb NMS integrat (export simplify=True + nms=True):
      shape: (1, num_deteccions, 6)  → (x1,y1,x2,y2,conf,class_id)

    Aquesta funció gestiona els dos casos automàticament.

    Retorna llista de (x1, y1, x2, y2, confiança) en coordenades originals.
    """
    # Elimina dimensió de batch
    out = output[0]  # (4+nc, N) o (N, 6) o (N, 4+nc)

    # ── Cas 1: sortida amb NMS integrat → shape (N, 6) ──────────────────────
    if out.ndim == 2 and out.shape[1] == 6:
        deteccions = []
        for det in out:
            x1, y1, x2, y2, conf, _ = det
            if conf < conf_threshold:
                continue
            x1 = int((x1 - pad_left) / scale)
            y1 = int((y1 - pad_top)  / scale)
            x2 = int((x2 - pad_left) / scale)
            y2 = int((y2 - pad_top)  / scale)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(orig_w, x2), min(orig_h, y2)
            deteccions.append((x1, y1, x2, y2, float(conf)))
        return deteccions

    # ── Cas 2: sortida sense NMS → shape (4+nc, N) o (N, 4+nc) ─────────────
    if out.ndim == 2:
        # Normalitzem a (N, 4+nc)
        if out.shape[0] < out.shape[1]:
            out = out.T   # era (4+nc, N)

        # Filtra per confiança màxima de classe
        num_classes = out.shape[1] - 4
        boxes_raw   = out[:, :4]            # cx, cy, w, h (format YOLO)
        scores_raw  = out[:, 4:]            # (N, nc)
        class_conf  = scores_raw.max(axis=1)

        mask = class_conf >= conf_threshold
        if not mask.any():
            return []

        boxes_f = boxes_raw[mask]
        confs_f = class_conf[mask]

        # cx,cy,w,h → x1,y1,x2,y2 en coordenades de la imatge entrada
        cx, cy, bw, bh = boxes_f[:, 0], boxes_f[:, 1], boxes_f[:, 2], boxes_f[:, 3]
        x1s = cx - bw / 2
        y1s = cy - bh / 2
        x2s = cx + bw / 2
        y2s = cy + bh / 2

        # NMS manual amb cv2.dnn.NMSBoxes
        bboxes_cv = np.stack([x1s, y1s, bw, bh], axis=1).tolist()
        idxs = cv2.dnn.NMSBoxes(
            bboxes_cv, confs_f.tolist(), conf_threshold, iou_threshold
        )
        if len(idxs) == 0:
            return []

        deteccions = []
        for i in np.array(idxs).flatten():
            x1 = int((x1s[i] - pad_left) / scale)
            y1 = int((y1s[i] - pad_top)  / scale)
            x2 = int((x2s[i] - pad_left) / scale)
            y2 = int((y2s[i] - pad_top)  / scale)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(orig_w, x2), min(orig_h, y2)
            deteccions.append((x1, y1, x2, y2, float(confs_f[i])))
        return deteccions

    return []


# ---------------------------------------------------------------------------
# Selecció objectiu (idèntica al codi original)
# ---------------------------------------------------------------------------
def seleccionar_objectiu(deteccions: list) -> Objectiu:
    if not deteccions:
        return None
    millor, millor_area = None, -1.0
    for x1, y1, x2, y2, conf in deteccions:
        area = float((x2 - x1) * (y2 - y1))
        if area > millor_area:
            millor_area = area
            millor = (x1, y1, x2, y2, area)
    return millor


# ---------------------------------------------------------------------------
# Offset tilt per compensar distància càmera-canó (idèntica al codi original)
# ---------------------------------------------------------------------------
def calcular_offset_tilt(area_px: float, frame_w: int, frame_h: int, cfg) -> float:
    if area_px <= 0:
        return cfg.tilt_canon_offset_deg
    radi_px  = math.sqrt(area_px / math.pi)
    focal_px = (frame_w / 2.0) / math.tan(math.radians(62.0) / 2.0)
    dist_m   = max(0.5, (0.125 * focal_px) / max(radi_px, 1.0))
    return math.degrees(math.atan(cfg.canon_offset_cm / 100.0 / dist_m))


# ---------------------------------------------------------------------------
# Preview de depuració (idèntica al codi original)
# ---------------------------------------------------------------------------
def dibuixar_preview(frame, state, objectiu: Objectiu, cfg) -> np.ndarray:
    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    cv2.line(frame, (cx_frame - 20, cy_frame), (cx_frame + 20, cy_frame), (0, 255, 255), 1)
    cv2.line(frame, (cx_frame, cy_frame - 20), (cx_frame, cy_frame + 20), (0, 255, 255), 1)

    if objectiu:
        x1, y1, x2, y2, area = objectiu
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        color = (0, 255, 0) if state.on_target else (0, 140, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, -1)
        cv2.line(frame, (cx_frame, cy_frame), (cx, cy), (100, 100, 255), 1)

    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (280, 120), (15, 15, 25), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
    lines = [
        f"FPS:  {state.fps:.1f}",
        f"PAN:  {state.pan_deg:.1f} graus",
        f"TILT: {state.tilt_deg:.1f} graus",
        f"Globus: {'SI' if state.target_detected else 'NO'}",
        f"{'>>> A PUNT <<<' if state.on_target else ''}",
    ]
    for i, txt in enumerate(lines):
        color = (0, 255, 120) if (state.on_target and i == 4) else (200, 200, 200)
        cv2.putText(frame, txt, (8, 20 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return frame


# ---------------------------------------------------------------------------
# Classe principal Detector
# ---------------------------------------------------------------------------
class Detector:
    """
    Encapsula càmera + model ONNX (YOLOv8 exportat).

    Exemple d'ús:
        det = Detector(cfg)
        frame, obj = det.llegir_frame()
        # obj: (x1,y1,x2,y2,area) o None
        det.alliberar()
    """

    def __init__(self, cfg):
        self._cfg = cfg

        model_path = trobar_model(cfg.model_path)

        # Sessió ONNX — usa CPU (òptim per a RPi 4)
        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = 4          # aprofita els 4 cores
        sess_opts.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        self._sess = ort.InferenceSession(
            str(model_path),
            sess_options=sess_opts,
            providers=["CPUExecutionProvider"],
        )

        # Nom i forma de l'entrada del model
        inp = self._sess.get_inputs()[0]
        self._input_name = inp.name
        _, _, self._input_h, self._input_w = inp.shape  # ex: 1,3,320,320
        self._input_size = (self._input_h, self._input_w)

        log.info(
            f"Model ONNX carregat: {model_path.name} | "
            f"entrada: {self._input_w}x{self._input_h}"
        )

        self._cap = obrir_camera(cfg)
        self.w    = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h    = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cx   = self.w // 2
        self.cy   = self.h // 2
        log.info(f"Resolució: {self.w}x{self.h} | Centre: ({self.cx},{self.cy})")

    # ------------------------------------------------------------------
    def llegir_frame(self) -> Tuple[Optional[np.ndarray], Objectiu]:
        """
        Llegeix un frame, executa inferència ONNX i retorna (frame, objectiu).
        Retorna (None, None) si no hi ha frame disponible.
        """
        ret, frame = self._cap.read()
        if not ret:
            log.warning("Frame perdut.")
            return None, None

        # Pre-processat
        tensor, scale, pad_top, pad_left = _preprocessar(frame, self._input_size)

        # Inferència
        output = self._sess.run(None, {self._input_name: tensor})

        # Post-processat
        deteccions = _postprocessar(
            output[0],
            scale, pad_top, pad_left,
            self._cfg.conf_threshold,
            self._cfg.iou_threshold,
            self.w, self.h,
        )

        objectiu = seleccionar_objectiu(deteccions)
        return frame, objectiu

    # ------------------------------------------------------------------
    def alliberar(self):
        """Allibera la càmera."""
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        log.info("Càmera alliberada.")
