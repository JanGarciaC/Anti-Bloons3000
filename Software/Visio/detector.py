"""
Visio/detector.py - Deteccio de globus amb model ONNX (onnxruntime)
====================================================================
El model ha d'estar exportat en format ONNX des de YOLOv8:
    yolo export model=best.pt format=onnx imgsz=320 simplify=True
"""

import sys
import math
import logging
import subprocess
from pathlib import Path
from typing import Optional, Tuple

log = logging.getLogger("robot.visio")


def _install(pkgs):
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


Objectiu = Optional[Tuple[int, int, int, int, float]]


def trobar_model(model_path: str) -> Path:
    if model_path:
        p = Path(model_path)
        if p.exists():
            return p
        print(f"[Detector] ERROR: Model no trobat a la ruta indicada: {p}")
        log.error(f"Model no trobat: {p}")
        sys.exit(1)
    candidates = [
        Path("runs/train_run/weights/best.onnx"),
        *sorted(Path(".").rglob("best.onnx")),
    ]
    for c in candidates:
        if c.exists():
            print(f"[Detector] Model trobat: {c}")
            log.info(f"Model trobat: {c}")
            return c
    print("[Detector] ERROR: No s'ha trobat cap model .onnx.")
    print("[Detector] Exporta'l amb: yolo export model=best.pt format=onnx imgsz=320 simplify=True")
    log.error("No s'ha trobat cap model .onnx.")
    sys.exit(1)


def obrir_camera(cfg):
    """
    Prova els metodes d'obertura de camera en ordre de prioritat:
      1. picamera2       - API nativa per a cameras CSI (recomanada RPi 4+)
      2. libcamerasrc    - pipeline GStreamer per camera CSI
      3. rpicam-vid      - eina de linia de comandes de libcamera
      4. V4L2 /dev/video - mode generic Linux
      5. CAP_ANY         - OpenCV tria automaticament
    """

    w, h, fps = cfg.frame_width, cfg.frame_height, cfg.fps_target
    print(f"[Camera] Intentant obrir camera ({w}x{h} a {fps} fps)...")

    # 1. picamera2
    try:
        from picamera2 import Picamera2
        import threading, time as _time

        class _Picamera2Cap:
            def __init__(self):
                self._picam = Picamera2()
                config = self._picam.create_video_configuration(
                    main={"size": (w, h), "format": "BGR888"},
                    controls={"FrameRate": float(fps)},
                )
                self._picam.configure(config)
                self._picam.start()
                _time.sleep(0.3)
                self._frame   = None
                self._lock    = threading.Lock()
                self._running = True
                self._thread  = threading.Thread(target=self._capturar, daemon=True)
                self._thread.start()
                deadline = _time.time() + 3.0
                while _time.time() < deadline:
                    with self._lock:
                        if self._frame is not None:
                            break
                    _time.sleep(0.05)

            def _capturar(self):
                while self._running:
                    frame = self._picam.capture_array("main")
                    with self._lock:
                        self._frame = frame

            def isOpened(self):
                with self._lock:
                    return self._frame is not None

            def read(self):
                with self._lock:
                    if self._frame is None:
                        return False, None
                    return True, self._frame.copy()

            def get(self, prop_id):
                if prop_id == cv2.CAP_PROP_FRAME_WIDTH:  return w
                if prop_id == cv2.CAP_PROP_FRAME_HEIGHT: return h
                if prop_id == cv2.CAP_PROP_FPS:          return fps
                return 0

            def set(self, *_): pass

            def release(self):
                self._running = False
                self._picam.stop()
                self._picam.close()

        cap = _Picamera2Cap()
        if cap.isOpened():
            print("[Camera] Camera oberta via picamera2 (CSI natiu)")
            log.info("Camera oberta via picamera2 (CSI natiu)")
            return cap
    except Exception as exc:
        print(f"[Camera] picamera2 no disponible: {exc}")
        log.debug(f"picamera2 no disponible: {exc}")

    # 2-5. OpenCV VideoCapture
    pipelines = [
        (
            f"libcamerasrc ! video/x-raw,width={w},height={h},"
            f"framerate={fps}/1 ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink drop=1 sync=false",
            cv2.CAP_GSTREAMER,
            "libcamerasrc GStreamer",
        ),
        (
            f"rpicam-vid --width {w} --height {h} --framerate {fps} "
            f"--codec yuv420 -o - -t 0 2>/dev/null | "
            f"gst-launch-1.0 fdsrc ! rawvideoparse width={w} height={h} "
            f"format=i420 ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER,
            "rpicam-vid GStreamer",
        ),
        (cfg.camera_index, cv2.CAP_V4L2, f"V4L2 index {cfg.camera_index}"),
        (cfg.camera_index, cv2.CAP_ANY,  f"CAP_ANY index {cfg.camera_index}"),
    ]

    for src, backend, etiqueta in pipelines:
        try:
            print(f"[Camera] Provant: {etiqueta}...")
            cap = cv2.VideoCapture(src, backend)
            if not cap.isOpened():
                cap.release()
                print(f"[Camera] {etiqueta}: no s'ha pogut obrir")
                log.debug(f"[{etiqueta}] no s'ha pogut obrir")
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS,          fps)
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                print(f"[Camera] Camera oberta correctament: {etiqueta}")
                log.info(f"Camera oberta: {etiqueta}")
                return cap
            cap.release()
            print(f"[Camera] {etiqueta}: obert pero sense frame valid")
            log.debug(f"[{etiqueta}] obert pero sense frame valid")
        except Exception as exc:
            print(f"[Camera] {etiqueta}: excepcio: {exc}")
            log.debug(f"[{etiqueta}] excepcio: {exc}")

    _diagnosticar_camera()
    sys.exit(1)


def _diagnosticar_camera():
    print("[Camera] ERROR: No s'ha pogut obrir la camera amb cap metode.")
    print("[Camera] --- Diagnostic ---")

    try:
        dispositius = sorted(Path("/dev").glob("video*"))
        if dispositius:
            print(f"[Camera] Dispositius /dev/video* trobats: {[str(d) for d in dispositius]}")
        else:
            print("[Camera] Cap /dev/video* trobat. La camera no s'ha detectat.")
    except Exception:
        pass

    try:
        r = subprocess.run(
            ["libcamera-hello", "--list-cameras"],
            capture_output=True, text=True, timeout=5
        )
        if r.returncode == 0:
            print(f"[Camera] libcamera detecta: {r.stdout.strip()}")
        else:
            print("[Camera] libcamera-hello ha fallat. Comprova que la camera CSI esta connectada.")
    except FileNotFoundError:
        print("[Camera] libcamera-hello no instal-lat.")
    except Exception:
        pass

    try:
        import picamera2  # noqa
        print("[Camera] picamera2 instal-lat pero no ha funcionat.")
    except ImportError:
        print("[Camera] picamera2 no instal-lat. Prova: sudo apt install python3-picamera2")

    print("[Camera] Solucions habituals per a RPi 4:")
    print("[Camera]   1. Comprova que el cable de la camera CSI esta ben connectat")
    print("[Camera]   2. Activa la camera: sudo raspi-config -> Interface Options -> Camera")
    print("[Camera]   3. Instal-la picamera2: sudo apt install python3-picamera2")
    print("[Camera]   4. Prova: libcamera-hello --list-cameras")
    print("[Camera]   5. Si es camera USB, comprova: ls /dev/video*")


def _preprocessar(frame: np.ndarray, input_size: Tuple[int, int]):
    ih, iw = frame.shape[:2]
    th, tw = input_size

    scale = min(tw / iw, th / ih)
    nw, nh = int(iw * scale), int(ih * scale)
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

    canvas   = np.full((th, tw, 3), 114, dtype=np.uint8)
    pad_top  = (th - nh) // 2
    pad_left = (tw - nw) // 2
    canvas[pad_top:pad_top + nh, pad_left:pad_left + nw] = resized

    img = canvas[:, :, ::-1].astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return img, scale, pad_top, pad_left


def _postprocessar(output, scale, pad_top, pad_left,
                   conf_threshold, iou_threshold, orig_w, orig_h):
    out = output[0]

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

    if out.ndim == 2:
        if out.shape[0] < out.shape[1]:
            out = out.T

        boxes_raw  = out[:, :4]
        scores_raw = out[:, 4:]
        class_conf = scores_raw.max(axis=1)

        mask = class_conf >= conf_threshold
        if not mask.any():
            return []

        boxes_f = boxes_raw[mask]
        confs_f = class_conf[mask]

        cx, cy = boxes_f[:, 0], boxes_f[:, 1]
        bw, bh = boxes_f[:, 2], boxes_f[:, 3]
        x1s = cx - bw / 2
        y1s = cy - bh / 2

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
            x2 = int(((x1s[i] + bw[i]) - pad_left) / scale)
            y2 = int(((y1s[i] + bh[i]) - pad_top)  / scale)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(orig_w, x2), min(orig_h, y2)
            deteccions.append((x1, y1, x2, y2, float(confs_f[i])))
        return deteccions

    return []


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


def calcular_offset_tilt(area_px: float, frame_w: int, frame_h: int, cfg) -> float:
    if area_px <= 0:
        return cfg.tilt_canon_offset_deg
    radi_px  = math.sqrt(area_px / math.pi)
    focal_px = (frame_w / 2.0) / math.tan(math.radians(62.0) / 2.0)
    dist_m   = max(0.5, (0.125 * focal_px) / max(radi_px, 1.0))
    return math.degrees(math.atan(cfg.canon_offset_cm / 100.0 / dist_m))


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


class Detector:
    """
    Encapsula camera + model ONNX (YOLOv8 exportat).

    Us:
        det = Detector(cfg)
        frame, obj = det.llegir_frame()
        det.alliberar()
    """

    def __init__(self, cfg):
        self._cfg = cfg

        print("[Detector] Carregant model ONNX...")
        model_path = trobar_model(cfg.model_path)
        sess_opts  = ort.SessionOptions()
        sess_opts.intra_op_num_threads = 4
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._sess = ort.InferenceSession(
            str(model_path),
            sess_options=sess_opts,
            providers=["CPUExecutionProvider"],
        )
        inp = self._sess.get_inputs()[0]
        self._input_name = inp.name
        _, _, self._input_h, self._input_w = inp.shape
        self._input_size = (self._input_h, self._input_w)
        print(f"[Detector] Model carregat: {model_path.name} | entrada: {self._input_w}x{self._input_h}")
        log.info(f"Model ONNX: {model_path.name} | entrada: {self._input_w}x{self._input_h}")

        self._cap = obrir_camera(cfg)
        self.w  = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.h  = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.cx = self.w // 2
        self.cy = self.h // 2
        print(f"[Detector] Resolucio: {self.w}x{self.h} | Centre: ({self.cx},{self.cy})")
        log.info(f"Resolucio: {self.w}x{self.h} | Centre: ({self.cx},{self.cy})")

    def llegir_frame(self) -> Tuple[Optional[np.ndarray], Objectiu]:
        ret, frame = self._cap.read()
        if not ret or frame is None:
            log.warning("Frame perdut.")
            return None, None

        tensor, scale, pad_top, pad_left = _preprocessar(frame, self._input_size)
        output = self._sess.run(None, {self._input_name: tensor})
        deteccions = _postprocessar(
            output[0], scale, pad_top, pad_left,
            self._cfg.conf_threshold, self._cfg.iou_threshold,
            self.w, self.h,
        )
        objectiu = seleccionar_objectiu(deteccions)
        return frame, objectiu

    def alliberar(self):
        if self._cap:
            self._cap.release()
        cv2.destroyAllWindows()
        print("[Detector] Camera alliberada.")
        log.info("Camera alliberada.")
