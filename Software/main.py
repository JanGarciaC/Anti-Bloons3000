"""
main.py — Robot detector i destructor de globus
================================================
Raspberry Pi 4 + càmera CSI + YOLOv8 ONNX + 2 servos + 2 motors + disparador

Arquitectura de mòduls:
  software/
  ├── config.py                  ← tots els paràmetres
  ├── estat.py                   ← estat global compartit
  ├── main.py                    ← AQUEST FITXER (bucle principal + FSM)
  ├── Visio d'elements/
  │   └── detector.py            ← càmera + inferència YOLO
  ├── Tir/
  │   └── apuntament.py          ← PID + servos + disparo
  └── Desplaçament/
      └── motors.py              ← motors de rodes + màquina d'estats

Comportament:
  1. EXPLORANT  → avança en línia recta fins que detecta un globus
  2. APROXIMANT → s'acosta al globus tot apuntant-hi amb els servos
  3. APUNTANT   → para, afina l'apuntament amb PID
  4. DISPARAT   → dispara, cooldown, torna a explorar

Execució:
    python main.py
    python main.py --dry-run            # sense moure res físicament
    python main.py --preview            # finestra de depuració
    python main.py --modo gpio          # forçar GPIO directe
    python main.py --modo pca9685       # forçar PCA9685
    python main.py --conf 0.4           # umbral de confiança YOLO
    python main.py --model best.pt      # ruta al model
"""

import sys
import time
import argparse
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("robot")

# Importem els mòduls del projecte
sys.path.insert(0, __file__.replace("main.py", ""))

from config import CFG
from estat  import EstatRobot

# Importa els mòduls amb noms de carpeta amb accents / espais
import importlib, sys as _sys

_visio      = importlib.import_module("Visio.detector")
_tir        = importlib.import_module("Tir.apuntament")
_deplac     = importlib.import_module("Desplaçament.motors")

Detector        = _visio.Detector
dibuixar_preview = _visio.dibuixar_preview
AimController   = _tir.AimController
MotorController = _deplac.MotorController
RobotFSM        = _deplac.RobotFSM

try:
    import cv2
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install",
                           "opencv-python-headless", "-q"])
    import cv2


# ===========================================================================
# BUCLE PRINCIPAL
# ===========================================================================

def bucle_principal(cfg, modo: str, dry_run: bool, show_preview: bool):
    # ── Inicialitza subsistemes ──────────────────────────────────────────
    log.info("Inicialitzant subsistemes...")
    detector = Detector(cfg)
    aim      = AimController(cfg, modo=modo, dry_run=dry_run)
    motors   = MotorController(cfg, dry_run=dry_run)
    fsm      = RobotFSM(cfg, motors)
    state    = EstatRobot()

    w, h  = detector.w, detector.h
    cx_f  = detector.cx
    cy_f  = detector.cy

    t_last    = time.time()
    log_timer = time.time()

    log.info("Robot llest. Prem Ctrl+C per aturar.")
    print()

    try:
        while True:
            # ── Captura i detecció ───────────────────────────────────────
            frame, objectiu = detector.llegir_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            now = time.time()
            state.fps  = 1.0 / max(now - t_last, 1e-6)
            t_last = now

            # ── Actualitza estat de detecció ─────────────────────────────
            prou_a_prop = False

            if objectiu is not None:
                x1, y1, x2, y2, area = objectiu
                cx_obj = (x1 + x2) // 2
                cy_obj = (y1 + y2) // 2

                state.target_detected = True
                state.target_cx       = cx_obj
                state.target_cy       = cy_obj
                state.target_area     = area

                prou_a_prop = area >= cfg.area_prou_a_prop

                # Errors en píxels respecte al centre del frame
                error_x = cx_obj - cx_f   # + = dreta
                error_y = cy_obj - cy_f   # + = avall

                # Apunta (PID + servos) i comprova disparo
                disparo_produit = aim.actualitzar(
                    error_x, error_y,
                    on_target  = (abs(error_x) < cfg.dead_zone_px and
                                  abs(error_y) < cfg.dead_zone_px),
                    area_px    = area,
                    frame_w    = w,
                    frame_h    = h,
                )

                state.on_target  = (abs(error_x) < cfg.dead_zone_px and
                                    abs(error_y) < cfg.dead_zone_px)
                state.pan_deg    = aim.pan_deg
                state.tilt_deg   = aim.tilt_deg

            else:
                state.target_detected = False
                state.on_target       = False
                state.target_area     = 0.0
                disparo_produit       = False
                aim.sense_objectiu()
                state.pan_deg  = aim.pan_deg
                state.tilt_deg = aim.tilt_deg

            # ── Actualitza FSM (rodes) ───────────────────────────────────
            state.fase = fsm.actualitzar(
                objectiu_detectat = state.target_detected,
                prou_a_prop       = prou_a_prop,
                on_target         = state.on_target,
                disparo_produit   = disparo_produit,
            )

            # ── Preview (opcional) ───────────────────────────────────────
            if show_preview and frame is not None:
                dibuixar_preview(frame, state, objectiu, cfg)
                cv2.imshow("Robot Globus", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    log.info("Sortint per teclat...")
                    break

            # ── Log cada 5 s ─────────────────────────────────────────────
            if now - log_timer >= 5.0:
                log_timer = now
                log.info(
                    f"FPS={state.fps:.1f} | PAN={state.pan_deg:.1f}° | "
                    f"TILT={state.tilt_deg:.1f}° | "
                    f"Globus={'SI' if state.target_detected else 'NO'} | "
                    f"Estat={state.fase}"
                )

    except KeyboardInterrupt:
        log.info("Aturat per l'usuari (Ctrl+C).")

    finally:
        log.info("Aturant subsistemes...")
        motors.parar()
        aim.cleanup()
        detector.alliberar()
        motors.cleanup()
        if show_preview:
            cv2.destroyAllWindows()
        log.info("Robot aturat correctament.")


# ===========================================================================
# PUNT D'ENTRADA
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(description="Robot detector de globus")
    parser.add_argument("--model",    type=str,   default="")
    parser.add_argument("--conf",     type=float, default=CFG.conf_threshold)
    parser.add_argument("--modo",     type=str,   default="auto",
                        choices=["auto", "gpio", "pca9685"])
    parser.add_argument("--dry-run",  action="store_true",
                        help="No mou cap actuador (prova de lògica)")
    parser.add_argument("--preview",  action="store_true",
                        help="Mostra finestra de depuració (requereix pantalla)")
    parser.add_argument("--camera",   type=int,   default=0)
    args = parser.parse_args()

    CFG.conf_threshold = args.conf
    CFG.camera_index   = args.camera
    if args.model:
        CFG.model_path = args.model

    print("=" * 60)
    print("  ROBOT DETECTOR DE GLOBUS — Raspberry Pi 4")
    print("=" * 60)
    print(f"  Mode servos  : {args.modo}")
    print(f"  Dry-run      : {args.dry_run}")
    print(f"  Preview      : {args.preview}")
    print(f"  Confiança    : {CFG.conf_threshold}")
    print(f"  PAN  rang    : [{CFG.pan_min_deg}°, {CFG.pan_max_deg}°]  (centre {CFG.pan_center_deg}°)")
    print(f"  TILT rang    : [{CFG.tilt_min_deg}°, {CFG.tilt_max_deg}°]  (centre {CFG.tilt_center_deg}°)")
    print(f"  Vel. crucero : {CFG.velocitat_avancar}%")
    print(f"  Vel. apropar : {CFG.velocitat_apropar}%")
    print(f"  Àrea a prop  : {CFG.area_prou_a_prop:.0f} px²")
    print("=" * 60)
    print()

    bucle_principal(CFG, args.modo, args.dry_run, args.preview)


if __name__ == "__main__":
    main()
