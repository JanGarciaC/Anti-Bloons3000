"""
estat.py — Estat global del robot
==================================
Dataclass llegera que els mòduls poden llegir i modificar
per passar informació entre ells sense acoblament fort.
"""

from dataclasses import dataclass, field
from desplacament.motors import EXPLORANT


@dataclass
class EstatRobot:
    # Servos
    pan_deg:          float = 90.0
    tilt_deg:         float = 90.0

    # Detecció
    target_detected:  bool  = False
    target_cx:        int   = 0
    target_cy:        int   = 0
    target_area:      float = 0.0
    on_target:        bool  = False

    # FSM
    fase:             str   = EXPLORANT

    # Rendiment
    fps:              float = 0.0
