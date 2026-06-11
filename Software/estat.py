"""
estat.py - Estat global del robot
==================================
Dataclass compartida entre moduls.
"""

from dataclasses import dataclass
from Desplacament.motors import EXPLORANT


@dataclass
class EstatRobot:
    pan_deg:          float = 90.0
    tilt_deg:         float = 90.0
    target_detected:  bool  = False
    target_cx:        int   = 0
    target_cy:        int   = 0
    target_area:      float = 0.0
    on_target:        bool  = False
    fase:             str   = EXPLORANT
    fps:              float = 0.0
