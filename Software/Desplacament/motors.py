"""
Desplacament/motors.py - Control de motors DC via transistors
=============================================================
Cada motor esta controlat per un transistor NPN (per exemple BC547 o 2N2222)
amb una resistencia de 1k a la base i un diode en paral-lel al motor.

El pin GPIO en HIGH activa el transistor i engega el motor.
El pin GPIO en LOW desactiva el transistor i para el motor.

Connexions GPIO:
  Motor rodes esquerre : GPIO definit a config (motor_esq_pin)
  Motor rodes dret     : GPIO definit a config (motor_dre_pin)
  Motor disparo        : GPIO definit a config (motor_disparo_pin)

Esquema de connexio per a cada motor:
  GPIO -- [R 1k] -- Base transistor NPN
                    Col-lector -- Motor -- VCC
                    Emitter -- GND
                    Diode en paral-lel al motor (catode a VCC, anode a col-lector)
"""

import sys
import time
import logging
from typing import Optional

log = logging.getLogger("robot.desplacament")


def _install(pkgs):
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", *pkgs, "-q"])


EXPLORANT  = "EXPLORANT"
APROXIMANT = "APROXIMANT"
APUNTANT   = "APUNTANT"
DISPARAT   = "DISPARAT"


class MotorController:
    """
    Controla els tres motors DC via transistors.

    motor_esq_pin     -> motor roda esquerra
    motor_dre_pin     -> motor roda dreta
    motor_disparo_pin -> motor de disparo

    HIGH = motor engegat
    LOW  = motor aturat
    """

    def __init__(self, cfg, dry_run: bool = False):
        self._cfg     = cfg
        self._dry_run = dry_run
        self._GPIO    = None

        if dry_run:
            print("[MotorController] Mode dry-run activat. Els motors no es mouraan.")
            log.info("MotorController: MODE DRY-RUN")
            return

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            _install(["RPi.GPIO"]); import RPi.GPIO as GPIO

        self._GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (cfg.motor_esq_pin, cfg.motor_dre_pin, cfg.motor_disparo_pin):
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        print(
            f"[MotorController] Inicialitzat via transistors. "
            f"ESQ=GPIO{cfg.motor_esq_pin} | "
            f"DRE=GPIO{cfg.motor_dre_pin} | "
            f"DISPARO=GPIO{cfg.motor_disparo_pin}"
        )
        log.info("MotorController inicialitzat via transistors.")

    def avancar(self, velocitat: Optional[int] = None):
        """Engega els dos motors de les rodes endavant."""
        if self._dry_run:
            log.debug("[DRY] AVANCAR")
            return
        GPIO = self._GPIO
        GPIO.output(self._cfg.motor_esq_pin, GPIO.HIGH)
        GPIO.output(self._cfg.motor_dre_pin, GPIO.HIGH)

    def parar(self):
        """Atura els dos motors de les rodes."""
        if self._dry_run:
            log.debug("[DRY] PARAR")
            return
        GPIO = self._GPIO
        GPIO.output(self._cfg.motor_esq_pin, GPIO.LOW)
        GPIO.output(self._cfg.motor_dre_pin, GPIO.LOW)

    def disparar(self, durada_s: float = None):
        """Engega el motor de disparo durant durada_s segons i el para."""
        if self._dry_run:
            print("[MotorController] DISPARO SIMULAT (dry-run)")
            log.debug("[DRY] DISPARAR")
            return
        durada = durada_s if durada_s is not None else self._cfg.fire_pulse_ms / 1000.0
        print(f"[MotorController] Engegant motor de disparo durant {durada:.2f} s")
        self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.HIGH)
        time.sleep(durada)
        self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.LOW)
        print("[MotorController] Motor de disparo aturat.")
        log.info("Motor de disparo aturat.")

    def cleanup(self):
        self.parar()
        if self._GPIO:
            self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.LOW)
        print("[MotorController] Motors aturats.")
        log.info("MotorController aturat.")


class RobotFSM:
    """
    Coordina el comportament global del robot.

    EXPLORANT  -> avanca en linia recta fins que detecta un globus
    APROXIMANT -> s'acosta al globus (motors actius) tot apuntant amb els servos
    APUNTANT   -> para els motors, afina l'apuntament amb PID
    DISPARAT   -> dispara, cooldown, torna a EXPLORANT
    """

    COOLDOWN_DISPARO_S = 1.5

    def __init__(self, cfg, motors: MotorController):
        self._cfg    = cfg
        self._motors = motors
        self.fase    = EXPLORANT
        self._disparo_time: Optional[float] = None
        print(f"[RobotFSM] Inicialitzat. Estat inicial: {self.fase}")
        log.info(f"RobotFSM inicialitzat. Estat inicial: {self.fase}")

    def actualitzar(self,
                    objectiu_detectat: bool,
                    prou_a_prop: bool,
                    on_target: bool,
                    disparo_produit: bool) -> str:
        fase_anterior = self.fase

        if self.fase == EXPLORANT:
            if objectiu_detectat:
                self._canviar_fase(APROXIMANT)
            else:
                self._motors.avancar()

        elif self.fase == APROXIMANT:
            if not objectiu_detectat:
                self._canviar_fase(EXPLORANT)
            elif prou_a_prop:
                self._motors.parar()
                self._canviar_fase(APUNTANT)
            else:
                self._motors.avancar()

        elif self.fase == APUNTANT:
            if not objectiu_detectat:
                self._canviar_fase(EXPLORANT)
            elif disparo_produit:
                self._canviar_fase(DISPARAT)
                self._disparo_time = time.time()

        elif self.fase == DISPARAT:
            self._motors.parar()
            if time.time() - self._disparo_time >= self.COOLDOWN_DISPARO_S:
                print("[RobotFSM] Cooldown acabat. Tornant a explorar.")
                log.info("Cooldown acabat. Tornant a explorar.")
                self._canviar_fase(EXPLORANT)

        if self.fase != fase_anterior:
            print(f"[RobotFSM] Estat: {fase_anterior} -> {self.fase}")
            log.info(f"FSM: {fase_anterior} -> {self.fase}")

        return self.fase

    def _canviar_fase(self, nova: str):
        self.fase = nova
        if nova in (APUNTANT, DISPARAT):
            self._motors.parar()
