"""
Desplacament/motors.py - Control de motors DC via transistors i FSM del robot
==============================================================================
Cada motor esta controlat per un transistor NPN amb resistencia de 1k a la base
i un diode en paral-lel al motor.

Els motors de les rodes s'engeguen via PWM per software (GPIO.PWM), que
permet controlar la velocitat amb un duty cycle (0-100%) configurable a
config.py (velocitat_rodes). El motor de disparo es sempre on/off (HIGH/LOW).

Connexions GPIO:
  Motor roda esquerra : GPIO definit a config (motor_esq_pin)
  Motor roda dreta    : GPIO definit a config (motor_dre_pin)
  Motor disparo       : GPIO definit a config (motor_disparo_pin)

Estats de la FSM:
  EXPLORANT  -> el robot avanca contínuament a poc a poc mentre el servo PAN
                fa un swipe continu cercant un globus.
  APROXIMANT -> s'acosta al globus (motors actius) tot apuntant amb els servos
  APUNTANT   -> para els motors, afina l'apuntament amb PID
  DISPARAT   -> dispara, cooldown, torna a EXPLORANT
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

    Les rodes s'engeguen amb PWM per software a la velocitat definida a
    config.velocitat_rodes (0-100). El motor de disparo es sempre HIGH/LOW.
    """

    def __init__(self, cfg, dry_run: bool = False, no_move: bool = False):
        self._cfg     = cfg
        self._dry_run = dry_run
        self._no_move = no_move
        self._GPIO    = None
        self._pwm_esq = None
        self._pwm_dre = None

        if dry_run:
            log.info("MotorController: MODE DRY-RUN")
            return

        try:
            import RPi.GPIO as GPIO
        except ImportError:
            _install(["RPi.GPIO"]); import RPi.GPIO as GPIO

        self._GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(cfg.motor_esq_pin,     GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(cfg.motor_dre_pin,     GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(cfg.motor_disparo_pin, GPIO.OUT, initial=GPIO.LOW)

        self._pwm_esq = GPIO.PWM(cfg.motor_esq_pin, cfg.motors_pwm_freq_hz)
        self._pwm_dre = GPIO.PWM(cfg.motor_dre_pin, cfg.motors_pwm_freq_hz)
        self._pwm_esq.start(0)
        self._pwm_dre.start(0)

        log.info(f"MotorController inicialitzat. Velocitat rodes: {cfg.velocitat_rodes}%")

    def avancar(self):
        if self._dry_run or self._no_move:
            log.debug("[DRY/NO-MOVE] AVANCAR")
            return
        vel = max(0, min(100, self._cfg.velocitat_rodes))
        self._pwm_esq.ChangeDutyCycle(vel)
        self._pwm_dre.ChangeDutyCycle(vel)

    def parar(self):
        if self._dry_run or self._no_move:
            log.debug("[DRY/NO-MOVE] PARAR")
            return
        self._pwm_esq.ChangeDutyCycle(0)
        self._pwm_dre.ChangeDutyCycle(0)

    def disparar(self, durada_s: float = None):
        if self._dry_run:
            print("DISPARO SIMULAT (dry-run)")
            return
        durada = durada_s if durada_s is not None else self._cfg.fire_pulse_ms / 1000.0
        print("DISPARO")
        self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.HIGH)
        time.sleep(durada)
        self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.LOW)
        log.info("Motor de disparo aturat.")

    def cleanup(self):
        self.parar()
        if self._pwm_esq:
            self._pwm_esq.stop()
        if self._pwm_dre:
            self._pwm_dre.stop()
        if self._GPIO:
            self._GPIO.output(self._cfg.motor_disparo_pin, self._GPIO.LOW)
        log.info("MotorController aturat.")


class RobotFSM:
    """
    Maquina d'estats del robot.

    En mode EXPLORANT el robot avanca contínuament a poc a poc mentre el
    servo PAN fa un swipe continu d'esquerra a dreta i viceversa (gestionat
    per un thread propi dins AimController, independent dels FPS).

    Quan es detecta un globus, la FSM surt immediatament de EXPLORANT.
    """

    COOLDOWN_DISPARO_S = 1.5

    def __init__(self, cfg, motors: MotorController):
        self._cfg    = cfg
        self._motors = motors
        self.fase    = EXPLORANT
        self._disparo_time: Optional[float] = None

        log.info(f"RobotFSM inicialitzat. Estat inicial: {self.fase}")

    def actualitzar(self,
                    objectiu_detectat: bool,
                    prou_a_prop: bool,
                    on_target: bool,
                    disparo_produit: bool) -> str:
        fase_anterior = self.fase

        if self.fase == EXPLORANT:
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
                log.info("Cooldown acabat. Tornant a explorar.")
                self._canviar_fase(EXPLORANT)

        if self.fase != fase_anterior:
            log.info(f"FSM: {fase_anterior} -> {self.fase}")

        # Si hem detectat un globus mentre exploravem, passem a aproximar
        if self.fase == EXPLORANT and objectiu_detectat:
            self._motors.parar()
            self._canviar_fase(APROXIMANT)
            log.info(f"FSM: {EXPLORANT} -> {APROXIMANT}")

        return self.fase

    def _canviar_fase(self, nova: str):
        self.fase = nova
        if nova in (APUNTANT, DISPARAT):
            self._motors.parar()
